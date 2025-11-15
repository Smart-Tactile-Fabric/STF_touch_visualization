from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter  # 用于控制仿真步进频率
import mink  # pip install mink
import queue  # 用于线程间安全传递数据
import serial  # 串口通信
import threading  # 用于多线程操作
import os
from tf.transformations import *  # 导入坐标变换工具

# 定义模型路径
model_xml_path = "/home/hjx/hjx_file/STF/STF_touch_visualization/python/real2sim/human_computer_interaction/car.xml"

# 初始化串口接收的触觉数据（16x16矩阵）
contact_data_norm = np.zeros((16, 16))  # 初始化16x16矩阵
data_queue = queue.Queue()  # 用于线程间安全传递数据
data_buffer = []  # 缓存一行传感器数据


def readThread(serDev):
    """
    串口读取线程，读取触觉数据并通过队列传递到主线程
    在此函数中，我们不断地从串口读取数据并将其存入队列中
    """
    global contact_data_norm, data_buffer
    while True:
        if serDev.in_waiting > 0:  # 如果串口有数据
            try:
                line = serDev.readline().decode('utf-8').strip()  # 读取一行数据
            except Exception as e:
                print(f"Error reading data: {e}")
                continue  # 读取失败时继续尝试读取

            # 如果读取到的行数据有效（长度为16）
            if len(line.split()) == 16:
                int_values = [int(val) for val in line.split()]  # 转换为整数
                data_buffer.append(int_values)  # 将这一行添加到缓存

                if len(data_buffer) == 16:  # 缓存已满，准备更新
                    contact_data_norm = np.array(data_buffer)  # 更新传感器数据矩阵
                    data_buffer.clear()  # 清空缓存，准备下一次读取
                    # 进行数据重排并通过队列传递给主线程
                    data_queue.put(rearrange_data(contact_data_norm))  # 调用重排函数并传递数据


def rearrange_data(current_array):
    """
    对读取到的数据进行重排处理
    将传感器数据的行进行交换，形成新的矩阵
    """
    new = np.zeros((16, 16))  # 创建一个新的零矩阵，用于存储处理后的数据

    # 处理当前数据并将其重新排列
    new[:8, :] = current_array[:8, :]  # 前8行保持不变
    new[8, :] = current_array[15, :]  # 16行变为第9行
    new[9, :] = current_array[14, :]  # 15行变为第10行
    new[10, :] = current_array[13, :]  # 14行变为第11行
    new[11, :] = current_array[12, :]  # 13行变为第12行
    new[12, :] = current_array[11, :]  # 12行变为第13行
    new[13, :] = current_array[10, :]  # 11行变为第14行
    new[14, :] = current_array[9, :]  # 10行变为第15行
    new[15, :] = current_array[8, :]  # 16行变为第9行

    return new  # 返回重排后的数据


def split_matrix(contact_data_norm):
    """
    将16x16矩阵分解为四个分块矩阵
    左上角、右上角、左下角、右下角
    """
    # 16x16矩阵分解为4个子矩阵
    top_left = contact_data_norm[:8, :8]  # 左上角
    top_right = contact_data_norm[:8, 8:]  # 右上角
    bottom_left = contact_data_norm[8:, :8]  # 左下角
    bottom_right = contact_data_norm[8:, 8:]  # 右下角

    return top_left, top_right, bottom_left, bottom_right


# 串口配置
BAUD = 1000000  # 设置串口波特率
serDev = serial.Serial('/dev/ttyUSB0', BAUD)  # 初始化串口
serDev.flush()  # 清空串口缓冲区

# 启动串口读取线程
serialThread = threading.Thread(target=readThread, args=(serDev,))
serialThread.daemon = True  # 设置为守护线程
serialThread.start()

if __name__ == "__main__":
    # 加载MuJoCo模型
    model = mujoco.MjModel.from_xml_path(model_xml_path)
    model.opt.timestep = 0.005  # 设置仿真步长（默认为0.001；可以加快仿真步速）

    # 初始化配置
    configuration = mink.Configuration(model)

    tasks = [
        base_task := mink.FrameTask(
            frame_name="car",  # 设定任务为车体
            frame_type="body",  # 物体类型为 body
            position_cost=0.1,  # 位置成本
            orientation_cost=1.0,  # 姿态成本
        ),
    ]

    model = configuration.model
    data = configuration.data
    solver = "quadprog"  # 求解器
    circle_radius = 0.5  # 圆半径

    # 查找底盘左右轮的控制器 index
    left_wheel_actuator_id = model.actuator("left").id
    right_wheel_actuator_id = model.actuator("right").id

    # 启动渲染器并进入仿真循环
    with mujoco.viewer.launch_passive(
            model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)  # 设置默认相机视角

        # Initialize to the home keyframe
        configuration.update_from_keyframe("home")  # 更新到起始关键帧
        base_task.set_target_from_configuration(configuration)  # 设置目标配置
        assert base_task.transform_target_to_world is not None

        # 设置仿真速率
        rate = RateLimiter(frequency=200.0, warn=False)  # 控制仿真更新速率
        dt = rate.period
        t = 0.0

        wheel_base = 1  # 车轮基距

        # 仿真主循环
        while viewer.is_running():

            # 获取触觉数据
            stf_max_data = np.max(contact_data_norm) / 200  # 最大压力数据缩放
            stf_sum_data = np.sum(contact_data_norm) / 100  # 总压力数据缩放
            # print(f"max:{stf_max_data}")
            # print(f"sum:{stf_sum_data}")
            # 统计矩阵中非零元素的个数
            stf_non_zero_count = np.count_nonzero(contact_data_norm)
            # print(f"Non-zero elements count: {stf_non_zero_count}")
            # print(contact_data_norm)

            # 获取分块矩阵
            top_left, top_right, bottom_left, bottom_right = split_matrix(contact_data_norm)
            # 打印每个分块矩阵
            # print("Top Left Block:")
            # print(top_left)
            # print("Top Right Block:")
            # print(top_right)
            # print("Bottom Left Block:")
            # print(bottom_left)
            # print("Bottom Right Block:")
            # print(bottom_right)

            top_left_max_data = np.max(top_left) / 200  # 最大压力数据缩放
            top_left_sum_data = np.sum(top_left) / 100  # 总压力数据缩放
            top_left_non_zero_count = np.count_nonzero(top_left)
            # print(f"Top left max:{top_left_max_data}")
            # print(f"Top left sum:{top_left_sum_data}")
            # print(f"Top left Non-zero elements count: {top_left_non_zero_count}")

            top_right_max_data = np.max(top_right) / 200  # 最大压力数据缩放
            top_right_sum_data = np.sum(top_right) / 100  # 总压力数据缩放
            top_right_non_zero_count = np.count_nonzero(top_right)
            # print(f"Top right max:{top_right_max_data}")
            # print(f"Top right sum:{top_right_sum_data}")
            # print(f"Top right Non-zero elements count: {top_right_non_zero_count}")

            bottom_left_max_data = np.max(bottom_left) / 200  # 最大压力数据缩放
            bottom_left_sum_data = np.sum(bottom_left) / 100  # 总压力数据缩放
            bottom_left_non_zero_count = np.count_nonzero(bottom_left)
            # print(f"Bottom left max:{bottom_left_max_data}")
            # print(f"Bottom left sum:{bottom_left_sum_data}")
            # print(f"Bottom left Non-zero elements count: {bottom_left_non_zero_count}")

            bottom_right_max_data = np.max(bottom_right) / 200  # 最大压力数据缩放
            bottom_right_sum_data = np.sum(bottom_right) / 100  # 总压力数据缩放
            bottom_right_non_zero_count = np.count_nonzero(bottom_right)
            # print(f"Bottom right max:{bottom_right_max_data}")
            # print(f"Bottom right sum:{bottom_right_sum_data}")
            # print(f"Bottom right Non-zero elements count: {bottom_right_non_zero_count}")

            # 控制速度和角速度（基于触觉数据）
            forward_speed, yaw_rate = 0.0, 0.0

            # 设定前进、后退及转向的条件
            if top_left_non_zero_count > 12 and top_left_max_data > 0.15:
                forward_speed += 0.2  # 前进
            if top_right_non_zero_count > 12 and top_right_max_data > 0.15:
                forward_speed -= 0.2  # 后退
            if bottom_left_non_zero_count > 12 and bottom_left_max_data > 0.15:
                yaw_rate += 0.2  # 逆时针转向
            if bottom_right_non_zero_count > 12 and bottom_right_max_data > 0.15:
                yaw_rate -= 0.2  # 顺时针转向

            # 根据控制信号计算左右轮速度
            v_l = forward_speed - yaw_rate * wheel_base / 2
            v_r = forward_speed + yaw_rate * wheel_base / 2

            # 更新车轮控制器的速度
            data.ctrl[left_wheel_actuator_id] = v_l
            data.ctrl[right_wheel_actuator_id] = v_r

            # 执行仿真步进
            mujoco.mj_step(model, data)

            # 固定帧率进行可视化
            viewer.sync()
            rate.sleep()  # 保持仿真速率一致
            t += dt
