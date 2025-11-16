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

# 线性加速函数
def get_accelerating_forward_speed(t, max_speed=0.2, start_time=2, end_time=6):
    """
    根据时间段控制前进速度，支持线性加速
    :param t: 当前时间
    :param max_speed: 最大前进速度
    :param start_time: 开始加速的时间
    :param end_time: 停止加速的时间
    :return: 计算得到的前进速度
    """
    if start_time <= t < end_time:
        # 线性加速：从0加速到max_speed
        return max_speed * (t - start_time) / (end_time - start_time)
    else:
        return max_speed  # 达到最大速度后，保持最大速度


# 线性减速函数
def get_decelerating_forward_speed(t, max_speed=0.2, start_time=6, end_time=10):
    """
    根据时间段控制前进速度，支持线性减速
    :param t: 当前时间
    :param max_speed: 最大前进速度
    :param start_time: 开始减速的时间
    :param end_time: 停止减速的时间
    :return: 计算得到的前进速度
    """
    if start_time <= t < end_time:
        # 线性减速：从max_speed降到0
        return max_speed * (1 - (t - start_time) / (end_time - start_time))
    else:
        return 0  # 停止


# 转向加速函数
def get_accelerating_yaw_rate(t, max_yaw_rate=0.5, start_time=11, end_time=15):
    """
    根据时间段控制转向速度，支持线性加速
    :param t: 当前时间
    :param max_yaw_rate: 最大转向速度
    :param start_time: 开始加速的时间
    :param end_time: 停止加速的时间
    :return: 计算得到的转向速度
    """
    if start_time <= t < end_time:
        # 线性加速：逐渐从0增加到max_yaw_rate
        return max_yaw_rate * (t - start_time) / (end_time - start_time)
    else:
        return max_yaw_rate  # 达到最大转向速度后，保持最大速度


# 转向减速函数
def get_decelerating_yaw_rate(t, max_yaw_rate=0.5, start_time=15, end_time=20):
    """
    根据时间段控制转向速度，支持线性减速
    :param t: 当前时间
    :param max_yaw_rate: 最大转向速度
    :param start_time: 开始减速的时间
    :param end_time: 停止减速的时间
    :return: 计算得到的转向速度
    """
    if start_time <= t < end_time:
        # 线性减速：逐渐从max_yaw_rate减小到0
        return max_yaw_rate * (1 - (t - start_time) / (end_time - start_time))
    else:
        return 0  # 停止转向


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

        # 初始化到“home”关键帧
        configuration.update_from_keyframe("home")  # 更新到起始关键帧
        base_task.set_target_from_configuration(configuration)  # 设置目标配置
        assert base_task.transform_target_to_world is not None

        # 设置仿真速率
        rate = RateLimiter(frequency=200.0, warn=False)  # 控制仿真更新速率
        dt = rate.period
        t = 0.0  # 仿真时间初始化

        wheel_base = 1  # 车轮基距

        # 仿真主循环
        while viewer.is_running():
            t += dt  # 更新时间

            # 基于时间控制小车的运动
            if 2 <= t < 6:
                # 前进 线性加速：从0加速到0.4
                forward_speed = get_accelerating_forward_speed(t, max_speed=0.4, start_time=2, end_time=6)
                yaw_rate = 0.0  # 不转向
            elif 6 <= t < 9:
                # 前进 线性减速：从0.4降到0
                forward_speed = get_decelerating_forward_speed(t, max_speed=0.4, start_time=6, end_time=9)
                yaw_rate = 0.0  # 不转向
            elif 11 <= t < 14:
                # 后退 线性加速：从0加速到0.4
                forward_speed = -1 * get_accelerating_forward_speed(t, max_speed=0.4, start_time=11, end_time=14)
                yaw_rate = 0.0  # 不转向
            elif 14 <= t < 17:
                # 后退 线性减速：从0.4降到0
                forward_speed = -1 * get_decelerating_forward_speed(t, max_speed=0.4, start_time=14, end_time=17)
                yaw_rate = 0.0  # 不转向
            elif 19 <= t < 26:
                # 左转 线性加速
                forward_speed = 0.0  # 停止
                yaw_rate = get_accelerating_yaw_rate(t, max_yaw_rate=0.5, start_time=19, end_time=26)
            elif 26 <= t < 33:
                # 左转 线性减速
                forward_speed = 0.0  # 前进
                yaw_rate = get_decelerating_yaw_rate(t, max_yaw_rate=0.5, start_time=26, end_time=33)
            elif 35 <= t < 40:
                # 右转 线性加速
                forward_speed = 0.0  # 停止
                yaw_rate = -1 * get_accelerating_yaw_rate(t, max_yaw_rate=0.5, start_time=35, end_time=40)
            elif 40 <= t < 45:
                # 右转 线性减速
                forward_speed = 0.0  # 前进
                yaw_rate = -1 * get_decelerating_yaw_rate(t, max_yaw_rate=0.5, start_time=40, end_time=45)
            else:
                forward_speed = 0.0  # 停止
                yaw_rate = 0.0  # 停止

            # 计算左右轮速度
            v_l = forward_speed - yaw_rate * wheel_base / 2
            v_r = forward_speed + yaw_rate * wheel_base / 2

            # 更新轮速
            data.ctrl[left_wheel_actuator_id] = v_l
            data.ctrl[right_wheel_actuator_id] = v_r

            # 执行仿真步进
            mujoco.mj_step(model, data)

            # 同步显示
            viewer.sync()
            rate.sleep()  # 保持仿真速率一致
