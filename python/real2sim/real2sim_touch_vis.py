import time
import mujoco
import mujoco.viewer
import cv2
import numpy as np
import serial
import threading
import queue

# 初始化串口接收的触觉数据
contact_data_norm = np.zeros((16, 16))  # 16x16矩阵
data_queue = queue.Queue()  # 用于线程间安全传递数据
data_buffer = []  # 缓存一行传感器数据

# 平滑参数
ALPHA = 0.1  # 影响当前数据和平滑数据的比例

def readThread(serDev):
    """串口读取线程，读取触觉数据并通过队列传递到主线程"""
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
                    data_queue.put(contact_data_norm)  # 调用重排函数: old
                    # data_queue.put(rearrange_data(contact_data_norm))  # 调用重排函数: new
                    # print("Updated contact data:", contact_data_norm)

def rearrange_data(current_array):
    """对读取到的数据进行重排处理"""
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

    return new  # 返回重排后的数据

# 串口配置
BAUD = 1000000
serDev = serial.Serial('/dev/ttyUSB0', BAUD)
serDev.flush()

# 启动串口读取线程
serialThread = threading.Thread(target=readThread, args=(serDev,))
serialThread.daemon = True
serialThread.start()

# 加载MuJoCo模型
m = mujoco.MjModel.from_xml_path('/home/hjx/hjx_file/STF/STF_touch_visualization/python/real2sim/real2sim_touch_stf.xml')
d = mujoco.MjData(m)

# 获取触觉传感器的ID
sensor_id_stf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "touch_point_stf")
if sensor_id_stf == -1:
    print("------")

# 存储传感器地址
touch_point_adr_stf = np.zeros((16, 16), dtype=int)
for x in range(16):
    for y in range(16):
        touch_point_adr_stf[x, y] = m.sensor_adr[x * 16 + y]  # 更新传感器地址

# 启动渲染器并进入仿真循环
with mujoco.viewer.launch_passive(m, d) as viewer:
    prev_data = np.zeros((16, 16))  # 初始化上一帧数据

    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(m, d)  # 更新仿真步骤

        # 从队列中获取最新的触觉数据
        touch_stf = np.zeros((16, 16))  # 默认空数据
        if not data_queue.empty():
            touch_stf = data_queue.get()

        # 使用平滑滤波处理触觉数据
        touch_stf = ALPHA * touch_stf + (1 - ALPHA) * prev_data  # 平滑数据
        prev_data = touch_stf.copy()  # 更新上一帧数据

        # 标准化触觉数据并映射到0-255
        touch_stf_normalized = np.clip(touch_stf, 0, 3) / 3 * 255
        # print(f"Normalized touch data: {touch_stf_normalized}")  # 打印标准化后的数据

        # 使用OpenCV的COLORMAP_VIRIDIS来创建热力图效果
        touch_colored_stf = cv2.applyColorMap(touch_stf_normalized.astype(np.uint8), cv2.COLORMAP_VIRIDIS)

        # 调整热力图大小以适应窗口
        touch_colored_stf_resized = cv2.resize(touch_colored_stf, (480, 480))

        # 显示热力图
        # cv2.imshow("Touch Heatmap - Sensor", touch_colored_stf_resized)

        # 更新MuJoCo仿真中的物体位置（按压力变化）
        for i in range(256):  # 更新为225个物体（15x15矩阵）
            z_offset = touch_stf[i // 16, i % 16] * 0.0015  # 映射传感器数据到 Z 轴高度

            qpos_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"obj_{i+1}")
            if qpos_id >= 0:  # 确保该物体存在
                d.qpos[qpos_id] = 0.02 + z_offset  # 更新物体的Z坐标
                # print(f"Object_{i+1} Z position: {d.qpos[qpos_id]}")  # 打印物体的Z位置

        # 同步仿真和渲染状态
        viewer.sync()

        # 保持与物理仿真步长同步
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

        cv2.waitKey(1)  # 等待1毫秒，刷新显示
