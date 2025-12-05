import numpy as np
import serial
import threading
import time
from scipy.ndimage import gaussian_filter
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 初始化接触数据矩阵
contact_data_norm = np.zeros((16, 16))  # 初始化16x16的接触数据矩阵

# 设置阈值和噪声缩放因子
THRESHOLD = 12
NOISE_SCALE = 60

# 定义全局变量
flag = False

# 串口数据读取线程
def readThread(serDev):
    global contact_data_norm, flag
    data_tac = []  # 存储接触数据的帧
    num = 0  # 已处理的帧数
    t1 = 0  # 用于计算帧率（FPS）
    backup = None  # 保存上一帧数据
    # flag = False  # 初始化状态标志
    current = None  # 当前帧数据

    while True:
        if serDev.in_waiting > 0:  # 如果串口有数据
            try:
                line = serDev.readline().decode('utf-8').strip()  # 从串口读取一行数据
            except:
                line = ""

            if len(line) < 10:  # 如果当前帧数据已经完整
                if current is not None and len(current) == 16:
                    backup = np.array(current)  # 备份当前帧
                    print("fps", 1 / (time.time() - t1))  # 打印帧率
                    t1 = time.time()  # 重置计时器
                    data_tac.append(backup)  # 将当前帧添加到数据列表
                    num += 1
                    if num > 30:  # 如果处理了30帧，结束初始化
                        break
                current = []  # 重置当前帧
                continue

            if current is not None:
                str_values = line.split()  # 分割字符串
                int_values = [int(val) for val in str_values]  # 转换为整数
                current.append(int_values)  # 添加当前行数据到当前帧

    data_tac = np.array(data_tac)
    median = np.median(data_tac, axis=0)
    flag = True  # 初始化完成
    print("初始化完成！")
    new = np.zeros((16, 16))  # 初始化新的一帧数据
    while True:
        if serDev.in_waiting > 0:  # 如果串口有数据
            try:
                line = serDev.readline().decode('utf-8').strip()  # 从串口读取数据
            except:
                line = ""
            if len(line) < 10:  # 如果当前帧完整
                if current is not None and len(current) == 16:
                    current_array = np.array(current)  # 转换为numpy数组
                    temp = 0
                    # 重新排列行数据，调整帧的顺序
                    # 标准版布料
                    new[:15, :] = current_array[:15, :] + temp  # 前15行保持不变

                    # 定制版布料
                    # new[:8, :] = current_array[:8, :] + temp  # 前8行保持不变
                    # new[8, :] = current_array[15, :] + temp
                    # new[9, :] = current_array[14, :] + temp
                    # new[10, :] = current_array[13, :] + temp
                    # new[11, :] = current_array[12, :] + temp
                    # new[12, :] = current_array[11, :] + temp
                    # new[13, :] = current_array[10, :] + temp
                    # new[14, :] = current_array[9, :] + temp
                    # new[15, :] = current_array[8, :] + temp

                backup = np.array(new)  # 备份当前帧
                current = []  # 重置当前帧
                if backup is not None:
                    contact_data = backup - median - THRESHOLD
                    contact_data = np.clip(contact_data, 0, 100)
                    if np.max(contact_data) < THRESHOLD:
                        contact_data_norm = contact_data / NOISE_SCALE
                    else:
                        contact_data_norm = contact_data / np.max(contact_data)
                continue
            if current is not None:
                str_values = line.split()
                int_values = [int(val) for val in str_values]
                current.append(int_values)


# 设置串口
PORT = '/dev/ttyUSB0'
BAUD = 1000000
serDev = serial.Serial(PORT, BAUD)  # 打开串口
serDev.flush()  # 清空串口输入缓冲区
serialThread = threading.Thread(target=readThread, args=(serDev,))
serialThread.daemon = True
serialThread.start()

# 初始化图表
fig, ax = plt.subplots()  # 创建图表和坐标轴
x_vals = []  # 用于存储时间点
y_vals = []  # 用于存储sum_contact_data值

# 初始化计数器
frame_count = 0


# 更新曲线图的函数
def update_plot(frame):
    global frame_count, x_vals, y_vals

    # 计算当前的sum_contact_data
    sum_contact_data = np.sum(contact_data_norm)

    # 添加当前的时间和sum_contact_data值
    x_vals.append(frame_count)
    y_vals.append(sum_contact_data)

    # 限制图表的显示范围（只显示最后100个数据点）
    if len(x_vals) > 100:
        x_vals = x_vals[1:]
        y_vals = y_vals[1:]

    # 清空当前图表
    ax.clear()

    # 绘制新的曲线
    ax.plot(x_vals, y_vals, label="sum_contact_data")

    # 设置图表标题和标签
    ax.set_title("Real-Time sum_contact_data Visualization")
    ax.set_xlabel("Time (frames)")
    ax.set_ylabel("sum_contact_data")

    # 增加图例
    ax.legend()

    # 更新计数器
    frame_count += 1

# 创建动画
ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False)

# 显示图表
plt.show()
