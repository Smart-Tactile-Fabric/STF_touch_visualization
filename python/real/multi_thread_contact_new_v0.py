import numpy as np
import serial
import threading
import cv2
import time
from scipy.ndimage import gaussian_filter

# 初始化默认的接触数据数组（16x16）和窗口大小
contact_data_norm = np.zeros((16, 16))
WINDOW_WIDTH = contact_data_norm.shape[1] * 30
WINDOW_HEIGHT = contact_data_norm.shape[0] * 30

# 创建 OpenCV 窗口来显示接触数据
cv2.namedWindow("Contact Data_left", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Contact Data_left", WINDOW_WIDTH, WINDOW_HEIGHT)

# 定义阈值和噪声缩放因子
THRESHOLD = 12
NOISE_SCALE = 60

# 定义全局变量
flag = False

# 用于读取和处理串口数据的线程函数
def readThread(serDev):
    global contact_data_norm, flag
    data_tac = []  # 存储接触数据的帧
    num = 0  # 已处理的帧数
    t1 = 0  # 用于计算帧率（FPS）
    backup = None  # 保存上一帧数据
    # flag = False  # 初始化状态标志
    current = None  # 当前帧数据

    while True:
        # 如果串口有数据，开始读取
        if serDev.in_waiting > 0:
            try:
                line = serDev.readline().decode('utf-8').strip()  # 从串口读取一行数据
            except:
                line = ""

            if len(line) < 10:
                # 如果当前帧数据已经完整，保存并进行处理
                if current is not None and len(current) == 16:
                    backup = np.array(current)  # 备份当前帧
                    print("fps", 1 / (time.time() - t1))  # 打印帧率
                    t1 = time.time()  # 重置计时器
                    data_tac.append(backup)  # 将当前帧添加到数据列表
                    num += 1
                    if num > 30:  # 初始化完成后，处理30帧数据
                        break
                current = []  # 重置当前帧
                continue

            # 如果当前帧没有完成，继续读取数据
            if current is not None:
                str_values = line.split()
                int_values = [int(val) for val in str_values]  # 将字符串转换为整数
                current.append(int_values)  # 添加当前行数据到当前帧

    # 计算收集数据的中值，用于背景去除
    data_tac = np.array(data_tac)
    median = np.median(data_tac, axis=0)
    flag = True  # 设置标志为True，表示初始化完成
    print("初始化完成！")

    new = np.zeros((16, 16))  # 初始化新的帧数据

    # 主循环，持续读取并处理数据
    while True:
        if serDev.in_waiting > 0:
            try:
                line = serDev.readline().decode('utf-8').strip()  # 从串口读取一行数据
            except:
                line = ""

            if len(line) < 10:
                if current is not None and len(current) == 16:  # 如果当前帧完整
                    current_array = np.array(current)  # 将当前帧转为numpy数组
                    temp = 0
                    # 重新排列行数据，调整帧的顺序
                    # 标准版布料
                    new[:15, :] = current_array[:15, :] + temp  # 前15行保持不变

                    # 定制版布料
                    # new[8, :] = current_array[15, :] + temp
                    # new[9, :] = current_array[14, :] + temp
                    # new[10, :] = current_array[13, :] + temp
                    # new[11, :] = current_array[12, :] + temp
                    # new[12, :] = current_array[11, :] + temp
                    # new[13, :] = current_array[10, :] + temp
                    # new[14, :] = current_array[9, :] + temp
                    # new[15, :] = current_array[8, :] + temp

                    # print(current_array[:])  # 布料标定测试用
                    print("current_array:")
                    for i in range(16):
                        print("{", end=" ")
                        print(*current_array[i], sep=", ", end=" },\n")  # 布料标定测试用

                backup = np.array(new)  # 备份当前帧

                current = []  # 重置当前帧
                if backup is not None:
                    # 去除背景并应用阈值
                    contact_data = backup - median - THRESHOLD
                    contact_data = np.clip(contact_data, 0, 100)  # 将数据限制在0到100之间

                    # 数据归一化
                    if np.max(contact_data) < THRESHOLD:
                        contact_data_norm = contact_data / NOISE_SCALE  # 如果最大值小于阈值，应用噪声缩放
                    else:
                        contact_data_norm = contact_data / np.max(contact_data)  # 否则按最大值归一化

                continue

            if current is not None:
                str_values = line.split()
                int_values = [int(val) for val in str_values]  # 将字符串转换为整数
                current.append(int_values)  # 将当前行数据添加到当前帧


# 设置串口
PORT = '/dev/ttyUSB0'
BAUD = 1000000
serDev = serial.Serial(PORT, BAUD)  # 打开串口
serDev.flush()  # 清空串口输入缓冲区

# 启动串口读取线程
serialThread = threading.Thread(target=readThread, args=(serDev,))
serialThread.daemon = True  # 设置为守护线程，主程序退出时线程也会退出
serialThread.start()


# 高斯模糊函数，用于平滑处理
def apply_gaussian_blur(contact_map, sigma=0.1):
    return gaussian_filter(contact_map, sigma=sigma)


# 时间滤波器，用于对帧进行平滑处理
def temporal_filter(new_frame, prev_frame, alpha=0.2):
    """
    应用时间滤波器进行平滑。
    'alpha' 决定了混合因子。
    较高的alpha给当前帧更多权重，而较低的alpha给前一帧更多权重。
    """
    return alpha * new_frame + (1 - alpha) * prev_frame


# 初始化前一帧的缓冲区
prev_frame = np.zeros_like(contact_data_norm)

# 主循环，显示处理后的接触数据
if __name__ == '__main__':
    print('开始接收数据测试')

    while True:
        for i in range(300):
            if flag:  # 检查初始化是否完成
                # 应用时间滤波器平滑数据
                temp_filtered_data = temporal_filter(contact_data_norm, prev_frame)
                prev_frame = temp_filtered_data  # 更新前一帧

                # 将数据缩放到0-255并转换为uint8类型
                temp_filtered_data_scaled = (temp_filtered_data * 255).astype(np.uint8)

                # 去掉最后一行和最后一列，生成15x15的矩阵
                contact_data_to_display = temp_filtered_data_scaled[:15, :15]

                # 使用颜色映射进行可视化
                # colormap = cv2.applyColorMap(temp_filtered_data_scaled, cv2.COLORMAP_VIRIDIS)
                colormap = cv2.applyColorMap(contact_data_to_display, cv2.COLORMAP_VIRIDIS)

                # 使用OpenCV显示处理后的接触数据
                cv2.imshow("Contact Data_left", colormap)
                cv2.waitKey(1)  # 每次显示1毫秒

            time.sleep(0.01)  # 小的延迟，确保平滑执行
