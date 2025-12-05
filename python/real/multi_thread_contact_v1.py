import numpy as np  # 导入NumPy库，用于处理数组和矩阵
import serial  # 导入串口通信库，用于与外部设备进行数据交换
import threading  # 导入线程库，用于多线程编程
import cv2  # 导入OpenCV库，用于图像处理和显示
import time  # 导入时间库，用于时间操作（例如延时和计算fps）
from scipy.ndimage import gaussian_filter  # 导入高斯滤波器，用于图像平滑
import queue  # 导入队列库，用于线程间安全地传递数据

# 初始化一个16x16的零矩阵，用于存储归一化后的传感器数据
contact_data_norm = np.zeros((16, 16))

# 根据contact_data_norm的尺寸来计算显示窗口的宽度和高度
WINDOW_WIDTH = contact_data_norm.shape[1] * 30
WINDOW_HEIGHT = contact_data_norm.shape[0] * 30

# 创建一个名为“Contact Data_left”的OpenCV窗口，并设置窗口大小为指定尺寸
cv2.namedWindow("Contact Data_left", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Contact Data_left", WINDOW_WIDTH, WINDOW_HEIGHT)

# 设置阈值和噪声缩放常数
THRESHOLD = 6
NOISE_SCALE = 20

# 定义全局变量
flag = False

# 创建一个队列用于线程间传递数据
data_queue = queue.Queue()

# 定义一个用于读取串口数据的线程函数
def readThread(serDev):
    global contact_data_norm, flag  # 使用全局变量
    data_tac = []  # 存储接收到的数据
    num = 0  # 记录已读取的数据数量
    t1 = 0  # 时间戳，用于计算fps
    backup = None  # 用于备份当前的传感器数据
    # flag = False  # 标记是否完成初始化
    current = None  # 存储当前读取的行数据

    while True:
        if serDev.in_waiting > 0:  # 如果串口有数据等待读取
            try:
                line = serDev.readline().decode('utf-8').strip()  # 读取一行数据并去掉末尾的换行符
            except:
                line = ""  # 如果读取出错，设置为空字符串
            if len(line) < 10:  # 如果行数据太短，说明读取到一行数据
                if current is not None and len(current) == 16:  # 如果当前数据行已完整
                    backup = np.array(current)  # 将当前数据转换为数组并备份
                    print("fps", 1 / (time.time() - t1))  # 计算并打印每秒帧数
                    t1 = time.time()  # 更新时间戳
                    data_tac.append(backup)  # 将备份数据添加到数据列表
                    num += 1  # 数据数量加1
                    if num > 30:  # 如果读取了超过30行数据
                        break  # 停止读取数据
                current = []  # 重置当前数据行
                continue  # 继续下一次循环
            if current is not None:
                str_values = line.split()  # 将读取的行数据按空格分割
                int_values = [int(val) for val in str_values]  # 将字符串转换为整数
                matrix_row = int_values  # 当前行数据
                current.append(matrix_row)  # 将当前行数据添加到current中

    data_tac = np.array(data_tac)  # 将收集到的数据转换为NumPy数组
    median = np.median(data_tac, axis=0)  # 对数据求中位数，去除噪声
    flag = True  # 标记数据已初始化完成
    print("Finish Initialization!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")  # 打印初始化完成的消息
    new = np.zeros((16, 16))  # 创建一个新的零矩阵，用于存储处理后的数据
    while True:
        if serDev.in_waiting > 0:  # 如果串口有数据等待读取
            try:
                line = serDev.readline().decode('utf-8').strip()  # 读取数据并去掉换行符
            except:
                line = ""  # 如果读取失败，设置为空字符串
            if len(line) < 10:  # 如果读取到一行数据并且长度小于10
                if current is not None and len(current) == 16:  # 如果当前行数据已完整
                    current_array = np.array(current)  # 将当前数据行转为NumPy数组
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
                current = []  # 重置当前数据行
                if backup is not None:
                    contact_data = backup - median - THRESHOLD  # 从备份数据中减去中位数和阈值
                    contact_data = np.clip(contact_data, 0, 100)  # 对数据进行裁剪，确保在0到100之间

                    # 如果最大数据小于阈值，进行噪声归一化
                    if np.max(contact_data) < THRESHOLD:
                        contact_data_norm = contact_data / NOISE_SCALE
                    else:
                        contact_data_norm = contact_data / np.max(contact_data)  # 否则对数据进行归一化

                    # 将处理后的数据加入队列
                    data_queue.put(contact_data_norm)
                continue  # 继续下一次循环
            if current is not None:
                str_values = line.split()  # 将读取的行数据按空格分割
                int_values = [int(val) for val in str_values]  # 将字符串转换为整数
                matrix_row = int_values  # 当前行数据
                current.append(matrix_row)  # 将当前行数据添加到current中

                continue  # 继续下一次循环

# 配置串口端口和波特率
BAUD = 1000000
serDev = serial.Serial('/dev/ttyUSB0', BAUD)  # 创建串口对象
exitThread = False  # 标记是否退出线程
serDev.flush()  # 清空串口缓存
# 创建并启动一个读取串口数据的线程
serialThread = threading.Thread(target=readThread, args=(serDev,))
serialThread.daemon = True  # 设置为守护线程，主线程结束时会自动结束
serialThread.start()  # 启动线程

# 定义高斯滤波函数
def apply_gaussian_blur(contact_map, sigma=0.1):
    return gaussian_filter(contact_map, sigma=sigma)  # 使用高斯滤波平滑图像

# 定义时间平滑滤波函数
def temporal_filter(new_frame, prev_frame, alpha=0.2):
    """
    Apply temporal smoothing filter.
    'alpha' determines the blending factor.
    A higher alpha gives more weight to the current frame, while a lower alpha gives more weight to the previous frame.
    """
    return alpha * new_frame + (1 - alpha) * prev_frame  # 计算时间滤波结果，平滑当前帧和上一帧

# 初始化上一帧数据
prev_frame = np.zeros_like(contact_data_norm)

# 主程序入口
if __name__ == '__main__':
    print('receive data test')  # 打印数据接收测试的消息

    while True:
        for i in range(300):  # 重复300次处理过程
            if not data_queue.empty():  # 检查队列中是否有新数据
                contact_data_norm = data_queue.get()  # 从队列中获取最新的数据
                # 对当前数据和上一帧进行时间滤波
                temp_filtered_data = temporal_filter(contact_data_norm, prev_frame)
                prev_frame = temp_filtered_data  # 更新上一帧为当前帧

                # 将数据缩放到0-255范围并转换为uint8类型
                temp_filtered_data_scaled = (temp_filtered_data * 255).astype(np.uint8)

                # 去掉最后一行和最后一列，生成15x15的矩阵
                contact_data_to_display = temp_filtered_data_scaled[:15, :15]

                # 使用颜色映射进行可视化
                # colormap = cv2.applyColorMap(temp_filtered_data_scaled, cv2.COLORMAP_VIRIDIS)
                colormap = cv2.applyColorMap(contact_data_to_display, cv2.COLORMAP_VIRIDIS)

                # 调整图像大小以适应窗口
                enlarged_image = cv2.resize(colormap, (WINDOW_WIDTH, WINDOW_HEIGHT), interpolation=cv2.INTER_LINEAR)

                # 显示彩色图像
                # cv2.imshow("Contact Data_left", colormap)
                cv2.imshow("Contact Data_left", enlarged_image)
                cv2.waitKey(1)  # 等待1毫秒，刷新显示
            time.sleep(0.01)  # 等待10毫秒
