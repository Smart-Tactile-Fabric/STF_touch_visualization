import numpy as np
import serial
import threading
import cv2
import time
from scipy.ndimage import gaussian_filter

# import matplotlib.pyplot as plt
# import seaborn as sns
# os.system('cls')

contact_data_norm = np.zeros((16, 16))
WINDOW_WIDTH = 800  # 调整窗口大小以适应新的布局
WINDOW_HEIGHT = 600
cv2.namedWindow("Tac", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Tac", WINDOW_WIDTH, WINDOW_HEIGHT)
THRESHOLD = 12
NOISE_SCALE = 20


def readThread(serDev):
    global contact_data_norm, flag
    data_tac = []
    num = 0
    t1 = 0
    backup = None
    flag = False
    current = None
    while True:
        if serDev.in_waiting > 0:
            try:
                line = serDev.readline().decode('utf-8').strip()
            except:
                line = ""
            if len(line) < 10:
                if current is not None and len(current) == 16:
                    backup = np.array(current)
                    print("fps", 1 / (time.time() - t1))
                    t1 = time.time()
                    data_tac.append(backup)
                    num += 1
                    if num > 30:
                        break
                current = []
                continue
            if current is not None:
                str_values = line.split()
                int_values = [int(val) for val in str_values]
                matrix_row = int_values
                current.append(matrix_row)

    data_tac = np.array(data_tac)
    median = np.median(data_tac, axis=0)
    flag = True
    print("Finish Initialization!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    new = np.zeros((16, 16))
    while True:
        if serDev.in_waiting > 0:
            try:
                line = serDev.readline().decode('utf-8').strip()
            except:
                line = ""
            if len(line) < 10:
                if current is not None and len(current) == 16:
                    current_array = np.array(current)
                    new = current_array  # 前8行保持不变

                    backup = np.array(new)
                current = []
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
                matrix_row = int_values
                current.append(matrix_row)
                continue


def apply_gaussian_blur(contact_map, sigma=0.1):
    return gaussian_filter(contact_map, sigma=sigma)


def temporal_filter(new_frame, prev_frame, alpha=0.5):
    return alpha * new_frame + (1 - alpha) * prev_frame


def split_and_display_fingers(contact_data):
    """
    将触觉矩阵分割显示:
    - 前3行与后12行分割开
    - 前3行每3列分割开，形成5个3x3的手指区域
    """
    # 确保数据是15x15
    if contact_data.shape != (15, 15):
        contact_data = contact_data[:15, :15]

    # 分割矩阵
    top_part = contact_data[:, :3]  # 前3行
    bottom_part = contact_data[:, 3:]  # 后12行
    bottom_part = bottom_part.T

    # 创建显示图像5
    display_height = 600
    display_width = 400

    # 创建黑色背景
    display_img = np.zeros((display_height, display_width, 3), dtype=np.uint8)

    # 设置区域参数
    top_height = 150  # 顶部区域高度
    bottom_height = 400  # 底部区域高度
    margin = 20  # 边距

    # 显示顶部区域（5个手指，每个3x3）
    finger_width = (display_width - 1 * margin) // 5

    for i in range(5):
        # 每个手指的3列数据
        start_col = i * 3
        end_col = (i + 1) * 3

        # 提取手指数据
        finger_data = top_part[start_col:end_col, :]

        # 缩放为3x3显示区域
        finger_display = cv2.resize(finger_data, (finger_width - margin, top_height - margin), interpolation=cv2.INTER_NEAREST)

        # 转换为彩色图
        finger_colored = cv2.applyColorMap((finger_display * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)

        # 计算显示位置
        x_pos = margin + i * (finger_width)
        y_pos = margin

        # 将手指数据放到显示图像上
        display_img[y_pos:y_pos + top_height - margin, x_pos:x_pos + finger_width - margin] = finger_colored

        # 添加手指标签
        cv2.putText(display_img, f'F{i + 1}',
                    (x_pos + 10, y_pos + top_height - margin - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # 显示分割线
    cv2.line(display_img, (0, top_height + margin // 2), (display_width, top_height + margin // 2),
             (100, 100, 100), 2)

    # 显示底部区域（后12行）
    bottom_display = cv2.resize(bottom_part,
                                (display_width - 2 * margin, bottom_height - 2 * margin), interpolation=cv2.INTER_NEAREST)
    bottom_colored = cv2.applyColorMap((bottom_display * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)

    bottom_y_pos = top_height + margin
    display_img[bottom_y_pos:bottom_y_pos + bottom_height - 2 * margin,
    margin:margin + display_width - 2 * margin] = bottom_colored

    # 添加区域标签
    cv2.putText(display_img, 'Top: 5 Fingers (3x3 each)', (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(display_img, 'Bottom: Palm Area', (10, bottom_y_pos + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    return display_img


# 初始化串口通信
PORT = '/dev/ttyUSB0'
BAUD = 1000000
serDev = serial.Serial(PORT, BAUD)
exitThread = False
serDev.flush()
serialThread = threading.Thread(target=readThread, args=(serDev,))
serialThread.daemon = True
serialThread.start()

# 初始化前一帧缓冲区
prev_frame = np.zeros_like(contact_data_norm)

if __name__ == '__main__':
    print('receive data test')

    while True:
        for i in range(300):
            if flag:
                # 应用时间滤波
                temp_filtered_data = temporal_filter(contact_data_norm, prev_frame)
                prev_frame = temp_filtered_data

                # 获取15x15的数据
                display_data = temp_filtered_data[:15, :15]

                # 分割并显示手指
                display_img = split_and_display_fingers(display_data)

                cv2.imshow("Tac", display_img)
                cv2.waitKey(1)
            time.sleep(0.01)
