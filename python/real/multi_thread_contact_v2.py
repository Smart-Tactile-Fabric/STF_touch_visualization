import numpy as np
import serial
import threading
import cv2
import time
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 初始化数据
contact_data_norm = np.zeros((16, 16))
WINDOW_WIDTH = contact_data_norm.shape[1] * 30
WINDOW_HEIGHT = contact_data_norm.shape[0] * 30
cv2.namedWindow("Contact Data_left", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Contact Data_left", WINDOW_WIDTH, WINDOW_HEIGHT)

# 设置阈值和噪声缩放常数
THRESHOLD = 6
NOISE_SCALE = 20

# 定义全局变量
flag = False

# 初始化3D图形
plt.ion()
fig = plt.figure(figsize=(8, 6))  # 调整为更合适的尺寸

# 只创建一个3D子图
ax_3d = fig.add_subplot(111, projection='3d')

# 创建网格坐标
x = np.arange(contact_data_norm.shape[1])
y = np.arange(contact_data_norm.shape[0])
x, y = np.meshgrid(x, y)

# 动画参数
amplitude = 0.5  # 跳动幅度
frequency = 0.5  # 跳动频率
start_time = time.time()

# 定义统一的颜色映射
COLORMAP = plt.cm.hot  # 使用与OpenCV的HOT colormap对应的matplotlib colormap


def update_3d_visualization(z_data):
    """更新3D动态可视化 - 只显示3D视图"""
    ax_3d.clear()

    # 1. 数据处理优化
    # 添加轻微的高斯平滑使表面更连续
    z_smoothed = gaussian_filter(z_data, sigma=0.8)

    # 2. 动态效果优化
    current_time = time.time() - start_time
    # 使用更复杂的动态因子组合
    dynamic_factor = 0.3 * np.sin(2 * np.pi * 0.5 * current_time) + \
                     0.1 * np.sin(2 * np.pi * 1.2 * current_time + 1) + 1
    z_dynamic = z_smoothed * dynamic_factor

    # 3. 颜色映射优化 - 使用与2D相同的颜色映射
    norm = plt.Normalize(vmin=0, vmax=1)
    colors = COLORMAP(norm(z_dynamic))

    # 4. 3D绘图优化
    surf = ax_3d.plot_surface(
        x, y, z_dynamic, facecolors=colors,
        rstride=1, cstride=1, linewidth=0.1,
        edgecolor='white', alpha=0.9,
        antialiased=True, shade=True
    )

    # 5. 光照效果增强
    ax_3d.set_facecolor((0.95, 0.95, 0.95))
    ax_3d.zaxis.set_rotate_label(False)

    # 6. 专业标注和布局
    ax_3d.set_title('3D Tactile Pressure Map\n', fontsize=12, pad=15, fontweight='bold')
    ax_3d.set_xlabel('\nX Position', fontsize=10, labelpad=10)
    ax_3d.set_ylabel('\nY Position', fontsize=10, labelpad=10)
    ax_3d.set_zlabel('\nPressure Intensity', fontsize=10, labelpad=10, rotation=90)

    # 7. 视角和范围优化
    ax_3d.set_zlim(0, 1.5)
    ax_3d.view_init(elev=38, azim=-135)
    ax_3d.dist = 9  # 调整相机距离

    plt.tight_layout()
    plt.draw()
    plt.pause(0.00001)


def readThread(serDev):
    global contact_data_norm, flag
    data_tac = []
    num = 0
    t1 = 0
    backup = None
    # flag = False
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
    new = np.zeros((16, 16))  # 创建一个新的零矩阵，用于存储处理后的数据
    while True:
        if serDev.in_waiting > 0:
            try:
                line = serDev.readline().decode('utf-8').strip()
                # print("fps",1/(time.time()-t1))
                # t1 =time.time()
            except:
                line = ""
            if len(line) < 10:
                if current is not None and len(current) == 16:
                    # backup = np.array(current)
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
                current = []
                if backup is not None:
                    contact_data = backup - median - THRESHOLD
                    contact_data = np.clip(contact_data, 0, 100)

                    if np.max(contact_data) < THRESHOLD:
                        contact_data_norm = contact_data / NOISE_SCALE
                    else:
                        # contact_data_norm = np.log(contact_data + 1) / np.log(2.0)
                        contact_data_norm = contact_data / np.max(contact_data)

                continue
            if current is not None:
                str_values = line.split()
                int_values = [int(val) for val in str_values]
                matrix_row = int_values
                current.append(matrix_row)

                continue


# PORT = "left_gripper_right_finger"
PORT = '/dev/ttyUSB0'
BAUD = 1000000
serDev = serial.Serial(PORT, BAUD)
exitThread = False
serDev.flush()
serialThread = threading.Thread(target=readThread, args=(serDev,))
serialThread.daemon = True
serialThread.start()


def apply_gaussian_blur(contact_map, sigma=0.1):
    return gaussian_filter(contact_map, sigma=sigma)


def temporal_filter(new_frame, prev_frame, alpha=0.2):
    """
    Apply temporal smoothing filter.
    'alpha' determines the blending factor.
    A higher alpha gives more weight to the current frame, while a lower alpha gives more weight to the previous frame.
    """
    return alpha * new_frame + (1 - alpha) * prev_frame


# Initialize previous frame buffer
prev_frame = np.zeros_like(contact_data_norm)

if __name__ == '__main__':
    print('开始接收数据测试')
    flag = True
    zoom_factor = 45
    prev_frame = np.zeros_like(contact_data_norm)

    while True:
        if flag:
            # 数据处理流程保持不变
            temp_filtered_data = temporal_filter(contact_data_norm, prev_frame)
            prev_frame = temp_filtered_data
            temp_filtered_data_enhanced = np.power(temp_filtered_data, 0.7)
            smoothed_data = gaussian_filter(temp_filtered_data_enhanced, sigma=0.2)

            # 统一颜色处理
            normalized_data = (smoothed_data * 255).astype(np.uint8)

            # 去掉最后一行和最后一列，生成15x15的矩阵
            contact_data_to_display = normalized_data[:15, :15]

            # 2D显示 - 使用HOT colormap
            # colormap = cv2.applyColorMap(normalized_data, cv2.COLORMAP_HOT)
            colormap = cv2.applyColorMap(contact_data_to_display, cv2.COLORMAP_HOT)

            # 显示原始2D触觉图像
            height, width = colormap.shape[:2]
            enlarged_colormap = cv2.resize(colormap,
                                           (width * zoom_factor, height * zoom_factor),
                                           interpolation=cv2.INTER_LANCZOS4)
            cv2.imshow("Contact Data_left", enlarged_colormap)
            cv2.waitKey(1)

            # 3D显示 - 使用相同的颜色映射
            update_3d_visualization(smoothed_data)

        time.sleep(0.01)