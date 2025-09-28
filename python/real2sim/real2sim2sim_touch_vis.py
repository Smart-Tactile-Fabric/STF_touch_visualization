import time
import mujoco
import mujoco.viewer
import cv2
import numpy as np
import serial
import threading
import queue

# ========== 全局参数 ==========
GRID_SIZE = 16
ALPHA = 0.1                      # 时间平滑参数
BAUD_RATE = 1000000              # 串口波特率
SERIAL_PORT = '/dev/ttyUSB0'     # 串口端口
INIT_HEIGHT = -0.06              # 仿真物体初始Z轴高度
Z_SCALE = -0.0015                # Z轴映射比例（负号表示下凹）

# ========== 全局数据结构 ==========
raw_contact_data = np.zeros((GRID_SIZE, GRID_SIZE))
data_queue = queue.Queue()
data_buffer = []


# ========== 读取串口线程 ==========
def serial_reader_thread(ser):
    """ 从串口读取16x16触觉数据，并送入线程安全队列 """
    global raw_contact_data, data_buffer

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            values = [int(val) for val in line.split()]
            if len(values) == GRID_SIZE:
                data_buffer.append(values)
                if len(data_buffer) == GRID_SIZE:
                    raw_contact_data = np.array(data_buffer)
                    data_buffer.clear()
                    # data_queue.put(rearrange_data(raw_contact_data))  # 新版本——布料
                    data_queue.put(raw_contact_data)  # 旧版本


# ========== 数据重排逻辑 ==========
def rearrange_data(data):
    """ 将原始16x16数据中后8行倒序排布 """
    new_data = np.zeros_like(data)
    new_data[:8] = data[:8]
    new_data[8] = data[15]
    new_data[9] = data[14]
    new_data[10] = data[13]
    new_data[11] = data[12]
    new_data[12] = data[11]
    new_data[13] = data[10]
    new_data[14] = data[9]
    return new_data


# ========== 主程序入口 ==========
def main():
    # 初始化串口
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    ser.flush()
    threading.Thread(target=serial_reader_thread, args=(ser,), daemon=True).start()

    # 加载模型
    model = mujoco.MjModel.from_xml_path('/home/hjx/hjx_file/STF/STF_touch_visualization/python/real2sim/real2sim2sim_touch_stf.xml')
    data = mujoco.MjData(model)

    # 构建触觉传感器地址索引
    touch_sensor_addr = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            touch_sensor_addr[x, y] = model.sensor_adr[x * GRID_SIZE + y]

    # 初始化上一帧数据
    prev_touch_data = np.zeros((GRID_SIZE, GRID_SIZE))

    # 启动MuJoCo可视化
    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)

            # 从队列获取传感器数据
            touch_data = np.zeros_like(prev_touch_data)
            if not data_queue.empty():
                touch_data = data_queue.get()

            # 时间平滑处理
            touch_smoothed = ALPHA * touch_data + (1 - ALPHA) * prev_touch_data
            prev_touch_data = touch_smoothed.copy()

            # 将触觉值映射到仿真物体的Z轴高度
            for i in range(GRID_SIZE * GRID_SIZE):
                x_idx, y_idx = i // GRID_SIZE, i % GRID_SIZE
                z_offset = touch_smoothed[x_idx, y_idx] * Z_SCALE
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj_{i+1}")
                if joint_id >= 0:
                    data.qpos[joint_id] = INIT_HEIGHT + z_offset

            # 获取MuJoCo内部触觉传感器数据（sim）
            touch_sim_data = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
            for x in range(GRID_SIZE):
                for y in range(GRID_SIZE):
                    adr = touch_sensor_addr[x, y]
                    force_val = mujoco.mju_norm3(data.sensordata[adr:adr + 3])
                    touch_sim_data[x, y] = mujoco.mju_clip(force_val, 0.0, 3.0)

            # 显示仿真触觉热力图（sim）
            sim_heatmap = generate_heatmap(touch_sim_data)
            cv2.imshow("Touch Heatmap - Sim", sim_heatmap)

            viewer.sync()

            # 控制仿真步长
            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)

            cv2.waitKey(1)


# ========== 热力图生成 ==========
def generate_heatmap(matrix, colormap=cv2.COLORMAP_VIRIDIS, size=(480, 480)):
    """ 将16x16矩阵标准化后转为OpenCV热力图 """
    normalized = np.clip(matrix, 0, 3) / 3 * 255
    heatmap = cv2.applyColorMap(normalized.astype(np.uint8), colormap)
    return cv2.resize(heatmap, size)


# ========== 程序执行 ==========
if __name__ == "__main__":
    main()

