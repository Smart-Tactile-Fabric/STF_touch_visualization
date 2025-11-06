import time
import mujoco
import mujoco.viewer
import cv2
import numpy as np

# 加载模型
m = mujoco.MjModel.from_xml_path('/home/hjx/hjx_file/STF/STF_touch_visualization/python/sim/sim_touch_stf.xml')
d = mujoco.MjData(m)

# 打印传感器数量
print(f"sensor_num: {m.nsensor}")

# 获取第一个触觉传感器的ID
sensor_id_stf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "touch_point_stf")

# 存储传感器的内存地址
touch_point_adr_stf = [[0] * 16 for _ in range(16)]

for x in range(16):
    for y in range(16):
        idx = x * 16 + y
        touch_point_adr_stf[x][y] = m.sensor_adr[idx]  # touch 传感器

# 启动渲染器并进入仿真循环
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():
        step_start = time.time()
        mujoco.mj_step(m, d)

        # 创建两个16x16的矩阵来存储两个触觉传感器的数据
        touch_stf = np.zeros((16, 16), dtype=np.float32)

        for x in range(16):
            for y in range(16):
                adr_stf = touch_point_adr_stf[x][y]

                # 获取第一个触觉传感器的数据
                data_stf = mujoco.mju_norm3(d.sensordata[adr_stf:adr_stf + 3])
                touch_stf[x, y] = mujoco.mju_clip(data_stf, 0.0, 3.0)

        # 将触觉传感器数据标准化到0到255之间
        touch_stf_normalized = np.clip(touch_stf, 0, 3) / 3 * 255

        # 使用OpenCV的COLORMAP_VIRIDIS来创建热力图效果
        touch_colored_stf = cv2.applyColorMap(touch_stf_normalized.astype(np.uint8), cv2.COLORMAP_VIRIDIS)

        # 调整热力图大小
        touch_colored_stf_resized = cv2.resize(touch_colored_stf, (480, 480))

        # 分别显示两个热力图在不同的窗口
        cv2.imshow("Touch Heatmap - Sensor", touch_colored_stf_resized)

        cv2.waitKey(1)

        # 同步仿真和渲染状态
        viewer.sync()

        # 简单的时间控制，保持与物理仿真步长同步
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
