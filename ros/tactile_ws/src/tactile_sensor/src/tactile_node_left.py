#!/usr/bin/env python

import serial
import numpy as np
import cv2
import time
import rospy
import threading
from std_msgs.msg import Float64MultiArray


def temporal_filter(new_frame, prev_frame, alpha=1):
    """
    Apply temporal smoothing filter.
    'alpha' determines the blending factor.
    A higher alpha gives more weight to the current frame, while a lower alpha gives more weight to the previous frame.
    """
    return alpha * new_frame + (1 - alpha) * prev_frame


def tactile_publisher(tactile_name, alpha=None):
    mean = np.zeros((16, 16))
    # Open the serial port
    serDev = serial.Serial(f'{tactile_name}', 1000000)

    serDev.flush()

    # Create a publisher object
    tactile_pub = rospy.Publisher(f'{tactile_name}', Float64MultiArray, queue_size=10)
    # Define the rate of publishing
    rate = rospy.Rate(30)  # 30Hz
    print("Start")
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
                    if num > 20:
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
    print(f"{tactile_name} Finish Initialization!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    current = None
    prev_frame = np.zeros_like(median)

    new = np.zeros((16, 16))  # 初始化新的帧数据

    while True:
        if serDev.in_waiting > 0:
            try:
                line = serDev.readline().decode('utf-8').strip()
            except:
                line = ""
            # print(len(line))
            if len(line) < 10:
                if current is not None and len(current) == 16:
                    current_array = np.array(current)  # 将当前帧转为numpy数组
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

                backup = np.array(new)
                current = []
                if backup is not None:
                    contact_data = backup - median - 12
                    contact_data = np.clip(contact_data, 0, 100)

                    if np.max(contact_data) < 12:
                        contact_data_norm = contact_data / 100
                    else:
                        contact_data_norm = contact_data / np.max(contact_data)
                    if alpha is not None:
                        contact_data_norm = temporal_filter(contact_data_norm, prev_frame, alpha=alpha)
                        prev_frame = contact_data_norm
                    tactile_msg = Float64MultiArray()
                    tactile_msg.data = contact_data_norm.flatten().tolist()
                    # Publish the message
                    tactile_pub.publish(tactile_msg)
                    # print("fps",1/(time.time()-t1))
                    # t1 =time.time()
                continue
            if current is not None:
                str_values = line.split()
                int_values = [int(val) for val in str_values]
                matrix_row = int_values
                current.append(matrix_row)
                continue


if __name__ == '__main__':
    rospy.init_node('tactile_publisher', anonymous=True)
    # tactile_reading('right_robot_left_finger')
    right_robot_left_finger_Thread = threading.Thread(target=tactile_publisher, args=('left_robot_left_finger', 0.5,))
    right_robot_left_finger_Thread.daemon = True
    right_robot_left_finger_Thread.start()
    right_robot_right_finger_Thread = threading.Thread(target=tactile_publisher, args=('left_robot_right_finger', 0.5,))
    right_robot_right_finger_Thread.daemon = True
    right_robot_right_finger_Thread.start()
    while not rospy.is_shutdown():
        rospy.spin()
