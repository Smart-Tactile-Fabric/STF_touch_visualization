# Smart-Tactile-Fabric(智能触觉布料)

#### Project Website(项目网页): https://smart-tactile-fabric.github.io/STF.github.io/

【淘宝】https://e.tb.cn/h.SQ6rjlgiow5ob5s?tk=MJN9f2bd29s CZ225 「【新品预售】压力传感器智能绑缚足底鞋垫机器人触觉柔性感知布料」点击链接直接打开 或者 淘宝搜索直接打开

## 1. Firmware

(1) Load the [arduino code](/arduino_code) to the arduino. 

      MatrixArray_naive 为初始版本的arduino代码，无标定；
      MatrixArray_normal 为正常标定版本的arduino代码，适合普适范围（可感知的受压面大，但串扰偏大一些）；
      MatrixArray_update 为更新标定版本的arduino代码，适合精细范围（仅限于感知受压面小的物体，串扰小），同时MuJoCo仿真效果好

(2) 触觉布料的标定教程：https://tcnemfaap5t4.feishu.cn/wiki/BSDXwU5HOibmS8kywGxcVirmnVh

补充说明：

(1) 淘宝购买的布料均已出厂标定（采用MatrixArray_update），如果想要采用MatrixArray_normal正常标定版本，可以自行探索布料标定教程中的操作流程(兼容Linux系统与Windows系统).

(2) 目前触觉布料有2个不同的版本：定制版与标准版，效果都差不多，定制版属于纯手工制作（可以制作不同大小尺寸/间距的布料，制作费时）；而标准版的制作工艺升级（固定间距，极限只能做15*15阵列大小的布料，制作效率提升）--需要注意排线不要插反，具体的实物图片如下：
![img.png](/image/img0.jpg)

## 2. Python

触觉布料的基础使用教程（面向新手）：https://e1rist1l032.feishu.cn/wiki/ZfWIw2AS1igHVOkU2vScT5oanZI?from=message&source_type=message

(1) Setup environment

        conda create --name stf python=3.10
        conda activate stf
        
        pip install pyserial
        pip install opencv-python==4.6.0.66
        pip install scipy
        pip install numpy==1.23.0
        pip install mujoco==3.3.0


(2) Start python visualization(Test Real)

注意：运行触觉布料的可视化测试脚本需要将【布料硬件】连接上位机（电脑）。建议烧入MatrixArray_normal代码。

        cd python/real
        python3 multi_thread_contact_v0.py

声明：关于标准版的布料，应该运行``multi_thread_contact_new_v0.py``， 这是由于代码中应遵循以下代码逻辑：

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

为了上位机触觉感知可视化图像效果正常，对于标准版布料，其余的运行代码都要使用该逻辑，可以自行注释更改代码；而应用换行代码的是定制版布料。


(3) 将真实布料触感在MuJoCo仿真环境中反馈并显示感受区域(Real2Sim2Sim)

触觉布料感应区域在mujoco仿真中的可视化效果展示：真实布料的感应区域映射至仿真中的粉色点阵（real2sim），由粉色点阵下压至mujoco仿真环境中的触觉传感器形成高亮可视化图像（sim2sim）

注意：仿真界面卡死、无反应或中间部分存在死点，建议关闭程序后重新运行。建议烧入MatrixArray_update代码。

        cd python/real2sim
        python3 real2sim_touch_vis.py
        python3 real2sim2sim_touch_vis.py

运行``real2sim2sim_touch_vis.py``代码的测试效果如下：
![img.png](/image/img2.png)

(4)【额外补充】基于MuJoCo仿真的触觉传感器测试(Test Sim)

        cd python/sim
        python3 sim_touch_vis.py

运行``sim_touch_vis.py``代码的测试效果如下：
![img.png](/image/img1.png)

(5)【额外补充】基于MuJoCo仿真的触觉人机交互测试(利用触觉控制小车)

        cd python/real2sim/human_computer_interaction
        python3 STF_touch_control_car.py

运行``STF_touch_control_car.py``代码的测试效果如下：
![img.png](/image/img3.jpg)

注意：触觉控制小车的原理是基于布料上四个象限的触觉数据同时控制前进、后退和转向.

## 3. Error Collection

注意： 

1) 串口端口的设置（``Linux``系统和``Windows``系统设置不同），例如
        
        Linux系统: '/dev/ttyUSB1'
        Windows系统: 'COM6'

2) 在MuJoCo仿真中，不同的XML文件路径要与自己的电脑一致，比如项目其中的一个XML文件路径设置为：

        '/home/hjx/hjx_file/STF/STF_touch_visualization/python/sim/sim_touch_stf.xml'

3) 在``multi_thread_contact_v0.py``代码中，阈值和噪声缩放因子可以自行调试，更改不同的数值来调整可视化中的噪声串扰：

        # 定义阈值和噪声缩放因子
        THRESHOLD = 6
        NOISE_SCALE = 20

``Reference``: https://binghao-huang.github.io/3D-ViTac/

## Citation
If you use this project in your work, please cite the following:

      @misc{jnu2025STF,
        title={Smart-Tactile-Fabric},
        author={Wen, Jiayi and Ma, Yunsheng and Huang, Jiaxin and Liu, Hanyu and Li, Xinran and Song, Zhigong},
        year={2025},
        howpublished={\url{https://github.com/Smart-Tactile-Fabric/STF_touch_visualization}},
}
