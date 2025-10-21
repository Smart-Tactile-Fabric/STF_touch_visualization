# Smart-Tactile-Fabric(智能触觉布料)

#### Project Website: https://smart-tactile-fabric.github.io/STF.github.io/

【淘宝】https://e.tb.cn/h.SQ6rjlgiow5ob5s?tk=MJN9f2bd29s CZ225 「【新品预售】压力传感器智能绑缚足底鞋垫机器人触觉柔性感知布料」点击链接直接打开 或者 淘宝搜索直接打开

## 1. Firmware

(1) Load the [arduino code](/arduino_code/MatrixArray.ino) to the arduino. 


## 2. Python
(1) Setup environment

        conda create --name stf python=3.10
        conda activate stf
        
        pip install pyserial
        pip install opencv-python==4.6.0.66
        pip install threading
        pip install scipy
        pip install numpy==1.23.0
        pip install mujoco==3.3.0


(2) Start python visualization(Test Real)

        cd python/real
        python3 multi_thread_contact_v2.py

(3) 基于MuJoCo仿真的触觉传感器测试(Test Sim)

        cd python/sim
        python3 sim_touch_vis.py

Reference: https://binghao-huang.github.io/3D-ViTac/
