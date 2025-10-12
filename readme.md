# Smart-Tactile-Fabric(智能触觉布料)

#### Project Website: https://smart-tactile-fabric.github.io/STF.github.io/

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


(2) Start python visualization(Test)

        cd python/real
        python3 multi_thread_contact_v2.py


Reference: https://binghao-huang.github.io/3D-ViTac/
