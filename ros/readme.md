
## ROS
1. compile ros package(Assuming you have already installed ROS1)

        cd ros/tactile_ws
        catkin_make

注意：``cd ros/tactile_ws``文件路径中后需要删除``build``和``devel``文件，然后再``catkin_make``编译;
后续在.bashrc文件中添加``source /home/hjx/hjx_file/STF/STF_touch_visualization/ros/tactile_ws/devel/setup.sh``内容.

2. rosrun tactile_sensor tactile_sensor.py

## Multi Sensors name setup 

1. setup multi thread tactile board name(Optional):
- One issue that arises is the port each robot binds to can change over time, e.g. a robot that
is initially ``ttyUSB0`` might suddenly become ``ttyUSB5``. 

- Take ``right_robot_left_finger``: right master robot as an example:
  1. Find the port that the right master robot is currently binding to, e.g. ``ttyUSB0``
  2. run ``udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial`` to obtain the serial number. Use the first one that shows up, the format should look similar to ``FT6S4DSP``.
  3. ``sudo vim /etc/udev/rules.d/99-tactile.rules`` and add the following line: 

         SUBSYSTEM=="tty", ATTRS{serial}=="<serial number here>", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="right_robot_left_finger"

  4. This will make sure the tactile in right_robot_left_finger is *always* binding to ``right_robot_left_finger``
  5. Repeat with the rest of 3 arms.
- To apply the changes, run ``sudo udevadm control --reload && sudo udevadm trigger``
- If successful, you should be able to find ``right_robot_left_finger`` in your ``/dev``

