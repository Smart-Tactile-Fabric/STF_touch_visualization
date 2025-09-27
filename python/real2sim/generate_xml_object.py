# xml object

def generate_mujoco_objects(start_idx, end_idx, pos_start=(0, 0), step=0.02, file_name="generated_objects.xml"):
    """生成指定范围内物体的XML代码并保存到文件"""

    # 打开文件进行写入
    with open(file_name, 'w') as file:
        # 写入 XML 头部
        file.write("<mujoco>\n")
        # file.write("  <worldbody>\n")

        num = 16
        # 物体的位置步长
        x_pos, y_pos = pos_start
        for i in range(start_idx, end_idx + 1):
            # 根据物体的编号来计算x、y坐标
            x_offset = (i - 1) % num * step  # 每16个物体换行，沿x轴递增
            y_offset = (i - 1) // num * step  # 每16个物体换行，沿y轴递增

            # 写入每个物体的XML定义
            file.write(f'    <body name="object_{i}" pos="{x_pos + x_offset:.2f} {y_pos + y_offset:.2f} 0.2">\n')
            file.write(f'      <joint name="obj_{i}" type="slide" range="-1 1" damping="100" frictionloss="10"/>\n')
            file.write(f'      <geom type="box" size="0.0099 0.0099 0.01" rgba=".62 .42 .62 0.58"/>\n')
            file.write(f'    </body>\n')

        # 写入 XML 结束部分
        # file.write("  </worldbody>\n")
        file.write("</mujoco>\n")

    print(f"XML file '{file_name}' has been generated.")


def generate_actuator_xml(start_idx, end_idx, file_name="actuator_generated.xml"):
    """生成指定范围内的 actuator XML 代码并保存到文件"""

    with open(file_name, 'w') as file:
        # 写入 XML 头部
        file.write("<mujoco>\n")
        file.write("  <actuator>\n")

        # 生成每个物体的 actuator 代码
        for i in range(start_idx, end_idx + 1):
            file.write(f'    <position name="obj_{i}" joint="obj_{i}" kp="200" ctrlrange="-0.088 0.1"/>\n')

        # 写入 XML 结束部分
        file.write("  </actuator>\n")
        file.write("</mujoco>\n")

    print(f"Actuator XML file '{file_name}' has been generated.")


if __name__ == "__main__":

    end_idx = 256
    # 生成 object_1 到 object_256 的代码并保存为XML文件
    generate_mujoco_objects(1, end_idx, file_name="/home/hjx/hjx_file/STF/STF_mujoco/python/real2sim/generated_objects.xml")

    # 生成 obj_1 到 obj_256 的 actuator XML
    generate_actuator_xml(1, end_idx, file_name="/home/hjx/hjx_file/STF/STF_mujoco/python/real2sim/actuator_generated.xml")
