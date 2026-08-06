#!/usr/bin/env python3
"""按实例渲染一份"PX4 iris(带mavlink_interface+电机，真正能被PX4控制) + Mid-360雷达/IMU"
的融合SDF模型，spawn进Gazebo用。

背景：sim-world 原来 spawn 的是 mighty 自己的 quadrotor.urdf.xacro——只有雷达+IMU的
传感器载具，没有 libgazebo_mavlink_interface.so 插件、没有电机模型，PX4 SITL 因此永远
连不上任何仿真物理实体（日志卡在"Waiting for simulator to accept connection on TCP
port 4560"不动，MAVROS 自然 connected:false）。真正带这个插件的是 PX4 官方自己的
iris 系列模型（Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/
iris.sdf.jinja），照着 PX4 官方 iris_rplidar 那个"iris+外挂传感器"组合模型的先例
（用 <include>+<joint fixed> 把电机机体和传感器焊在一起），把 Mid-360 雷达/IMU 焊上去。

iris.sdf.jinja 本身要按实例渲染不同的 mavlink_tcp_port/mavlink_udp_port（否则两架
飞机的 Gazebo<->PX4 TCP 桥接端口会撞），端口公式照抄 PX4 官方
Tools/simulation/gazebo-classic/sitl_multiple_run.sh 里 spawn_model() 的规则：
tcp=4560+instance, udp=14560+instance, mavlink_id=1+instance。

Mid-360 的传感器 SDF（射线雷达+IMU）内容照抄 mighty 自己
livox_laser_simulation_ros2/urdf/mid360.xacro 打了 livox_mid360_imu.patch +
livox_pointcloud_timestamp.patch + livox_mid360_visual_gravity.patch 之后的最终版本，
只是从 xacro 变量替换改成了 Python 字符串格式化（不再走 xacro/URDF，直接生成
最终 SDF，因为要跟 iris 这个纯 SDF 模型拼在一起，URDF include 不了 SDF 模型）。
水平安装(rpy全0)/z偏移0.06(紧贴iris机体上表面，机体collision box顶面在
z=0.055，见iris.sdf.jinja里base_link_inertia_collision的<size>0.47 0.47
0.11</size>、pose在原点，留了5mm给安装支架厚度)跟 mighty_sensor_mount.patch、
dlio_extrinsics_mount.patch 用的是同一套外参，保持一致。

原来是30度前倾——实测发现这个角度反而有负面影响：mid360本身原生垂直FOV是
-7.2°~+55.2°（本来就整体偏上），前倾30度之后变成22.8°~85.2°，几乎完全偏离
水平面，飞机正前方同高度的障碍物大概率根本不在扫描范围内，只有机头先转向、
把整个偏置的FOV锥转过去才可能扫到——这正是"飞机快到障碍物跟前机头还没转过来
就撞上"这个问题的物理根因之一（另一部分原因是yaw跟踪速率跟不上，见README
"机头转向跟不上"那一节，是两个不同层面但会叠加的问题）。改成水平安装后，
-7.2°~+55.2°这个原生FOV本身就跨越了水平面，不需要靠机头转向对准才能看见
正前方的障碍物，覆盖水平方向的能力理论上不再依赖偏航角，全方位（360度
水平旋转）都有类似的覆盖。

雷达传感器/插件这两处 <visualize> 故意都是 false（不是1000个采样点的射线扫描线
可视化）——实测过 true 会导致 PX4 连不稳：单独对照测试过，同一个融合模型只把这
两处从 true 改成 false，PX4 就从反复 `ERROR [simulator_mavlink] poll timeout 0, 25`
变成正常启动到 `Ready for takeoff!`。原因是无GUI订阅者时 Gazebo 依然会为射线可视化
准备渲染数据，拖慢整个物理仿真循环的实时性，导致 mavlink_interface 插件发给 PX4
的数据流延迟、PX4 那边 poll 超时。这跟雷达本体的可见外观（上面 <link> 里的
<visual> 球体，GUI 里能看到扣在机体上的半球形整流罩）是两回事，那个不影响
性能，继续保留。
"""

import argparse
import subprocess
import sys

PX4_ROOT = "/opt/PX4-Autopilot"
SITL_GAZEBO_DIR = f"{PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
JINJA_GEN = f"{SITL_GAZEBO_DIR}/scripts/jinja_gen.py"
IRIS_JINJA = f"{SITL_GAZEBO_DIR}/models/iris/iris.sdf.jinja"
MID360_CSV = "/opt/mighty_ws/install/ros2_livox_simulation/share/ros2_livox_simulation/scan_mode/mid360.csv"

# cfg/dlio.yaml 外参、雷达本体挂载位置等都是照这几个数来的
MID360_MOUNT_XYZ = "0 0 0.06"
MID360_MOUNT_RPY = "0 0 0"  # 水平安装，跟 mighty_sensor_mount.patch 保持一致

MID360_SDF_TEMPLATE = """
    <link name='{ns}_mid360_link'>
      <pose>{xyz} {rpy}</pose>
      <inertial>
        <mass>0.4</mass>
        <inertia>
          <ixx>0.000493</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.000493</iyy><iyz>0</iyz><izz>0.00048</izz>
        </inertia>
      </inertial>
      <!-- 半球形外观：SDF没有现成的半球图元，用一个完整球体、球心正好卡在
           机体安装面(z=0)上做近似——下半球被机体自己的visual挡住看不见，
           露出来的上半球看起来就是扣在机体上表面的一个半球形整流罩，不用
           额外做/找半球mesh。纯视觉近似，没有配对的<collision>（原来的
           box也没配collision，这里保持一致，不新增碰撞体）。 -->
      <visual name='{ns}_mid360_visual'>
        <geometry><sphere><radius>0.025</radius></sphere></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <sensor type='ray' name='{ns}_livox'>
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>100</update_rate>
        <plugin name='{ns}_livox_plugin' filename='libros2_livox.so'>
          <ray>
            <scan>
              <horizontal>
                <samples>100</samples><resolution>1</resolution>
                <min_angle>0</min_angle><max_angle>6.2831852</max_angle>
              </horizontal>
              <vertical>
                <samples>360</samples><resolution>1</resolution>
                <min_angle>-0.126000</min_angle><max_angle>0.963900</max_angle>
              </vertical>
            </scan>
            <range><min>0.001</min><max>40.0</max><resolution>0.15</resolution></range>
            <noise><type>gaussian</type><mean>0.0</mean><stddev>0.0</stddev></noise>
          </ray>
          <visualize>false</visualize>
          <samples>20000</samples>
          <downsample>1</downsample>
          <csv_file_name>{mid360_csv}</csv_file_name>
          <topic>mid360</topic>
          <namespace>{ns}</namespace>
        </plugin>
      </sensor>
      <sensor name='{ns}_livox_imu' type='imu'>
        <always_on>true</always_on>
        <update_rate>500</update_rate>
        <!-- 原来这里完全没配<imu>噪声块——Gazebo这个IMU传感器默认就是零噪声，
             跟上面雷达那个显式stddev=0.0是同一个模式。用户反馈：真实硬件上
             用DLIO，哪怕IMU只有几十Hz也很稳，从来不用去碰dt钳位这类参数；
             而这套仿真里470Hz+完全理论精确值的IMU反而会让DLIO静止发散。真实
             传感器天然带噪声，某些依赖噪声做数值正则化的估计器/偏置估计逻辑
             在零噪声输入下反而容易出问题（比如把浮点舍入误差当成有意义的
             "真信号"）。补一组常见MEMS IMU量级的高斯噪声（角速度噪声密度
             ~2e-4 rad/s、加速度噪声密度~1.7e-2 m/s²，是仿真里常用的量级参考
             值，不是Livox Mid-360实测datasheet数字，如果有真实标定数据应该
             换成那个）。 -->
        <imu>
          <angular_velocity>
            <x><noise type='gaussian'><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
            <y><noise type='gaussian'><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
            <z><noise type='gaussian'><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type='gaussian'><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
            <y><noise type='gaussian'><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
            <z><noise type='gaussian'><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
          </linear_acceleration>
        </imu>
        <plugin name='{ns}_livox_imu_plugin' filename='libimu_plugin.so'>
          <ros>
            <namespace>{ns}</namespace>
            <remapping>~/out:=mid360/imu</remapping>
          </ros>
        </plugin>
      </sensor>
    </link>
    <joint name='{ns}_mid360_joint' type='fixed'>
      <parent>base_link</parent>
      <child>{ns}_mid360_link</child>
    </joint>
"""


def render_iris(instance: int, output_file: str) -> None:
    subprocess.run(
        [
            sys.executable, JINJA_GEN, IRIS_JINJA, SITL_GAZEBO_DIR,
            "--mavlink_tcp_port", str(4560 + instance),
            "--mavlink_udp_port", str(14560 + instance),
            "--mavlink_id", str(1 + instance),
            "--gst_udp_port", str(5600 + instance),
            "--mavlink_cam_udp_port", str(14530 + instance),
            "--output-file", output_file,
        ],
        check=True,
    )


def merge_mid360(iris_sdf_path: str, namespace: str, output_path: str) -> None:
    with open(iris_sdf_path) as f:
        content = f.read()

    if content.count("</model>") != 1:
        raise RuntimeError(
            f"expected exactly one </model> in {iris_sdf_path}, found {content.count('</model>')}"
        )

    mid360_block = MID360_SDF_TEMPLATE.format(
        ns=namespace,
        xyz=MID360_MOUNT_XYZ,
        rpy=MID360_MOUNT_RPY,
        mid360_csv=MID360_CSV,
    )
    content = content.replace("</model>", mid360_block + "\n  </model>")

    with open(output_path, "w") as f:
        f.write(content)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", required=True, help="e.g. NX01")
    parser.add_argument("--instance", type=int, required=True, help="0-based PX4 -i instance index")
    parser.add_argument("--output", required=True, help="final merged sdf output path")
    args = parser.parse_args()

    iris_tmp = f"/tmp/iris_{args.namespace}.sdf"
    render_iris(args.instance, iris_tmp)
    merge_mid360(iris_tmp, args.namespace, args.output)
    print(f"[gen_iris_mid360_sdf] wrote {args.output}")


if __name__ == "__main__":
    main()
