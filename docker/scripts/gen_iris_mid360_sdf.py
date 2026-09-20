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
        <!-- 2026-09-09改成银色外观（用户要求），原来是近黑色0.1 0.1 0.1。
             加<specular>让金属感更明显一点，不是必须的，纯锦上添花。 -->
        <material>
          <ambient>0.75 0.75 0.78 1</ambient>
          <diffuse>0.75 0.75 0.78 1</diffuse>
          <specular>0.9 0.9 0.9 1</specular>
        </material>
      </visual>
      <sensor type='ray' name='{ns}_livox'>
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <!-- 2026-09-15：从100Hz降到10Hz——真实Livox MID360帧率就是约
             10Hz（20万点/秒÷10Hz=2万点/帧，正好对上下面<samples>20000
             </samples>这个参数），100Hz是比真实硬件快10倍的过度仿真，
             实测是gzserver CPU瓶颈的主因（100Hz×3.6万根射线/帧=每秒
             360万次光线投射，纯CPU raycasting，两架飞机各一份）。降到
             10Hz不影响点云图案/密度，只是采样节奏更贴近真实硬件，同时
             应该能显著降低gzserver负担、把RTF拉回接近1.0（详见DEBUG_
             JOURNAL.md同日期"仿真real-time-factor"那条记录）。 -->
        <update_rate>10</update_rate>
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
        <!-- 2026-09-15：500Hz降到200Hz（用户直接要求），减轻gzserver负担
             ——跟下面噪声块是两件独立的事：噪声块解决的是"零噪声理论精确值
             导致DLIO静止发散"，这次只是降更新频率，噪声配置不变，不会
             重新引入那个问题。200Hz也是常见MEMS IMU/Livox系列硬件的实际
             量级，不算脱离真实硬件规格。 -->
        <update_rate>200</update_rate>
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

# 2026大赛任务系统阶段2.1：前视/下视相机，焊接方式完全照抄上面mid360的做法
# （<link>+<sensor>+<joint fixed>拼进iris这个纯SDF模型，不走mighty的
# quadrotor.urdf.xacro/generic_camera.urdf.xacro——那条xacro链路根本不在这个
# 项目实际spawn的模型里，见本文件开头的背景说明，PX4 SITL飞的是这里生成的
# iris+mid360融合SDF，mighty自己的quadrotor URDF早就被弃用了。执行方案原文
# "阶段2.1"写的是"修复generic_camera.urdf.xacro的命名空间问题"，验证过那份
# xacro确实完全没有被这条构建/spawn链路引用，是文档编写时对架构的误判，
# 这里改成在真正生效的地方（这个脚本）加相机，效果等价、位置改了。
#
# 相机朝向：SDF camera sensor在<pose>无旋转时看向本地+X（前）——mid360那样
# 水平安装时rpy全0就是"前视"；下视相机要看向-Z，用绕Y轴+90度(pitch=+π/2)：
# 旋转矩阵Ry(θ)把+X轴变成(cosθ,0,-sinθ)，θ=π/2时得到(0,0,-1)，正好是-Z——
# 这是SDF/Gazebo camera sensor摆位的标准写法（PX4/Gazebo生态里下视相机demo
# 一贯的姿态写法），没有在这个具体模型里实机/GUI肉眼验证过朝向，如果后续
# 发现前视相机拍到的是天花板或下视相机拍到的是墙面，第一嫌疑就是这个pitch
# 符号，翻个负号即可。
#
# 用`libgazebo_ros_camera.so`（跟mighty的generic_camera.urdf.xacro用的是
# 同一个官方插件，参数名`<cameraName>`/`<imageTopicName>`/
# `<cameraInfoTopicName>`/`<frameName>`照抄过来），命名空间修复方式照抄本
# 文件mid360 IMU那段已经验证工作的`<ros><namespace>{ns}</namespace></ros>`
# 写法（不是mighty那份xacro漏掉的部分——mighty那份的问题正是完全没有这个
# <ros>块，只传了裸的topic名）。
# 2026-09-16：原来每路相机（front/down）各自焊一个独立的
# <sensor type='camera'>，实测确认Gazebo Classic的
# SensorManager::ImageSensorContainer是整个gzserver进程唯一一个、按
# 传感器注册顺序严格FIFO处理的共享队列（直接拉gazebo11源码`gazebo/
# sensors/SensorManager.cc`确认：只有一个全局`ImageSensorContainer`，
# `AddSensor()`只看类别不看归属哪个model，`Update()`就是个简单for
# 循环，没有轮询/公平调度）——谁先声明谁在每次渲染竞争里赢，实测反转
# `--cameras front,down`→`down,front`声明顺序，赢家精确对调，两架
# 飞机结果一致，坐实了这个机制，详见DEBUG_JOURNAL.md 2026-09-16"down
# 相机问题继续深挖"系列记录。
#
# 改成合并成一个`<sensor type='multicamera'>`，两个`<camera>`子块各自
# 独立`<pose>`——查过`MultiCameraSensor.cc`源码确认子相机pose是完整
# 6DOF `ignition::math::Pose3d`复合，不限于小基线双目场景，front水平
# 朝前+down垂直朝下这种90度差异的挂载没有问题。`libgazebo_ros_camera.so`
# 本身检测到挂在`MultiCameraSensor`上会自动切到多相机发布逻辑（不需要
# 换成另一个插件文件），话题命名规则是`<camera_name>/<子相机name>/
# image_raw`（源码`gazebo_ros_camera.cpp`第179行`camera_name_ = _sdf->
# Get<std::string>("camera_name", ...)`确认，读的是`<camera_name>`这个
# 下划线小写标签，不是旧的驼峰式`<cameraName>`）——两路相机在
# `MultiCameraSensor::UpdateImpl()`同一次调用里一起渲染完，不再是
# `SensorManager`队列里两个独立排队的sensor，从根上绕开这个FIFO问题。
#
# ⚠️ 跟之前`<imageTopicName>`标签被ROS2移植版静默忽略同一个教训——这次
# 改动生成的实际话题名（预期是`/{ns}/{ns}_camera/front/image_raw`这种
# 形式）build之后必须实测`ros2 topic list`确认，不能只信源码读到的
# 逻辑就直接认为对，历史上已经在这个坑上摔过一次。

# 2026-09-16续：先把两路相机合并进同一个`<sensor type='multicamera'>`，
# down实测4种姿态组合（pitch=+90/+80度×Z=-0.0217/-0.0317/-0.30、
# pitch=-90度×Z=-0.0217/-0.30）全部失败——正俯仰角不管多远都是纯背景色
# （跟世界<background>色值精确吻合），负俯仰角近距离拍到自身雷达底部
# （自遮挡），远距离干脆连image_raw都不发布了；front全程正常。
# 换成"down独立`<sensor type='camera'>`+front留在multicamera"的混合
# 方案，实测更糟——两路相机全部只在gzserver刚spawn那一瞬间各成功收到
# 一帧，之后彻底沉默，稳定复现（连续两次干净重启都一样），说明
# `type='camera'`跟`type='multicamera'`混用本身在这套gzserver/插件
# 版本组合里有问题（很可能是SensorManager处理两种不同图像传感器类型时
# 有共享状态/线程冲突），不是靠拆分姿态能绕开的。
# 改成两路都用multicamera、但各自独立一个`<sensor type='multicamera'>`
# （每个里面只放它自己一个子相机）——解决了假死，但补了实测像素统计
# （mean/std）才发现down的内容其实还是没修好：不管merge成一个还是各自
# 独立，`multicamera`这个传感器类型对down这个90度俯仰姿态就是渲染不出
# 内容（纯背景色178），front（水平朝前，pitch=0）在同一套结构里全程
# 正常，说明问题精确锁定在`multicamera`类型本身处理大角度俯仰子相机的
# 渲染路径上，不是姿态数值/距离/跟谁共享的问题。
# 2026-09-16最终定案：两路相机全部退回最初就验证过内容可靠的独立
# `<sensor type='camera'>`（放弃multicamera这整条路）。公平性（谁被
# SensorManager全局FIFO队列挤占渲染机会）靠"声明顺序决定优先级"兜底
# （sim-world-entrypoint.sh的CAMERAS_DEFAULT="down,front"，down先声明
# 就赢）——运行时动态开关相机（`<camera_name>/set_enabled`话题）这条路
# 2026-09-17已经整体删除，排查不出稳定性问题的根因、用户直接要求删掉，
# 不再作为兜底手段，见DEBUG_JOURNAL.md同日期记录。
CAMERA_SDF_TEMPLATE = """
    <link name='{ns}_{cam}_camera_link'>
      <pose>{pose}</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz>
        </inertia>
      </inertial>
      <visual name='{ns}_{cam}_visual'>
        <geometry><box><size>0.04 0.02 0.02</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
      <sensor type='camera' name='{ns}_{cam}_camera'>
        <update_rate>{update_rate}</update_rate>
        <camera name='{cam}'>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{res_x}</width>
            <height>{res_y}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>50</far></clip>
        </camera>
        <plugin name='{ns}_{cam}_camera_controller' filename='libgazebo_ros_camera.so'>
          <ros>
            <namespace>{ns}</namespace>
          </ros>
          <camera_name>{ns}_{cam}_camera</camera_name>
        </plugin>
      </sensor>
    </link>
    <joint name='{ns}_{cam}_camera_joint' type='fixed'>
      <parent>base_link</parent>
      <child>{ns}_{cam}_camera_link</child>
    </joint>
"""

# 2026-09-18新增：单相机+可动关节两档预设视角（前视/下视），彻底避开
# Gazebo Classic多相机共享渲染队列这个联网核实过的架构级限制（见
# DEBUG_JOURNAL.md 2026-09-17/09-18记录——PX4/gazebo-classic官方仓库
# 都独立确认过，`SensorManager.cc`源码注释里维护者自己写明image类
# 传感器必须跑在同一个主线程，不是能调参数解决的）。整机全局只有一个
# `type='camera'`真实传感器在渲染，不存在"谁挤占谁"这个前提条件。
#
# 挂载点(x,y,z)取`CAMERA_MOUNTS['front']`的X偏移(0.09)+`CAMERA_MOUNTS
# ['down']`的Z偏移(-0.0217)叠加——这两个偏移量分别是各自方向上独立
# 实测验证过"不自遮挡"的经验值（见CAMERA_MOUNTS注释里front/down各自
# 5-6轮微调历史），叠加使用理论上应该两个朝向都不会被自己的机体挡住
# （前视看的是水平方向，不受Z偏移影响；下视看的是竖直方向，不受X偏移
# 影响），但**没有像历史上那样逐轮肉眼确认过实际画面**，如果后续发现
# 某个朝向被自遮挡，参照CAMERA_MOUNTS的调整方式继续微调
# SWITCHABLE_CAMERA_MOUNT_XYZ即可，不需要改这份模板结构。
#
# 关节角度约定（跟`set_camera_view()`/`gazebo_ros_joint_pose_trajectory`
# 配合）：0弧度=前视（本地+X轴=机体正前方，跟原来固定的front挂载同一个
# 朝向约定），1.5707963弧度(90°)=下视（绕本地Y轴转90度，跟原来固定的
# down挂载pitch=1.5707963是同一个旋转方向，已经反复实测验证过朝向正确）。
# 关节限位留了一点余量（-0.05~1.62），不卡死在精确的0/90度边界上，
# 避免浮点误差导致SetPosition()被限位拒绝。
SWITCHABLE_CAMERA_SDF_TEMPLATE = """
    <link name='{ns}_switchable_camera_link'>
      <pose>{pose}</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz>
        </inertia>
      </inertial>
      <visual name='{ns}_switchable_visual'>
        <geometry><box><size>0.04 0.02 0.02</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
      <sensor type='camera' name='{ns}_switchable_camera'>
        <update_rate>{update_rate}</update_rate>
        <camera name='switchable'>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{res_x}</width>
            <height>{res_y}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.05</near><far>50</far></clip>
        </camera>
        <plugin name='{ns}_switchable_camera_controller' filename='libgazebo_ros_camera.so'>
          <ros>
            <namespace>{ns}</namespace>
          </ros>
          <camera_name>{ns}_switchable_camera</camera_name>
        </plugin>
      </sensor>
    </link>
    <joint name='{ns}_switchable_camera_joint' type='revolute'>
      <parent>base_link</parent>
      <child>{ns}_switchable_camera_link</child>
      <pose>0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.05</lower>
          <upper>1.62</upper>
        </limit>
      </axis>
    </joint>
    <plugin name='{ns}_switchable_camera_joint_trajectory' filename='libgazebo_ros_joint_pose_trajectory.so'>
      <ros>
        <namespace>{ns}</namespace>
      </ros>
      <update_rate>50</update_rate>
    </plugin>
"""

# front挂载X偏移(见CAMERA_MOUNTS['front'])+down挂载Z偏移(见
# CAMERA_MOUNTS['down'])叠加，见SWITCHABLE_CAMERA_SDF_TEMPLATE上方说明。
SWITCHABLE_CAMERA_MOUNT_XYZ = "0.09 0 -0.0217"
# front/down共用同一套hfov/分辨率/update_rate，直接复用CAMERA_MOUNTS
# ['front']的值（down那份数值完全一样，两个取哪个都可以）。
SWITCHABLE_CAMERA_PARAMS = {
    "hfov": "1.3963", "res_x": "640", "res_y": "480", "update_rate": "10",
    "color": "0.6 0.2 0.6 1",
}
# 关节角度：0=前视，1.5707963(90°)=下视，见SWITCHABLE_CAMERA_SDF_TEMPLATE
# 上方"关节角度约定"说明。
CAMERA_VIEW_JOINT_ANGLE = {"front": 0.0, "down": 1.5707963}


def merge_switchable_camera(sdf_path: str, namespace: str, initial_view: str = "down") -> None:
    """焊接单相机+可动关节（前视/下视两档预设角度）。`initial_view`是这架
    飞机spawn出来那一刻关节的初始角度对应哪个朝向——注意这只决定"关节
    初始角度写几"，不是运行时切换用的（运行时切换靠contest_sdk的
    `set_camera_view()`发`set_joint_trajectory`消息，两者是完全独立的
    两条路径，只是复用同一个角度约定表`CAMERA_VIEW_JOINT_ANGLE`）。

    SDF本身不支持给revolute关节声明"非零初始角度"这个语义（关节永远从
    零位开始），这里的做法是让`<link>`的`<pose>`直接就是目标初始朝向对应
    的最终位姿（等价于"关节从一开始就转到了这个角度"），不依赖spawn后
    再发一次消息去摆正——sim-world-entrypoint.sh不需要额外为这件事补一次
    "开机自动纠正朝向"的时序脆弱操作。
    """
    if initial_view not in CAMERA_VIEW_JOINT_ANGLE:
        raise ValueError(f"initial_view必须是{list(CAMERA_VIEW_JOINT_ANGLE)}之一，收到{initial_view!r}")

    with open(sdf_path) as f:
        content = f.read()

    if content.count("</model>") != 1:
        raise RuntimeError(
            f"expected exactly one </model> in {sdf_path}, found {content.count('</model>')}"
        )

    x, y, z = SWITCHABLE_CAMERA_MOUNT_XYZ.split()
    angle = CAMERA_VIEW_JOINT_ANGLE[initial_view]
    # 前视：无旋转；下视：绕Y轴转90度——跟CAMERA_MOUNTS['down']的pose写法
    # （"0 0 -0.0217 0 1.5707963 0"）保持同一种rpy格式，只是roll/yaw恒为0，
    # 只有pitch随initial_view变化。
    pose = f"{x} {y} {z} 0 {angle} 0"

    block = SWITCHABLE_CAMERA_SDF_TEMPLATE.format(
        ns=namespace, pose=pose, **SWITCHABLE_CAMERA_PARAMS
    )
    content = content.replace("</model>", block + "\n  </model>")

    with open(sdf_path, "w") as f:
        f.write(content)


# 2026大赛任务系统：对地测距雷达(单点向下测距，喂PX4 EKF2做地形跟随/精准
# 定高，跟下视相机是两回事——下视相机出的是RGB图像，给AprilTag/二维码
# 识别用，不产生距离数据）。
#
# ⚠️ 这颗传感器**必须**用`<include><uri>model://lidar</uri></include>`
# 引用PX4官方`sitl_gazebo-classic/models/lidar/model.sdf`（2026-09-10
# 曾经短暂切换到`model://sonar`又紧急改回来，原因见下面
# RANGEFINDER_MOUNT_XYZ那段"最终方案"说明——sonar会把整个gzserver
# 直接crash掉，不是数据链路问题，是更严重的引擎级问题），不能像
# mid360/相机那样手写等价的`<link>+<sensor>`直接
# 焊进来——PX4负责"发现传感器→转发MAVLink DISTANCE_SENSOR消息"的机制
# 是靠正则匹配"嵌套子模型的名字"（不是link名、不是sensor名）来自动
# 接线的，且两边话题名拼接规则（发布方`~/<根模型名>/link/<子模型名>`、
# 订阅方同一套拼法）只有在"真的是一个嵌套子模型、名字匹配对应正则"时
# 才能对上——手写一个"看起来等价"的同名link不满足这个条件，逐行核对
# 过两边的话题字符串拼接逻辑，确实对不上，不是猜测。GAZEBO_MODEL_PATH
# 里已经包含`sitl_gazebo-classic/models`（起容器时`echo
# $GAZEBO_MODEL_PATH`确认过），`<include>`能正常解析到。
#
# 挂载位姿：`model://lidar`自己的sensor在其内部link坐标系下已经带了
# pitch=+π/2（看官方model.sdf内容，跟本文件下视相机同一个"pitch=+π/2=
# 转到-Z"的约定一致，见下视相机CAMERA_MOUNTS注释的旋转矩阵推导），所以
# `<include>`这一级只需要给平移，不能再叠加一次旋转（会转两次变成朝
# 前/朝后）。机体collision box中心在原点、<size>0.47 0.47 0.11</size>
# 即顶面z=+0.055/底面z=-0.055（见mid360挂载注释），挂载点必须留在这个
# 范围之外（往下超过-0.055），否则射线原点埋在自己机体碰撞箱里会自
# 遮挡，具体挂多少见下面RANGEFINDER_MOUNT_XYZ的调整记录。
#
# 每架飞机都应该有这颗传感器（不像相机是可选的），main()里无条件调用，
# 不受--cameras参数控制。
#
# 2026-09-09用户看了真机截图后要求"定高雷达往上缩2/3个雷达机身高度"——
# "机身"指`model://lidar`自己那颗黑色圆柱体视觉体
# （`<cylinder><radius>0.006</radius><length>0.05</length></cylinder>`，
# 见PX4官方model.sdf），高度0.05米，2/3约0.0333米，从-0.06上移到
# -0.06+0.0333≈-0.0267。
# 2026-09-09第四轮：用户在Gazebo GUI里人工看了实际重建出来的NX01/NX02后
# 要求"测距雷达往下伸3cm"，在上面-0.0267的基础上再往下（更负）移0.03米：
# -0.0267-0.03=-0.0567。
# 2026-09-09第五轮："测距雷达往上缩1cm"，-0.0567+0.01=-0.0467。
# 2026-09-09第六轮："测距雷达往上缩1cm，往前移1cm"——Z: -0.0467+0.01=
# -0.0367；X: 0+0.01=0.01。
# 2026-09-09第七轮："Z上缩1cm，X前移2cm"——Z: -0.0367+0.01=-0.0267；
# X: 0.01+0.02=0.03。
# 2026-09-09第八轮："Z上缩0.5cm"——Z: -0.0267+0.005=-0.0217。
#
# 2026-09-10排查MAVLink DISTANCE_SENSOR数据链路"PX4内部distance_sensor
# uORB话题从来没发布过"（`px4-listener --instance 0 distance_sensor`
# 显示`never published`，见DEBUG_JOURNAL.md）时发现：上面几轮纯粹按
# GUI截图肉眼调的Z坐标（最终-0.0217）**已经调到了机体自身碰撞体内部**——
# 机体`base_link_inertia_collision`是个以原点为中心的0.47x0.47x0.11米
# 箱体，箱底在z=-0.055（见mid360挂载注释里的推导），-0.0217比-0.055更
# 靠近机体中心，也就是说测距雷达的射线原点整个埋在自己机体的碰撞箱
# 内部——射线一出去大概率先打中自己的机身（自遮挡）。当时改到明确留出
# 机体碰撞箱之外净空的位置——-0.10米，比箱底-0.055还要再往下探4.5厘米，
# 想验证这是不是根因。**这个假设最后被实测证伪**（挪到碰撞箱外之后
# `distance_sensor`仍然`never published`，`ranges not constructed yet
# (zero sized)`警告原样还在）——连同"只查NestedModels()[0]"、"headless
# 环境visualize=true"这两版patch，一共三个有依据的假设都被逐一证伪，见
# DEBUG_JOURNAL.md完整排查记录。
#
# 2026-09-10曾经短暂改用`model://sonar`（三个ray类型假设都证伪之后，
# 用户直接要求"换用sonar传感器类型替换"），实测（用gdb挂到gzserver上
# 手动spawn复现，见DEBUG_JOURNAL.md）发现**sonar会把gzserver直接
# 段错误crash掉**，跟mid360的Livox自定义射线插件冲突：崩溃点在
# `LivoxOdeMultiRayShape::UpdateRays()`→ODE的`dSpaceCollide2`内部，
# 栈顶在`libgazebo_ode.so`里，是ODE碰撞检测库本身的问题（大概率是
# sonar传感器在ODE里创建的几何体类型，没有在ODE的collider函数指针表里
# 跟Livox自定义射线类注册过合法的碰撞对，dSpaceCollide2按未注册的
# 类型组合去查表拿到野指针）——不是数据链路层面能解决的问题，是Gazebo
# 引擎级的bug。对照实验：同一个挂载位置换回`model://lidar`（ray类型）
# 单独spawn、反复测试都不会崩，只有sonar会崩，两次独立复现都是同一条
# 崩溃栈，不是偶发的。结论：sonar方案放弃，改回lidar，
# DISTANCE_SENSOR数据链路的"never published"问题维持之前三个假设都
# 证伪的状态，暂时没有新的根因方向，需要用户决定下一步（比如换其他
# 类型的测距传感器、或者干脆放弃Gazebo原生传感器改用ROS2节点合成假
# 数据喂给PX4）。
#
# 2026-09-10：自遮挡假设已证伪，-0.10这个位置除了排查用途、对数据链路
# 没有任何实际帮助，反而偏离了用户前面8轮在GUI里手动调出来满意的外观
# 位置——既然功能上不占便宜，改回用户上次调定的-0.0217（第八轮结果），
# 不再继续停留在排查用的临时位置。之后如果确实要根治"埋进碰撞箱"这件事，
# 需要用户重新明确要不要牺牲外观换取净空，而不是默认保留排查值。
RANGEFINDER_MOUNT_XYZ = "0.03 0 -0.0157"

RANGEFINDER_SDF_TEMPLATE = """
    <include>
      <uri>model://lidar</uri>
      <pose>{xyz} 0 0 0</pose>
    </include>
    <joint name='{ns}_lidar_joint' type='fixed'>
      <parent>base_link</parent>
      <child>lidar::link</child>
    </joint>
"""

# 2026-09-09用户要求"测距雷达改为红色"。`model://lidar`内部那个视觉体
# （黑色圆柱体，radius 0.006/length 0.05）是嵌套子模型自己的属性，
# Gazebo Classic的`<include>`没有覆盖嵌套模型内部sensor/visual属性的
# 机制（本文件上面RANGEFINDER注释里已经写过这条限制）——不能直接改它的
# 颜色。做法是额外焊一个纯视觉、无碰撞体、无传感器的红色圆柱体
# （radius 0.008, length 0.052，比lidar那颗0.006/0.05大一圈），挂在
# 同一个`{xyz}`平移位置——把官方那个视觉体整个包在里面挡住，肉眼看到的
# 就是红色。（2026-09-10曾经短暂换成sonar又改回lidar，这颗覆盖圆柱体
# 尺寸够大，两种传感器的官方视觉体都能盖住，不用跟着改。）
# 姿态原样不转（0 0 0），照抄官方model.sdf里那颗视觉体本身的写法——
# 它自己也没有额外旋转，尽管旁边的sensor有pitch=+π/2，视觉体跟sensor
# 用的不是同一个pose（这颗视觉体是竖直摆放的一根小圆柱，不是指向下方，
# 纯粹是外观标识，不代表实际测距方向）。
RANGEFINDER_VISUAL_TEMPLATE = """
    <link name='{ns}_rangefinder_visual_link'>
      <pose>{xyz} 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-7</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-7</iyy><iyz>0</iyz><izz>1e-7</izz>
        </inertia>
      </inertial>
      <visual name='{ns}_rangefinder_visual'>
        <geometry><cylinder><radius>0.008</radius><length>0.052</length></cylinder></geometry>
        <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
    <joint name='{ns}_rangefinder_visual_joint' type='fixed'>
      <parent>base_link</parent>
      <child>{ns}_rangefinder_visual_link</child>
    </joint>
"""


def merge_rangefinder(sdf_path: str, namespace: str) -> None:
    """在已经焊过mid360的sdf上再焊对地测距雷达（in-place，读了再写回自己）。"""
    with open(sdf_path) as f:
        content = f.read()

    if content.count("</model>") != 1:
        raise RuntimeError(
            f"expected exactly one </model> in {sdf_path}, found {content.count('</model>')}"
        )

    rangefinder_block = (
        RANGEFINDER_SDF_TEMPLATE.format(ns=namespace, xyz=RANGEFINDER_MOUNT_XYZ)
        + RANGEFINDER_VISUAL_TEMPLATE.format(ns=namespace, xyz=RANGEFINDER_MOUNT_XYZ)
    )
    content = content.replace("</model>", rangefinder_block + "\n  </model>")

    with open(sdf_path, "w") as f:
        f.write(content)


# 挂载位置：前视相机在机头前方水平朝前(+X)；下视相机在机体下方朝下(-Z)。
# hfov=1.3963弧度(80度)，跟mighty那份被弃用的generic_camera调用参数
# (hfov=90度)取同一量级，不是精确复刻。res 640x480/15Hz是仿真侧"够用于
# AprilTag/二维码解码"的保守选择，不是照抄任何真机参数——真机vision-stack
# 用的是1280x720（见docker_real/docker_sim/src/vision_stack/
# image_publisher_node.py），仿真这边分辨率没有必须对齐真机的理由，但如果
# 后续发现640x480下tag解码不稳定，第一步可以尝试调高这个分辨率。
# color: 纯视觉区分用（前视/下视不容易从姿态一眼看出区别），不代表任何
# 物理含义，前视选蓝、下视选绿，跟mid360银色整流罩/测距雷达黑色圆柱都
# 区分得开。
#
# front的挂载X坐标2026-09-09从0.15改成0.08——用户反馈截图里蓝色视觉体
# 悬空在机身外面一截，肉眼看着不像"装在机身上"。iris机体真实的可视化
# 网格比它自己的碰撞体(0.47x0.47x0.11，见mid360挂载注释)小不少——第一次
# 只改到0.08米/-0.05米，用户截图复查后反馈下视那颗依然太远，而且明确
# 说"两个小方块可以有一部分嵌入到机体中"，不用死抠着刚好贴合机身表面
# 这条线，往机身里面再收一截更保险。改成front=0.04米、down=-0.02米，
# 都在mid360挂载注释那个0.11米高碰撞体范围以内，纯粹是根据反馈肉眼调的
# 经验值，没有对应到机体真实网格的精确尺寸数据，视觉体故意让它跟机体
# 网格有重叠。
CAMERA_MOUNTS = {
    # 2026-09-09第三轮调整：真机（不是临时测试实体）截图里发现front/down
    # 都整个缩没入机腹了，用户给了精确指令——"前视摄像头往前伸一个摄像头
    # 长度，下视摄像头往下伸半个摄像头长度"。视觉体box的"长度"是沿取景
    # 方向的0.04米那条边（`<size>0.04 0.02 0.02</size>`，见
    # CAMERA_SDF_TEMPLATE），在当前挂载值基础上front再往+X移0.04米、
    # down再往-Z移0.02米（半个0.04）。
    # 2026-09-09第四轮：用户在Gazebo GUI里人工看了实际重建出来的NX01/NX02
    # 之后给的精确微调——"前视摄像头可以往前再伸2cm，下视摄像头需要往上
    # 缩2cm"。都是在第三轮基础上再叠加：front 0.08→0.10，down
    # -0.04→-0.02。
    # 2026-09-09第五轮：继续人工看实机微调——"前视摄像头往回缩1cm，下视
    # 摄像头往下伸1cm"。front 0.10→0.09，down -0.02→-0.03。
    # 2026-09-16：update_rate从15降到10——查出`front`/`down`两路相机在
    # Gazebo Classic里共用同一个OpenGL/OGRE渲染上下文，架构上没法并行
    # 渲染，是`down`比`front`稀疏这个现象的真根因（不是ROS2/Python这层
    # 能修的，详见DEBUG_JOURNAL.md 2026-09-16"down相机问题继续深挖"）。
    # 降低两路相机各自的标称帧率，减轻共享渲染管线的总负载，缓解这个
    # 争抢，不是从根上消除（架构限制本身还在）。
    "front": {"pose": "0.09 0 0 0 0 0", "hfov": "1.3963", "res_x": "640", "res_y": "480", "update_rate": "10", "color": "0.1 0.3 0.9 1"},
    # 2026-09-15实测排查全过程（最终结论见本段末尾）：
    # 1. 原挂载`0 0 -0.03 0 1.5707963 0`（pitch=+90度）拍到的画面完全
    #    均匀纯灰色（std=0.0，像素值178，跟世界文件<background>0.7 0.7
    #    0.7 1(0.7×255≈178.5)几乎精确吻合），一度怀疑是pitch符号错了
    #    （朝上看天空，不是朝下看地面），翻成-1.5707963。
    # 2. 翻转pitch之后改拍到自己机身的桨叶/起落架——排查发现`CAMERA_
    #    MOUNTS`历史上5轮微调全是在Gazebo GUI里肉眼看"这个小方块贴合
    #    机身好不好看"调出来的挂载位置，从来没人真正检查过实际拍到的
    #    画面，贴得越紧看着越"像装上去"，但相机视角也跟着被机体挡住。
    #    把Z从-0.03大幅挪远到-0.30（挪出机体阴影范围，具体数值是实拍
    #    验证出来的经验值，不是理论精确算出来的）。
    # 3. pitch=-1.5707963+Z=-0.30这个组合实测多次拍到的还是机身（不管
    #    悬停在1.5米还是2米高度，画面都一样，说明相机看的是跟自己机体
    #    绑定的固定视角，不是随高度变化的外部世界）。
    # 4. 用户提出"直接换回pitch=90度试试"——**这才是最终正确答案**：
    #    pitch保持原始的+1.5707963不变，只要Z挪远到-0.30就够了，不需要
    #    翻转pitch符号。第1步观察到的"纯灰色天空"现象，回头看应该是
    #    Z太近(-0.03)时camera clipping/自身遮挡跟pitch共同作用出的一个
    #    误导性结果，不是pitch符号本身错——**真正唯一的根因自始至终
    #    都是挂载离机身太近，跟pitch符号无关**，中间翻转pitch这一步是
    #    多绕的弯路。最终实拍验证：悬停在地面火情点`fire_point_marker`
    #    正上方，干净拍到红色底座+AprilTag黑白图案，不再是机身/天空。
    # 2026-09-15用户指出-0.30米对真机不现实（相机吊在机身下方30厘米，
    # 容易被撞断），要求验证更小的、更接近真机可行安装距离的值够不够
    # 清空自身遮挡——0.30是"挪到能看清为止"的经验值上限，不是必须要
    # 这么大。实测依次验证了0.1米（干净）、对齐测距仪挂载高度
    # `RANGEFINDER_MOUNT_XYZ`的-0.0217米（同样干净，且是目前测过最
    # 贴近机身、最现实的值——原本担心这个值比最早失败的-0.03米还要
    # 靠近机体、可能复现近裁剪面卡进机体网格的渲染异常，实测证明这个
    # 担心多余，效果完全正常）——**最终定案就用这个值**，用户确认。
    # 2026-09-16：multicamera合并之后down画面变回纯灰色（178，等于世界
    # 背景色），翻过pitch符号测试——翻转后用户在GUI里直接确认"看的是
    # 天"，证实翻转方向是错的，原始+1.5707963才是朝下的正确方向，跟
    # 最早那次下视相机排查的最终结论一致（pitch从来不是问题，问题是
    # 挂载离机身太近）。pitch改回原始值，Z距离先维持-0.0217不变，下一步
    # 单独排查是不是multicamera结构下这个距离不够用了。
    # 2026-09-16再续：确认multicamera这条路对down走不通之后，退回独立
    # sensor结构（STANDALONE_CAMERAS），挂载位姿恢复2026-09-15那轮最终
    # 定案值——用户直接要求"独立sensor恢复原来安装位置"。
    "down": {"pose": "0 0 -0.0217 0 1.5707963 0", "hfov": "1.3963", "res_x": "640", "res_y": "480", "update_rate": "10", "color": "0.1 0.8 0.2 1"},
}


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


def merge_cameras(
    sdf_path: str, namespace: str, camera_names: list, camera_type: str = "camera"
) -> None:
    """在已经焊过mid360的sdf上再焊相机传感器（in-place，sdf_path读了再
    写回自己）。2026-09-16几轮multicamera改造全部证明对down无效（见
    CAMERA_SDF_TEMPLATE上面那段注释），最终定案退回最初就验证过内容
    可靠的独立`<sensor type='camera'>`，按camera_names的顺序依次插入
    （用于配合声明顺序决定FIFO优先级，down要声明在front前面）。

    `camera_type`目前只接受`'camera'`（真实图像渲染）——2026-09-17曾经
    新增过`'logical_camera'`（纯几何视锥判断，不进共享渲染队列）这个
    选项，排查了一整天确认插件在gzserver进程内publish()之后消息完全
    送不到任何订阅者、根因未定位到，2026-09-18已彻底放弃这条路、相关
    代码删除，见DEBUG_JOURNAL.md 2026-09-17/09-18记录。
    """
    if not camera_names:
        return
    if camera_type != "camera":
        raise ValueError(f"camera_type必须是'camera'，收到{camera_type!r}")

    with open(sdf_path) as f:
        content = f.read()

    if content.count("</model>") != 1:
        raise RuntimeError(
            f"expected exactly one </model> in {sdf_path}, found {content.count('</model>')}"
        )

    blocks = []
    for cam in camera_names:
        if cam not in CAMERA_MOUNTS:
            raise ValueError(f"unknown camera name '{cam}', expected one of {list(CAMERA_MOUNTS)}")
        mount = dict(CAMERA_MOUNTS[cam])
        blocks.append(CAMERA_SDF_TEMPLATE.format(ns=namespace, cam=cam, **mount))

    content = content.replace("</model>", "\n".join(blocks) + "\n  </model>")

    with open(sdf_path, "w") as f:
        f.write(content)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", required=True, help="e.g. NX01")
    parser.add_argument("--instance", type=int, required=True, help="0-based PX4 -i instance index")
    parser.add_argument("--output", required=True, help="final merged sdf output path")
    parser.add_argument(
        "--cameras", default="",
        help="逗号分隔的相机列表，取值来自CAMERA_MOUNTS的key（front/down），"
             "空字符串=不挂相机。例：--cameras front,down",
    )
    parser.add_argument(
        "--camera-type", default="switchable", choices=("camera", "switchable"),
        help="这架飞机相机用哪种方案：'camera'（真实图像渲染，`--cameras`"
             "列表里每个名字各焊一个独立传感器）、'switchable'"
             "（2026-09-18新增，默认值——单个真实相机+可动关节，`--cameras`"
             "参数对这个模式无效，见merge_switchable_camera()说明）。",
    )
    parser.add_argument(
        "--camera-initial-view", default="down", choices=("front", "down"),
        help="仅camera-type=switchable时生效：这架飞机spawn出来那一刻，"
             "唯一那个相机的关节初始朝向。默认down——需要按飞机分别指定时"
             "由调用方（sim-world-entrypoint.sh的CAMERA_INITIAL_VIEW_${NS}）"
             "覆盖。",
    )
    args = parser.parse_args()

    iris_tmp = f"/tmp/iris_{args.namespace}.sdf"
    render_iris(args.instance, iris_tmp)
    merge_mid360(iris_tmp, args.namespace, args.output)
    merge_rangefinder(args.output, args.namespace)
    if args.camera_type == "switchable":
        merge_switchable_camera(args.output, args.namespace, initial_view=args.camera_initial_view)
        print(
            f"[gen_iris_mid360_sdf] wrote {args.output} "
            f"(camera_type=switchable, initial_view={args.camera_initial_view})"
        )
    else:
        camera_names = [c.strip() for c in args.cameras.split(",") if c.strip()]
        merge_cameras(args.output, args.namespace, camera_names, camera_type=args.camera_type)
        print(
            f"[gen_iris_mid360_sdf] wrote {args.output} "
            f"(cameras={camera_names or 'none'}, camera_type={args.camera_type})"
        )


if __name__ == "__main__":
    main()
