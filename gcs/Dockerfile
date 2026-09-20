# 真机部署地面站(GCS)专用镜像，2026-08-14新增（真机部署第一批骨架代码）。
#
# 本机（Ubuntu 24.04）没有ROS2 Humble的官方apt二进制包（Humble只对应
# 22.04），原生装不了，实际可行路径是本机也跑一个轻量容器——但跟
# flight-stack镜像完全不是一回事：这里只装ROS2底座+GCS需要跨机订阅的
# 自定义消息定义包，不编译DLIO/mighty/ego_planner/px4ctrl这些算法实现。
# GCS只需要能parse消息类型、加入DDS域看话题/调service，不需要真的跑这些
# 算法节点。
#
# 目前只包含2026-08-14确认的真机首飞组合（LOCALIZATION_SOURCE=
# single_uwb_slam PLANNER=ego_planner CONTROLLER=px4ctrl SLAM_BACKEND=dlio）
# 实际会用到的自定义消息包：
#   - quadrotor_msgs（vendor/px4ctrl_ros2）：px4ctrl发布Px4ctrlDebug、接收
#     PositionCommand等用到的消息类型。
#   - traj_utils（staging/ego-planner-swarm/src/planner）：ego_planner的
#     Bspline/MultiBsplines/DataDisp消息类型。
# 如果以后GCS要监控mighty特有话题，需要额外加dynus_interfaces/
# fla_interfaces两个包（都在staging/mighty_ws_src下）；uwb_imu模式相关的
# 消息目前都是标准geometry_msgs，不需要额外的自定义包。
#
# 这两个包都验证过是"干净"的（package.xml检查过，见DEBUG_JOURNAL.md
# 2026-08-14相关记录）：quadrotor_msgs只依赖std_msgs/geometry_msgs；
# traj_utils依赖PCL/Eigen3，都不依赖这个仓库里其它任何自定义包，可以
# 单独拿出来编译，不会把DLIO/mighty那些重的算法实现一起拖进来。
# 2026-08-18实测更正：Eigen3确实ros-base已有，但PCL不在——之前这条注释
# 断言"两个系统库镜像里都已经有"是没实测过的猜测，实际build时
# `find_package(PCL REQUIRED)`报错找不到`PCLConfig.cmake`。只装
# `libpcl-dev`（不装`ros-humble-pcl-ros`/`pcl-conversions`那两个——那是
# 给"跟ROS消息类型转换"用的，traj_utils这里只是拿PCL类型定义msg，不需要
# 那层转换，装了徒增体积，跟这个镜像"保持轻量"的设计目标矛盾），跟
# docker/Dockerfile.base里flight-stack装PCL是同一个来源问题，只是那边是
# 三个包一起装、这里没那两个转换包的需求。
FROM ros:humble-ros-base-jammy

# 2026-08-14自查时补的——这台开发机build镜像访问Docker Hub/apt源需要走代理
# （见docker-compose.yml/Dockerfile.base同样的写法），漏了这两行的话在这台
# 机器上会直接build失败，不是Jetson/真机那边的事，是"在哪build这个GCS镜像"
# 的问题（这个镜像就是给本机/开发机用的，不会在Jetson上build）。
ARG HTTP_PROXY
ARG HTTPS_PROXY
ENV http_proxy=${HTTP_PROXY} \
    https_proxy=${HTTPS_PROXY}

# 2026-08-19踩坑：rviz2拆成单独一层装。原来5个包在一条RUN里，某次build
# 装到最后一个包（ros-humble-rviz-assimp-vendor）时packages.ros.org
# 通过本机代理（127.0.0.1:7897）返回502 Bad Gateway——纯粹是这次抓取
# 495MB/605个包（rviz2把整条Qt5+VTK+assimp/ogre-vendor依赖链都拉进来）
# 过程中的瞬时网络问题，不是Dockerfile写法的问题，但因为是同一条RUN，
# 一失败整层作废，重新build要把这495MB/28分钟全部重新下一遍。拆成两层后
# 前面这些体积小、依赖稳定的包失败率低，装成功就能被Docker缓存住，真正
# 容易碰到代理抖动的rviz2单独一层，重试只需要重下它自己那条依赖链。
# 同时加`-o Acquire::Retries=3`让apt自己对瞬时5xx错误做几次重试，减少
# 需要手动重跑build的情况。
#
# 2026-08-25再踩一次同类坑，这次是libpcl-dev：build卡在下载阶段20多分钟
# 之后，最后3个包(libxkbcommon-x11-0/libqt5quickshapes5/libcbor0.8，
# 都是libpcl-dev可视化模块拖进来的Qt5依赖链上的)502 Bad Gateway失败，
# 整条RUN(跟下面几个体积小的ROS包挤在一起)全部作废，之前已经下载成功
# 的479MB也要重新来一遍——libpcl-dev单独一个包就会拖出577个包/482MB
# （PCL可视化模块依赖VTK+Qt5+GDAL+Boost-all+OpenMPI+JDK11一整套，比上面
# rviz2那次495MB还夸张），体积/依赖复杂度已经跟rviz2是同一个量级，混在
# "体积小、依赖稳定"那一层里不合适，跟rviz2同样的道理单独拆出来，重试
# 只用重下它自己这条依赖链，不用连累旁边几个小包重新验证一遍；
# Acquire::Retries从3提到6，给瞬时5xx多几次自我恢复的机会。
RUN apt-get update && apt-get install -y --no-install-recommends \
    -o Acquire::Retries=6 \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-mavros-msgs \
    ros-humble-rosbridge-suite \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    -o Acquire::Retries=6 \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    -o Acquire::Retries=6 \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# 2026-08-20新增：decomp_rviz_plugins（走廊多面体可视化插件）——2026-08-19
# 验证rviz2+X11链路时实测发现的缺口，两份rviz配置（gcs/rviz/下）都引用了
# `decomp_rviz_plugins/PolyhedronArray`这个Display类型，但GCS镜像当时只装了
# 官方`ros-humble-rviz2`，没编译这个插件，RViz2打开时对应Display报
# `PluginlibFactory`加载失败（non-fatal，其余Display如TF/PointCloud2/Path
# 不受影响，但"走廊可视化"这一路看不到）。跟sim-world/flight-stack镜像
# 编译`decomp_rviz_plugins`同一套依赖，需要单独补装`ros-humble-pcl-
# conversions`——这个包镜像顶部注释里之前明确说过"不需要"（那是针对
# traj_utils只用PCL类型定义msg、不需要ROS消息转换层的场景），但
# decomp_rviz_plugins的package.xml确实<depend>了pcl_conversions，是不同
# 包的真实依赖，不是重复踩同一个坑。
RUN apt-get update && apt-get install -y --no-install-recommends \
    -o Acquire::Retries=6 \
    ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

COPY vendor/px4ctrl_ros2/quadrotor_msgs /opt/gcs_ws/src/quadrotor_msgs
COPY staging/ego-planner-swarm/src/planner/traj_utils /opt/gcs_ws/src/traj_utils

RUN . /opt/ros/humble/setup.sh && \
    cd /opt/gcs_ws && \
    colcon build --symlink-install

# 只搬decomp_util/decomp_ros_msgs/decomp_rviz_plugins三个包（不要
# decomp_test_node——那是给独立测试节点用的demo，GCS只需要rviz插件本身，
# 跟这个镜像"只装监控必需最小集合"的设计目标一致，sim-world/flight-stack
# 那两个镜像整个DecompROS2目录搬过去是因为它们本来就要build mighty全套
# 依赖，体积不是主要考量，这里不一样）。decomp_util不装到gcs_ws（跟
# sim-world/flight-stack一样的顺序问题：decomp_rviz_plugins的package.xml
# 没声明依赖decomp_util，colcon不会自动把它的安装路径注入
# CMAKE_PREFIX_PATH，必须先单独build它、手动source一次它的install/
# setup.bash，下一步的find_package(decomp_util)才找得到）。
COPY staging/decomp_ws_src/DecompROS2/DecompUtil /opt/decomp_ws/src/DecompUtil
COPY staging/decomp_ws_src/DecompROS2/decomp_ros_msgs /opt/decomp_ws/src/decomp_ros_msgs
COPY staging/decomp_ws_src/DecompROS2/decomp_rviz_plugins /opt/decomp_ws/src/decomp_rviz_plugins
RUN . /opt/ros/humble/setup.sh && \
    cd /opt/decomp_ws && \
    colcon build --packages-select decomp_util --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    . /opt/decomp_ws/install/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/gcs_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/decomp_ws/install/setup.bash" >> /root/.bashrc

# 2026-08-18新增：最简网页地面站——rosbridge把ROS2话题转成WebSocket，
# 前端是一个不需要构建步骤的静态HTML，entrypoint.sh同时把两者起在同一个
# 容器里（rosbridge监听9090，静态文件用python3 http.server起在8080，
# network_mode:host所以两个端口直接是宿主机端口）。这是《网页端地面站
# 控制面板方案.md》阶段1+3+4的最小实现，不含起降/机队控制（那是后续
# 阶段，需要单独的容器B后端）。
# roslib.js不走运行时CDN，打进镜像——热点现场大概率没有公网出口，浏览器
# 要能直接从GCS本机8080端口把这个依赖也一起拿到，不能依赖运行时再去连
# CDN。2026-08-25改：原来是build阶段现场curl，跟docker CLI/compose插件
# 那两个大文件同样的"网络慢、来回build要重下"的问题（虽然这个文件只有
# 66KB，量级差很多，但道理一样），改成COPY本地已经下载好的
# gcs/frontend/roslib.min.js（这个文件不大，直接提交进git，不用像那两个
# 几十MB的二进制一样单独走.gitignore+fetch脚本那一套）。
COPY gcs/frontend /opt/frontend

# 2026-08-19新增：gcs/rviz/下两份"从容器现场导出确认为最新版本"的RViz
# 配置（见gcs/rviz/README.md）一并打进镜像，配合上面新装的ros-humble-
# rviz2——之前只把文件拷贝到宿主机就停了，镜像里没有rviz2也没这两个
# 文件，实际打不开，这次补齐让它真正能用。⚠️2026-08-19实测确认
# `docker exec -it <容器名> rviz2 ...`这种直接指定命令（不经过bash）的
# 调用方式不会读`/root/.bashrc`，ROS2/decomp_ws这些overlay的source不会
# 生效（找不到`ros2`/`rviz2`可执行文件本身依赖的AMENT_PREFIX_PATH），
# 实测能跑通的写法是显式`bash -c`：
#   docker exec -it <容器名> bash -c "source /opt/ros/humble/setup.bash && \
#     source /opt/gcs_ws/install/setup.bash && \
#     source /opt/decomp_ws/install/setup.bash && \
#     rviz2 -d /opt/rviz/multi_mighty.rviz"
# （decomp_ws这行2026-08-20新增，装的是`decomp_rviz_plugins`——不source
# 这一份，两份rviz配置里的`PolyhedronArray`走廊可视化Display会报
# PluginlibFactory加载失败，其余Display不受影响，见DEBUG_JOURNAL.md
# 2026-08-19"新发现的缺口"那条记录）。或者用`docker exec -it <容器名>
# bash`先进容器交互式shell（.bashrc在这种调用方式下才会被读取，三个
# source在里面已经预先写好了），再手敲`rviz2 -d ...`。
# 需要宿主机开着X11转发（docker-compose.yml已配DISPLAY+/tmp/.X11-unix）。
COPY gcs/rviz/multi_mighty.rviz gcs/rviz/multi_ego_planner.rviz /opt/rviz/

# build阶段用完代理就重置成空——运行时这个容器不应该带着build环境的代理
# 设置，跟Dockerfile.base结尾同样的做法。
ENV http_proxy= \
    https_proxy=

COPY gcs/entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

# 长驻容器，日常用`docker exec -it <container> bash`进去用ros2 CLI工具/
# 未来的GCS运维脚本；entrypoint.sh负责起rosbridge+静态文件服务。
ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["sleep", "infinity"]
