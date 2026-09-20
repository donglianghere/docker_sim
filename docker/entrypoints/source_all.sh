# 统一 source 所有 colcon workspace，供 flight-stack-entrypoint.sh 启动时用，
# 也供手动 `docker exec -it <container> bash` 进容器调试时用（一行代替十行）。
#
# ⚠️ 必须用 `source /opt/source_all.sh`（或 `. /opt/source_all.sh`）执行，
# 不能 `./source_all.sh`/`bash /opt/source_all.sh` 直接运行——这里改的环境变量
# （PATH/AMENT_PREFIX_PATH等）只有在当前shell里source才会生效，用子进程执行的
# 话，这些变量只在那个子进程里有效，子进程一退出就全部消失，调用者什么都拿不到。
#
# 顺序不是随便排的：
# 1. /opt/ros/humble 必须最先——后面每个workspace的setup.bash都是在它之上
#    做overlay（叠加ament前缀路径），底层环境没建好，后面全部作废。
# 2. livox_ws 必须排在 point_lio_ws/fastlio_ws 之前——point_lio和FAST-LIO2都
#    原生依赖livox_ros_driver2::msg::CustomMsg，可执行文件运行时(不只是编译时)
#    链接了这个类型支持库，顺序错了会找不到符号，表现是
#    "Package 'point_lio'/'fast_lio' not found"，2026-08-14实测踩过（见
#    DEBUG_JOURNAL.md）。fastlio_ws（2026-09-07新增，第三个可选SLAM后端）同理。
# 3. 其余几个（mighty_ws/dlio_ws/ros2_px4_stack_ws/px4ctrl_ws/ego_planner_ws/
#    uwb_origin_ws/nlink_ws）互相之间没有依赖关系，顺序不重要。nlink_ws
#    （真实UWB驱动nlink_parser2+nlink_uwb_bridge，2026-08-13编译进镜像）
#    之前一直没有被source过——仿真里从来不用它（uwb_sim是仿真替身），
#    真机DEPLOY_TARGET=hw模式下entrypoint会启动这个包的节点，不source
#    会重演point_lio当初"编译了但找不到包"的同类问题，这里补上。
# 4. contest_mission_ws（2026-09-08新增）是个例外：它依赖px4ctrl_ws里的
#    quadrotor_msgs（PositionCommand消息类型），必须排在px4ctrl_ws之后，
#    不能套用上面第3条"顺序不重要"。
#
# ROS2/colcon 生成的 setup.bash 内部会引用一堆没给默认值的变量（比如第一个就会
# 炸的 AMENT_TRACE_SETUP_FILES），跟 `set -u` 天生冲突——source 期间关掉 -u，
# source完再由调用者（flight-stack-entrypoint.sh）决定要不要重新打开。
set +u
source /opt/ros/humble/setup.bash
source /opt/decomp_ws/install/setup.bash 2>/dev/null || true
source /opt/mighty_ws/install/setup.bash
source /opt/dlio_ws/install/setup.bash
source /opt/livox_ws/install/setup.bash
source /opt/point_lio_ws/install/setup.bash
source /opt/fastlio_ws/install/setup.bash
source /opt/ros2_px4_stack_ws/install/setup.bash
source /opt/px4ctrl_ws/install/setup.bash
# contest_mission（2026大赛任务系统专属包，2026-09-08新增）用到
# quadrotor_msgs（PositionCommand消息类型），排在px4ctrl_ws（quadrotor_msgs
# 在这个workspace里编译，见Dockerfile.flight-stack）之后source，
# 依赖提供方先于依赖方，跟这个文件其余地方的排序原则一致。
source /opt/contest_mission_ws/install/setup.bash
source /opt/ego_planner_ws/install/setup.bash
source /opt/uwb_origin_ws/install/setup.bash
source /opt/nlink_ws/install/setup.bash
