# GCS用RViz配置

2026-08-19新增，供真机地面站（GCS）显示用。这两份文件**不是新导出的**，
是`docker/Dockerfile.sim-world`已经在用、确认是"最新版本"的同一份文件的
原样拷贝，来源见该文件里`COPY patches/mighty_rviz_multi_mighty.rviz ...`/
`COPY patches/mighty_rviz_multi_ego_planner.rviz ...`两行及其上方注释：

- `multi_mighty.rviz`——2026-08-10从一次实际跑起来的sim-world容器里现场
  导出，PLANNER=mighty时用。
- `multi_ego_planner.rviz`——2026-08-13同样从实际跑起来的sim-world容器里
  现场导出（导出时是NUM_AGENTS=1单机彩排模式，TF树/SyncSource/Fixed
  Frame只有NX01，没有NX02，是当时真实运行状态），PLANNER=ego_planner时用。

`patches/`目录下的原件是给`docker/Dockerfile.sim-world`构建镜像用的
构建输入，这里额外放一份是为了真机GCS场景直接引用，不用因为找rviz配置
文件而去翻`patches/`（那个目录混着几十个`.patch`文件，语义上是"构建
输入"而不是"GCS要用的资源"）。**两份文件内容完全一致，以后如果要更新
（比如又从某次容器里现场导出了新配置），两处都要同步覆盖**——`patches/`
那份是构建镜像的输入，不能删；这里这份是GCS本机直接引用的副本，不能漏更。

**2026-08-19更新，缺口已补上**：`gcs/Dockerfile`加装了`ros-humble-
rviz2`，这两份文件也一起`COPY`进镜像`/opt/rviz/`下，`gcs/docker-
compose.yml`配了`DISPLAY`+`/tmp/.X11-unix`转发。容器起来后用：

```bash
docker exec -it docker_sim-gcs-1 rviz2 -d /opt/rviz/multi_mighty.rviz       # PLANNER=mighty
docker exec -it docker_sim-gcs-1 rviz2 -d /opt/rviz/multi_ego_planner.rviz  # PLANNER=ego_planner
```

宿主机侧需要`xhost +local:docker`（或等效授权）允许容器连接X server，
这一步还没有实测过。
