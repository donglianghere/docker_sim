# GCS后端(容器B)镜像，2026-08-20新增。
#
# 不需要ROS2——这个容器只做"改配置文件+重启"这类浏览器/rosbridge做不到
# 的操作（见app.py文件头），不发布/订阅任何ROS2话题，不用跟flight-stack/
# sim-world共享DDS域，python:3.11-slim基础镜像足够，保持轻量。
FROM python:3.11-slim

# 跟这个项目其它Dockerfile一样的proxy写法——build阶段需要代理访问外网时
# 生效，build完重置成空不留在最终镜像里。
ARG HTTP_PROXY
ARG HTTPS_PROXY
ENV http_proxy=${HTTP_PROXY} \
    https_proxy=${HTTPS_PROXY}

# 2026-08-25删除了openssh-client——这个后端原来靠它SSH到Jetson做机队
# 启停/规划避障调参，两个功能当天都被要求删除了，app.py里已经没有任何
# 代码路径会调用ssh，不再需要装这个包。
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# 只需要docker CLI客户端本身（对着挂进来的/var/run/docker.sock发命令），
# 不需要完整docker-ce套件（dockerd/containerd等只有宿主机才需要）——用
# 官方静态二进制包，比额外配一份docker官方apt源+GPG key更轻量、构建
# 步骤更少。
#
# 2026-08-25改：原来是build阶段现场curl这两个文件（docker CLI约74MB、
# 下面的compose插件约64MB），实测这条链路只要Dockerfile里*任何*更早的
# 层缓存失效（改了requirements.txt/app.py之外的东西，比如那次删
# openssh-client），这两个大文件就要跟着重新下载一遍，加上这台机器的
# 出网带宽本来就慢，一次好几分钟——起因是用户反馈"来回build都要下载，
# 效率超低"，见DEBUG_JOURNAL.md 2026-08-25记录。改成COPY本地预先下载好
# 的文件（用gcs/backend/vendor/fetch.sh在宿主机上、用宿主机自己的网络/
# 代理提前拉一次，这两个文件本身不进git，见.gitignore），Docker的COPY
# 层缓存是按文件内容哈希算的，只要vendor/下这两个文件没变，这一层永远
# 命中缓存，不再依赖网络、也不再受"更早的层缓存失效"连带影响。
COPY gcs/backend/vendor/docker-26.1.4.tgz /tmp/docker.tgz
RUN tar -xz -C /usr/local/bin --strip-components=1 -f /tmp/docker.tgz docker/docker \
    && rm /tmp/docker.tgz

# 2026-08-21新增：`docker compose`子命令（v2是独立的CLI插件，不含在上面
# 那份docker CLI静态包里）——"仿真机队"启动改成`docker compose up -d`
# 动态传LOCALIZATION_SOURCE/PLANNER/CONTROLLER这几个环境变量，不再是
# 单纯`docker start`已经建好的容器（那样没法应用网页上新选的参数）。
# docker CLI发现插件的标准路径之一是`/usr/local/lib/docker/cli-plugins/`，
# 文件名必须是`docker-compose`、要有执行权限，`docker compose ...`才能
# 识别到这个子命令。同样2026-08-25改成COPY本地预下载文件，理由跟上面
# docker CLI那步一样。
RUN mkdir -p /usr/local/lib/docker/cli-plugins
COPY gcs/backend/vendor/docker-compose-linux-x86_64 /usr/local/lib/docker/cli-plugins/docker-compose
RUN chmod +x /usr/local/lib/docker/cli-plugins/docker-compose

WORKDIR /app
COPY gcs/backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY gcs/backend/app.py .

# 起飞后自标定动作需要的两个脚本——POST /api/sim/{ns}/takeoff会docker cp
# 这两份到flight-stack容器里执行，见app.py"4. 起飞（含自标定动作）"一节。
# docker cp的本地源路径是相对这个容器自己的文件系统（这个容器只是拿
# 挂进来的docker.sock跟宿主机dockerd说话，不代表能读到宿主机的文件），
# 必须先COPY进镜像，不能指望宿主机的scripts/目录。
COPY scripts/ros2_env_setup.sh scripts/ros2_env_setup.sh
COPY scripts/auto_calibration_flight.py scripts/auto_calibration_flight.py

# 构建阶段用完代理就重置成空，不让代理设置留在最终镜像里——跟
# gcs/Dockerfile结尾同样的做法。
ENV http_proxy= \
    https_proxy=

EXPOSE 8000
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8000"]
