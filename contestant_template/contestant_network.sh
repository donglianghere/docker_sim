# 选手容器的网络参数（被 运行仿真.sh 和 /home/robots/ai_uav/start_*.sh 用 source 引入，
# 不单独运行）。
#
# 仿真和真机用两个不同的 ROS_DOMAIN_ID，互相隔离，选手代码不会连错目标：
#   仿真 = 21：镜像自带的 DDS 配置只走本机回环（lo），跟同一台电脑上的仿真容器通信。
#   真机 = 20：DDS 走这台电脑连飞机的网卡。网卡 IP 和飞机 IP 取自地面站"GCS网络设置"
#              面板保存的 gcs/gcs_network_state.json（地面站真机模式用的就是这一份），
#              也可以用环境变量 REAL_IFACE / REAL_PEERS 直接指定。
#
# 用法：
#   source contestant_network.sh
#   contestant_net_args sim|real <仿真项目目录docker_sim，可为空> <生成配置文件放哪个目录>
#   docker run ... "${CONTESTANT_NET_ARGS[@]}" ...

contestant_net_args() {
    local mode="$1" sim_dir="$2" out_dir="$3"
    CONTESTANT_NET_ARGS=()
    CONTESTANT_NET_DESC=""

    if [[ "$mode" == "sim" ]]; then
        CONTESTANT_NET_ARGS=(-e ROS_DOMAIN_ID=21)
        CONTESTANT_NET_DESC="仿真（ROS_DOMAIN_ID=21，本机回环）"
        return 0
    fi
    if [[ "$mode" != "real" ]]; then
        echo "!! contestant_net_args: 模式只能是 sim 或 real，收到 '$mode'" >&2
        return 1
    fi

    local iface="${REAL_IFACE:-}" peers="${REAL_PEERS:-}"
    local state="$sim_dir/gcs/gcs_network_state.json"
    if [[ -z "$iface" && -n "$sim_dir" && -f "$state" ]]; then
        iface="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("real",{}).get("interface_address",""))' "$state" 2>/dev/null)"
        [[ -z "$peers" ]] && peers="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("real",{}).get("peer_addresses",""))' "$state" 2>/dev/null)"
    fi
    if [[ -z "$iface" ]]; then
        echo "!! 真机模式需要知道这台电脑连飞机用的网卡IP：" >&2
        echo "   在地面站网页'GCS网络设置'里保存一次，或者运行前设置环境变量，比如" >&2
        echo "   REAL_IFACE=192.168.2.103 REAL_PEERS=192.168.2.101,192.168.2.102" >&2
        return 1
    fi
    if command -v ip >/dev/null 2>&1 && ! ip -4 addr | grep -qw "inet $iface"; then
        echo "!! 这台电脑上没有 IP 为 $iface 的网卡（没连上飞机那个 WiFi？），用 ip -4 addr 确认。" >&2
        return 1
    fi

    local peers_xml="" p
    for p in ${peers//,/ }; do
        peers_xml+="        <Peer address=\"$p\"/>"$'\n'
    done
    mkdir -p "$out_dir"
    local xml="$out_dir/cyclonedds_real.xml"
    cat > "$xml" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!-- contestant_network.sh 自动生成（真机模式），每次运行都会重写，不要手改 -->
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="false" address="$iface" priority="default" multicast="true"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
$peers_xml      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
    CONTESTANT_NET_ARGS=(
        -e ROS_DOMAIN_ID=20
        -e CYCLONEDDS_URI=file:///opt/contest_real_cyclonedds.xml
        -v "$xml":/opt/contest_real_cyclonedds.xml:ro
    )
    CONTESTANT_NET_DESC="真机（ROS_DOMAIN_ID=20，网卡 $iface，飞机 ${peers:-靠组播自动发现}）"
}
