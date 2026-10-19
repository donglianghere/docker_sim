# 选手容器的网络参数（用 source 引入，不单独运行）。
#
# ⚠️ 仓库外有三个脚本用**绝对路径** source 这个文件，别挪别改名：
#     /home/robots/ai_uav/start_contestant_task.sh
#     /home/robots/ai_uav/start_contestant_shell.sh
#     /home/robots/ai_uav/start_sound_light_server.sh   ← 声光常驻程序靠它
# 原来还有 contestant_template/运行仿真.sh 也 source 它，那套旧入口已于
# 2026-10-01 删除（统一用 一键仿真/run.sh）。
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
    # local：**真机域（20）但只在本机通信**，不钉飞机那张网卡、不列飞机对等体。
    # 给两类东西用：
    #   · 声光常驻程序——板子在地面站上，它只跟同一台机器上的选手容器说话，
    #     飞机从不发声光请求。原来它跟选手容器一样钉在"连飞机的网卡"上，
    #     结果地面站一换网（2026-10-04 换到另一个 SSID）它的 DDS 就卡死刷
    #     `ddsi_udp_conn_write to udp/192.168.2.101:... failed`，对本机也不通，
    #     板子整个变砖。
    #   · 声明了 NEEDS_AIRCRAFT=False 的测试程序（比如 t6 只测声光装置）。
    # 回环必须是**显式的网卡 + 显式的对等体**：只给域号（像 sim 那样）时
    # CycloneDDS 会 autodetermine 挑一张真网卡，两边挑得不一样就发现不了。
    if [[ "$mode" == "local" ]]; then
        # local：真机域（20）但**不钉任何网卡**——交给 CycloneDDS 自己选，跟 sim
        # 模式同一种做法（sim 也是只给域号），只是域号不同。
        #
        # 给两类东西用：
        #   · 声光常驻程序——板子在地面站上，它只跟同一台机器上的选手容器说话，
        #     飞机从不发声光请求。原来它跟选手容器一样钉死在"连飞机那张网卡"上，
        #     地面站一换 SSID 它的 DDS 就卡在
        #     `ddsi_udp_conn_write to udp/192.168.2.101:... failed` 上刷屏，
        #     对本机也不通，板子整个变砖（2026-10-04 实测）。
        #   · 声明了 NEEDS_AIRCRAFT=False 的测试程序（比如 t6 只测声光装置）。
        #
        # ⚠️ 为什么不显式写回环：试过 `<NetworkInterface address="127.0.0.1">`，
        # CycloneDDS 会打 `selected interface "lo" is not multicast-capable:
        # disabling multicast` 并把组播整个关掉。纯单播又要 `ParticipantIndex`
        # 才能被对等体找到，而且**真机模式的容器仍然钉在 LAN 网卡上、碰不到
        # 回环**——所以显式回环只能自己跟自己通，解决不了"真飞时也要能发声"。
        # 让两边都 autodetermine，它们会选到同一张网卡，有没有飞机都能碰上。
        CONTESTANT_NET_ARGS=(-e ROS_DOMAIN_ID=20)
        CONTESTANT_NET_DESC="本机（ROS_DOMAIN_ID=20，网卡由 CycloneDDS 自选，不依赖飞机网段）"
        return 0
    fi
    # device：**声光常驻程序专用**——同时监听 lo 和（存在时）连飞机那张网卡。
    #
    # 为什么要两张：声光板在地面站上，请求来自两类参与者
    #   · 真飞时的选手容器 —— 走 real 模式，钉在 $iface 上，只能在 LAN 上碰面
    #   · 不连飞机的测试（t6）和 local 模式 —— 只在 lo 上
    # 原来声光容器直接用 real 模式（只有 LAN），所以地面站一换 SSID 它就变砖；
    # 只用 lo 又会让真飞时的选手容器找不到它。两张都挂才能满足"没有飞机 /
    # 一架 / 两架"三种情况都能发声（2026-10-04 用户要求）。
    #
    # ⚠️ **优先级必须拉开**：lo 给 0、LAN 给 10。两个都写 priority="default" 时
    # CycloneDDS 会挑 lo 当主网卡并打
    # `selected interface "lo" is not multicast-capable: disabling multicast`，
    # 把组播整个关掉——2026-10-04 我在 real 模式里这么干过一次，结果所有发往
    # 192.168.2.101/102 的写入全部 retcode -3，**到飞机的链路直接断**，
    # basic_test 在 takeoff:set_height 上 5 秒超时。
    #
    # LAN 那张**不存在就不写**（地面站没连 uav 时），否则 autodetermine="false"
    # 指向一个不存在的地址会让 DDS 起不来。
    if [[ "$mode" == "device" ]]; then
        local dif="${REAL_IFACE:-}"
        local dstate="$sim_dir/gcs/gcs_network_state.json"
        if [[ -z "$dif" && -n "$sim_dir" && -f "$dstate" ]]; then
            dif="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("real",{}).get("interface_address",""))' "$dstate" 2>/dev/null)"
        fi
        local lan_xml=""
        if [[ -n "$dif" ]] && command -v ip >/dev/null 2>&1 && ip -4 addr | grep -qw "inet $dif"; then
            lan_xml="        <NetworkInterface autodetermine=\"false\" address=\"$dif\" priority=\"10\" multicast=\"true\"/>"$'\n'
        fi
        mkdir -p "$out_dir"
        local dxml="$out_dir/cyclonedds_device.xml"
        cat > "$dxml" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!-- contestant_network.sh 自动生成（声光设备模式），每次运行都会重写，不要手改 -->
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
$lan_xml        <NetworkInterface name="lo" priority="0" multicast="true"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
EOF
        CONTESTANT_NET_ARGS=(
            -e ROS_DOMAIN_ID=20
            -e CYCLONEDDS_URI=file:///opt/contest_device_cyclonedds.xml
            -v "$dxml":/opt/contest_device_cyclonedds.xml:ro
        )
        if [[ -n "$lan_xml" ]]; then
            CONTESTANT_NET_DESC="声光设备（ROS_DOMAIN_ID=20，网卡 lo + $dif）"
        else
            CONTESTANT_NET_DESC="声光设备（ROS_DOMAIN_ID=20，仅 lo——没检测到 ${dif:-飞机网段} 那张网卡，真飞前要重起一次）"
        fi
        return 0
    fi
    if [[ "$mode" != "real" ]]; then
        echo "!! contestant_net_args: 模式只能是 sim / real / local / device，收到 '$mode'" >&2
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
      <!-- ⚠️ 2026-10-04：这里**不要再加回环网卡**。试过加一条
           「address="127.0.0.1" priority="default"」，CycloneDDS 直接把 lo 当成主
           网卡、打出 「selected interface "lo" is not multicast-capable: disabling
           multicast「，然后所有发往 192.168.2.101/102 的写入全部 」retcode -3」，
           **整条到飞机的链路断掉**（basic_test 在 takeoff:set_height 上 5 秒超时，
           因为 pt4ctrl 在飞机上）。本机内部那条链路另走 local 模式，见上面。 -->
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
