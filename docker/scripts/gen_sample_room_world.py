#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 sample_room_layout.yaml 渲染样题版场景 worlds/sample_room.world。

2026-09-29 新建，对标 2026-09-27 商定的大赛样题。

为什么这份 world 是**生成**的，而 fire_drill_room.world 是手写的：
样题版每根立柱正东都紧挨一根孪生柱（3 根变 6 根），每根柱子还要每米画一圈
楼层线（6 根 x 5 圈 = 30 个视觉体）。手工把这些 <pose> 跟 yaml 对齐，第一次
就得敲几百行，以后改一个坐标要同步改十几处——必错。所以这里落成脚本：
**改布局只改 yaml，然后重跑这个脚本**。
（fire_drill_room_layout.yaml 顶上那段注释早就写了"等真要做随机化布局时再写
一个从 yaml 生成 world 的小脚本"，就是这个。）

坐标系：原点在房间**西南角**，房间占 x∈[0,20]、y∈[0,25]——跟 fire_drill_room
的"原点在房间中心"不一样，原因见 yaml 里的说明。两套 world 各自独立。

用法：
    python3 gen_sample_room_world.py \\
        --layout src/contest_mission/config/sample_room_layout.yaml \\
        --output docker/worlds/sample_room.world
"""
import argparse
import math

import yaml

WALL_MAT = "Gazebo/White"      # 照抄 fire_drill_room：墙是白的，地面才是灰的
GROUND_MAT = "Gazebo/Grey"
PILLAR_RGBA = (0.72, 0.55, 0.36)      # 浅褐色（2026-09-29 用户要求，原来是深褐 0.40/0.26/0.13）
CYLINDER_MAT = "Gazebo/Blue"
TERRAIN_MAT = "Gazebo/Yellow"

HEADER = """<?xml version="1.0" ?>
<!-- 本文件由 docker/scripts/gen_sample_room_world.py 生成，不要手工改。
     改布局请改 src/contest_mission/config/sample_room_layout.yaml 后重新生成。 -->
<sdf version='1.7'>
  <world name='default'>
    <plugin name='gazebo_ros_state' filename='libgazebo_ros_state.so'>
      <ros>
        <namespace>/plug</namespace>
        <argument>model_states:=model_states_plug</argument>
        <argument>link_states:=link_states_plug</argument>
      </ros>
      <update_rate>100.0</update_rate>
    </plugin>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <!-- max_step_size 必须保持 0.001：调大（0.002/0.004）会让 gzserver 在
         spawn 带 mavlink_interface 的 iris 时崩溃，详见 fire_drill_room.world
         同一处注释和 DEBUG_JOURNAL.md 2026-09-15 记录。 -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <island_threads>8</island_threads>
          <thread_position_correction>1</thread_position_correction>
        </solver>
      </ode>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""


def script_mat(name, indent):
    p = " " * indent
    return (f"{p}<material>\n"
            f"{p}  <script>\n"
            f"{p}    <uri>file://media/materials/scripts/gazebo.material</uri>\n"
            f"{p}    <name>{name}</name>\n"
            f"{p}  </script>\n"
            f"{p}</material>\n")


def rgba_mat(rgb, indent):
    p = " " * indent
    c = "%.2f %.2f %.2f 1" % tuple(rgb)
    return (f"{p}<material>\n"
            f"{p}  <ambient>{c}</ambient>\n"
            f"{p}  <diffuse>{c}</diffuse>\n"
            f"{p}</material>\n")


def marker_mat(name, indent):
    p = " " * indent
    return (f"{p}<material>\n"
            f"{p}  <script>\n"
            f"{p}    <uri>file://media/materials/scripts/contest_markers.material</uri>\n"
            f"{p}    <name>{name}</name>\n"
            f"{p}  </script>\n"
            f"{p}</material>\n")


def ground_plane(room):
    sx, sy = room["size_x"], room["size_y"]
    # 地面 plane 的几何中心要落在房间中心（原点在西南角，所以是 sx/2, sy/2）
    return f"""
    <model name='ground_plane_map'>
      <static>1</static>
      <pose>{sx/2} {sy/2} 0 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>{sx} {sy}</size></plane></geometry>
          <surface>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <ode>
                <kp>1000000.0</kp><kd>100.0</kd><max_vel>1.0</max_vel><min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction><ode><mu>100</mu><mu2>50</mu2></ode><torsional><ode/></torsional></friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{sx} {sy}</size></plane></geometry>
{script_mat(GROUND_MAT, 10)}        </visual>
        <self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic>
      </link>
    </model>
"""


def box_model(name, pose, size, mat_xml, static=True, collision=True):
    col = ""
    if collision:
        col = (f"        <collision name='collision'>\n"
               f"          <geometry><box><size>{size}</size></box></geometry>\n"
               f"        </collision>\n")
    return (f"\n    <model name='{name}'>\n"
            f"      <static>{1 if static else 0}</static>\n"
            f"      <pose>{pose}</pose>\n"
            f"      <link name='link'>\n"
            f"{col}"
            f"        <visual name='visual'>\n"
            f"          <geometry><box><size>{size}</size></box></geometry>\n"
            f"{mat_xml}"
            f"        </visual>\n"
            f"      </link>\n"
            f"    </model>\n")


def walls_and_ceiling(room):
    sx, sy, h = room["size_x"], room["size_y"], room["height"]
    t = room["wall_thickness"]
    cx, cy = sx / 2, sy / 2
    out = []
    # 墙体中心线压在房间边界上，跟 fire_drill_room 的做法一致（内净空 sx-t）
    out.append(box_model("room_wall_south", f"{cx} 0 {h/2} 0 0 0",
                         f"{sx+t} {t} {h}", script_mat(WALL_MAT, 10)))
    out.append(box_model("room_wall_north", f"{cx} {sy} {h/2} 0 0 0",
                         f"{sx+t} {t} {h}", script_mat(WALL_MAT, 10)))
    out.append(box_model("room_wall_west", f"0 {cy} {h/2} 0 0 0",
                         f"{t} {sy+t} {h}", script_mat(WALL_MAT, 10)))
    out.append(box_model("room_wall_east", f"{sx} {cy} {h/2} 0 0 0",
                         f"{t} {sy+t} {h}", script_mat(WALL_MAT, 10)))
    # 天花板照抄 fire_drill_room：**只有 collision，没有 visual**。
    # 这是有意的——它是一块看不见的盖子，物理上把飞机关在屋里，视觉上不挡
    # 从上往下看房间内部（GUI 里俯视、以及从上方架相机时都要能看进去）。
    # 给它加 visual 就会变成一块盖住全场的实心板，整个场景没法看。
    # surface 那一大段也照抄，不是可省的——原版就带着。
    out.append(f"""
    <model name='ceiling'>
      <static>1</static>
      <pose>{cx} {cy} {h} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{sx+t} {sy+t} 0.1</size></box></geometry>
          <surface>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <ode>
                <kp>1000000.0</kp><kd>100.0</kd><max_vel>1.0</max_vel><min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction><ode><mu>100</mu><mu2>50</mu2></ode><torsional><ode/></torsional></friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <self_collide>0</self_collide><enable_wind>0</enable_wind><kinematic>0</kinematic>
      </link>
    </model>
""")
    return "".join(out)


def pillar_model(name, x, y, size, height, rings, divider=None):
    """一根立柱：带 collision 的柱身 + 若干**纯视觉**的楼层线环。

    楼层线没有 <collision> 是有意的：Gazebo 的射线传感器打的是 collision
    几何，加了 collision 这圈线就会被雷达扫成障碍点，凭空给规划器制造
    一圈"柱子比实际粗 6 厘米"的假目标。纯视觉则完全不进点云。
    """
    body = (f"        <collision name='collision'>\n"
            f"          <geometry><box><size>{size} {size} {height}</size></box></geometry>\n"
            f"        </collision>\n"
            f"        <visual name='visual'>\n"
            f"          <geometry><box><size>{size} {size} {height}</size></box></geometry>\n"
            f"{rgba_mat(PILLAR_RGBA, 10)}"
            f"        </visual>\n")
    # 单元分隔线：压在本柱 +x 面（两柱贴合的缝）上的一条竖线，通顶。
    # 只给"带孪生柱的那一根"画，孪生柱自己不画，否则一栋楼会出现两条。
    div_xml = ""
    if divider and divider.get("enabled"):
        dw = float(divider["width_m"])
        dp = float(divider["protrude_m"])
        div_xml = (f"        <visual name='unit_divider'>\n"
                   f"          <pose>{size/2:.3f} 0 0 0 0 0</pose>\n"
                   f"          <geometry><box><size>{dw} {size + 2*dp:.3f} {height}</size></box></geometry>\n"
                   f"{rgba_mat(divider['color'], 10)}"
                   f"        </visual>\n")
    ring_xml = ""
    if rings["enabled"]:
        w = size + 2 * rings["protrude_m"]
        th = rings["thickness_m"]
        z = rings["spacing_m"]
        i = 1
        while z < height - 1e-6:                    # 柱顶那一圈不画（贴着天花板）
            ring_xml += (f"        <visual name='floor_ring_{i}'>\n"
                         f"          <pose>0 0 {z - height/2:.3f} 0 0 0</pose>\n"
                         f"          <geometry><box><size>{w:.3f} {w:.3f} {th}</size></box></geometry>\n"
                         f"{rgba_mat(rings['color'], 10)}"
                         f"        </visual>\n")
            z += rings["spacing_m"]
            i += 1
    return (f"\n    <model name='{name}'>\n"
            f"      <static>1</static>\n"
            f"      <pose>{x} {y} {height/2} 0 0 0</pose>\n"
            f"      <link name='link'>\n"
            f"{body}{div_xml}{ring_xml}"
            f"      </link>\n"
            f"    </model>\n")


def pillars(layout):
    size = layout["pillar_size"]
    h = layout["pillar_height"]
    rings = layout["floor_rings"]
    out = []
    div = layout.get("unit_divider")
    for p in layout["pillars"]:
        # 分隔线只画在本柱上（缝在它的 +x 面），孪生柱不画，一栋楼一条
        out.append(pillar_model(p["id"], p["x"], p["y"], size, h, rings,
                                divider=div if p.get("twin_east") else None))
        if p.get("twin_east"):
            # 正东紧挨着一根同样的：中心 +size，两柱侧面正好贴合
            out.append(pillar_model(p["id"] + "_e", p["x"] + size, p["y"], size, h, rings))
    return "".join(out)


def cylinder_model(layout):
    c = layout["obstacle_cylinder"]
    return (f"\n    <model name='obstacle_cylinder'>\n"
            f"      <static>1</static>\n"
            f"      <pose>{c['x']} {c['y']} {c['height']/2} 0 0 0</pose>\n"
            f"      <link name='link'>\n"
            f"        <collision name='collision'>\n"
            f"          <geometry><cylinder><radius>{c['diameter']/2}</radius>"
            f"<length>{c['height']}</length></cylinder></geometry>\n"
            f"        </collision>\n"
            f"        <visual name='visual'>\n"
            f"          <geometry><cylinder><radius>{c['diameter']/2}</radius>"
            f"<length>{c['height']}</length></cylinder></geometry>\n"
            f"{script_mat(CYLINDER_MAT, 10)}"
            f"        </visual>\n"
            f"      </link>\n"
            f"    </model>\n")


def floor_marker(name, x, y, tag_material):
    """地面圆盘 + 上面一张 AprilTag 贴片（物资点/地面火情点共用这个形状）。"""
    return (f"\n    <model name='{name}'>\n"
            f"      <static>1</static>\n"
            f"      <pose>{x} {y} 0 0 0 0</pose>\n"
            f"      <link name='link'>\n"
            f"        <visual name='visual_base'>\n"
            f"          <cast_shadows>0</cast_shadows>\n"
            f"          <pose>0 0 0.01 0 0 0</pose>\n"
            f"          <geometry><cylinder><radius>0.4</radius><length>0.02</length></cylinder></geometry>\n"
            f"{script_mat('Gazebo/Grey', 10)}"
            f"        </visual>\n"
            f"        <visual name='visual_apriltag'>\n"
            f"          <cast_shadows>0</cast_shadows>\n"
            f"          <pose>0 0 0.025 0 0 0</pose>\n"
            f"          <geometry><box><size>0.5 0.5 0.01</size></box></geometry>\n"
            f"{marker_mat(tag_material, 10)}"
            f"        </visual>\n"
            f"      </link>\n"
            f"    </model>\n")


def helipads(layout):
    out = []
    for pad in layout["takeoff_landing_pads"]:
        # yaw 1.5707963 跟 fire_drill_room 一致：H 贴图本身上下左右对称，
        # 这个 yaw 只是沿用原写法，不影响观感。
        out.append(f"\n    <model name='helipad_{pad['id']}'>\n"
                   f"      <static>1</static>\n"
                   f"      <pose>{pad['x']} {pad['y']} 0.005 0 0 1.5707963</pose>\n"
                   f"      <link name='link'>\n"
                   f"        <visual name='visual'>\n"
                   f"          <cast_shadows>0</cast_shadows>\n"
                   f"          <geometry><box><size>0.5 0.5 0.01</size></box></geometry>\n"
                   f"{marker_mat('ContestMarkers/HelipadMarker', 10)}"
                   f"        </visual>\n"
                   f"      </link>\n"
                   f"    </model>\n")
    return "".join(out)


def fire_apriltag(layout):
    """高层着火点标识：贴在指定立柱的指定面上，法线朝外。

    face='-y' 时贴南面：薄片 <box>0.5 0.01 0.5</box> 的薄轴本来就是 y，
    所以 yaw=0 即可；贴东西面才需要转 90 度（见 fire_drill_room 那份）。
    """
    m = layout["fire_apriltag_marker"]
    pid = m["pillar_id"]
    p = next(q for q in layout["pillars"] if q["id"] == pid)
    d = m["mount_standoff_m"]
    face = m["face"]
    # 这张表必须跟 scenario_reset_node.FIRE_TAG_FACE_LOCAL_OFFSETS 一致：
    # 容器起来 5 秒后那个节点会把 tag 重新摆一次，两边不一致的话静态 world
    # 和重置后的位姿对不上。**yaw 不是可有可无的**——薄片本身左右对称，转
    # 180° 看着一样，但贴图会镜像，AprilTag 镜像了就识别不出来。
    # 约定：'+Y' 面对应 yaw=0（贴图正面朝 +Y），所以 '-Y' 面要转 π。
    dx, dy, yaw = {"-y": (0.0, -d, math.pi), "+y": (0.0, d, 0.0),
                   "-x": (-d, 0.0, math.pi / 2), "+x": (d, 0.0, -math.pi / 2)}[face]
    z = layout["fire_apriltag_height_m"]
    return (f"\n    <model name='fire_apriltag_marker'>\n"
            f"      <static>1</static>\n"
            f"      <pose>{p['x']+dx:.2f} {p['y']+dy:.2f} {z} 0 0 {yaw:.7f}</pose>\n"
            f"      <link name='link'>\n"
            f"        <visual name='visual'>\n"
            f"          <cast_shadows>0</cast_shadows>\n"
            f"          <geometry><box><size>0.5 0.01 0.5</size></box></geometry>\n"
            f"{marker_mat('ContestMarkers/AprilTagFirePillar', 10)}"
            f"        </visual>\n"
            f"      </link>\n"
            f"    </model>\n")


def terrain(layout):
    """梯形坡道：Gazebo classic 的 box 做不出梯形，用平顶 + 两块斜板拼。

    斜板绕 X 轴转 ±26.57°（= atan(0.5/1.0)，1 米水平对 0.5 米垂直），
    多余部分埋进地里。数值跟 fire_drill_room 那份一致，只是位置换了。
    """
    t = layout["terrain_module"]
    sx, sy, sz, top = t["size_x"], t["size_y"], t["size_z"], t["top_width_y"]
    slope_run = (sy - top) / 2                       # 单侧水平投影长度
    ang = math.atan2(sz, slope_run)                  # 坡角
    slab = math.hypot(slope_run, sz)                 # 斜板沿 y 的长度
    off = top / 2 + slope_run / 2                    # 斜板中心的 y 偏移
    zc = sz / 2 - slab * math.sin(ang) / 2           # 让斜面上沿接平顶、下沿入地

    def part(nm, ypos, rot, mat_indent=10):
        return (f"        <collision name='{nm}_col'>\n"
                f"          <pose>0 {ypos:.3f} {zc:.3f} {rot:.6f} 0 0</pose>\n"
                f"          <geometry><box><size>{sx} {slab:.3f} {sz*1.2:.3f}</size></box></geometry>\n"
                f"        </collision>\n"
                f"        <visual name='{nm}_vis'>\n"
                f"          <pose>0 {ypos:.3f} {zc:.3f} {rot:.6f} 0 0</pose>\n"
                f"          <geometry><box><size>{sx} {slab:.3f} {sz*1.2:.3f}</size></box></geometry>\n"
                f"{script_mat(TERRAIN_MAT, mat_indent)}"
                f"        </visual>\n")

    return (f"\n    <model name='terrain_module'>\n"
            f"      <static>1</static>\n"
            f"      <pose>{t['x']} {t['y']} {t['center_z']} 0 0 0</pose>\n"
            f"      <link name='link'>\n"
            f"        <collision name='top_col'>\n"
            f"          <pose>0 0 {sz/2} 0 0 0</pose>\n"
            f"          <geometry><box><size>{sx} {top} {sz}</size></box></geometry>\n"
            f"        </collision>\n"
            f"        <visual name='top_vis'>\n"
            f"          <pose>0 0 {sz/2} 0 0 0</pose>\n"
            f"          <geometry><box><size>{sx} {top} {sz}</size></box></geometry>\n"
            f"{script_mat(TERRAIN_MAT, 10)}"
            f"        </visual>\n"
            f"{part('ramp_north', off, -ang)}"
            f"{part('ramp_south', -off, ang)}"
            f"      </link>\n"
            f"    </model>\n")


def footer(room):
    # 相机放在西南角外侧斜看全场，取景思路照抄 fire_drill_room
    return (f"\n    <gui fullscreen='0'>\n"
            f"      <camera name='user_camera'>\n"
            f"        <pose>{room['size_x']/2} -8 16 0 0.75 1.5707963</pose>\n"
            f"        <view_controller>orbit</view_controller>\n"
            f"        <projection_type>perspective</projection_type>\n"
            f"      </camera>\n"
            f"    </gui>\n\n  </world>\n</sdf>\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--layout", required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    with open(args.layout, encoding="utf-8") as f:
        L = yaml.safe_load(f)
    room = L["room"]

    parts = [HEADER, ground_plane(room), walls_and_ceiling(room),
             cylinder_model(L), pillars(L),
             floor_marker("supply_point_marker", L["supply_point"]["x"],
                          L["supply_point"]["y"], "ContestMarkers/AprilTagSupply"),
             floor_marker("fire_point_marker", L["ground_fire_point"]["x"],
                          L["ground_fire_point"]["y"], "ContestMarkers/AprilTagGroundFire"),
             fire_apriltag(L), terrain(L), helipads(L), footer(room)]
    with open(args.output, "w", encoding="utf-8") as f:
        f.write("".join(parts))

    n_pillars = sum(2 if p.get("twin_east") else 1 for p in L["pillars"])
    print(f"[gen_sample_room_world] wrote {args.output}")
    print(f"  房间 {room['size_x']}x{room['size_y']}x{room['height']} m，原点在西南角")
    print(f"  立柱 {n_pillars} 根（{len(L['pillars'])} 根 + 各自正东孪生柱），"
          f"每根 {int((L['pillar_height'] - 1e-6) // L['floor_rings']['spacing_m'])} 圈楼层线")


if __name__ == "__main__":
    main()
