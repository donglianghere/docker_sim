# -*- coding: utf-8 -*-
"""把 track_land 的超时判据提到 elif 链最前面。"""
import io, os, shutil, sys

OLD = u"""            if self._track_xy is not None and agl <= handoff + tol_m and centered:
                why = (f'已降到离地 {agl:.2f} m'
                       + (f'、对准 {px:.0f} px' if px is not None else ''))
            elif (self._track_xy is not None and agl <= handoff + tol_m
                  and not centered):
                # 高度到了但没对准：**不交棒**，停在这个高度接着修水平。
                # 这一拍下面照常把 target_z 压在 handoff 上，所以飞机不会继续下降。
                # 兜底是 track_land_timeout_s（下面那条），不会无限等。
                if (self._px_hold_log_t is None
                        or now_m - self._px_hold_log_t >= 1.0):
                    self._px_hold_log_t = now_m
                    self.get_logger().info(
                        f'track_land：已到交接高度 {agl:.2f} m，但目标离主点还有'
                        f' {px:.0f} px（要求 {px_tol:.0f}），先停在这个高度对准')
            elif tmo > 0.0 and elapsed > tmo:
                why = (f'用满 {tmo:.0f} s'
                       + (f'（高度 {agl:.2f} m、离主点 {px:.0f} px）'
                          if px is not None and agl < 1e8 else ''))
            elif lost_for > grace:"""

NEW = u"""            # \U0001f534 2026-10-18：超时判据必须排在**最前面**。原来它排在“高度到了但
            #    没对准”之后，而那条分支只打印日志、不设 why，于是飞机一悬在交接
            #    高度就反复命中它，`elapsed > tmo` 永远执行不到——实测 30 s 的时限
            #    跑成 70 s（68 条“先停在这个高度对准”，飞机期间一路沉到 agl 0.03 m，
            #    直到 tag 出视场、_track_xy 变空才掉进这条补报“用满 30 s”）。
            #    SDK 侧等 LAND_ON_TIMEOUT_S+40=70 s，比节点早 6 秒放弃，所以
            #    servo_status 全程 None、每次都走“没等到机载降落确认”兜底。
            if tmo > 0.0 and elapsed > tmo:
                why = (f'用满 {tmo:.0f} s'
                       + (f'（高度 {agl:.2f} m、离主点 {px:.0f} px）'
                          if px is not None and agl < 1e8 else ''))
            elif self._track_xy is not None and agl <= handoff + tol_m and centered:
                why = (f'已降到离地 {agl:.2f} m'
                       + (f'、对准 {px:.0f} px' if px is not None else ''))
            elif (self._track_xy is not None and agl <= handoff + tol_m
                  and not centered):
                # 高度到了但没对准：**不交棒**，停在这个高度接着修水平。
                # 这一拍下面照常把 target_z 压在 handoff 上，所以飞机不会继续下降。
                # 兜底是**上面那条** track_land_timeout_s，不会无限等。
                if (self._px_hold_log_t is None
                        or now_m - self._px_hold_log_t >= 1.0):
                    self._px_hold_log_t = now_m
                    self.get_logger().info(
                        f'track_land：已到交接高度 {agl:.2f} m，但目标离主点还有'
                        f' {px:.0f} px（要求 {px_tol:.0f}），先停在这个高度对准')
            elif lost_for > grace:"""

paths = sys.argv[1:]
done = 0
for p in paths:
    if not os.path.exists(p):
        print('SKIP (not found) %s' % p); continue
    s = io.open(p, encoding='utf-8').read()
    if NEW.split(u'\n')[0].strip() in s:
        print('ALREADY patched %s' % p); done += 1; continue
    n = s.count(OLD)
    if n != 1:
        print('FAIL %s : expected 1 occurrence, got %d' % (p, n)); continue
    shutil.copy(p, p + '.bak-20261018-timeout')
    io.open(p, 'w', encoding='utf-8').write(s.replace(OLD, NEW, 1))
    print('PATCHED %s' % p); done += 1
print('total patched/ok = %d' % done)
