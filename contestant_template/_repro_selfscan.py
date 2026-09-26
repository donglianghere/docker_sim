"""自扫描伪点诊断：起飞到指定高度、原地悬停一段时间、降落。

悬停在 2.5 米时，点云里任何"离飞机本体 1 米以内"的点都不可能是地面或墙面，
只能是雷达扫到自己机身/桨叶的伪点——这样就能干净地量出来有多少、在哪。
配套的统计脚本是 scratchpad 里的 selfscan_diag.py。
"""
import argparse
import time

from contest_sdk import DroneSDK

HOVER_AGL_M = 2.5
HOVER_S = 45.0


def main():
    ap = argparse.ArgumentParser(description='悬停自扫描诊断')
    ap.add_argument('--namespace', default='NX01')
    ap.add_argument('--role', default='leader')
    ap.add_argument('--teammate', default='NX02')
    args = ap.parse_args()

    sdk = DroneSDK(namespace=args.namespace,
                   role='recon' if args.role == 'leader' else 'supply',
                   teammate_namespace=args.teammate)
    try:
        sdk.takeoff(height_m=HOVER_AGL_M)
        print(f'[{sdk.namespace}] 悬停 {HOVER_S:.0f} 秒采数据…', flush=True)
        time.sleep(HOVER_S)
        sdk.land()
    finally:
        sdk.shutdown()


if __name__ == '__main__':
    main()
