# -*- coding: utf-8 -*-
"""让 MJPEG 预览也画 AprilTag 框（原来只画 YOLO 的 dets）。"""
import io, os, shutil, sys

MARK = u'at_boxes'

def patch(p):
    if not os.path.exists(p):
        return 'SKIP (not found) %s' % p
    s = io.open(p, encoding='utf-8').read()
    if MARK in s:
        return 'ALREADY patched %s' % p
    lines = s.split(u'\n')
    out = []
    i = 0
    n_at_init = n_append = n_call = n_sig = n_draw = 0
    while i < len(lines):
        ln = lines[i]
        st = ln.strip()
        # 1) apriltag 块之前初始化收集列表
        if (n_at_init == 0 and st == u'if self.apriltag_detector is not None:'
                and i + 1 < len(lines) and lines[i + 1].strip() == u'self._at_tick += 1'):
            out.append(u'        # 2026-10-18：AprilTag 的框也要画到 MJPEG 预览上。原来 _update_preview()')
            out.append(u'        #   只画 YOLO 的 dets，而 tag 是下面单独检测、直接填进检测消息的，')
            out.append(u'        #   于是抓灭火弹那段（画面里只有 tag、没有 YOLO 认识的类别）拉流一个框都没有。')
            out.append(u'        at_boxes = []')
            n_at_init = 1
            out.append(ln); i += 1; continue
        # 2) 每个 tag 检测后记下框
        if st == u'msg.detections.append(det)' and n_append == 0:
            out.append(ln)
            indent = ln[:len(ln) - len(ln.lstrip())]
            out.append(indent + u'at_boxes.append((x1, y1, x2, y2, int(tid)))')
            n_append = 1
            i += 1; continue
        # 3) 调用点传进去
        if st == u'self._update_preview(frame, dets)':
            out.append(ln.replace(u'self._update_preview(frame, dets)',
                                  u'self._update_preview(frame, dets, at_boxes)'))
            n_call += 1; i += 1; continue
        # 4) 函数签名
        if st == u'def _update_preview(self, frame, dets):':
            out.append(ln.replace(u'def _update_preview(self, frame, dets):',
                                  u'def _update_preview(self, frame, dets, at_boxes=()):'))
            n_sig += 1; i += 1; continue
        # 5) 画 tag 框（插在把 preview 交出去之前）
        if st == u'self.mjpeg_server.update_frame(preview)' and n_draw == 0:
            indent = ln[:len(ln) - len(ln.lstrip())]
            out.append(indent + u'# AprilTag 用橙色，YOLO 是绿色，一眼分得出哪条链在出结果。')
            out.append(indent + u'for (ax1, ay1, ax2, ay2, tid) in at_boxes:')
            out.append(indent + u'    q1 = (int(ax1), int(ay1))')
            out.append(indent + u'    q2 = (int(ax2), int(ay2))')
            out.append(indent + u'    cv2.rectangle(preview, q1, q2, (0, 165, 255), 2)')
            out.append(indent + u"    cv2.putText(preview, f'apriltag:{tid}', (q1[0], max(q1[1] - 5, 0)),")
            out.append(indent + u'                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)')
            n_draw = 1
            out.append(ln); i += 1; continue
        out.append(ln); i += 1
    counts = (n_at_init, n_append, n_call, n_sig, n_draw)
    if counts != (1, 1, 1, 1, 1):
        return 'FAIL %s : anchors(init,append,call,sig,draw)=%s' % (p, counts)
    shutil.copy(p, p + '.bak-20261018-tagbox')
    io.open(p, 'w', encoding='utf-8').write(u'\n'.join(out))
    return 'PATCHED %s' % p

for path in sys.argv[1:]:
    print(patch(path))
