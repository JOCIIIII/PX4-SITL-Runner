#!/usr/bin/env python3
"""Person detector: YOLOX (Megvii, Apache-2.0) ONNX models from its 0.1.1rc0 release (yolox_tiny.onnx 416 px,
yolox_s.onnx 640 px; COCO class 0 = person) on onnxruntime, plus the ground point of a detection from the camera
pose AirSim returns with the image.

    det = PersonDetector('/home/user/workspace/ros2/models/yolox_s.onnx')
    boxes = det.detect(bgr)                 # [(x1, y1, x2, y2, score), ...] in image pixels
    boxes = det.detect_multi(bgr)           # + the image turned 90 / 270 deg: (x1, y1, x2, y2, score, rot) -
                                            # people lying on the ground, seen from above, are found turned upright
    xyz = ground_point(u, v, w, h, fov_deg, cam_pos_ned, cam_q_wxyz)   # NED [m] where the pixel ray meets z = 0

    python3 person_detector.py MODEL IMAGE...   (prints detections, writes IMAGE.det.jpg with boxes)
"""
import math
import sys

import cv2
import numpy as np
import onnxruntime as ort


class PersonDetector:
    def __init__(self, model, score_thr=0.35, nms_thr=0.45, providers=None):
        avail = ort.get_available_providers()
        want = providers or [p for p in ('CUDAExecutionProvider', 'CPUExecutionProvider') if p in avail]
        self.sess = ort.InferenceSession(model, providers=want)
        inp = self.sess.get_inputs()[0]
        self.name = inp.name
        self.h, self.w = int(inp.shape[2]), int(inp.shape[3])
        self.score_thr, self.nms_thr = score_thr, nms_thr
        grids, strides = [], []
        for s in (8, 16, 32):
            hs, ws = self.h // s, self.w // s
            xv, yv = np.meshgrid(np.arange(ws), np.arange(hs))
            grids.append(np.stack((xv, yv), 2).reshape(-1, 2))
            strides.append(np.full((hs * ws, 1), s))
        self.grids = np.concatenate(grids).astype(np.float32)
        self.strides = np.concatenate(strides).astype(np.float32)

    def detect(self, bgr):
        r = min(self.h / bgr.shape[0], self.w / bgr.shape[1])
        img = cv2.resize(bgr, (int(bgr.shape[1] * r), int(bgr.shape[0] * r)), interpolation=cv2.INTER_LINEAR)
        pad = np.full((self.h, self.w, 3), 114, np.uint8)
        pad[:img.shape[0], :img.shape[1]] = img
        x = pad.transpose(2, 0, 1)[None].astype(np.float32)      # BGR 0..255, no normalisation (YOLOX >= 0.1.1)
        p = self.sess.run(None, {self.name: x})[0][0]
        xy = (p[:, :2] + self.grids) * self.strides
        wh = np.exp(p[:, 2:4]) * self.strides
        score = p[:, 4] * p[:, 5]                                 # objectness x P(person)
        keep = score > self.score_thr
        if not keep.any():
            return []
        xy, wh, score = xy[keep] / r, wh[keep] / r, score[keep]
        boxes = np.concatenate([xy - wh / 2, wh], 1)
        idx = cv2.dnn.NMSBoxes(boxes.tolist(), score.tolist(), self.score_thr, self.nms_thr)
        out = []
        for i in np.array(idx).reshape(-1):
            x1, y1, bw, bh = boxes[i]
            out.append((float(x1), float(y1), float(x1 + bw), float(y1 + bh), float(score[i])))
        return sorted(out, key=lambda b: -b[4])

    def detect_multi(self, bgr, rotations=(0, 90, 270)):
        """detect() on the image and on it turned; boxes in the original image, merged by NMS; rot = the turn that
        found it (0: upright, else most likely a person lying down)"""
        h, w = bgr.shape[:2]
        allb = []
        for rot in rotations:
            if rot == 0:
                allb += [b + (0,) for b in self.detect(bgr)]
            elif rot == 90:          # cv2.ROTATE_90_CLOCKWISE: rotated (u', v') <- original (v', h - 1 - u')
                for x1, y1, x2, y2, s in self.detect(cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE)):
                    allb.append((y1, h - 1 - x2, y2, h - 1 - x1, s, 90))
            elif rot == 270:         # cv2.ROTATE_90_COUNTERCLOCKWISE: rotated (u', v') <- original (w - 1 - v', u')
                for x1, y1, x2, y2, s in self.detect(cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)):
                    allb.append((w - 1 - y2, x1, w - 1 - y1, x2, s, 270))
        if not allb:
            return []
        xywh = [[b[0], b[1], b[2] - b[0], b[3] - b[1]] for b in allb]
        idx = cv2.dnn.NMSBoxes(xywh, [b[4] for b in allb], self.score_thr, self.nms_thr)
        return sorted((allb[i] for i in np.array(idx).reshape(-1)), key=lambda b: -b[4])


def quat_rot(q_wxyz):
    w, x, y, z = q_wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def ground_point(u, v, width, height, fov_deg, cam_pos, cam_q, ground_z=0.0):
    """NED point where the ray through pixel (u, v) meets the plane z = ground_z; None when it points above the
    horizon. AirSim camera frame: x forward (optical axis), y right, z down; fov_deg is horizontal."""
    f = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    d_cam = np.array([1.0, (u - width / 2.0) / f, (v - height / 2.0) / f])
    d = quat_rot(cam_q) @ d_cam
    if d[2] <= 1e-6:
        return None
    t = (ground_z - cam_pos[2]) / d[2]
    return np.asarray(cam_pos, dtype=float) + t * d


if __name__ == '__main__':
    det = PersonDetector(sys.argv[1])
    for path in sys.argv[2:]:
        img = cv2.imread(path)
        t0 = cv2.getTickCount()
        boxes = det.detect(img)
        ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1e3
        print(f'{path}: {len(boxes)} person(s) in {ms:.0f} ms', [tuple(round(v, 2) for v in b) for b in boxes])
        for x1, y1, x2, y2, s in boxes:
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(img, f'{s:.2f}', (int(x1), int(y1) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imwrite(path + '.det.jpg', img)
