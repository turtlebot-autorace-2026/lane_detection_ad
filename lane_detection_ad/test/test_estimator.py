'''
Offline checks of lane_estimator (no ROS, no YOLO).

Synthetic "perfect" lane masks are drawn in the senior's BEV from a known world centre line,
clipped to the area the real camera actually sees in the BEV. The closed-loop test drives a
kinematic robot with a command delay over straight / U-bend / zigzag and reports the lateral
error. This checks the geometry, signs and controller stability; it says nothing about YOLO
mask quality or Gazebo/real dynamics.
'''
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lane_detection_ad.lane_estimator import (BEV_H, BEV_M, BEV_W, WHITE, YELLOW,  # noqa: E402
                                              LaneConfig, LaneEstimator)

LANE = 0.286          # sim line-centre spacing (260 inner + 26 line)
LINE_W = 0.026
VISIBLE = cv2.warpPerspective(np.full((480, 640), 255, np.uint8), BEV_M, (BEV_W, BEV_H)) > 0


# ---------------------------------------------------------------- course + rendering

def course(segments, step=0.005):
    '''segments: list of (length, curvature) or ('kink', deg). Returns Nx3 (x, y, heading).'''
    x = y = th = 0.0
    pts = [(x, y, th)]
    for seg in segments:
        if seg[0] == 'kink':
            th += math.radians(seg[1])
            continue
        length, k = seg
        for _ in range(int(round(length / step))):
            th += k * step
            x += step * math.cos(th)
            y += step * math.sin(th)
            pts.append((x, y, th))
    return np.array(pts)


def boundaries(c, off):
    n = np.c_[-np.sin(c[:, 2]), np.cos(c[:, 2])]     # left normal
    return c[:, :2] + off * n


def render(cfg, c, pose, which=(YELLOW, WHITE)):
    x, y, yaw = pose
    cs, sn = math.cos(yaw), math.sin(yaw)
    dets = []
    for cls, side in ((YELLOW, +1), (WHITE, -1)):
        if cls not in which:
            continue
        b = boundaries(c, side * LANE / 2)
        dx, dy = b[:, 0] - x, b[:, 1] - y
        d, l = cs * dx + sn * dy, -sn * dx + cs * dy
        ok = (d > 0.05) & (d < 0.8) & (np.abs(l) < 0.5)
        if ok.sum() < 2:
            continue
        r, col = cfg.d_to_row(d[ok]), cfg.y_to_x(l[ok])
        mask = np.zeros((BEV_H, BEV_W), np.uint8)
        thick = max(1, int(round(LINE_W / cfg.m_per_px)))
        cv2.polylines(mask, [np.int32(np.c_[col, r])], False, 255, thick)
        mask = (mask > 0) & VISIBLE
        if mask.any():
            dets.append((cls, mask, 0.9))
    return dets


def lateral_error(c, pose):
    dx, dy = c[:, 0] - pose[0], c[:, 1] - pose[1]
    i = int(np.argmin(dx * dx + dy * dy))
    th = c[i, 2]
    return -math.sin(th) * (pose[0] - c[i, 0]) + math.cos(th) * (pose[1] - c[i, 1]), i


# ---------------------------------------------------------------- unit checks

def straight():
    return course([(2.0, 0.0)])


def test_straight_centered():
    cfg, c = LaneConfig(), straight()
    est = LaneEstimator(cfg)
    for k in range(5):
        v, w = est.update(render(cfg, c, (0.0, 0.0, 0.0)), (0.0, 0.0, 0.0), 0.07 * k)
    assert est.debug['mode'] == 'both'
    assert abs(est.debug['e_y']) < 0.005
    assert abs(w) < 0.05 and v > 0


def test_offset_sign():
    cfg, c = LaneConfig(), straight()
    est = LaneEstimator(cfg)
    pose = (0.0, -0.02, 0.0)            # robot 2 cm right of the centre -> path is to the left
    for k in range(5):
        v, w = est.update(render(cfg, c, pose), pose, 0.07 * k)
    assert est.debug['e_y'] > 0.015 and w > 0


def test_single_line_centre():
    cfg, c = LaneConfig(), straight()
    for which in ((YELLOW,), (WHITE,)):
        est = LaneEstimator(cfg)
        est.update(render(cfg, c, (0, 0, 0)), (0, 0, 0), 0.0)        # learn width with both lines
        for k in range(1, 4):
            est.update(render(cfg, c, (0, 0, 0), which), (0, 0, 0), 0.07 * k)
        assert est.debug['mode'].endswith('_only')
        assert abs(est.debug['e_y']) < 0.01, (which, est.debug['e_y'])


def fork_dets(cfg, c, pose, widen):
    '''Yellow where it belongs, white `widen` m further right: a pair that is still inside the
    0.6-1.4 lane-width check, like the first part of a fork.'''
    yellow = render(cfg, c, pose, (YELLOW,))
    white = render(cfg, c, (pose[0], pose[1] + widen, pose[2]), (WHITE,))
    return yellow + white


def test_follow_line_at_fork():
    # true lane width, no learning, so the expected centres are exact
    cfg = LaneConfig(learn_lane_width=False, lane_width_px=320.0 * LANE / 0.258)
    c, pose = straight(), (0.0, 0.0, 0.0)

    def path_y(follow):
        est = LaneEstimator(cfg)
        for k in range(4):
            est.update(fork_dets(cfg, c, pose, 0.07), pose, 0.07 * k, follow_line=follow)
        d, y, _ = est.path
        ahead = d >= 0.30                          # the shifted white line is visible from d ~0.29
        return est.debug['mode'], float(np.mean(y[ahead]))

    mode, y = path_y('both')                        # normal: centre pulled 3.5 cm toward the fork
    assert mode == 'both' and -0.045 < y < -0.025, y
    mode, y = path_y('yellow')                      # forced yellow: stays on the yellow-side centre
    assert mode == 'yellow_only' and abs(y) < 0.008, y
    mode, y = path_y('white')                       # forced white: follows the diverging white line
    assert mode == 'white_only' and -0.08 < y < -0.06, y


def test_lost_memory_then_search():
    cfg, c = LaneConfig(), straight()
    est = LaneEstimator(cfg)
    t, pose = 0.0, [0.0, 0.0, 0.0]
    for _ in range(5):
        est.update(render(cfg, c, tuple(pose)), tuple(pose), t)
        t += 0.07
    v, w = est.update([], tuple(pose), t)
    assert est.debug['mode'] == 'memory' and v > 0
    pose[0] += cfg.hold_distance + 0.05                 # drove too far on memory
    t += 0.07
    est.update([], tuple(pose), t)
    assert est.debug['mode'] == 'lost'
    t += cfg.search_after + 0.1
    v, w = est.update([], tuple(pose), t)
    assert est.debug['mode'] == 'search' and v == 0.0 and w != 0.0


# ---------------------------------------------------------------- closed loop

def run_course(c, cfg=None, latency=0.15, dt=1.0 / 15, seed=0, start=(0.0, 0.0, 0.0)):
    cfg = cfg or LaneConfig()
    est = LaneEstimator(cfg)
    rng = np.random.default_rng(seed)
    pose = list(start)
    queue = [(0.0, 0.0)] * max(1, int(round(latency / dt)))   # command delay
    t, errs, modes = 0.0, [], []
    for _ in range(4000):
        v, w = est.update(render(cfg, c, tuple(pose)), tuple(pose), t)
        queue.append((v, w))
        va, wa = queue.pop(0)
        va *= 1.0 + 0.03 * rng.standard_normal()
        wa *= 1.0 + 0.05 * rng.standard_normal()
        for _ in range(4):                                         # integrate in sub-steps
            h = dt / 4
            pose[0] += va * h * math.cos(pose[2])
            pose[1] += va * h * math.sin(pose[2])
            pose[2] += wa * h
        t += dt
        e, i = lateral_error(c, pose)
        errs.append((i, e))
        modes.append(est.debug['mode'])
        if i >= len(c) - 60:
            break
    return np.array(errs), modes, t


def report(name, c, errs, sections):
    idx, e = errs[:, 0].astype(int), errs[:, 1]
    out = []
    for label, lo, hi in sections:
        sel = (idx >= lo) & (idx < hi)
        if sel.any():
            out.append('%s max|e|=%.1fmm' % (label, 1000 * np.abs(e[sel]).max()))
    print('[%s] ' % name + ', '.join(out))
    return np.abs(e).max()


def map_course():
    '''Shapes measured from autorace_gazebo course_surface.png: tightest bends have a centre-line
    radius of ~0.28 m; the zigzag is 27 deg legs of 0.33 / 0.65 / 0.33 m (kinks 27, 54, 54, 27 deg).'''
    R = 0.28
    bends = [(0.6, 0.0), (math.pi * R, 1 / R), (0.3, 0.0), (math.pi / 2 * R, -1 / R),
             (math.pi / 2 * R, 1 / R), (0.3, 0.0)]
    zig = [('kink', 27), (0.33, 0.0), ('kink', -54), (0.65, 0.0), ('kink', 54), (0.33, 0.0),
           ('kink', -27), (0.5, 0.0)]
    return course(bends + zig), len(course(bends)) - 1


def test_closed_loop_course():
    c, n_bend = map_course()
    worst = 0.0
    for seed in (0, 1, 2):
        errs, modes, t = run_course(c, seed=seed)
        assert errs[-1, 0] >= len(c) - 60, 'did not finish (index %d, modes %s)' % (errs[-1, 0], modes[-5:])
        worst = max(worst, report('seed %d' % seed, c, errs, [('straight', 0, 120), ('bends', 120, n_bend),
                                                               ('zigzag', n_bend, len(c))]))
        print('  lap %.1fs, modes %s' % (t, {m: modes.count(m) for m in sorted(set(modes))}))
    # offline with perfect masks: bends ~17 mm, zigzag ~30 mm; 40 mm = wheel touches the inner line edge
    assert worst < 0.04
