'''
Lane following for TurtleBot3 AutoRace on top of the senior's YOLO-seg lane model
(turtlebot-autorace-2026/lane_detection_dl @ f1a7a74, BEV + model reused unchanged).

Pure numpy/OpenCV (no ROS) so it can be unit-tested offline.

Robot frame used below: d = forward distance from the wheel axle [m], y = lateral [m], +y = left.
BEV image (640x480) is the senior's metric-isotropic BEV of the undistorted /camera image.

Per frame:
  1. lane masks (yellow = left, white = right) inside the ROI
  2. row-by-row tracing of each line (nearest mask run to the previous row), no global polynomial
     -> keeps zigzag kinks and partly visible curves
  3. centre path: each visible line shifted half a lane width along its normal, averaged
     (senior: 0.6 lane width for a single line -> 26 mm centre jump on every both/single switch)
  4. path memory in the odom frame: the part of the path that left the camera view (below the
     image, sharp curves, kinks) stays usable for a short distance
  5. control, near/far split:
       near (axle, latency-compensated): lateral error e_y, heading error e_th, path curvature
         w = v * (kappa + e_y / L^2 + 2 e_th / L)   (critically damped in distance, L = control_length)
       far (preview up to ~0.55 m): curvature / kink -> slow down BEFORE the bend
       steering saturates -> slow down instead of running wide (kinks become slow pivot turns)
  6. lost lane: memory hold -> stop -> rotate-search toward the last known side (30 s rule)
'''
import math

import cv2
import numpy as np

# ---- BEV (senior's bev_params.yaml, unchanged) ----
BEV_W, BEV_H = 640, 480
BEV_SRC = np.float32([[18, 451], [633, 477], [445, 157], [216, 153]])
BEV_DST = np.float32([[166, 474], [474, 474], [474, 122], [166, 122]])
BEV_M = cv2.getPerspectiveTransform(BEV_SRC, BEV_DST)
BEV_PX_PER_CAMERA_HEIGHT = 160.0
BEV_BOTTOM_AHEAD = 0.76   # ground distance of the BEV bottom row from the camera [camera heights]

# model class ids
WHITE, YELLOW = 0, 1


# BEV pixels that come from inside the camera image (the BEV is a trapezoid of real pixels)
VISIBLE = cv2.erode(cv2.warpPerspective(np.full((480, 640), 255, np.uint8), BEV_M, (BEV_W, BEV_H)),
                    np.ones((5, 5), np.uint8)) > 0


def to_bev(undistorted):
    if undistorted.shape[:2] != (480, 640):
        undistorted = cv2.resize(undistorted, (640, 480))
    return cv2.warpPerspective(undistorted, BEV_M, (BEV_W, BEV_H))


DEFAULTS = {
    # geometry (senior's BEV calibration)
    'robot_x': 326.0,              # BEV x of the robot centre line [px]
    'camera_height': 0.129,        # m, sets the BEV metric scale
    'camera_ahead_of_axle': 0.076,  # m
    'lane_width_px': 320.0,        # initial line-centre spacing [BEV px]; learned online
    'learn_lane_width': True,
    'lane_width_alpha': 0.05,
    'line_width': 0.026,           # m, painted line width
    'robot_half_width': 0.09,      # m, outer half width (wheel edge) for the margin estimate
    # line tracing
    'roi_top_row': 220,            # BEV rows below this only
    'roi_half_width': 250,         # +-px around robot_x
    'row_step': 8,
    'min_points': 5,
    'max_jump_px': 30.0,           # allowed x change between traced rows
    'single_offset': 0.5,          # centre = line + single_offset * lane width (senior: 0.6)
    # path memory
    'memory_behind': 0.25,         # m kept behind the axle
    'hold_distance': 0.10,         # m that may be driven on memory only (line loss is expected only in mission zones)
    'hold_time': 1.0,              # s
    # control
    'control_length': 0.12,        # m, L: k_y = 1/L^2, k_th = 2/L
    'latency': 0.15,               # s, image -> wheel delay compensated by prediction
    'v_max': 0.12,
    'v_max_intersection': 0.08,
    'v_min': 0.03,
    'a_lat_max': 0.03,             # m/s^2, v <= sqrt(a_lat_max / kappa)
    'preview_max': 0.55,           # m
    'preview_decel': 0.05,         # m/s^2 assumed when slowing for a bend ahead (small = early)
    'kink_deg': 30.0,              # heading change per 4 cm treated as a kink
    'kink_pivot': False,           # experimental: stop + turn in place at kinks (offline: erratic, 24-76 mm)
    'pivot_dist': 0.01,            # m, start the in-place turn when the kink vertex is this close
    'pivot_done_deg': 8.0,         # in-place turn ends when the next segment is this well aligned
    'pivot_timeout': 4.0,          # s, safety exit
    'v_single': 0.8,               # speed factor, one line only
    'v_memory': 0.6,               # speed factor, memory only
    'slow_margin': 0.02,           # m, wheel margin below which speed is scaled down
    'max_ang': 1.2,                # rad/s
    'max_ang_acc': 6.0,            # rad/s^2
    'max_lin_acc': 0.3,            # m/s^2
    'search_after': 1.0,           # s stopped before rotate-search
    'search_ang': 0.4,             # rad/s
}


class LaneConfig:
    def __init__(self, **kw):
        for k, v in DEFAULTS.items():
            setattr(self, k, v)
        for k, v in kw.items():
            setattr(self, k, v)
        self.roi_top_row = int(self.roi_top_row)
        self.row_step = int(self.row_step)
        self.rows = np.arange(BEV_H - 10, self.roi_top_row - 1, -self.row_step)   # bottom -> top

    @property
    def m_per_px(self):
        return self.camera_height / BEV_PX_PER_CAMERA_HEIGHT

    def row_to_d(self, r):
        return (BEV_BOTTOM_AHEAD + (BEV_H - np.asarray(r, float)) / BEV_PX_PER_CAMERA_HEIGHT) \
            * self.camera_height + self.camera_ahead_of_axle

    def d_to_row(self, d):
        return BEV_H - ((np.asarray(d, float) - self.camera_ahead_of_axle) / self.camera_height
                        - BEV_BOTTOM_AHEAD) * BEV_PX_PER_CAMERA_HEIGHT

    def x_to_y(self, x):
        return -(np.asarray(x, float) - self.robot_x) * self.m_per_px

    def y_to_x(self, y):
        return self.robot_x - np.asarray(y, float) / self.m_per_px


# ----------------------------------------------------------------------------- perception

def _runs(cols, gap=3):
    if cols.size == 0:
        return []
    splits = np.flatnonzero(np.diff(cols) > gap) + 1
    return [(int(c[0]), int(c[-1])) for c in np.split(cols, splits)]


def trace_line(mask, rows, x_expect, first_tol, max_jump, valid=None, half_w=0.0):
    '''Follow one line from the bottom row up: in each row take the mask run nearest to the
    previous point. A run cut by the edge of the visible BEV area (valid == False next to it) is
    clipped, so its centre is taken from its uncut edge instead. Returns [(row, x)].'''
    pts = []
    x_prev, tol = float(x_expect), float(first_tol)
    w = mask.shape[1]
    for r in rows:
        cols = np.flatnonzero(mask[max(r - 2, 0):r + 3].any(axis=0))
        best, best_dist = None, None
        for a, b in _runs(cols):
            dist = max(0.0, a - x_prev, x_prev - b)     # distance from x_prev to the run interval
            if best_dist is None or dist < best_dist:
                best, best_dist = (a, b), dist
        if best is not None and best_dist <= tol:
            a, b = best
            cut_l = cut_r = False
            if valid is not None:
                cut_l = a - 3 < 0 or not valid[r, a - 3]
                cut_r = b + 3 >= w or not valid[r, b + 3]
            if cut_l and cut_r:
                x = None
            elif cut_l:
                x = b - half_w
            elif cut_r:
                x = a + half_w
            else:
                x = 0.5 * (a + b)
            if x is not None:
                pts.append((int(r), x))
                x_prev, tol = x, max_jump
                continue
        if pts:
            tol += max_jump           # allow gaps
    return pts


def _offset(d, y, off):
    '''Shift a curve y(d) by off [m] along its normal, + = to the right (-y).'''
    s = np.gradient(y, d) if len(d) > 1 else np.zeros_like(d)
    n = np.sqrt(1.0 + s * s)
    od, oy = d + off * s / n, y - off / n
    order = np.argsort(od)
    return od[order], oy[order]


def _monotonic(d, y):
    d, idx = np.unique(np.round(d, 4), return_index=True)
    return d, y[idx]


# ----------------------------------------------------------------------------- memory

class PathMemory:
    '''Centre-path points kept in the odom frame.'''

    def __init__(self):
        self.pts = np.zeros((0, 2))
        self.last_obs_t = None
        self.last_obs_xy = None

    @staticmethod
    def to_robot(pts, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        return c * dx + s * dy, -s * dx + c * dy

    @staticmethod
    def to_world(d, l, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.c_[x + c * d - s * l, y + s * d + c * l]

    def update(self, obs, pose, t, cfg):
        '''obs: (d, y) centre points in the robot frame or None. Returns (d, y, source) or None.'''
        md, my = self.to_robot(self.pts, pose) if len(self.pts) else (np.zeros(0), np.zeros(0))
        if obs is not None:
            od, oy = obs
            keep = (md < od[0] - 0.01) & (md > -cfg.memory_behind)
            d, y, source = np.r_[md[keep], od], np.r_[my[keep], oy], 'obs'
            self.last_obs_t, self.last_obs_xy = t, (pose[0], pose[1])
        else:
            if self.last_obs_t is None or t - self.last_obs_t > cfg.hold_time:
                return None
            if math.hypot(pose[0] - self.last_obs_xy[0], pose[1] - self.last_obs_xy[1]) > cfg.hold_distance:
                return None
            keep = md > -cfg.memory_behind
            d, y, source = md[keep], my[keep], 'memory'
            if d.size < 3 or d.max() < 0.05:
                return None
        order = np.argsort(d)
        d, y = d[order], y[order]
        self.pts = self.to_world(d, y, pose)
        return d, y, source


# ----------------------------------------------------------------------------- estimator

class LaneEstimator:
    def __init__(self, cfg=None):
        self.cfg = cfg or LaneConfig()
        self.reset()

    def set_params(self, values):
        '''Change controller parameters at run time (section profiles); perception state is kept.'''
        for k, v in values.items():
            if k in DEFAULTS:
                setattr(self.cfg, k, v)

    def reset(self):
        cfg = self.cfg
        self.lane_width = cfg.lane_width_px * cfg.m_per_px     # m, line-centre spacing
        self.lane_width0 = self.lane_width
        self.n_width = 0                                         # paired width measurements so far
        self.memory = PathMemory()
        self.v_cmd = self.w_cmd = 0.0
        self.t_prev = None
        self.lost_since = None
        self.last_side = 1.0
        self.pivoting, self.pivot_target, self.pivot_t0 = False, 0.0, 0.0
        self.travel = 0.0            # m driven (from commands), for kink bookkeeping
        self.ignore_kinks_until = -1.0
        self.line_px = {YELLOW: None, WHITE: None}   # traced [(row, x)] for drawing / next frame
        self.line_t = {YELLOW: -1e9, WHITE: -1e9}
        self.path = None
        self.debug = {'mode': 'init'}

    # ---- perception -------------------------------------------------------------------

    def roi_mask(self):
        cfg = self.cfg
        roi = np.zeros((BEV_H, BEV_W), bool)
        x0 = int(max(0, cfg.robot_x - cfg.roi_half_width))
        x1 = int(min(BEV_W, cfg.robot_x + cfg.roi_half_width))
        roi[cfg.roi_top_row:, x0:x1] = True
        return roi & VISIBLE

    def _trace(self, mask, cls, t):
        cfg = self.cfg
        half = 0.5 * self.lane_width / cfg.m_per_px
        prev = self.line_px[cls]
        if prev and t - self.line_t[cls] < 0.5:
            x_expect, first_tol = prev[0][1], 80.0
        else:
            x_expect = cfg.robot_x - half if cls == YELLOW else cfg.robot_x + half
            first_tol = float(cfg.roi_half_width)
        pts = trace_line(mask, cfg.rows, x_expect, first_tol, cfg.max_jump_px,
                         valid=self.roi_mask(), half_w=0.5 * cfg.line_width / cfg.m_per_px)
        if len(pts) < cfg.min_points:
            return None, None
        self.line_px[cls], self.line_t[cls] = pts, t
        r, x = np.array(pts, float).T
        d, y = cfg.row_to_d(r), cfg.x_to_y(x)
        if len(y) >= 5:
            y = np.convolve(np.r_[y[0], y, y[-1]], np.ones(3) / 3.0, mode='valid')
        return d, y

    def _centre(self, detections, t, lane_mode, follow_line='both'):
        cfg = self.cfg
        roi = self.roi_mask()
        masks = {YELLOW: np.zeros_like(roi), WHITE: np.zeros_like(roi)}
        for cls, mask, _ in detections:
            if cls in masks:
                masks[cls] |= mask & roi
        left = self._trace(masks[YELLOW], YELLOW, t)
        right = self._trace(masks[WHITE], WHITE, t)
        have_l, have_r = left[0] is not None, right[0] is not None
        # forced single line (mission FSM, e.g. intersection branch): ignore the other line
        if follow_line == 'yellow':
            have_r = False
        elif follow_line == 'white':
            have_l = False

        # pair check: order and width
        if have_l and have_r:
            lo, hi = max(left[0][0], right[0][0]), min(left[0][-1], right[0][-1])
            if hi - lo > 0.03:
                dd = np.linspace(lo, hi, 8)
                gap = np.interp(dd, *left) - np.interp(dd, *right)
                g = float(np.median(gap))
                if not (0.6 * self.lane_width < g < 1.4 * self.lane_width):
                    if lane_mode:
                        have_r = False
                    else:
                        have_l = False
                elif cfg.learn_lane_width and hi - lo > 0.08:
                    slope = np.median(np.r_[np.gradient(left[1], left[0]), np.gradient(right[1], right[0])])
                    width = g * math.cos(math.atan(slope))
                    self.n_width += 1
                    a = max(cfg.lane_width_alpha, 1.0 / self.n_width) if self.n_width <= 20 \
                        else cfg.lane_width_alpha               # fast start, then slow EMA
                    self.lane_width = float(np.clip((1 - a) * self.lane_width + a * width,
                                                    0.75 * self.lane_width0, 1.25 * self.lane_width0))

        off = cfg.single_offset * self.lane_width
        curves = []
        if have_l:
            curves.append(_offset(left[0], left[1], +off))
        if have_r:
            curves.append(_offset(right[0], right[1], -off))
        if not curves:
            return None, 'lost'
        mode = 'both' if len(curves) == 2 else ('yellow_only' if have_l else 'white_only')
        lo = min(c[0][0] for c in curves)
        hi = max(c[0][-1] for c in curves)
        if hi - lo < 0.04:
            return None, 'lost'
        grid = np.arange(lo, hi + 1e-9, 0.02)
        acc, cnt = np.zeros_like(grid), np.zeros_like(grid)
        for od, oy in curves:
            od, oy = _monotonic(od, oy)
            ok = (grid >= od[0]) & (grid <= od[-1])
            acc[ok] += np.interp(grid[ok], od, oy)
            cnt[ok] += 1
        ok = cnt > 0
        return (grid[ok], acc[ok] / cnt[ok]), mode

    # ---- control ----------------------------------------------------------------------

    def _control(self, d, y, t, mode, intersection, pose):
        cfg = self.cfg
        v0, w0 = self.v_cmd, self.w_cmd
        # latency compensation: express the path in the pose predicted after cfg.latency
        th = w0 * cfg.latency
        if abs(w0) > 1e-4:
            px, py = v0 / w0 * math.sin(th), v0 / w0 * (1 - math.cos(th))
        else:
            px, py = v0 * cfg.latency, 0.0
        c, s = math.cos(th), math.sin(th)
        dd, yy = d - px, y - py
        d1, y1 = _monotonic(c * dd + s * yy, -s * dd + c * yy)
        if d1.size < 3:
            return None

        # far: slow down before bends / kinks
        v_top = cfg.v_max_intersection if intersection else cfg.v_max
        v_allow, kappa_prev, kink, kink_d, kink_turn = v_top, 0.0, False, None, 0.0
        lo, hi = max(d1[0], 0.0), min(d1[-1], cfg.preview_max)
        if hi - lo >= 0.12:
            g = np.arange(lo, hi + 1e-9, 0.02)
            gy = np.interp(g, d1, y1)
            hd = np.arctan2(np.diff(gy), np.diff(g))       # segment headings
            dth = np.diff(hd)                              # turn at vertices g[1:-1]
            pair = dth[:-1] + dth[1:]                      # turn over 4 cm, vertices i+1, i+2
            pos = 0.5 * (g[1:-2] + g[2:-1])
            kap = np.abs(pair) / 0.04
            is_kink = np.abs(pair) > math.radians(cfg.kink_deg)
            if self.travel < self.ignore_kinks_until:
                is_kink &= pos > 0.08                      # the kink just turned at is behind us
            v_lim = np.where(is_kink, cfg.v_min, np.sqrt(cfg.a_lat_max / np.maximum(kap, 1e-6)))
            v_allow = min(v_allow, float(np.min(np.sqrt(v_lim ** 2 + 2 * cfg.preview_decel * np.maximum(pos, 0)))))
            kappa_prev, kink = float(kap.max()), bool(is_kink.any())
            if kink:
                i = int(np.flatnonzero(is_kink)[0])
                vtx = i if abs(dth[i]) >= abs(dth[i + 1]) else i + 1
                kink_d = float(g[vtx + 1])
                kink_turn = float(np.mean(hd[vtx + 1:vtx + 4]))   # heading after the vertex
                # refine the vertex: intersection of straight-line fits before / after it
                pre = (d1 > kink_d - 0.12) & (d1 < kink_d - 0.02)
                post = (d1 > kink_d + 0.02) & (d1 < kink_d + 0.12)
                if pre.sum() >= 3 and post.sum() >= 3:
                    b1, a1 = np.polyfit(d1[pre], y1[pre], 1)
                    b2, a2 = np.polyfit(d1[post], y1[post], 1)
                    if abs(b1 - b2) > 0.1:
                        dk = (a2 - a1) / (b1 - b2)
                        if abs(dk - kink_d) < 0.04:
                            kink_d = float(dk)
                            kink_turn = float(math.atan(b2))

        # kink right in front of the axle: stop and turn in place toward the next segment.
        # The target heading is kept in the odom frame so it survives frames without a kink detection.
        yaw_pred = pose[2] + th
        if kink_d is not None and (self.pivoting or (cfg.kink_pivot and kink_d <= cfg.pivot_dist)):
            if not self.pivoting:
                self.pivoting, self.pivot_t0 = True, t
            self.pivot_target = yaw_pred + kink_turn
        if self.pivoting:
            turn = math.atan2(math.sin(self.pivot_target - yaw_pred), math.cos(self.pivot_target - yaw_pred))
            if abs(turn) < math.radians(cfg.pivot_done_deg) or t - self.pivot_t0 > cfg.pivot_timeout:
                self.pivoting = False
                self.ignore_kinks_until = self.travel + 0.08
            else:
                w = float(np.clip(2.5 * turn, -cfg.max_ang, cfg.max_ang))
                self.debug.update(e_y=0.0, e_th=turn, kappa=0.0, kappa_prev=kappa_prev, kink=True,
                                  margin=float('nan'), kappa_cmd=0.0)
                return 0.0, w, 'pivot'

        # near: e_y, e_th, curvature at the axle (only the part before an approaching kink)
        lim = (kink_d - 0.02) if (cfg.kink_pivot and kink_d is not None) else 1e9
        near = (d1 > -0.10) & (d1 < min(0.30, lim))
        if kink_d is not None and cfg.kink_pivot:
            sel = near if near.sum() >= 3 else (np.arange(d1.size) < 3)
            b, a = np.polyfit(d1[sel], y1[sel], 1)
            kappa = 0.0
        elif (d1 < 0.05).any() and near.sum() >= 5 and np.ptp(d1[near]) > 0.12:
            c2, b, a = np.polyfit(d1[near], y1[near], 2)
            kappa = 2 * c2 / (1 + b * b) ** 1.5
        else:   # nothing near the axle yet: line fit on the nearest 12 cm, extrapolated
            sel = d1 < d1[0] + 0.12
            if sel.sum() < 3:
                sel = np.arange(d1.size) < 3
            b, a = np.polyfit(d1[sel], y1[sel], 1)
            wide = d1 < d1[0] + 0.25
            kappa = 0.0
            if wide.sum() >= 5 and np.ptp(d1[wide]) > 0.10:
                c2, b2, _ = np.polyfit(d1[wide], y1[wide], 2)
                kappa = 2 * c2 / (1 + b2 * b2) ** 1.5
        e_y, e_th = float(a), float(math.atan(b))
        kappa = float(np.clip(kappa, -10.0, 10.0))

        v = min(v_top, v_allow, math.sqrt(cfg.a_lat_max / max(abs(kappa), 1e-6)))
        if mode in ('yellow_only', 'white_only'):
            v *= cfg.v_single
        elif mode == 'memory':
            v *= cfg.v_memory
        margin = 0.5 * (self.lane_width - cfg.line_width) - cfg.robot_half_width - abs(e_y)
        if margin < cfg.slow_margin:
            v *= max(0.3, margin / cfg.slow_margin)
        v = max(v, cfg.v_min)

        L = cfg.control_length
        kappa_cmd = kappa + e_y / (L * L) + 2.0 * e_th / L
        if abs(kappa_cmd) * v > cfg.max_ang:            # steering saturated: slow down, keep the radius
            v = cfg.max_ang / abs(kappa_cmd)
        w = float(np.clip(v * kappa_cmd, -cfg.max_ang, cfg.max_ang))

        self.debug.update(e_y=e_y, e_th=e_th, kappa=kappa, kappa_prev=kappa_prev, kink=kink,
                          margin=margin, kappa_cmd=kappa_cmd)
        self.last_side = 1.0 if e_y >= 0 else -1.0
        return v, w, mode

    # ---- main -------------------------------------------------------------------------

    def update(self, detections, pose, t, lane_mode=True, intersection=False, follow_line='both'):
        '''
        detections: list of (cls, mask HxW bool in BEV coords, conf)
        pose: (x, y, yaw) in the odom frame (odometry or integrated commands)
        t: time [s]
        lane_mode: True -> keep the yellow (left) line when the pair is inconsistent
        follow_line: 'both' (normal) | 'yellow' | 'white' -> use only that line
        returns (linear_x, angular_z)
        '''
        cfg = self.cfg
        dt = 0.07 if self.t_prev is None else float(np.clip(t - self.t_prev, 0.02, 0.3))
        self.t_prev = t
        self.debug = {}

        obs, mode = self._centre(detections, t, lane_mode, follow_line)
        res = self.memory.update(obs, pose, t, cfg)
        out = None
        if res is not None:
            d, y, source = res
            if source == 'memory':
                mode = 'memory'
            self.path = (d, y, source)
            out = self._control(d, y, t, mode, intersection, pose)
        else:
            self.path = None

        if out is None:
            if self.lost_since is None:
                self.lost_since = t
            if t - self.lost_since < cfg.search_after:
                v, w, mode = 0.0, 0.0, 'lost'
            else:
                v, w, mode = 0.0, cfg.search_ang * self.last_side, 'search'
        else:
            self.lost_since = None
            v, w, mode = out
            v = min(v, self.v_cmd + cfg.max_lin_acc * dt)
            w = float(np.clip(w, self.w_cmd - cfg.max_ang_acc * dt, self.w_cmd + cfg.max_ang_acc * dt))

        self.v_cmd, self.w_cmd = float(v), float(w)
        self.travel += self.v_cmd * dt
        self.debug.update(mode=mode, v=self.v_cmd, w=self.w_cmd, lane_width=self.lane_width)
        return self.v_cmd, self.w_cmd

    # ---- debug view -------------------------------------------------------------------

    def draw(self, bev, detections=None):
        cfg = self.cfg
        img = bev.copy()
        if detections:
            ov = img.copy()
            for cls, mask, _ in detections:
                ov[mask] = (0, 255, 255) if cls == YELLOW else (255, 0, 255)
            img = cv2.addWeighted(ov, 0.4, img, 0.6, 0)
        for cls, col in ((YELLOW, (0, 200, 255)), (WHITE, (255, 255, 255))):
            pts = self.line_px[cls]
            if pts and self.t_prev is not None and self.line_t[cls] == self.t_prev:
                for r, x in pts:
                    cv2.circle(img, (int(x), int(r)), 3, col, -1)
        if self.path is not None:
            d, y, source = self.path
            r, x = cfg.d_to_row(d), cfg.y_to_x(y)
            col = (0, 255, 0) if source == 'obs' else (0, 140, 255)
            cv2.polylines(img, [np.int32(np.c_[x, r])], False, col, 2)
        cv2.line(img, (int(cfg.robot_x), BEV_H), (int(cfg.robot_x), BEV_H - 60), (255, 0, 0), 2)
        x0 = int(max(0, cfg.robot_x - cfg.roi_half_width))
        x1 = int(min(BEV_W - 1, cfg.robot_x + cfg.roi_half_width))
        cv2.rectangle(img, (x0, cfg.roi_top_row), (x1, BEV_H - 1), (0, 255, 255), 1)
        for i, txt in enumerate(self.debug_lines()):
            cv2.putText(img, txt, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        return img

    def debug_lines(self):
        d = self.debug
        lines = ['%s v=%.3f w=%+.2f' % (d.get('mode', ''), d.get('v', 0.0), d.get('w', 0.0))]
        if 'e_y' in d:
            lines.append('e_y=%+.0fmm e_th=%+.1fdeg margin=%.0fmm'
                         % (d['e_y'] * 1000, math.degrees(d['e_th']), d['margin'] * 1000))
            lines.append('k=%+.1f k_prev=%.1f kink=%d lw=%.0fmm'
                         % (d['kappa'], d['kappa_prev'], d['kink'], d['lane_width'] * 1000))
        return lines
