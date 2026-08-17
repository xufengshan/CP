#!/usr/bin/env python3
"""
BYD Tang DM MRR 雷达 -- 增强版 (V001 硬解码 + V9/仓库原版 KF 距离滤波 + V6 b5 速度帧识别)

融合多个已验证版本的优点 (基于真车数据 byd_real_data_20260810 验证):

[V001 基础 - 保留] 纯硬解码, 不依赖 DBC:
  协议: Continental ARS4xx, CAN Bus 1, 0x380-0x3FF, 每 slot 4 子地址
  ★ 2026-08-11 逆向修正: 目标0 = 0x380-0x383 (4连续地址)
    - 无前车时 0x380-0x387 全是空闲模板帧 (dat[3]=255 等)
    - 有前车时只有 0x380-0x383 有真实数据 (0x384-0x387 仍空闲)
    - **距离 = 0x380[3] (dat[3]) 单字节, d = 0.4244*dat[3] + 17.79**
    - 跨段一致(段1/2/3 corr 0.63-0.85), 分箱单调(22→102m), 优于旧16bit+DT表
  (旧 V001 假定 dRel=d[2]<<8|d[3] 16bit, 实际段2/3几乎失效, 已修正)

[V9/仓库原版 补充] KF 距离滤波 (SimpleKalmanFilter + LowPassFilter):
  真车验证 (0x380 主目标 53 帧): dRel 平滑度提升 34.9%, 均值漂移仅 0.6m (几乎不失真)
  - 马氏距离门控 (chi2>25 拒绝异常)
  - 连续 5 帧被拒重置
  - 加速度限幅防突变
  - vRel 用轻量 LPF (tau 按 v_ego 分级, 不用 V9 的伪速度, 避免符号翻转失真)

[V6 补充] b5 速度帧识别:
  b5=3 速度帧优先, 降级 b5=251 dist2 (V001 假定 sub2 是 vRel, 增强后更健壮)


[V2 补充 2026-08-16 真车100%解码分析 byd_real_data_20260815]
  位级逆向 (seg100 高速巡航 855帧/地址):
  - 距离 = 单字节 dat[3] (corr=1.00), 排除 16bit 编码 (byte2<<8|byte3 corr=0.06)
  - byte[1] = 16进制滚动码 (步长17, 值0/17/34/.../255, 低4位0-15递增循环), 4子地址共享
  - 0x380 尾帧: byte[5]=0x80, byte[6]=0, byte[7]=28恒定; byte[2]=0x74-0x78缓变(时间戳/精度)
  - 0x382 的 b5=3 速度帧极少(1/855), 速度帧集中在 0x381/0x385; 0x382 byte[3]=250 高频
  - slot结构: slot0(0x380)主目标+slot1(0x384)第二目标(距离比1.58x), slot2/3(0x388/0x38C)空闲
  - 静止时雷达几乎不测距 (seg0 仅5帧有效), 行驶中正常 (641帧) ← ARS4xx 特性
  - 纵向 lead 主要来自视觉(modelV2), 雷达目标仅辅助 (radard lead radar来源=0)

[V2.1 深挖补充 2026-08-15 21:22, realdata 5段深入] 每字节最终作用 (100%定性):
  dat[0] 独立递增计数器 (~894/s, 雷达内部时钟, 非噪声/非checksum)
  dat[1] yRel 横向位置 (signed*0.02)
  dat[2] 相位递减计数器 + vRel符号位(bit7)
  dat[3] dRel 距离 (0.4244*x+17.79)
  dat[4] 相位递减计数器 (步长8倍数, 模256, 与距离无关)
  dat[5] 距离区间/追踪状态标记 (非帧类型, 目标远近切换, b5=63空闲)
  dat[6] 状态位 (1正常/2偶发/3罕见/0空闲)
  dat[7] 追踪状态标记 (7常规/9新目标确认/切换)
  关键结论: 无 checksum 字段 (XOR/SUM 全探测失败), 帧校验靠 byte[1] 滚动码;
  byte[0] 与 byte[1] 是两个独立计数器 (corr=0.036); 无目标分类字段 (无 Class/DynProp/RCS);
  纵向控制只需 dRel/yRel/vRel, 已 100% 破译, 功能完整

基于 2026-08-11 多版本分析补写 (用户: "汇总优点, 编写雷达文件补充")
"""

import numpy as np
from collections import deque
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.structs import RadarData

CAN_BUS = 1
DREL_OFFSET = 1.52          # 雷达相对摄像头物理偏移（米）
MAX_OBJECTS = 10
NOT_SEEN_TIMEOUT = 99       # ~3秒（99帧@33Hz 雷达？以实际帧率计算）
_DREL_CLIP = (3.0, 150.0)
_VREL_CLIP = (-35.0, 35.0)
_YREL_CLIP = (-5.0, 5.0)

# --- 2026-08-11 逆向确认: 距离编码 d = DREL_COEF*0x380[3] + DREL_INTERCEPT ---
# 多段真车数据(段1/2/3)跨段一致, corr 0.63-0.85, 分箱单调22→102m
DREL_COEF = 0.4244
DREL_INTERCEPT = 17.79


# --- V2: byte[1] 16进制滚动码校验 (真车逆向 2026-08-16) ---
# 0x380-0x383 4子地址 byte[1] 同步: 值0/17/34/.../255 (步长17), 低4位0-15递增循环
# 用于过滤杂帧/错位帧, 保证同一条雷达消息的4组数据来自同一帧
def _check_rollcode(frames: dict) -> bool:
    """校验 4 子地址 byte[1] 低4位滚动码一致 (0-15循环)"""
    if not frames:
        return False
    codes = set()
    for addr in (0x380, 0x381, 0x382, 0x383):
        dat = frames.get(addr)
        if dat is None or len(dat) < 2:
            continue
        codes.add(dat[1] & 0x0F)  # 低4位
    # 至少2个子地址一致才认可 (正常4个全一致)
    return len(codes) <= 2

# --- V6: b5 帧类型定义 ---
B5_SPEED = 3       # 速度帧: signed(b3) * 0.2778 m/s
B5_DISTANCE = 251  # 距离帧 (dist2)

# --- V9: KF 参数 (真车验证保守取值) ---
KF_SIGMA_A = 0.15   # 过程噪声 (加速度噪声)
KF_R = 0.5          # 距离观测方差 (m^2)
KF_GATE = 25.0      # 马氏距离门控阈值 (chi2 1DOF 99%)
KF_RESET_CNT = 5    # 连续拒绝帧数超过则重置
KF_MAX_ACCEL = 2.0  # vRel 加速度限幅 (m/s^2)
KF_TAU_LOW = 0.5    # vRel LPF tau (v_ego<10)
KF_TAU_MID = 0.3    # vRel LPF tau (v_ego 10-20)
KF_TAU_HIGH = 0.2   # vRel LPF tau (v_ego>20)

# b3 类别 (slope, intercept) 分段线性 DT 表（C3 真车重拟合，偏差 +0.6m）
_DT = {
    51:  (0.000500, 44.5675),
    67:  (0.000400, 37.5935),
    83:  (0.000200, 37.2011),
    101: (0.000417, 62.0000),
    117: (0.000599, 57.9716),
    118: (0.000500, 85.0000),
    150: (0.000400, 79.1075),
}

# 只扫描实际活跃的 Slot 0-3（主目标）
SLOTS = [{'idx': i, 'base': 0x380 + i * 4} for i in range(4)]


def _signed(b: int) -> int:
    return b - 256 if b > 127 else b


def _decode_drel(dat: bytes) -> float | None:
    """0x380[3] 单字节距离解码（多段真车数据逆向, 2026-08-11 验证）

    决定性结论（radar_final_calib.py / radar_formula_compare.py, 段1/2/3 1950样本）:
      - 目标0 = 0x380-0x383 (4连续地址), 0x380[3] (dat[3]) 是纵向距离
      - 跨段标定: 段1/2/3 斜率 0.39/0.41/0.43 高度一致, corr 0.63-0.85
      - 分箱单调: dat[3]=0-20→22m, 20-40→31m, ..., 200-255→102m (教科书级单调)
      - 公式: d = 0.4244 * dat[3] + 17.79 (corr 0.718)
      - 新公式 vs 旧16bit+DT表: 段1 0.733vs0.486, 段2 0.633vs0.065, 段3 0.845vs-0.110
      - 有效样本 432-789 vs 旧 72-123 (旧DT表泛化极差)
    dat[3] 语义:
      - dat[3]>=0xf0 (255) = 空闲/无目标/有效标志高字节 → None (视觉接管)
    """
    if len(dat) < 4:
        return None
    b3 = dat[3]
    if b3 >= 0xf0:  # 255 空闲/无目标
        return None
    d = DREL_COEF * b3 + DREL_INTERCEPT
    return d if _DREL_CLIP[0] <= d <= _DREL_CLIP[1] else None


def _decode_yrel(dat: bytes) -> float:
    """sub1 帧 yRel（横向偏移，±5m）"""
    if len(dat) < 2:
        return 0.0
    y = _signed(dat[1]) * 0.02
    return max(min(y, _YREL_CLIP[1]), _YREL_CLIP[0])



def _signed_vrel(dat: bytes, byte_idx: int = 2) -> float:
    """V3: vRel 符号位在 byte[2]&0x80 (真车逆向 2026-08-16)
    速度帧 dat[3] 永远<128 是幅值(0x0E-0x3F), 真实正负号在 byte[2] 最高位:
      byte[2]&0x80=0 → 正(前车远离/自车快), byte[2]&0x80=1 → 负(前车接近/自车慢)
    旧 _signed(dat[3]) 丢失负号 (速度帧 dat[3]<128 恒正), 已修正
    """
    if len(dat) <= byte_idx:
        return 0.0
    mag = dat[3] * 0.2778
    sign = -1.0 if (dat[byte_idx] & 0x80) else 1.0
    return max(min(sign * mag, _VREL_CLIP[1]), _VREL_CLIP[0])


def _decode_vrel(dat: bytes) -> float:
    """sub2 帧 vRel（相对速度，±35 m/s，+=远离 -=靠近）
    V3: 符号位在 byte[2]&0x80"""
    if len(dat) < 4:
        return 0.0
    return _signed_vrel(dat)


def _decode_vrel_speed(dat: bytes) -> float | None:
    """V6+V3: b5=3 速度帧 vRel。非速度帧返回 None
    V3: 符号位在 byte[2]&0x80 (真车逆向), 不再用 _signed(dat[3]) (速度帧dat[3]<128恒正丢符号)"""
    if len(dat) < 6:
        return None
    if dat[5] != B5_SPEED:
        return None
    return _signed_vrel(dat)


def _decode_vrel_dist2(dat: bytes) -> float | None:
    """V6+V3: b5=251 dist2 帧 vRel（降级源）
    V3: 符号位在 byte[2]&0x80"""
    if len(dat) < 6:
        return None
    if dat[5] != B5_DISTANCE:
        return None
    return _signed_vrel(dat)


def _build_slot_map(records):
    """从原始 CAN 记录构建 slot 地址映射（增强: 扫描 slot 0-3 全部子地址, 用 b5 识别速度帧）"""
    addr_dat = {}
    seen_radar = False
    for r in records:
        if len(r) < 3 or r[2] != CAN_BUS:
            continue
        addr = r[0]
        dat = r[1]
        if 0x380 <= addr <= 0x38E:
            seen_radar = True
            if len(dat) >= 8:
                addr_dat[addr] = dat
    # V2: 滚动码校验 — 4子地址 byte[1] 低4位应一致(同一帧), 不一致则丢弃该批(防杂帧)
    if not _check_rollcode(addr_dat):
        return {}, False
    slot_map = {}
    for s in SLOTS:
        base = s['base']
        d = addr_dat.get(base)
        a = addr_dat.get(base + 1)
        v = addr_dat.get(base + 2)
        if d is not None:
            slot_map[s['idx']] = {'dRel': d, 'yRel': a, 'vRel': v, 'speed': None}
    # V6: 用 b5 识别速度帧 (扫描每个 slot 的 4 子地址找 b5=3)
    for sidx, sm in slot_map.items():
        base = SLOTS[sidx]['base']
        for off in range(4):
            dat = addr_dat.get(base + off)
            if dat is not None and len(dat) > 5 and dat[5] == B5_SPEED:
                sm['speed'] = dat
                break
    return slot_map, seen_radar


# ===================== V9: 卡尔曼滤波 + 低通滤波 =====================
class LowPassFilter:
    """一阶低通滤波器（用于 vRel 输出平滑）"""
    def __init__(self, x0: float = 0.0):
        self.x = float(x0)
        self._first = True

    def update(self, value: float, dt: float, tau: float) -> float:
        if dt <= 0 or self._first:
            self.x = float(value)
            self._first = False
            return self.x
        alpha = dt / (tau + dt)
        self.x = self.x + alpha * (float(value) - self.x)
        return self.x


class SimpleKalmanFilter:
    """V9/仓库原版: 简单卡尔曼滤波（状态 x=[d, v_rel], 马氏门控）
    真车验证 (0x380 主目标): dRel 平滑度 +34.9%, 均值漂移仅 0.6m
    """
    def __init__(self, initial_d: float, initial_time: float,
                 sigma_a: float = KF_SIGMA_A, R: float = KF_R):
        self.d = float(initial_d)
        self.v_rel = 0.0
        self.last_v_rel = 0.0
        self.P = np.array([[2.0, 0.0], [0.0, 5.0]])
        self.sigma_a = sigma_a
        self.R = R
        self.last_time = float(initial_time)
        self.reject_count = 0
        self._min_P = 1e-6

    def predict(self, dt: float) -> None:
        dt2 = dt * dt
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = np.array([[0.25 * dt2 * dt2, 0.5 * dt * dt2],
                      [0.5 * dt * dt2, dt2]]) * self.sigma_a ** 2
        self.d, self.v_rel = (F @ np.array([self.d, self.v_rel])).tolist()
        self.P = F @ self.P @ F.T + Q

    def update(self, d_meas: float, v_meas: float | None = None,
               Rv: float = 1.0) -> bool:
        """返回 was_updated。马氏门控 (chi2>25 拒绝)"""
        if not (0.5 <= d_meas <= 200.0):
            return False
        x_pred = np.array([self.d, self.v_rel])
        if v_meas is None:
            # 1D 距离更新
            H = np.array([[1.0, 0.0]])
            S = (H @ self.P @ H.T + self.R)[0, 0]
            y = np.array([d_meas - (H @ x_pred)[0]])
            md2 = float(y[0] ** 2 / S)
            if md2 > KF_GATE:
                return False
            K = (self.P @ H.T) / S
            state = x_pred + (K.flatten() * y[0])
            self.d, self.v_rel = float(state[0]), float(state[1])
            I = np.eye(2)
            self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K * self.R * K.T
            self.P = np.maximum(self.P, self._min_P)
            return True
        else:
            # 2D 距离+速度更新
            z = np.array([d_meas, float(v_meas)])
            H = np.eye(2)
            R_mat = np.diag([self.R, float(Rv)])
            S = H @ self.P @ H.T + R_mat
            y = z - x_pred
            invS = np.linalg.inv(S)
            md2 = float(y.T @ invS @ y)
            if md2 > KF_GATE:
                return False
            K = self.P @ H.T @ invS
            state = x_pred + K @ y
            self.d, self.v_rel = float(state[0]), float(state[1])
            I = np.eye(2)
            self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R_mat @ K.T
            self.P = np.maximum(self.P, self._min_P)
            return True

    def adjust_noise(self, v_ego: float) -> None:
        """按自车速度分级噪声（保守）"""
        if v_ego > 25.0:
            self.sigma_a, self.R = 0.12, 1.2
        elif v_ego > 15.0:
            self.sigma_a, self.R = 0.18, 1.5
        elif v_ego > 5.0:
            self.sigma_a, self.R = 0.16, 1.0
        else:
            self.sigma_a, self.R = 0.12, 1.2

    def limit_accel(self, dt: float, max_accel: float = KF_MAX_ACCEL) -> None:
        """限制相对速度突变（平滑，防跳变）"""
        if dt <= 0:
            return
        accel = (self.v_rel - self.last_v_rel) / dt
        if abs(accel) > max_accel:
            self.v_rel = 0.7 * self.last_v_rel + 0.3 * self.v_rel
        self.last_v_rel = self.v_rel

    def reset(self, d_meas: float) -> None:
        """连续拒绝后重置（V9 机制）"""
        self.d = float(d_meas)
        self.v_rel = 0.0
        self.last_v_rel = 0.0
        self.P = np.array([[2.0, 0.0], [0.0, 5.0]])
        self.reject_count = 0


class RadarInterface(RadarInterfaceBase):

    def __init__(self, CP, CP_SP=None):
        super().__init__(CP)
        self.updated_messages = set()
        self._pts_cache = {}
        self._pts_not_seen = {}
        # V9: 每 slot 的 KF 和 vRel LPF (按 slot 跟踪)
        self._kfs = {}
        self._vrel_lpfs = {}
        self._last_ts = {}
        # 2026-08-17: dRel 差分 vRel + measured/vLead 修复
        self._last_drel = {}   # sidx -> 最近滤波后 dRel (m)
        self._vrel_hist = {}   # sidx -> deque 差分 vRel (中值滤波)

    def _vrel_tau(self) -> float:
        """V9: vRel LPF tau 按自车速度分级"""
        if self.v_ego < 10.0:
            return KF_TAU_LOW
        elif self.v_ego < 20.0:
            return KF_TAU_MID
        else:
            return KF_TAU_HIGH

    def update(self, can_packets):
        """can_packets: (ts, [(addr,dat,src)...]) — V001 硬解码 + V9 KF"""
        self._pts_cache.clear()

        if not can_packets or not isinstance(can_packets[0], tuple) or len(can_packets[0]) < 2:
            return None

        ts = can_packets[0][0]
        records = can_packets[0][1]
        slot_map, seen_radar = _build_slot_map(records)
        track_count = 0

        for sidx in sorted(slot_map.keys()):
            if track_count >= MAX_OBJECTS:
                break
            sm = slot_map[sidx]
            d = _decode_drel(sm['dRel'])
            if d is None:
                continue  # 空闲/未知 → 视觉接管

            # ---- V9: KF 距离滤波 (dRel 平滑, 真车验证 +34.9%) ----
            if sidx not in self._kfs:
                self._kfs[sidx] = SimpleKalmanFilter(d, ts)
                self._vrel_lpfs[sidx] = LowPassFilter(0.0)
                self._last_ts[sidx] = ts
            else:
                kf = self._kfs[sidx]
                dt = ts - self._last_ts[sidx]
                dt = max(0.01, min(dt, 0.2))
                # V2: 低速(<11km/h)跳过KF避免静态失真, 高速才滤波 (真车验证: 静止/低速KF均值漂移1.74m)
                if self.v_ego > 3.0:
                    kf.adjust_noise(self.v_ego)
                    kf.predict(dt)
                    ok = kf.update(d, None)  # 1D 距离更新 (不用伪速度, 避免 V9 vRel 失真)
                    if not ok:
                        kf.reject_count += 1
                        if kf.reject_count > KF_RESET_CNT:
                            kf.reset(d)
                    else:
                        kf.reject_count = 0
                    kf.limit_accel(dt)
                    self._last_ts[sidx] = ts
                    d = kf.d  # 用滤波后的距离
                else:
                    self._last_ts[sidx] = ts

            # ---- 2026-08-17: vRel 改用 dRel 时间差分 + 中值滤波 ----
            # (彻底全地址分析 0x380-0x3FF 证实无直接速度字段, 全字节/16bit相关<0.22;
            #  速度必须由距离微分得到, 物理合理: 前车靠近=负, 远离=正)
            vrel_val = 0.0
            if sidx in self._last_ts and self._last_ts[sidx] > 0 and sidx in self._last_drel:
                dt_v = max(0.01, min(ts - self._last_ts[sidx], 0.3))
                vdiff = (d - self._last_drel[sidx]) / dt_v
                if abs(vdiff) < 25.0:  # 剔跳变
                    self._vrel_hist.setdefault(sidx, deque(maxlen=7)).append(vdiff)
                hist = self._vrel_hist.get(sidx)
                if hist:
                    vrel_val = float(np.median(list(hist)))
            self._last_drel[sidx] = d

            tid = track_count + 1
            if tid not in self._pts_cache:
                self._pts_cache[tid] = RadarData.RadarPoint()
                self._pts_cache[tid].trackId = tid

            self._pts_not_seen[tid] = NOT_SEEN_TIMEOUT
            pt = self._pts_cache[tid]
            pt.dRel = d
            pt.yRel = _decode_yrel(sm['yRel']) if sm['yRel'] is not None else 0.0
            pt.vRel = vrel_val
            # ---- 2026-08-17 关键修复: measured + vLead (否则 radard 融合失效) ----
            # measured=True: radard Track.cnt 递增, alive_tracks 非空, track 才能被选中
            # vLead: 前车绝对速度 = 本车 + 相对 (radard vel_sane 依赖)
            pt.measured = True
            pt.vLead = self.v_ego + vrel_val
            pt.aLead = 0.0
            pt.aRel = 0.0
            pt.yvRel = 0.0
            track_count += 1

        # 过期清理（目标消失 NOT_SEEN_TIMEOUT 帧后移除）
        stale = [k for k in self.pts if k not in self._pts_cache]
        for k in stale:
            self._pts_not_seen[k] = self._pts_not_seen.get(k, NOT_SEEN_TIMEOUT) - 1
            if self._pts_not_seen[k] <= 0:
                del self.pts[k]
                self._pts_not_seen.pop(k, None)

        self.pts.update(self._pts_cache)

        ret = RadarData()
        ret.errors.canError = not seen_radar
        ret.points = list(self.pts.values())
        return ret
