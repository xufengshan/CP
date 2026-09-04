#!/usr/bin/env python3
"""
BYD 唐DM 车内雷达 (Continental ARS4xx) — RadarInterface 官方化 + 方位几何恢复版
================================================================================
★ 2026-09-04 15:35  官方化(3194ec) + 方位/三角几何恢复  (用户批准方向"按照建议优化", 只本地改文件)
  问题诊断(09-04 15:32 拍板): "官方化回归"执行过度, 把 carrot 底层拿不到的【硬件方位几何】
  也一并删了 → 系统不知道旁车方位 → 主目标(0x109)无效时段不会正确判断、也不放视觉接管.
  本版在原官方化架构上【只补回雷达侧必须提供的硬件几何】, 其余合成/fusion 仍交 carrot 系统层.

★ 架构契约 (与 toyota官方 / bases.interfaces.RadarInterfaceBase 一致):
    card.py  : RD = self.RI.update_carrot(CS.vEgo, CS.aEgo, rcv_time, can_list)
    base      : update_carrot(...) 内部: ① self.v_ego=self.v_ego_hist[0]; ② ret=self.update(can_packets)
                → ③ 对本帧 self.pts 里每个点按 trackId 建/续 MyTrack(跟踪/平滑/加速度)
                → ④ 覆盖 radar_point.aLead/jLead/yRel(平滑)/yvRel(平滑) 后返回 ret.
                → ⑤ ret(None 或 RadarData) 原样返回给 card.  radard/carrot 消费 RD.points.
    即: 本接口雷达只负责在 self.update() 里 decode CAN → 在 self.pts 放逐点(含方位/tri 几何/稳定trackId);
        KF/横向平滑/aLead = 系统层(MyTrack)工作, 本文件不再越权.

★ 本版相比"纯官方化 3194ec" 多恢复的 (雷达层必须自备的硬件几何, 每雷达点提供):
  1. 方位采集  : 池槽 base+1 (0x381/385/389/38D/391/395) 各自 SUB 帧 AzimSub{i}(b6) 方位
                 → DBC (1,-128) → 解码值 = b6-128 (128中心方位指数) → θ = 值×0.55°/LSB.
                 (主目标 0x340 AzimB7 恒≈128 中心、Main 锁纵向不参与横向 → 不另注册, 见下注)
  2. 三角 _triangle_dynamic: 副目标相对参考系(有主→主目标0x109; 无主→自车)给出:
                 y_rel(车道量化横向) / v_lateral(横向切入=dS·θ̇) / dx_dot(纵向贴近=dṠ·cosθ−dṀ)
  3. _azimuth_to_yrel 车道量化: 副目标横向量化到 4 车道区间中点(1.75/5.25/8.75/12.25m),
                 落在区间之间/超范围=运动中车 → 保留连续横向(不硬量化)
  4. 完整输出字段 (radard/carrot primary 需要):
                 dRel(距离) / yRel(副=车道横向, 主=0锁纵) /
                 vRel(副=横向切入速度, 主=dM_dot→逼近) / yvRel(副=切入) /
                 vLead(副=v_ego+dx_dot, 主=v_ego+vRel) /
                 measured=True / aLead=0.0 / aRel=0.0 (绝不塞安全等级1/2 — 会被 carrot predictor
                 误读成前车加速度 a_rel 污染前车轨迹预测; aRel/aLead 保持 0 加速度语义)

★ 保留的官方化合理部分 (09-04 定点保留, 不全退回复刻旧大版):
  ✅ 官方 CANParser 触发式架构(trigger=0x109, updated_messages 累积, canError 超时清 pts)
     + 基类 RadarInterfaceBase.update_carrot 做 MyTrack/KF/横向平滑 → 雷达不重复造轮子
  ✅ trackId 按【物理通道(地址)】持续分配: 同地址持续同 trackId; 消失重现=新 trackId(单调递增).
     Main 固定 trackId=1.  (注: pts 以地址为键 → 基类 MyTrack 要求每个存活点 trackId 唯一,
     故不做"跨槽位 4m 合并成同一对象"的旧大版机制; 地址一旦换槽新 ID 是官方 single-slot 语义)
  ✅ 主目标无效【短宽限】防闪: Main(0x109) 短时无效时短保持 track1, 让 UI/lead 不闪;
     🔴 但超宽限【必须清掉】自清, 绝不顶假点卡住视觉(09-01 "Main抖动假点卡起步"教训;
     09-04 用户拍板: 主目标持续无效超宽限必须清 → 放视觉接管, 且让旁车方位点继续喂)
  ✅ 池槽"有效在场才输出+短时槽保持": 15Hz 池槽按 25Hz 节拍抽样有间隙 → 极短保持(=1节拍估)
     避免目标仍在时 MyTrack 被该间隙重置闪断; 一旦收到空闲/哨兵/无效 → 立即清, 不留假点.

★ 解码公式 (唯一解码, 原样保留 100% 真车验证; DBC byd_han_dmev_2020):
    0x109 RADAR_MAIN_TARGET  MainDist = 0.5*b7 - 4    (主目标 25Hz 100%)
    池A base+1 (0x381..395)  dRel = 0.4244*b3 + 17.79 (槽连续距离)
                            AzimSub{i} = b6 - 128 → 0.55°/LSB
    bus1 (CanBus.MRR), DBC byd_han_dmev_2020, 触发=0x109
  📌 注: 主目标方位 0x340 AzimB7 实测恒≈128(正前方) → Main 锁纵向(yRel=0), 方位不参与三角参考系
     (旧大版注解同: "主=0x340恒≈128、副=各SUB帧b6 → 主锁纵向不参与横向") → 不额外注册 0x340.

🔴 纪律: 纯本地文件; 绝不连真机 / 不 pscp / 不 push / 不改其它文件。
"""
import math
import numpy as np
from typing import List, Set, Optional
from opendbc.can import CANParser
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.structs import RadarData

# ==================== DBC / 总线 配置 ====================
_DBC_NAME = "byd_han_dmev_2020"      # 官方 DBC (MainDist / dRel_slot*b + AzimSub*)
CAN_BUS = 1                          # 车内雷达总线 (CanBus.MRR = 1)
MAX_OBJECTS = 7                      # 当帧最大同时目标: Main + 6 池A base+1 槽
_TS_TIMEOUT = 2.0                    # 雷达数据超时(s): 超此未见 0x109 触发 -> 清 pts + canError

# ---- 消息地址 (DBC byd_han_dmev_2020, 真车验证) ----
_MAIN_MSG = 0x109                    # RADAR_MAIN_TARGET  主目标 MainDist = 0.5*b7 - 4 (25Hz)
# 池A 只读 base+1 (2026-08-24 深挖): base+0 = 离散量化档(假目标源)跳过;
#   base+1 = 连续真实距离 + 方位(b6 AzimSub) = 唯一信任源。槽4/5 (0x391/395) DBC 已补 dRel_slot4b/5b+AzimSub4/5
_POOLA_ADDRS: List[int] = []
for _i in range(6):
    _POOLA_ADDRS.append(0x381 + _i * 4)                   # 0x381/385/389/38D/391/395
_POOLA_DREL_SIG = ['dRel_slot0b', 'dRel_slot1b', 'dRel_slot2b',
                   'dRel_slot3b', 'dRel_slot4b', 'dRel_slot5b']
_POOLA_AZIM_SIG = ['AzimSub0', 'AzimSub1', 'AzimSub2',
                   'AzimSub3', 'AzimSub4', 'AzimSub5']

# ---- 有效窗 (真车验证/深挖) ----
_MAIN_VALID_MIN, _MAIN_VALID_MAX = 1.0, 120.0   # 主目标有效距离 (1.0m 下界 - 08-25 "近距离不刹车"根因修复)
_SLOT_VALID_MIN, _SLOT_VALID_MAX = 3.0, 120.0   # 槽连续距离有效窗
_IDLE_B3 = (4, 247, 255)                        # 槽空闲哨兵 dat[3] → d≈19.5/122.6/126m 假目标

# ---- 宽限 / 保持 (帧计数; trigger 25Hz) ----
_MAIN_GRACE_FRAMES = 8                 # 主目标无效防闪宽限帧数 (~0.32s); 超限【必须自清】放视觉接管
_SLOT_HOLD_FRAMES = 2                  # 池槽按 25Hz 节拍抽样间隙的最短保持 (1~2帧 ~40-80ms, 防 15Hz 目标被切缝闪断)

# ---- 方位几何参数 (0.55°/LSB; 野值窗) ----
_DEG_PER_LSB = 0.55
_AZ_OFF_VALID_MIN, _AZ_OFF_VALID_MAX = -38.0, 112.0   # 解码方位指数 (b6-128) 有效窗, 之外=无可靠方位
# 车道量化 (副目标落到车道区间 → 用区间中点档位; 中国路况最多 4 车道; 区间之间/超范围 → 运动中的车保留连续值)
_LANE_RANGES = ((0.3, 3.2, 1.75), (3.2, 7.2, 5.25),
                (7.2, 11.2, 8.75), (11.2, 200.0, 12.25))
_DM_DOT_CLIP = 15.0                    # 主目标距离变化率限幅 (m/s)
_VLAT_CLIP = 15.0                      # 横向切入速度限幅 (m/s)
_DXDOT_CLIP = 35.0                     # 纵向贴近 dx_dot 限幅 (m/s)
_SLOPE_WIN = 0.5                       # 滑动窗(s)


def _lsq_slope(rows) -> float:
    """最小二乘斜率 (最近 ~0.5s 滑动窗, 降 0.5/0.4244m 量化台阶噪声):
    斜率 = (n·Σ(tv) − Σt·Σv) / (n·Σ(t²) − (Σt)²).  <3 点 → 0."""
    if len(rows) < 3:
        return 0.0
    n = len(rows)
    st = sum(r[0] for r in rows)
    sv = sum(r[1] for r in rows)
    stv = sum(r[0] * r[1] for r in rows)
    stt = sum(r[0] * r[0] for r in rows)
    den = n * stt - st * st
    return (n * stv - st * sv) / den if abs(den) > 1e-9 else 0.0


def _make_radar_can_parser() -> CANParser:
    """注册 0x109(Main 25Hz) + 池A base+1 六槽(副 15Hz), bus1. 权威: 主25Hz 副15Hz (08-18路测)"""
    messages = [(_MAIN_MSG, 25)] + [(a, 15) for a in _POOLA_ADDRS]
    return CANParser(_DBC_NAME, messages, CAN_BUS)


class RadarInterface(RadarInterfaceBase):
    """BYD 唐DM 车内雷达接口 - 官方化 + 方位/三角几何恢复版
    职责 = decode CAN → self.pts 放逐点(含硬件方位车道横向 + 三角纵向/横向动态 + 稳定 trackId)
           + 主目标短路宽限防闪(超限自清放视觉); KF/横向平滑/加速度 = 基类 MyTrack 层.
    不越权: 不做 TTC/安全等级/跨槽对象级融合 — 都交 carrot(radard/primary/MyTrack)."""

    def __init__(self, CP):
        super().__init__(CP)
        self.rcp = None if getattr(CP, 'radarUnavailable', False) else _make_radar_can_parser()
        self.trigger_msg = _MAIN_MSG                 # 0x109 主目标 = 触发
        self.updated_messages: Set[int] = set()
        self._last_seen_ts = 0.0                     # 最近一次 0x109 触发时刻(s), 迟到判 canError

        # ---- 逐物理通道(地址)状态 ----
        self._addr_track: dict[int, int] = {}        # addr -> trackId (通道首次出现分配; 单调不回收)
        self._next_track_id = 2                      # trackId 单调递增 (2 起; Main 固定 1)
        # 短时宽限/保持 (均按触发帧计数)
        self._main_gone_cnt = 0                      # 主目标连续无效帧数 (超 _MAIN_GRACE_FRAMES 自清)
        self._main_last_d: Optional[float] = None    # 主目标最近一次真实有效距离(宽限重发用)
        self._main_last_vlead = 0.0                  # 主目标最近有效 vLead(宽限重发用)
        self._slot_hold: dict[int, int] = {}         # addr -> 连续未有效更新的保持计数 (超 _SLOT_HOLD_FRAMES 清)
        self._slot_last: dict[int, tuple] = {}       # addr -> 最近有效 (d, trackId) 供保持重发

        # ---- 滑动窗 (硬件几何 / 斜率) ----
        self._win_d: dict[int, list] = {}            # addr/ch 'Main' -> [(ts,d)] 距离窗 (dṠ / dM_dot / 主 vRel)
        self._win_th: dict[int, list] = {}           # addr -> [(ts,θ_deg)] 方位窗 (θ̇)
        self._main_track = _MAIN_MSG                 # 主目标地址即 0x109

    # ==================== 主入口 (供 base RadarInterfaceBase.update_carrot 调用) ====================
    def update(self, can_packets):
        """基类 update_carrot → self.update(can_packets). self.v_ego 已被基类在当前帧赋值.
        0x109 触发才产一帧(processed); 逐物理通道 decode → 续 pts / 宽限保持 / 消失清 / NEW trackId."""
        if self.rcp is None:
            return super().update(None)

        vls = self.rcp.update(can_packets)
        self.updated_messages.update(vls)

        try:
            now_s = can_packets[0][0] / 1e9
        except Exception:
            now_s = 0.0

        # 雷达超时: 超过 _TS_TIMEOUT 未见 0x109 触发 -> canError + 清 pts(不顶假点)
        if self.trigger_msg not in self.updated_messages:
            if (now_s - self._last_seen_ts) > _TS_TIMEOUT and self._last_seen_ts > 0.0:
                rr = RadarData()
                rr.errors.canError = True
                self.pts.clear()
                rr.points = list(self.pts.values())
                return rr
            return None

        self._last_seen_ts = now_s
        ret = self._process_frame(now_s)
        self.updated_messages.clear()
        return ret

    # ==================== 核心: 触发帧逐物理通道更新 ====================
    def _process_frame(self, now_s: float) -> RadarData:
        """在 0x109 触发帧上更新 self.pts (地址为键; 每点多几何字段 + 稳定 trackId).
        主目标无效: 短宽限防闪 -> 超限【必清】放视觉.
        池槽: 有效在场才输出(短时槽保持防 15Hz 抽样切缝); 收到无效/哨兵/空闲 → 立即清."""
        ret = RadarData()
        if not self.rcp.can_valid:
            ret.errors.canError = True

        # 本触发帧"实际更新"的通道 (来自 CANParser, 防 vl 残留旧值把离场目标当仍在)
        updated = set(self.updated_messages)

        # ================== 1) 主目标处理 (短宽限, 超限自清) ==================
        main_valid = False
        main_d = None
        if _MAIN_MSG in updated:                      # 0x109 更新过 (本帧触发帧必有)
            _mv, _md, _ = self._read_main(now_s)
            main_valid, main_d = _mv, _md
        if main_valid:
            self._main_gone_cnt = 0
            self._main_last_d = main_d
            self._emit_main(now_s, main_d)            # 真实 Main 点 (track1)
        else:
            # 主目标本帧无有效测量: 短宽限防闪 → 超限必清(放视觉接管, 不顶假点)
            self._main_gone_cnt += 1
            if self._main_last_d is not None and self._main_gone_cnt <= _MAIN_GRACE_FRAMES:
                self._emit_main_hold(now_s)           # 短保持 track1 (dRel=最后有效, UI/lead 不闪)
            else:
                # 超宽限 / 从未有效: 彻底清掉 track1, 让 carrot 视觉接管
                if _MAIN_MSG in self.pts:
                    del self.pts[_MAIN_MSG]
                self._main_last_d = None
                self._main_gone_cnt = 0

        # ================== 2) 池槽逐个 (方位/三角几何 + 槽短保持) ==================
        #   语义三态:
        #     addr 本帧已更新 & 距离有效   → 真实几何点(重算三角), 复位保持
        #     addr 本帧已更新 & 值无效/idle → 目标确认走了 → 立即清(不留假点)
        #     addr 本帧未更新(槽静默, 15Hz 抽样间隙) → 极短保持重发(防闪断), 超限清
        for i, addr in enumerate(_POOLA_ADDRS):
            if addr not in updated:
                # 槽本帧静默(未推送)——允许极短保持(防 25Hz 节拍把 15Hz 仍在目标闪断), 超限清
                hold = self._slot_hold.get(addr, 0) + 1
                self._slot_hold[addr] = hold
                last = self._slot_last.get(addr)
                if last is not None and hold <= _SLOT_HOLD_FRAMES:
                    self._emit_slot_hold(addr, now_s, last[0], last[1])
                else:
                    self._drop_slot(addr)
            else:
                # 槽本帧已更新: 读真正测量
                d, off = self._read_slot(addr, _POOLA_DREL_SIG[i], _POOLA_AZIM_SIG[i])
                if d is not None:
                    self._slot_hold[addr] = 0
                    self._slot_last[addr] = (d, off)
                    self._emit_slot(addr, now_s, d, off)
                else:
                    # 已更新但值无效/空闲/哨兵 = 目标确认离开 → 立即清, 绝不顶假点
                    self._drop_slot(addr)

        ret.points = list(self.pts.values())
        return ret

    def _drop_slot(self, addr: int) -> None:
        """清除某池槽点及其保持/状态 (目标确认离开或超保持)."""
        if addr in self.pts:
            del self.pts[addr]
        self._slot_hold.pop(addr, None)
        self._slot_last.pop(addr, None)

    # ==================== 读取: 主目标 / 槽 ====================
    def _read_main(self, now_s: float):
        """主目标 0x109 MainDist: 有效窗 1.0~120m 才认. 无效(空闲/超范围)→(False, …).
        主目标有效时把真实点在 pts 内替入, 并给 (main off 恒 0 锁纵)."""
        md = self.rcp.vl.get(_MAIN_MSG, {}).get('MainDist', None)
        if md is None or md != md:
            return False, None, 0.0
        md = float(md)
        if _MAIN_VALID_MIN <= md <= _MAIN_VALID_MAX:
            # 主距离窗更新 (dM_dot 滑动窗拟合; 供主 vRel 与副 dx_dot 参考)
            w = self._win_d.setdefault(self._main_track, [])
            w.append((now_s, md))
            while w and (now_s - w[0][0]) > _SLOPE_WIN:
                w.pop(0)
            return True, md, 0.0
        return False, None, 0.0

    def _read_slot(self, addr: int, d_sig: str, az_sig: str):
        """槽 base+1 地址: 连续距离 dRel-signal + 方位 AzimSub(b6→-128 解码). 
        空闲/越界/哨兵 dat[3]=4/247/255 → (None,None). 方位野值窗内置 off_valid."""
        d = self.rcp.vl.get(addr, {}).get(d_sig, None)
        if d is None or d != d:
            return None, None
        d = float(d)
        if not (_SLOT_VALID_MIN <= d <= _SLOT_VALID_MAX):
            return None, None
        b3 = round((d - 17.79) / 0.4244)            # 反解 dat[3] 过滤空闲哨兵码 (真车验证)
        if b3 in _IDLE_B3:
            return None, None
        # 方位 (DBC (1,-128) → 解码 = b6-128 方位指数; 0.55°/LSB)
        off = self.rcp.vl.get(addr, {}).get(az_sig, None)
        off = float(off) if off is not None and off == off else 0.0
        return d, off

    # ==================== 几何 / 动态 ====================
    @staticmethod
    def _theta_deg(off: float) -> float:
        """方位指数(解码 b6-128) → 相对车头角度 deg (负=左, 正=右; 0.55°/LSB)."""
        return off * _DEG_PER_LSB

    @staticmethod
    def _azimuth_to_yrel(off: float, d_rel: float) -> float:
        """方位 → 横向车道量化 (副目标车道档位容错, 2026-08-25 用户定稿):
        角度 θ = off×0.55°; 连续横向 = -d·sinθ (radard 右正左负, 取反).
        落在车道区间(第1~4车道) → 区间中点档位(1.75/5.25/8.75/12.25m, 容错不同路宽);
        落在区间之间/超范围 = 运动中的车(变道/插入/远离) → 保留连续横向, 不硬量化.
        野值方位(off 超出 [-38,112]) → 无可靠方位, 回正前方 0."""
        if d_rel < 0.1 or not (_AZ_OFF_VALID_MIN <= off <= _AZ_OFF_VALID_MAX):
            return 0.0
        yrel = -d_rel * math.sin(math.radians(off * _DEG_PER_LSB))
        ay = abs(yrel)
        sign = 1.0 if yrel >= 0 else -1.0
        for lo, hi, mid in _LANE_RANGES:
            if lo <= ay <= hi:
                return float(sign * mid)            # 车道中间: 固定档位(容错路宽)
        return float(np.clip(yrel, -15.0, 15.0))    # 区间之间/超范围: 保留连续横向

    def _slot_dynamics(self, addr: int, now_s: float, d: float, off: float,
                       theta_deg: float, main_d: Optional[float]) -> tuple:
        """副目标三角动态 (2026-08-25 用户定稿公式, 参考系=主目标或自车):
          dṠ: 本槽距离窗 lstsq 斜率 (0.5s 滑动窗, 降量化)
          θ̇ : 方位指数窗 lstsq 斜率 → rad/s → v_lateral = dS·θ̇  (横向切入, 全可靠)
          dṀ : 主目标距离窗斜率 (外层 _main_dm_dot)
          dx_dot = dṠ·cosθ − dṀ   (副相对参考系纵向贴近; <0=逼近)
          (此处用相位差垂直项 sinθ·θ̇ 时 dṠ 误差与 θ 噪声耦合大 → 采用户定稿简化, 见旧版注解)
        返回 (v_lateral, dx_dot)."""
        # 滑动窗: 距离 + 方位(各自独立, 避免互相污染)
        wd = self._win_d.setdefault(addr, [])
        wd.append((now_s, d))
        while wd and (now_s - wd[0][0]) > _SLOPE_WIN:
            wd.pop(0)
        wt = self._win_th.setdefault(addr, [])
        wt.append((now_s, theta_deg))
        while wt and (now_s - wt[0][0]) > _SLOPE_WIN:
            wt.pop(0)

        dS_dot = _lsq_slope(wd)                       # 槽自身距离变化率 (m/s)
        dth_deg = _lsq_slope(wt)                      # 方位角变化率 (deg/s)
        cos_t = math.cos(math.radians(theta_deg))
        v_lateral = d * math.radians(dth_deg)         # 横向切入速度 (m/s)

        # 参考系纵向变化: 主目标有效用主 dṀ; 否则参考自车 dṀ=0
        dM_dot = self._main_dm_dot()
        if main_d is None or main_d <= 0.0:
            dM_dot = 0.0
        dx_dot = dS_dot * cos_t - dM_dot              # 副相对(主/自车)纵向贴近 (m/s)

        return (float(np.clip(v_lateral, -_VLAT_CLIP, _VLAT_CLIP)),
                float(np.clip(dx_dot, -_DXDOT_CLIP, _DXDOT_CLIP)))

    def _main_dm_dot(self) -> float:
        """主目标距离变化率 (0.5s 窗 lstsq 斜率, 限幅 ±15 m/s). 负=逼近."""
        rows = self._win_d.get(self._main_track, [])
        if len(rows) < 3:
            return 0.0
        return float(np.clip(_lsq_slope(rows), -_DM_DOT_CLIP, _DM_DOT_CLIP))

    # ==================== 发射: Main ====================
    def _emit_main(self, now_s: float, d: float) -> None:
        """真实主目标点 (trackId=1, yRel=0 锁纵, vRel=dM_dot 逼近, vLead=v_ego+vRel)."""
        dM_dot = self._main_dm_dot()
        addr = self._main_track
        pt = self.pts.get(addr)
        if pt is None:
            pt = RadarData.RadarPoint()
            self.pts[addr] = pt
        pt.trackId = 1
        self._addr_track[addr] = 1
        pt.dRel = d
        pt.yRel = 0.0
        pt.vRel = float(np.clip(dM_dot, -35.0, 35.0)) if dM_dot else 0.0
        pt.yvRel = 0.0
        try:
            pt.vLead = self.v_ego + pt.vRel          # 前车绝对速度 (供 MyTrack 推 aLead; carrot _pick_two 过滤需 vLead>2)
        except Exception:
            pt.vLead = 0.0
        pt.measured = True
        self._set_zero_accel(pt)
        self._main_last_vlead = pt.vLead

    def _emit_main_hold(self, now_s: float) -> None:
        """主目标短路宽限防闪: 重发最后真实 Main 点 (dRel 不变). 超限由 _process_frame 清."""
        if self._main_last_d is None:
            return
        dM_dot = self._main_dm_dot()
        addr = self._main_track
        pt = self.pts.get(addr)
        if pt is None:
            pt = RadarData.RadarPoint()
            self.pts[addr] = pt
        pt.trackId = 1
        pt.dRel = self._main_last_d
        pt.yRel = 0.0
        pt.vRel = float(np.clip(dM_dot, -35.0, 35.0)) if dM_dot else 0.0
        pt.yvRel = 0.0
        try:
            pt.vLead = self.v_ego + pt.vRel
        except Exception:
            pt.vLead = self._main_last_vlead
        pt.measured = True
        self._set_zero_accel(pt)

    # ==================== 发射: 池槽 (方位/三角几何) ====================
    def _emit_slot(self, addr: int, now_s: float, d: float, off: float) -> None:
        """真实池槽点: 完整几何字段 (方位车道横向 + 三角横向切入/纵向贴近 + 稳定 trackId)."""
        theta_deg = self._theta_deg(off)
        main_d = self._main_last_d if self._main_last_d is not None else (
            self.pts.get(_MAIN_MSG).dRel if self.pts.get(_MAIN_MSG) is not None else None)
        main_d = main_d if (main_d is not None and self.pts.get(_MAIN_MSG) is not None) else None

        # 三角动态 (v_lateral 横向切入, dx_dot 纵向贴近)
        v_lateral, dx_dot = self._slot_dynamics(addr, now_s, d, off, theta_deg, main_d)
        # 车道量化横向 (可靠方位才量化; 野值 → 正前方)
        y_rel = self._azimuth_to_yrel(off, d) if (_AZ_OFF_VALID_MIN <= off <= _AZ_OFF_VALID_MAX) else 0.0

        pt = self.pts.get(addr)
        # 稳定 trackId: 首次/重现该地址 → 分配新 trackId (单调递增); 持续同 trackId
        if pt is None or pt.trackId == 0:
            pt = RadarData.RadarPoint()
            self.pts[addr] = pt
            tid = self._addr_track.get(addr)
            if tid is None:
                while self._next_track_id == 1:      # 避开 Main
                    self._next_track_id += 1
                tid = self._next_track_id
                self._next_track_id += 1
                self._addr_track[addr] = tid
            pt.trackId = tid
        pt = self.pts[addr]
        pt.dRel = d
        pt.yRel = y_rel
        pt.vRel = v_lateral                          # 副: 横向切入速度 (任务语义 vRel=切入)
        pt.yvRel = v_lateral                         # 副: 横向切入 (基类会平滑覆盖)
        try:
            pt.vLead = self.v_ego + dx_dot           # 副: 绝对速度 = 自车 + 纵向相对贴近
        except Exception:
            pt.vLead = 0.0
        pt.measured = True
        self._set_zero_accel(pt)

    def _emit_slot_hold(self, addr: int, now_s: float, d: float, off: float) -> None:
        """池槽极短保持 (防 15Hz 抽样切缝闪断 MyTrack): 重发最后几何点.
        这里不重算动态(无新测量), 重发最后 dRel/横向/几何; vRel/vLead 用短的轻量差分避免数据陈旧."""
        theta_deg = self._theta_deg(off)
        main_d = self.pts.get(_MAIN_MSG).dRel if self.pts.get(_MAIN_MSG) is not None else None
        # 保持重发: 纵向逼近未知 → dx_dot 0, v_lateral 0 (平滑交给 MyTrack, 不臆造)
        y_rel = self._azimuth_to_yrel(off, d) if (_AZ_OFF_VALID_MIN <= off <= _AZ_OFF_VALID_MAX) else 0.0
        pt = self.pts.get(addr)
        if pt is None:
            pt = RadarData.RadarPoint()
            self.pts[addr] = pt
            tid = self._addr_track.get(addr)
            if tid is None:
                while self._next_track_id == 1:
                    self._next_track_id += 1
                tid = self._next_track_id
                self._next_track_id += 1
                self._addr_track[addr] = tid
            pt.trackId = tid
        pt = self.pts[addr]
        pt.dRel = d
        pt.yRel = y_rel
        pt.vRel = pt.vRel if hasattr(pt, 'vRel') else 0.0
        try:
            pt.yvRel = pt.yvRel if hasattr(pt, 'yvRel') else 0.0
        except Exception:
            pt.yvRel = 0.0
        if main_d is not None:
            try:
                pt.vLead = self.v_ego + 0.0
            except Exception:
                pt.vLead = 0.0
        pt.measured = True
        self._set_zero_accel(pt)

    # ==================== 工具 ====================
    @staticmethod
    def _set_zero_accel(pt) -> None:
        """aLead/aRel 保持 0 加速度语义 (绝不塞安全等级1/2 → carrot predictor 会把 aRel 误读成
        前车加速度 a_rel 污染轨迹预测). 基类 MyTrack 会随后用 vLead 差分推 aLead/jLead 覆盖 aLead."""
        try:
            pt.aRel = 0.0
            pt.aLead = 0.0
        except Exception:
            pass

    # ==================== 重置 ====================
    def reset(self) -> None:
        """重置雷达接口 (全部当帧/通道/几何状态清空)"""
        self.pts = {}
        self.updated_messages = set()
        self._addr_track = {}
        self._next_track_id = 2
        self._main_gone_cnt = 0
        self._main_last_d = None
        self._main_last_vlead = 0.0
        self._slot_hold = {}
        self._slot_last = {}
        self._win_d.clear()
        self._win_th.clear()
        self._last_seen_ts = 0.0
        # 基类遗留成员一并复位
        try:
            self._pts_cache = {}
            self._prev_targets = []
            self._used_this_frame = set()
            self._next_track_id = 2
        except Exception:
            pass


# ==================== 冒烟测试 ====================
# 注: 需在完整 openpilot 环境跑 (真实 CANParser 解 bytes → 需 byd_han_dmev_2020.dbc / numpy)。
# 语义无真机 CANParser 时, 见 workspace 的 _byd_smoke.py (假 CANParser 驱动 update() 验分层)。
if __name__ == "__main__":
    from opendbc.car.structs import CarParams
    inst = RadarInterface(CarParams())
    # 空闲帧 (0x109 MainDist dat[7]=255 -> 0.5*255-4=123.5>120 无效; 池槽 dat[3]=255 ->126>120 无效)
    idle = [(0x109, bytes([0, 0, 0, 0, 0, 0, 0, 255]), 1),
            (0x381, bytes([0, 0, 0, 255, 0, 0, 0, 255]), 1)]
    # 真目标帧: 0x109 dat[7]=100 -> Main=46m; 0x381 dat[3]=50 -> 0.4244*50+17.79=39.01m; 0x385 dat[3]=60->43.25m
    #   0x381.b6 AzimSub0 = 115-128 = -13 指数 (~左 7°) ; 0x385.b6=141-128=+13 (右 7°)
    real = [(0x109, bytes([0, 0, 0, 0, 0, 0, 127, 100]), 1),
            (0x381, bytes([0, 0, 0, 50, 0, 0, 115, 7]), 1),
            (0x385, bytes([0, 0, 0, 60, 0, 0, 141, 9]), 1)]
    pkts = lambda f: [(int(1e9), f)]
    rd = inst.update(pkts(idle))
    print("空闲帧: 返回 %s / 目标数=%d (期望无目标, 触发帧)" % (type(rd).__name__, len(list(rd.points) if rd else [])))
    for i in range(3):
        rd = inst.update(pkts(real))
    pts = list(rd.points)
    print(f"真目标持续3帧: 目标数={len(pts)} (应>=3)")
    for p in sorted(pts, key=lambda q: q.dRel):
        print(f"  track{p.trackId}: dRel={p.dRel:.1f}m yRel={p.yRel:+.2f} vRel={p.vRel:+.1f} "
              f"vLead={p.vLead:+.1f} measured={p.measured} aRel={p.aRel}")
    # 主目标消失(空闲), 但仍有池槽目标 → 主宽限保持再清, 槽保持几何看得到(主无效系统仍知旁车方位)
    rd = inst.update(pkts(idle + [(0x381, bytes([0, 0, 0, 50, 0, 0, 115, 7]), 1)]))
    print(f"主消失帧(池槽仍在): 目标数={len(list(rd.points))} (主宽限保持/清, 槽应仍在)")
    print("冒烟测试完成 (完整路径需真实 CANParser 环境; 本脚本最少验证语法/基类契约)")