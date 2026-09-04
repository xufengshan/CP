#!/usr/bin/env python3
"""
BYD 唐DM 车内雷达 (Continental ARS4xx) — RadarInterface 官方回归版
================================================================================
★ 2026-09-04 官方方式回归 (用户批准; 方向 = 接口只做"解析 CAN → 逐点 RadarPoint + 稳定 trackId",
  合成/fusion 全部交 carrot 系统层: RadarInterfaceBase.update_carrot + MyTrack → radard)。
  参照官方样板 toyota/radar_interface.py 重构:
    __init__: 建 self.rcp(CANParser, Main0x109 + 池A base+1 六槽), trigger=0x109, updated set,
              track_id 单调计数, pts 按物理通道(地址)持续
    update(can_packets): rcp.update -> 0x109 触发才产一帧 -> 逐"物理通道"处理
              (Main=0x109 / 池槽=base+1地址): 当通道有有效测量 -> pts[channel] 延续+同 trackId;
               通道消失(该帧无有效测量)-> pts[channel] 删除; 重现=新目标 -> NEW trackId(单调递增)

  🔴 剪掉的越权合成 (user 定, 官方系统接管):
     - _triangle_dynamic (副目标相对主目标 dx_dot/横向切入 vlat)          删除
     - 车道区间量化 yRel + 安全 TTC 判定                                 删除
     - Main 宽限保持(_main_last/_main_hold_cnt) + 副目标 GONE_TIMEOUT 顶帧  删除
     - 跨帧 4m 距离合并续 ID (_prev_targets/_used_this_frame)           删除
     - dM_dot 滑动窗拟合限幅、三角 等自作融合                            删除
  ✅ 保留的唯一"最小可靠 vRel 来源" (对应用户定, 别过度):
     官方主目标因 DBC 无 REL_SPEED 字段, 我们保留各【物理通道自身距离】的滑动窗斜率
     (短窗降 0.5m/0.4244m 量化台阶) 给出该通道 vRel。只做本通道测量, 不做任何跨目标/三角/TTC。

  🔴 解码公式 (唯一解码, 原样保留 100% 真车验证; see DBC byd_han_dmev_2020):
     0x109 RADAR_MAIN_TARGET  MainDist = 0.5*b7 - 4            (主目标, 20Hz, 100%)
     池A base+1               dRel = 0.4244*b3 + 17.79         (槽连续距离)
                              AzimSub{i} = b6 - 128 (DBC(1,-128)) → 0.55°/LSB
     bus1 (CanBus.MRR), DBC byd_han_dmev_2020, 触发=0x109

【上下级接口契约】(与 card.py / interfaces.py 严格对应)
  card.py   : RD = self.RI.update_carrot(CS.vEgo, CS.aEgo, rcv_time, can_list)
  interfaces : RadarInterfaceBase.update_carrot(can_packets 版本) 调 self.update(can_packets)
               -> 对本帧 self.pts 里每个 trackId 建/续 MyTrack -> 填 aLead/jLead/yRel平滑/yvRel
               (追踪/KF/横向平滑 = 系统层工作, 雷达不再越权);
               vLead/vRel/dRel/yRel(raw)/measured 由本接口 update() 填。
  radard/controls: 消费 RD.points {measured/trackId/dRel/yRel/vRel/vLead/aRel/yvRel/aLead/jLead}

🔴 纪律: 纯本地文件; 绝不连真机 / 不 pscp / 不 push。
"""
import math
import numpy as np
from typing import List, Set
from opendbc.can import CANParser
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.structs import RadarData

# ==================== DBC / 总线 配置 ====================
_DBC_NAME = "byd_han_dmev_2020"      # 官方 DBC (含 MainDist / dRel_slot*b / AzimSub)
CAN_BUS = 1                          # 车内雷达总线 (CanBus.MRR = 1)
MAX_OBJECTS = 7                      # 当帧最大同时目标: Main + 6 池A base+1 槽
_TS_TIMEOUT = 2.0                    # 雷达数据超时(s): 超此未见 0x109 触发 -> 清 pts + canError

# ---- 消息地址 (DBC byd_han_dmev_2020, 真车验证) ----
_MAIN_MSG = 0x109                    # RADAR_MAIN_TARGET  主目标 MainDist = 0.5*b7 - 4 (20Hz)
_AZIM_MSG = 0x340                    # RADAR_340 AzimB7 (恒≈128 中心; Main 锁纵向, 不参与横向)

# 池A 只读 base+1 (2026-08-24 深挖): base+0 = 离散量化档(假目标源)跳过;
#   base+1 = 连续真实距离 + 方位(b6) = 唯一信任源。槽4/5 (0x391/395) DBC 已补 dRel_slot4b/5b+AzimSub4/5
_POOLA_ADDRS: List[int] = []                              # 6 个 base+1 地址
for _i in range(6):
    _POOLA_ADDRS.append(0x381 + _i * 4)                   # 0x381/385/389/38D/391/395
_POOLA_DREL_SIG = ['dRel_slot0b', 'dRel_slot1b', 'dRel_slot2b',
                   'dRel_slot3b', 'dRel_slot4b', 'dRel_slot5b']
_POOLA_AZIM_SIG = ['AzimSub0', 'AzimSub1', 'AzimSub2',
                   'AzimSub3', 'AzimSub4', 'AzimSub5']


def _make_radar_can_parser() -> CANParser:
  """注册 0x109(Main) + 池A base+1 六槽; 全部 20Hz, bus1. (官方 _create_radar_can_parser 对应)"""
  messages = [(_MAIN_MSG, 20)] + [(a, 20) for a in _POOLA_ADDRS]
  return CANParser(_DBC_NAME, messages, CAN_BUS)


class RadarInterface(RadarInterfaceBase):
  """BYD 唐DM 车内雷达接口 - 官方回归版
  职责 = 纯解析(CAN->当帧真实目标) + 逐物理通道持续跟踪分配 trackId;
  不做消失延迟/宽限/车道量化/TTC/跨帧距离合并/三角合成 — 那些交 CP(radard/MyTrack)。"""

  def __init__(self, CP):
    super().__init__(CP)
    self.rcp = None if getattr(CP, 'radarUnavailable', False) else _make_radar_can_parser()
    self.trigger_msg = _MAIN_MSG                 # 0x109 主目标 = 触发(节拍对)
    self.updated_messages: Set[int] = set()
    self.track_id = 0                            # trackId 单调计数 (官方 Toyota style)
    self._last_seen_ts = 0.0                     # 最近一次 0x109 触发时刻(s), 迟到判 canError
    self._dhist: dict[int, List[tuple]] = {}     # vRel 本通道距离滑动窗 [(ts, d)] (短窗降量化)
    self.pts = {}                                # 官方基类也建 pts; 这里按物理通道(addr)续存点对象

  # ---------- 主接口 (card.py 经基类 update_carrot -> 这里调用) ----------
  def update(self, can_packets):
    """官方入口: 供 RadarInterfaceBase.update_carrot 调用(它负责 MyTrack/平滑/KF fusion).
    0x109 触发才产帧; 逐物理通道 decode -> 续 pts[channel] / 消失清除 / NEW trackId."""
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_packets)
    self.updated_messages.update(vls)

    # 时间基准: 首包 nanots -> 秒
    try:
      now_s = can_packets[0][0] / 1e9
    except Exception:
      now_s = 0.0

    # 雷达超时: 超过 _TS_TIMEOUT 未见 0x109 触发 -> 报 canError 并清当帧目标(不顶假点)
    if self.trigger_msg not in self.updated_messages:
      if (now_s - self._last_seen_ts) > _TS_TIMEOUT and self._last_seen_ts > 0.0:
        rr = RadarData()
        rr.errors.canError = True
        self.pts.clear()
        rr.points = list(self.pts.values())
        return rr
      return None

    self._last_seen_ts = now_s
    ret = self._update(now_s)
    self.updated_messages.clear()
    return ret

  def _update(self, now_s: float):
    """在 0x109 触发帧上, 逐物理通道解码输出 (官方 Toyota 逐 slot 风格).
    规则 = 官方"当帧真实目标, 无顶帧": 仅在"最近一次触发以来的窗口"内出现过且有效的通道才输出;
    某通道该窗口无有效测量(NEW_TRACK/缺席/无效) -> 立即清除, 重现 = NEW trackId(单调递增).
    (Main 触发天然要求 0x109 该窗口在场; 池槽若在该雷达节拍缺席=该目标走了, 不留假点)"""
    ret = RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    chans = [self.trigger_msg] + _POOLA_ADDRS            # 物理通道: Main + 池槽
    cur_channels = set(self.updated_messages) & set(chans)

    # 1) Main (0x109): 触发即在场; 仅本帧距离有效才输出(主目标无宽限)
    if self.trigger_msg in cur_channels:
      md = self.rcp.vl[self.trigger_msg].get('MainDist', None)
      # 主目标有效距离 1.0~120m (100% 出席; 0.5m量化; <1m 几乎无帧)
      if md is not None and (md == md) and 1.0 <= float(md) <= 120.0:
        self._emit_channel(self.trigger_msg, float(md), 0.0, now_s)  # Main yRel 锁纵向=0
      else:
        self.pts.pop(self.trigger_msg, None)              # 主目标本帧无效 -> 消失
    else:
      self.pts.pop(self.trigger_msg, None)

    # 2) 池A base+1 六槽 (当帧真实目标; 空格/缺席/哨兵=无, 全部不留帧)
    for i, addr in enumerate(_POOLA_ADDRS):
      if addr not in cur_channels:
        self.pts.pop(addr, None)                          # 该槽本窗口无数据 -> 无目标
        continue
      d = self.rcp.vl[addr].get(_POOLA_DREL_SIG[i], None)
      if d is None or d != d:
        self.pts.pop(addr, None)
        continue
      d = float(d)
      # 有效距离窗 3~120m + 空闲哨兵码过滤 (dat[3]=4/247/255 -> d≈19.5/122.6/126m 假目标)
      b3 = round((d - 17.79) / 0.4244)                    # 反解 dat[3] 过滤空闲码 (真车验证)
      if not (3.0 <= d <= 120.0) or b3 in (4, 247, 255):
        self.pts.pop(addr, None)                          # 该槽此时无真实目标
        continue
      # 横向 Y (实测方位, 连续值; 不做车道量化)
      y_rel = 0.0
      az = self.rcp.vl[addr].get(_POOLA_AZIM_SIG[i], None)
      if az is not None and az == az:                     # DBC (1,-128) -> decoded = b6 - 128 (方位指数)
        o = float(az)
        # 野值窗口: b6 在 90~240 之外(即 o<-38 或 o>112) 属无可靠方位 -> 横向 0(正前方兜底)
        if -38.0 <= o <= 112.0:
          ang = math.radians(o * 0.55)                    # 0.55°/LSB (全帧标定); <0=左
          # 车载坐标: left 为 + (radard 约定), 右正左负注释下 yRel 用 -d*sin(与标定一致)
          y_rel = -d * math.sin(ang)
      self._emit_channel(addr, d, y_rel, now_s)

    ret.points = list(self.pts.values())
    return ret

  # ---------- 工具 ----------
  def _emit_channel(self, addr: int, d: float, y_rel: float, now_s: float):
    """把某物理通道本帧真实测得目标写进该通道点; 首次/重现 = NEW trackId(官方 NEW_TRACK 单调度)."""
    # vRel: 本通道自身距离短窗斜率 (唯一可靠最小来源; DBC 无 REL_SPEED; 不做三角/跨目标)
    v_rel = self._channel_vrel(addr, d, now_s)

    if addr not in self.pts:
      self.pts[addr] = RadarData.RadarPoint()
      self.pts[addr].trackId = self.track_id
      self.track_id += 1
    pt = self.pts[addr]
    pt.dRel = d
    pt.yRel = y_rel
    pt.vRel = v_rel
    pt.vLead = v_rel + self.v_ego   # 前车绝对速度 (与官方 Toyota 同式; 供 MyTrack 推 aLead)
    pt.aRel = float('nan')          # 官方 Toyota: aRel 恒 nan, 由 MyTrack 用 vLead 差分推 a_lead
    pt.yvRel = 0.0                  # 官方 Toyota 默认; 横向速度由 MyTrack 平滑得到
    pt.measured = True
    try:
      pt.aLead = 0.0
      pt.jLead = 0.0
    except Exception:
      pass

  def _channel_vrel(self, addr: int, d: float, now_s: float) -> float:
    """该物理通道自身 vRel = 短窗 dRel 斜率 (最近 ~0.45s, 降量化噪声; 本通道测量, 非合成).
    无独立 REL_SPEED 时按主目标也需一路径: 轻量斜率即可, 勿过度(不用三角/限幅级融合)."""
    hist = self._dhist.setdefault(addr, [])
    hist.append((now_s, d))
    while hist and (now_s - hist[0][0]) > 0.45:
      hist.pop(0)
    if len(hist) < 2:
      return 0.0
    if len(hist) == 2:
      dt = hist[1][0] - hist[0][0]
      if dt > 1e-3:
        return float(np.clip((hist[1][1] - hist[0][1]) / dt, -35.0, 35.0))
      return 0.0
    # 最小二乘斜率 (>=3 点)
    n = len(hist)
    st = sum(t for t, _d in hist); sv = sum(dd for _t, dd in hist)
    stv = sum(t * dd for t, dd in hist); stt = sum(t * t for t, _d in hist)
    den = n * stt - st * st
    if abs(den) < 1e-9:
      return 0.0
    return float(np.clip((n * stv - st * sv) / den, -35.0, 35.0))

  def reset(self) -> None:
    """重置雷达接口"""
    self.pts = {}
    self.updated_messages = set()
    self.track_id = 0
    self._last_seen_ts = 0.0
    self._dhist.clear()


# ==================== 冒烟测试 ====================
# 注: 需在完整 openpilot 环境跑 (真实 CANParser 解 bytes -> 需 byd_han_dmev_2020.dbc / numpy)。
# 语义无真机 CANParser 时,见 workspace 的 _byd_smoke.py 用假 CANParser 驱动 update() 验证分层。
if __name__ == "__main__":
  from opendbc.car.structs import CarParams
  inst = RadarInterface(CarParams())
  # 空闲帧 (0x109 MainDist dat[7]=255 -> 0.5*255-4=123.5>120 无效; 池槽 dat[3]=255 ->126>120 无效)
  idle = [(0x109, bytes([0, 0, 0, 0, 0, 0, 0, 255]), 1),
          (0x381, bytes([0, 0, 0, 255, 0, 0, 0, 255]), 1)]
  # 真目标帧: 0x109 dat[7]=100 -> Main=46m; 0x381 dat[3]=50 -> 0.4244*50+17.79=39.01m; 0x385 dat[3]=60->43.25m
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
  # 目标消失(空闲): 官方契约=当帧清除, 无宽限顶帧
  rd = inst.update(pkts(idle))
  print(f"目标消失帧(空闲): 目标数={len(list(rd.points))} (应0, 无宽限保持)")
  print("冒烟测试完成 (完整路径需真实 CANParser 环境; 本脚本最少验证语法/基类契约)")
