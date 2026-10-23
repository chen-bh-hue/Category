"""
HybridX 分层范畴框架（重写）

本文件只放类型和算法，相当于头文件。
具体场景在 hybridx_apply.py 里用表格填写，再由 compile_case 装配成对象。
0.1 与 0.2 是数学工具，不在应用文件里填写。
机器人与环境上的态射在第 1.2 节定义，定位设施和通信链路在第 1.3 节定义。
任务的分解与合成在第 3.1 节定义。
基元任务到机器人动作的翻译在第 4 节定义。
对照论文第二章逐步实现，旧脚本 solute.py 先不动。

已完成：
  0.2 图上的可靠性：路径积、并行通路、设施增益、通信是否仍闭合
  1–4. 状态、态射、设施与通信、任务偏序、动作翻译
  5. 任务求解：束搜索构造可行方案，自适应大邻域搜索优化行程
"""
from __future__ import annotations
import builtins
import copy
import itertools
import math
import random
from dataclasses import dataclass, field


def _indent(text: str, pad: str = '  ') -> str:
    return '\n'.join(pad + line for line in text.splitlines())


def _nums(values) -> str:
    return '(' + ', '.join(f'{v:g}' for v in values) + ')'


class Showable:
    """各组件共用的展示接口：text 返回结构文本，print 把它打出来。"""

    def text(self) -> str:
        raise NotImplementedError

    def print(self) -> None:
        builtins.print(self.text())

    def __str__(self) -> str:
        return self.text()


# ============================================================
# 0.1 态射基本数学工具
#     只提供箭头、恒等和复合。机器人与环境上的态射不在这里定义。
# ============================================================
@dataclass(frozen=True)
class Morphism(Showable):
    """
    范畴中的一个态射 f: src → tgt。
    reliability 是该态射成功的概率，取值 [0, 1]。
    """
    mid: str
    src: str
    tgt: str
    reliability: float = 1.0

    def __post_init__(self):
        if not 0.0 <= self.reliability <= 1.0:
            raise ValueError(f'态射 {self.mid} 的可靠性必须在 [0, 1] 内')

    def text(self) -> str:
        return f'{self.src} -{self.mid}[{self.reliability:.3f}]-> {self.tgt}'


def identity(obj: str) -> Morphism:
    """对象 obj 上的恒等态射，可靠性为 1。"""
    return Morphism(mid=f'id_{obj}', src=obj, tgt=obj, reliability=1.0)


def compose(f: Morphism, g: Morphism) -> Morphism:
    """
    先做 f 再做 g，得到复合 g∘f。
    可复合的条件是 f 的目标等于 g 的源。
    复合可靠性为两者之积：r(g∘f) = r(g) * r(f)。
    """
    if f.tgt != g.src:
        raise ValueError(f'不能复合：{f.mid} 的目标 {f.tgt} 与 {g.mid} 的源 {g.src} 不一致')
    return Morphism(
        mid=f'{g.mid}∘{f.mid}',
        src=f.src,
        tgt=g.tgt,
        reliability=f.reliability * g.reliability,
    )

# ============================================================
# 0.2 图分析计算工具
#     数学工具。具体的设施、链路和任务图不在这里填写，写在应用文件里。
#
#     结构：一条任务链是若干态射的复合。复合可靠性是乘积
#       r(path) = ∏ r(f_i)。
#     同一对状态之间可以有多条通路（里程计、定位设施、无线、视觉）。
#     至少一条成功时
#       r = 1 - ∏ (1 - r_j)。
#     新设施的作用是再添一条通路。它的增益是“新路成功且旧路都失败”：
#       Δr = r(new) · ∏ (1 - r(old))。
#     通信链路用同一公式。多台机器人必须同时在场时，联合状态上至少要有
#     一条未失效的通信态射，交换图才闭合；单台任务不依赖通信。
#     失效就是从通路里去掉被点名的设施或链路，再重算剩下的可靠性。
# ============================================================

def path_reliability(weights: list[float]) -> float:
    """一条链的可靠性：链上各态射成功率的乘积。空链的乘积为 1。"""
    value = 1.0
    for weight in weights:
        if not 0.0 <= weight <= 1.0:
            raise ValueError(f'可靠性 {weight} 必须在 [0, 1] 内')
        value *= weight
    return value


def parallel_reliability(weights: list[float]) -> float:
    """同一对状态之间多条通路的可靠性：至少一条成功。没有通路时为 0。"""
    if not weights:
        return 0.0
    miss = 1.0
    for weight in weights:
        if not 0.0 <= weight <= 1.0:
            raise ValueError(f'可靠性 {weight} 必须在 [0, 1] 内')
        miss *= 1.0 - weight
    return 1.0 - miss


def facility_gain(new: float, old: list[float]) -> float:
    """
    新通路带来的增益 Δr。
    它等于新路成功、原有各路都失败的概率。没有旧路时，增益就是新路本身。
    """
    if not 0.0 <= new <= 1.0:
        raise ValueError(f'可靠性 {new} 必须在 [0, 1] 内')
    return new * path_reliability([1.0 - weight for weight in old])


def odometry_reliability(distance: float, r0: float = 0.99, lam: float = 0.002) -> float:
    """
    没有定位设施时的原路径：里程计。
    距离越长，位置信念越不可靠，r = r0 · exp(-λ · d)。
    """
    if distance < 0 or not 0.0 <= r0 <= 1.0 or lam < 0:
        raise ValueError('里程计的距离、基准可靠性和衰减系数不合法')
    return max(0.0, min(1.0, r0 * math.exp(-lam * distance)))


@dataclass(frozen=True)
class SegmentEffect(Showable):
    """一段被略去的移动，加上盖住它的定位设施之后的可靠性。"""
    start: str
    end: str
    distance: float
    odometry: float
    used: tuple[str, ...]
    reliability: float
    gain: float

    def text(self) -> str:
        names = '、'.join(self.used) if self.used else '（无）'
        return (
            f'{self.start} → {self.end}  {self.distance:.2f} m\n'
            f'  里程计 {self.odometry:.3f}  设施 {names}\n'
            f'  段可靠性 {self.reliability:.3f}  增益 {self.gain:.3f}'
        )


@dataclass(frozen=True)
class CommEffect(Showable):
    """一组机器人是否还能靠剩余通信链路对齐联合状态。"""
    crew: int
    used: tuple[str, ...]
    reliability: float
    open: bool

    def text(self) -> str:
        if self.crew < 2:
            return '单台任务  不需要通信'
        names = '、'.join(self.used) if self.used else '（无剩余链路）'
        state = '闭合' if self.open else '断开'
        return f'{self.crew} 台协同  链路 {names}  可靠性 {self.reliability:.3f}  交换图{state}'


def segment_effect(
    start: str,
    end: str,
    distance: float,
    facilities: list,
    *,
    failed: tuple[str, ...] = (),
    r0: float = 0.99,
    lam: float = 0.002,
) -> SegmentEffect:
    """
    计算一段移动的可靠性。
    里程计是原路径。sites 同时包含两端的设施再各添一条感知通路。
    增益是这些设施相对“只有里程计”提高的成功率。
    """
    base = odometry_reliability(distance, r0, lam)
    used = tuple(
        item.name for item in facilities
        if item.name not in failed and item.covers(start, end)
    )
    extra = [item.reliability for item in facilities if item.name in used]
    total = parallel_reliability([base, *extra])
    return SegmentEffect(start, end, distance, base, used, total, total - base)


def comm_effect(links: list, crew: int, *, failed: tuple[str, ...] = ()) -> CommEffect:
    """
    协同任务的通信作用。
    少于两台机器人时交换图不依赖链路。
    否则，每条未失效且要求人数不超过当前人数的链路都是一条并行态射。
    一条都不剩时交换图断开，可靠性为 0。
    """
    if crew < 2:
        return CommEffect(crew, (), 1.0, True)
    used_links = [
        item for item in links
        if item.name not in failed and item.min_crew <= crew
    ]
    reliability = parallel_reliability([item.reliability for item in used_links])
    return CommEffect(
        crew,
        tuple(item.name for item in used_links),
        reliability,
        bool(used_links) and reliability > 0.0,
    )




# ============================================================
# 1.1 范畴与状态对象
#     R：机器人状态层；E：环境状态层。
# ============================================================
@dataclass
class RobotState(Showable):
    """机器人状态层的公共状态：位置、朝向、电量与信念。"""
    pos: tuple[float, float]   # 平面位置 (x, y)，单位：米
    theta: float               # 朝向角，单位：弧度
    bat: float                 # 剩余电量，取值范围 [0, 1]
    belief: dict = field(default_factory=dict)  # 对环境的信念分布

    def text(self) -> str:
        lines = [f'位置 {_nums(self.pos)}  朝向 {self.theta:.2f}  电量 {self.bat:.2f}']
        lines.append(f'信念 {self.belief or "（无）"}')
        return '\n'.join(lines)


@dataclass
class TransportRobot(RobotState):
    """运输车。在公共状态上增加装载量和累计行驶距离。"""
    full: float = 0.0          # 当前装载量（0 空载，1 满载）
    dis: float = 0.0           # 累计行驶距离

    def text(self) -> str:
        return super().text() + f'\n装载 {self.full:.2f}  里程 {self.dis:.2f}'


@dataclass
class ArmRobot(RobotState):
    """机械臂。在公共状态上增加三个关节角和夹爪开合。"""
    ang: tuple[float, float, float] = (0.0, 0.0, 0.0)
    grip: float = 0.0          # 夹爪开合：0 空，1 夹持

    def text(self) -> str:
        return super().text() + f'\n关节 {_nums(self.ang)}  夹爪 {self.grip:.2f}'


@dataclass
class Robot(Showable):
    """分类层 R 中的单个机器人对象（能力 + 可靠性参数）。"""
    rid: str
    rtype: str          # 机型：'transport' 运输车 | 'arm' 机械臂
    state: RobotState   # 实际放入 TransportRobot 或 ArmRobot
    # 能力向量：{任务类型: 能力值}，用于判断该机器人能否执行某类任务
    capability: dict[str, float] = field(default_factory=dict)
    # 可靠性参数：移动距离越长，单次动作可靠性越低
    r0: float = 0.99           # 零距离时的基准可靠性
    lam_r: float = 0.002       # 可靠性随距离的衰减系数（每米）
    lam_b: float = 0.0005      # 电量随距离的消耗系数（每米）

    def text(self) -> str:
        kind = {'transport': '运输车', 'arm': '机械臂'}.get(self.rtype, self.rtype)
        cap = '  '.join(f'{k}:{v:.2f}' for k, v in self.capability.items()) or '（无）'
        return (
            f'{self.rid}  {kind}\n'
            f'  状态\n{_indent(self.state.text(), "    ")}\n'
            f'  能力  {cap}\n'
            f'  可靠性  r0={self.r0:.2f}  λr={self.lam_r:g}  λb={self.lam_b:g}'
        )


@dataclass
class EnvState(Showable):
    """环境状态层 E 中的单个状态对象。"""
    occ: dict = field(default_factory=dict)      # 栅格占用：(x,y,z) -> 0/1
    mat: dict = field(default_factory=dict)      # 栅格材质：(x,y,z) -> 材料名
    sigma: dict = field(default_factory=dict)    # 结构化程度向量，用于估计交接可靠性

    def text(self) -> str:
        def body(data: dict) -> str:
            if not data:
                return '（无）'
            return '  '.join(f'{k}:{v}' for k, v in data.items())

        return f'环境\n  占用  {body(self.occ)}\n  材质  {body(self.mat)}\n  结构化  {body(self.sigma)}'


# ============================================================
# 1.2 机器人态射
#     arm 和 trans 通用态射：
#       a1(bat_decline_num)：耗电态射，作用在 Robot 上使其电量下降。
#       a2((src.x, src.y), (dst.x, dst.y))：自由移动态射，表征二维空间内的移动。
#           系统没有无人机，所以用二维简化。
#       a3：信念转移态射，表示机器人对环境的理解发生变化。
#           这个态射较为抽象，不做具体公式化定义，后面可以从通讯角度使用它。
#     arm 的专用态射：
#       m1(src_angle, dst_angle)：机械臂关节角变化态射。
#       m2：抓取/放下态射，专门表征夹爪动作。
#     trans 的专用态射：
#       t1(dis_delta_num)：一维移动态射，表征车辆在运输道路上的移动，变化 dis。
#       t2(load_num)：装载卸载态射，表征车厢当前占用量的变化，正为装载，负为卸载。
# ============================================================

def _close(a: float, b: float, eps: float = 1e-6) -> bool:
    return abs(a - b) <= eps


def _same_point(p: tuple[float, ...], q: tuple[float, ...], eps: float = 1e-6) -> bool:
    return len(p) == len(q) and all(_close(x, y, eps) for x, y in zip(p, q))


def _arm_state(robot: Robot) -> ArmRobot:
    if robot.rtype != 'arm' or not isinstance(robot.state, ArmRobot):
        raise TypeError(f'{robot.rid} 不是机械臂，不能使用 m1/m2')
    return robot.state


def _transport_state(robot: Robot) -> TransportRobot:
    if robot.rtype != 'transport' or not isinstance(robot.state, TransportRobot):
        raise TypeError(f'{robot.rid} 不是运输车，不能使用 t1/t2')
    return robot.state


@dataclass(frozen=True)
class A1(Showable):
    """a1：耗电。电量减少 bat_decline_num。"""
    bat_decline_num: float

    def __post_init__(self):
        if self.bat_decline_num < 0:
            raise ValueError('a1 的耗电量不能为负')

    def apply(self, robot: Robot) -> None:
        nxt = robot.state.bat - self.bat_decline_num
        if nxt < 0:
            raise ValueError(f'{robot.rid} 电量不足以执行 a1')
        robot.state.bat = nxt

    def text(self) -> str:
        return f'a1  电量 -{self.bat_decline_num:g}'


@dataclass(frozen=True)
class A2(Showable):
    """a2：自由移动。机器人当前应在 src，执行后位置变为 dst。"""
    src: tuple[float, float]
    dst: tuple[float, float]

    def apply(self, robot: Robot) -> None:
        if not _same_point(robot.state.pos, self.src):
            raise ValueError(f'{robot.rid} 不在 a2 的起点 {self.src}')
        robot.state.pos = self.dst

    def text(self) -> str:
        return f'a2  {_nums(self.src)} → {_nums(self.dst)}'


@dataclass
class A3(Showable):
    """
    a3：信念转移。
    框架不计算新信念。content 由 1.3 的通信链路填入，执行时并入当前信念。
    """
    content: dict = field(default_factory=dict)

    def apply(self, robot: Robot) -> None:
        robot.state.belief.update(self.content)

    def text(self) -> str:
        return f'a3  信念 ← {self.content or "（待通讯填入）"}'


@dataclass(frozen=True)
class M1(Showable):
    """m1：关节角从 src_angle 变到 dst_angle。"""
    src_angle: tuple[float, ...]
    dst_angle: tuple[float, ...]

    def __post_init__(self):
        if len(self.src_angle) != len(self.dst_angle) or not self.src_angle:
            raise ValueError('m1 的源关节角与目标关节角维度必须相同且非空')

    def apply(self, robot: Robot) -> None:
        state = _arm_state(robot)
        if not _same_point(state.ang, self.src_angle):
            raise ValueError(f'{robot.rid} 的关节角不是 m1 的起点')
        state.ang = self.dst_angle

    def text(self) -> str:
        return f'm1  {_nums(self.src_angle)} → {_nums(self.dst_angle)}'


@dataclass(frozen=True)
class M2(Showable):
    """m2：夹爪动作。grasp 闭合夹持，release 张开放下。"""
    act: str

    def __post_init__(self):
        if self.act not in ('grasp', 'release'):
            raise ValueError("m2 的动作只能是 grasp 或 release")

    def apply(self, robot: Robot) -> None:
        state = _arm_state(robot)
        state.grip = 1.0 if self.act == 'grasp' else 0.0

    def text(self) -> str:
        word = '抓取' if self.act == 'grasp' else '放下'
        return f'm2  {word}'


@dataclass(frozen=True)
class T1(Showable):
    """t1：沿运输道路的一维移动，累计里程增加 dis_delta_num。"""
    dis_delta_num: float

    def __post_init__(self):
        if self.dis_delta_num < 0:
            raise ValueError('t1 的里程增量不能为负')

    def apply(self, robot: Robot) -> None:
        state = _transport_state(robot)
        state.dis += self.dis_delta_num

    def text(self) -> str:
        return f't1  里程 +{self.dis_delta_num:g}'


@dataclass(frozen=True)
class T2(Showable):
    """t2：车厢占用量变化。load_num 为正是装载，为负是卸载。"""
    load_num: float

    def apply(self, robot: Robot) -> None:
        state = _transport_state(robot)
        nxt = state.full + self.load_num
        if not 0.0 <= nxt <= 1.0:
            raise ValueError(f'{robot.rid} 执行 t2 后的装载量 {nxt} 超出 [0, 1]')
        state.full = nxt

    def text(self) -> str:
        sign = '+' if self.load_num >= 0 else ''
        return f't2  装载 {sign}{self.load_num:g}'


# ============================================================
# 1.3 定位设施与通信链路
#     二者都不是任务节点，而是挂在已有任务边上的并行态射。
#
#     定位设施是感知函子 F: E → R 的一条具体通路。
#     它不搬运物体。机器人经过一段被略去的移动时，设施把环境状态
#     写成机器人的位置信念，和里程计并列。sites 写出它盖住的落点；
#     一段移动的两端都在 sites 里，这条设施才作用在该段上。
#     kind 只说明设施是什么：camera 固定相机，uwb 锚点，tag 视觉标签。
#
#     通信链路是群体状态上的态射
#       a: (s1, …, sn) → (s1', …, sn')。
#     它作用在必须至少 min_crew 台机器人同时在场的任务上，让这些车
#     对齐联合信念。同一任务上的多条链路互为备用：无线失效时，
#     视觉链路仍连接同一对联合状态，交换图保持闭合。
#     链路送达的内容随后由 a3 并入每一台机器人自己的信念。
# ============================================================

@dataclass(frozen=True)
class Facility(Showable):
    """一条定位通路。可靠性是这条感知通路本身的成功率。"""
    name: str
    sites: tuple[str, ...]
    reliability: float
    kind: str = 'camera'

    def __post_init__(self):
        if self.kind not in ('camera', 'uwb', 'tag'):
            raise ValueError(f'{self.name} 的设施种类只能是 camera、uwb 或 tag')
        if not 0.0 <= self.reliability <= 1.0:
            raise ValueError(f'{self.name} 的可靠性必须在 [0, 1] 内')
        if len(self.sites) < 2:
            raise ValueError(f'{self.name} 至少要盖住两个落点，才能作用在一段移动上')
        if len(set(self.sites)) != len(self.sites):
            raise ValueError(f'{self.name} 的落点重复')

    def covers(self, start: str, end: str) -> bool:
        """两端落点都在覆盖范围内时，这段移动可以用这条感知通路。"""
        return start in self.sites and end in self.sites and start != end

    def text(self) -> str:
        kind = {'camera': '相机', 'uwb': '锚点', 'tag': '视觉标签'}.get(self.kind, self.kind)
        where = '、'.join(self.sites)
        return f'{self.name}  {kind}  覆盖 {where}  可靠性 {self.reliability:.3f}'


@dataclass(frozen=True)
class CommLink(Showable):
    """一条通信通路。min_crew 是它开始起作用的最少同时在场机器人数。"""
    name: str
    reliability: float
    min_crew: int = 2

    def __post_init__(self):
        if not 0.0 <= self.reliability <= 1.0:
            raise ValueError(f'{self.name} 的可靠性必须在 [0, 1] 内')
        if self.min_crew < 2:
            raise ValueError(f'{self.name} 的通信至少作用于两台机器人')

    def text(self) -> str:
        return f'{self.name}  通信  至少 {self.min_crew} 台  可靠性 {self.reliability:.3f}'


# ============================================================
# 2. 任务节点
#    一个节点描述一件任务要谁来做、动手前必须成立什么、做完后留下什么。
# ============================================================

@dataclass
class RobotRole(Showable):
    """该任务需要的一台机器人。同机型要多台时，用 slot 把它们区分开。"""
    rtype: str                 # 'transport' 运输车 | 'arm' 机械臂
    capability: float          # 该机型完成此任务的能力，取值 [0, 1]
    slot: str = ''             # 同机型的区分名；空字符串表示这一型只有一台

    def __post_init__(self):
        if not 0.0 <= self.capability <= 1.0:
            raise ValueError(f'{self.rtype} 在任务上的能力必须在 [0, 1] 内')

    def key(self) -> str:
        """前置条件、后置效果和分配表都用这个名字指向这一台。"""
        return f'{self.rtype}:{self.slot}' if self.slot else self.rtype

    def text(self) -> str:
        kind = {'transport': '运输车', 'arm': '机械臂'}.get(self.rtype, self.rtype)
        mark = self.slot if self.slot else ''
        return f'{kind}{mark}  能力 {self.capability:.2f}'


@dataclass
class PreCondition(Showable):
    """
    一条前置条件。who 里写参与机型：
      一个机型表示该车自身的状态，两个机型表示二者之间的关系。
    key 是被约束的量，op 是比较符，value 是阈值。
    例如运输车空载、双方电量足够、机械臂与运输车距离足够近：
      PreCondition(('transport',), 'full', '<=', 0.0)
      PreCondition(('transport',), 'bat', '>=', 0.3)
      PreCondition(('arm',), 'bat', '>=', 0.3)
      PreCondition(('arm', 'transport'), 'dist', '<=', 1.0)
    """
    who: tuple[str, ...]
    key: str
    op: str
    value: float

    def __post_init__(self):
        if not 1 <= len(self.who) <= 2:
            raise ValueError('前置条件只能约束一台机器人，或两类机器人之间的关系')
        if self.op not in ('<', '<=', '>', '>=', '=='):
            raise ValueError(f'不支持的比较符: {self.op}')

    def text(self) -> str:
        if len(self.who) == 1:
            return f'{self.who[0]}.{self.key} {self.op} {self.value:g}'
        return f'{self.key}({self.who[0]}, {self.who[1]}) {self.op} {self.value:g}'


@dataclass
class PostEffect(Showable):
    """
    一条后置效果。target 指明改谁：
      ('robot', 机型) 改参与该任务的那台机器人
      ('env',)        改环境
    mode='set' 覆盖，'add' 累加（电量消耗），'merge' 合并字典（信念）。
    例如装载结束运输车满载、机械臂夹爪空、电量下降、信念更新；
    放置结束环境材质变化；配送结束运输车到达新位置。
    """
    target: tuple[str, ...]
    key: str
    value: object
    mode: str = 'set'

    def __post_init__(self):
        if self.mode not in ('set', 'add', 'merge'):
            raise ValueError(f'不支持的效果方式: {self.mode}')
        if not self.target or self.target[0] not in ('robot', 'env'):
            raise ValueError('后置效果的对象只能是 robot 或 env')
        if self.target[0] == 'robot' and len(self.target) != 2:
            raise ValueError('机器人效果需写成 (\'robot\', 机型)')
        if self.target[0] == 'env' and len(self.target) != 1:
            raise ValueError('环境效果需写成 (\'env\',)')

    def text(self) -> str:
        if self.target[0] == 'env':
            who = '环境'
        else:
            who = {'transport': '运输车', 'arm': '机械臂'}.get(self.target[1], self.target[1])
        if self.mode == 'add':
            return f'{who}.{self.key} += {self.value}'
        if self.mode == 'merge':
            return f'{who}.{self.key} ∪ {self.value}'
        return f'{who}.{self.key} = {self.value}'


@dataclass(frozen=True)
class Dependency(Showable):
    """偏序前驱：本任务开始前，ref 所指的任务必须已经完成。"""
    ref: str
    by: str = 'id'             # 'id' 按任务 id；'type' 按任务类型

    def __post_init__(self):
        if self.by not in ('id', 'type'):
            raise ValueError('依赖只能按 id 或 type 指向另一个任务')

    def text(self) -> str:
        kind = '编号' if self.by == 'id' else '类型'
        return f'{kind} {self.ref}'


@dataclass
class TaskNode(Showable):
    """一件任务：类型、编号、参与机型、前置条件、后置效果、偏序依赖。"""
    tid: str
    ttype: str
    roles: list[RobotRole] = field(default_factory=list)
    pre: list[PreCondition] = field(default_factory=list)
    post: list[PostEffect] = field(default_factory=list)
    deps: list[Dependency] = field(default_factory=list)
    reliability: float = 1.0   # 这一步骤本身的成功率；大物体转运会把它降低

    def __post_init__(self):
        if not 0.0 <= self.reliability <= 1.0:
            raise ValueError(f'任务 {self.tid} 的可靠性必须在 [0, 1] 内')
        keys = [role.key() for role in self.roles]
        if len(keys) != len(set(keys)):
            raise ValueError(f'任务 {self.tid} 的角色标识重复')
        known = set(keys)
        for cond in self.pre:
            missing = [name for name in cond.who if name not in known]
            if missing:
                raise ValueError(f'任务 {self.tid} 的前置条件用到了未参与的机型: {missing}')
        for effect in self.post:
            if effect.target[0] == 'robot' and effect.target[1] not in known:
                raise ValueError(f'任务 {self.tid} 的后置效果用到了未参与的机型: {effect.target[1]}')

    def text(self) -> str:
        def block(title: str, items: list) -> list[str]:
            lines = [f'  {title}']
            if not items:
                lines.append('    （无）')
            else:
                lines.extend(f'    {item.text()}' for item in items)
            return lines

        lines = [f'{self.tid}  [{self.ttype}]  可靠性 {self.reliability:.2f}']
        lines.extend(block('角色', self.roles))
        lines.extend(block('前置', self.pre))
        lines.extend(block('后置', self.post))
        lines.extend(block('依赖', self.deps))
        return '\n'.join(lines)


# ============================================================
# 3. 任务范畴
# ============================================================

# ============================================================
# 3.1 生成图（operad）
#     第 2 节里写明的任务是基元。
#     一个复杂任务对应一张有向无环图：节点是基元（或尚未展开的子任务），
#     边 u→v 表示 u 必须先于 v。没有边相连的节点互不依赖，可以并行。
#     对应论文中的 operad。外层依赖只接到子图的入口和出口，不把所有基元两两连上。
#
#     移动种类多，定义生成树时全部略去，不作为节点：
#       move       平面移动
#       move_ang   机械臂姿态变化
#       deliver    一维道路上的长途运输
#       nav        二维空间内的导航运输
#       roadtrans  deliver 的旧称，同样略去
#     分析可靠性、效率时，相邻两个基元之间的移动距离由
#     前一任务的终位置和后一任务的始位置计算，不在树上单独存放。
#
#     carve 采集原料：
#       move → dig → move
#       图中只留下 dig
#     长途转运：
#       load → move → deliver → move → unload
#       图中只留下 load → unload
#     避障运输：
#       load → move → nav → move → unload
#       图中只留下 load → unload
#     assemble 拼装：
#       move → move_ang → grasp → move_ang → move → move_ang → place
#       图中只留下 grasp → place
# ============================================================

# 这些类型是移动，生成树里不保留
MOVE_TYPES = frozenset({'move', 'move_ang', 'deliver', 'nav', 'roadtrans'})


def move_distance(end_pos: tuple[float, float], start_pos: tuple[float, float]) -> float:
    """前一基元任务的终位置到后一基元任务的始位置的移动距离。"""
    return math.hypot(start_pos[0] - end_pos[0], start_pos[1] - end_pos[1])


def _assert_dag(names: list[str], edges: tuple[tuple[str, str], ...]) -> None:
    """边是「先 → 后」。有环则拒绝。"""
    if len(names) != len(set(names)):
        raise ValueError('图中存在重复节点')
    succ = {name: [] for name in names}
    indeg = {name: 0 for name in names}
    for src, dst in edges:
        if src not in indeg or dst not in indeg:
            raise ValueError(f'边 {src} → {dst} 指向了图外的节点')
        succ[src].append(dst)
        indeg[dst] += 1
    ready = [name for name in names if indeg[name] == 0]
    seen = 0
    while ready:
        cur = ready.pop()
        seen += 1
        for nxt in succ[cur]:
            indeg[nxt] -= 1
            if indeg[nxt] == 0:
                ready.append(nxt)
    if seen != len(names):
        raise ValueError('依赖图中存在环')


def _labels(keys: list[str]) -> dict[str, str]:
    """同一名字出现多次时加上 #序号。"""
    total: dict[str, int] = {}
    for key in keys:
        total[key] = total.get(key, 0) + 1
    seen: dict[str, int] = {}
    out = {}
    for i, key in enumerate(keys):
        seen[key] = seen.get(key, 0) + 1
        out[i] = key if total[key] == 1 else f'{key}#{seen[key]}'
    return out


def _dag_lines(labels: dict[str, str], edges: list[tuple[str, str]]) -> list[str]:
    """把有向无环图收成若干条执行链。汇合点会同时出现在入边的链尾和出边的链头。"""
    pred: dict[str, list[str]] = {nid: [] for nid in labels}
    succ: dict[str, list[str]] = {nid: [] for nid in labels}
    for src, dst in edges:
        succ[src].append(dst)
        pred[dst].append(src)
    starts = []
    for nid in labels:
        incoming = pred[nid]
        if len(incoming) != 1 or len(succ[incoming[0]]) != 1:
            starts.append(nid)
    paths: list[list[str]] = []
    for nid in starts:
        path = [nid]
        cur = nid
        while len(succ[cur]) == 1:
            nxt = succ[cur][0]
            if len(pred[nxt]) != 1:
                path.append(nxt)
                break
            path.append(nxt)
            cur = nxt
        paths.append(path)
    shown = {nid for path in paths if len(path) > 1 for nid in path}
    lines = []
    for path in paths:
        if len(path) == 1 and path[0] in shown:
            continue
        lines.append(' → '.join(labels[nid] for nid in path))
    return lines or ['（无节点）']


@dataclass(frozen=True)
class GenNode:
    """生成图中的一个基元任务。"""
    nid: str
    ttype: str
    task: TaskNode | None = None


@dataclass(frozen=True)
class GenGraph(Showable):
    """
    复杂任务展开后的有向无环图。
    节点全是基元。边 earlier → later 表示 earlier 必须先完成。
    两条没有路径相连的基元互不依赖。
    """
    ttype: str
    nodes: tuple[GenNode, ...]
    edges: tuple[tuple[str, str], ...]

    def __post_init__(self):
        _assert_dag([node.nid for node in self.nodes], self.edges)

    def sources(self) -> tuple[GenNode, ...]:
        pointed = {dst for _, dst in self.edges}
        return tuple(node for node in self.nodes if node.nid not in pointed)

    def sinks(self) -> tuple[GenNode, ...]:
        pointing = {src for src, _ in self.edges}
        return tuple(node for node in self.nodes if node.nid not in pointing)

    def text(self) -> str:
        named = _labels([node.ttype for node in self.nodes])
        labels = {node.nid: named[i] for i, node in enumerate(self.nodes)}
        lines = [f'{self.ttype} ⇒']
        lines.extend(f'  {line}' for line in _dag_lines(labels, list(self.edges)))
        return '\n'.join(lines)


def gap_distances(graph: GenGraph, end_pos, start_pos) -> list[tuple[str, str, float]]:
    """
    只对有依赖边的相邻基元计算被略去的移动距离。
    没有边的两对基元不产生这段移动。
    end_pos(节点) 是前一任务的终位置，start_pos(节点) 是后一任务的始位置。
    """
    by_id = {node.nid: node for node in graph.nodes}
    return [
        (src, dst, move_distance(end_pos(by_id[src]), start_pos(by_id[dst])))
        for src, dst in graph.edges
    ]


class TaskOperad(Showable):
    """任务 operad：登记基元和依赖边，并展开成基元上的有向无环图。"""

    def __init__(self):
        self.primitives: set[str] = set()
        # 输出任务 → (去掉移动后的输入, 这些输入之间的直接依赖下标)
        self.generators: dict[str, tuple[tuple[str, ...], tuple[tuple[int, int], ...]]] = {}

    def add_primitive(self, ttype: str) -> None:
        """登记一个第 2 节里的基元任务类型。"""
        if ttype in MOVE_TYPES:
            raise ValueError(f'{ttype} 是移动，不能作为基元任务')
        if ttype in self.generators:
            raise ValueError(f'{ttype} 已是复杂任务')
        self.primitives.add(ttype)

    def add_generator(self, output: str, steps: list[str], deps: list[tuple[int, int]] | None = None) -> None:
        """
        登记一张生成图。steps 里的移动会被丢掉。
        deps 是去掉移动之后的下标对 (i, j)，表示第 i 个剩余任务必须先于第 j 个。
        不传 deps 时，只把相邻的剩余任务连成一条链，不相邻的不直接依赖。
        """
        if output in MOVE_TYPES:
            raise ValueError(f'{output} 是移动，不能作为复杂任务')
        if output in self.primitives:
            raise ValueError(f'{output} 已是基元任务')
        if output in self.generators:
            raise ValueError(f'{output} 的生成规则已存在')
        kept = tuple(step for step in steps if step not in MOVE_TYPES)
        if not kept:
            raise ValueError(f'{output} 去掉移动后没有剩余任务')
        if deps is None:
            dep_pairs = tuple((i, i + 1) for i in range(len(kept) - 1))
        else:
            dep_pairs = tuple(deps)
            for i, j in dep_pairs:
                if not 0 <= i < len(kept) or not 0 <= j < len(kept):
                    raise ValueError(f'{output} 的依赖下标超出剩余任务')
        self.generators[output] = (kept, dep_pairs)

    def decompose(self, ttype: str, tasks: dict[str, TaskNode] | None = None) -> GenGraph:
        """把复杂任务展开成基元上的有向无环图。tasks 用任务类型索引第 2 节的基元。"""
        return self._expand(ttype, ttype, {} if tasks is None else tasks)

    def _expand(self, ttype: str, nid: str, tasks: dict[str, TaskNode]) -> GenGraph:
        if ttype in self.primitives:
            node = GenNode(nid, ttype, tasks.get(ttype))
            return GenGraph(ttype, (node,), ())
        found = self.generators.get(ttype)
        if found is None:
            raise KeyError(f'未定义的任务: {ttype}')
        inputs, dep_pairs = found
        parts = tuple(
            self._expand(step, f'{nid}/{i}:{step}', tasks)
            for i, step in enumerate(inputs)
        )
        nodes = tuple(node for part in parts for node in part.nodes)
        edges = [edge for part in parts for edge in part.edges]
        for i, j in dep_pairs:
            for src in parts[i].sinks():
                for dst in parts[j].sources():
                    edges.append((src.nid, dst.nid))
        return GenGraph(ttype, nodes, tuple(edges))

    def text(self) -> str:
        prim = '  '.join(sorted(self.primitives)) or '（无）'
        lines = [f'基元  {prim}', '生成图']
        if not self.generators:
            lines.append('  （无）')
        for name, (steps, deps) in self.generators.items():
            if not deps:
                lines.append(f'  {name} ⇒ {steps[0]}')
                continue
            labels = {i: step for i, step in enumerate(steps)}
            shown = '    '.join(f'{steps[i]} → {steps[j]}' for i, j in deps)
            lines.append(f'  {name} ⇒ {shown}')
            del labels
        return '\n'.join(lines)


# ============================================================
# 4. 任务翻译
#    任务名映射到一张执行顺序的有向无环图，任务名本身不是图中的节点。
#    边 u→v 表示 u 做完才能做 v。每个主动作再分叉到自己的 a1（耗电）。
#    需要感知时，把 a3 作为主链上的一个节点插进去。
#    R→E 把动作结果写成环境变化，一般留空。
#
#    dig（arm）：
#      a2 → m1 → a3 → m2 → m1 → a2，每步分叉 a1
#    load（transport + arm）：
#      tr: t2 → a2 → t1 ← m2 ← m1 :ar
#    unload（transport + arm）：
#      tr: a2 → t2 → t1 ← m2 ← m1 ← a3 :ar，随后 t1 → m1 → m2（arm）
# ============================================================

_ROLE_TAG = {'transport': 'tr', 'arm': 'ar'}
_ACT_OPS = {
    'arm': frozenset({'a1', 'a2', 'a3', 'm1', 'm2'}),
    'transport': frozenset({'a1', 'a2', 'a3', 't1', 't2'}),
}


@dataclass(frozen=True)
class ActNode:
    """执行图中的一个机器人动作。"""
    nid: str
    op: str
    role: str


@dataclass(frozen=True)
class ActGraph(Showable):
    """执行顺序的有向无环图。指向 a1 的边是耗电分叉，其余边是主动作的先后。"""
    nodes: tuple[ActNode, ...]
    edges: tuple[tuple[str, str], ...]

    def __post_init__(self):
        _assert_dag([node.nid for node in self.nodes], self.edges)
        by_id = {node.nid: node for node in self.nodes}
        for node in self.nodes:
            allowed = _ACT_OPS.get(node.role)
            if allowed is None or node.op not in allowed:
                raise ValueError(f'{node.role} 不能执行 {node.op}')
        for src, dst in self.edges:
            if src not in by_id or dst not in by_id:
                raise ValueError('执行图的边指向了不存在的动作')

    def text(self) -> str:
        by_id = {node.nid: node for node in self.nodes}
        keys = [f'{_ROLE_TAG[node.role]}.{node.op}' for node in self.nodes]
        named = _labels(keys)
        labels = {node.nid: named[i] for i, node in enumerate(self.nodes)}
        forks = {node.nid for node in self.nodes if node.op == 'a1'}
        main_edges = [(src, dst) for src, dst in self.edges if dst not in forks]
        main_ids = {node.nid: labels[node.nid] for node in self.nodes if node.nid not in forks}
        lines = list(_dag_lines(main_ids, main_edges))
        fork_edges = [(src, dst) for src, dst in self.edges if dst in forks]
        if fork_edges:
            lines.append('分叉')
            lines.extend(f'  {labels[src]} → {labels[dst]}' for src, dst in fork_edges)
        return '\n'.join(lines)


class ActBuilder:
    """按执行顺序往一张有向图里加动作。每个主动作自动分叉到一个 a1。"""

    def __init__(self):
        self.nodes: list[ActNode] = []
        self.edges: list[tuple[str, str]] = []
        self._n = 0

    def node(self, role: str, op: str) -> str:
        allowed = _ACT_OPS.get(role)
        if allowed is None or op not in allowed:
            raise ValueError(f'{role} 不能执行 {op}')
        self._n += 1
        nid = f'{role}:{op}:{self._n}'
        self.nodes.append(ActNode(nid, op, role))
        if op != 'a1':
            self._n += 1
            fork = f'{role}:a1:{self._n}'
            self.nodes.append(ActNode(fork, 'a1', role))
            self.edges.append((nid, fork))
        return nid

    def seq(self, role: str, ops: tuple[str, ...]) -> list[str]:
        ids = [self.node(role, op) for op in ops]
        for src, dst in zip(ids, ids[1:]):
            self.edges.append((src, dst))
        return ids

    def link(self, src: str, dst: str) -> None:
        """增加一条先后边：src 完成之后才能做 dst。"""
        self.edges.append((src, dst))

    def graph(self) -> ActGraph:
        return ActGraph(tuple(self.nodes), tuple(self.edges))


@dataclass(frozen=True)
class TaskTranslation(Showable):
    """任务名到执行图的一条映射。图里没有以任务名为根的节点。"""
    ttype: str
    graph: ActGraph
    env: tuple[PostEffect, ...] = ()

    def __post_init__(self):
        for effect in self.env:
            if effect.target[0] != 'env':
                raise ValueError('R→E 只记录环境变化')

    def roles(self) -> set[str]:
        return {node.role for node in self.graph.nodes}

    def text(self) -> str:
        lines = [f'{self.ttype} ⇒']
        lines.extend(f'  {line}' for line in self.graph.text().splitlines())
        if self.env:
            lines.append('  R→E')
            lines.extend(f'    {effect.text()}' for effect in self.env)
        else:
            lines.append('  R→E  （不转化）')
        return '\n'.join(lines)


class TaskTranslator(Showable):
    """基元任务类型 → 执行有向图。翻译时核对车型是否与任务一致。"""

    def __init__(self):
        self.table: dict[str, TaskTranslation] = {}

    def add(self, translation: TaskTranslation) -> None:
        if translation.ttype in self.table:
            raise ValueError(f'{translation.ttype} 的翻译已存在')
        self.table[translation.ttype] = translation

    def translate(self, task: TaskNode) -> TaskTranslation:
        translation = self.table.get(task.ttype)
        if translation is None:
            raise KeyError(f'基元任务 {task.ttype} 还没有翻译')
        have = {role.rtype for role in task.roles}
        need = translation.roles()
        if have != need:
            raise ValueError(f'{task.ttype} 的车型是 {sorted(have)}，翻译用的是 {sorted(need)}')
        return translation

    def text(self) -> str:
        if not self.table:
            return '翻译  （无）'
        return '\n\n'.join(item.text() for item in self.table.values())


# ============================================================
# 5. 任务求解
#    输入是第 3.1 节展开后的基元依赖图，以及第 1.1 节的机器人和环境。
#    一个可行方案给每个基元派一台符合车型与能力的机器人，并给出依赖图的一个线性延伸。
#    执行前先补上被略去的接近移动：要改位置的机器人先走到目标点；
#    距离约束尚未满足时，参与的机器人汇合到中点。电量不够则该分支不可行。
#    随后检查前置条件，再写入后置效果。
#
#    搜索：束搜索。每一步只扩展依赖已满足的基元，每层保留行程最短的若干个部分方案。
#    每类机器人只尝试有限个候选，避免机器人数量增加后组合爆炸。
#    优化：自适应大邻域搜索。破坏一段分配，再用贪心或后悔值插入修复，
#    模拟退火决定是否接受更差的方案，从而跳出束搜索留下的候选。
#    优化目标是总接近行程。束宽、候选数和迭代次数都可以按规模调大。
# ============================================================

class _Reject(Exception):
    """当前分配或顺序不可行。"""


def _compare(left: float, op: str, right: float) -> bool:
    if op == '<':
        return left < right
    if op == '<=':
        return left <= right
    if op == '>':
        return left > right
    if op == '>=':
        return left >= right
    return abs(left - right) <= 1e-9


_ROLE_NAME = {'transport': '运输车', 'arm': '机械臂'}


def _crew_name(key: str) -> str:
    rtype, _, slot = key.partition(':')
    return _ROLE_NAME.get(rtype, rtype) + slot


@dataclass
class TaskPlan(Showable):
    """一个任务方案：执行顺序、每个基元的机器人分配，以及接近行程。"""
    order: tuple[str, ...]
    assign: dict[str, dict[str, str]]
    travel: float
    feasible: bool = True
    labels: dict[str, str] = field(default_factory=dict)

    def text(self) -> str:
        if not self.feasible:
            return '任务方案  （无可行解）'
        lines = [f'任务方案  行程 {self.travel:.2f}']
        for index, nid in enumerate(self.order, start=1):
            crew = '  '.join(
                f'{_crew_name(key)} {rid}' for key, rid in self.assign[nid].items()
            )
            lines.append(f'  {index}. {self.labels.get(nid, nid)}  {crew}')
        return '\n'.join(lines)


class TaskSolver(Showable):
    """用束搜索构造可行方案，再用自适应大邻域搜索缩短接近行程。"""

    def __init__(
        self,
        robots: dict[str, Robot],
        env: EnvState,
        graph: GenGraph,
        *,
        beam_width: int = 8,
        robot_limit: int = 4,
        iters: int = 120,
        destroy_ratio: float = 0.3,
        T0: float = 8.0,
        alpha: float = 0.97,
        seed: int = 0,
    ):
        if beam_width < 1 or robot_limit < 1 or iters < 1:
            raise ValueError('束宽、候选数和迭代次数至少为 1')
        self.graph = graph
        self.beam_width = beam_width
        self.robot_limit = robot_limit
        self.iters = iters
        self.destroy_ratio = destroy_ratio
        self.T0 = T0
        self.alpha = alpha
        self.rng = random.Random(seed)
        self.robots = {rid: copy.deepcopy(robot) for rid, robot in robots.items()}
        self.env = copy.deepcopy(env)
        self.by_id = {node.nid: node for node in graph.nodes}
        for node in graph.nodes:
            if node.task is None:
                raise ValueError(f'基元 {node.nid} 没有对应的任务定义')
        self.preds = {node.nid: [] for node in graph.nodes}
        self.succs = {node.nid: [] for node in graph.nodes}
        for src, dst in graph.edges:
            self.preds[dst].append(src)
            self.succs[src].append(dst)
        self._add_task_deps()
        names = [node.nid for node in graph.nodes]
        _assert_dag(names, tuple((src, dst) for dst in names for src in self.preds[dst]))
        self.best: TaskPlan | None = None

    def _add_task_deps(self) -> None:
        """把第 2 节写在任务上的偏序也补进前驱。编号指向 tid，类型指向全部同型基元。"""
        by_tid: dict[str, list[str]] = {}
        by_type: dict[str, list[str]] = {}
        for node in self.graph.nodes:
            task = node.task
            assert task is not None
            by_tid.setdefault(task.tid, []).append(node.nid)
            by_type.setdefault(task.ttype, []).append(node.nid)
        for node in self.graph.nodes:
            task = node.task
            assert task is not None
            for dep in task.deps:
                if dep.by == 'id':
                    sources = by_tid.get(dep.ref, [])
                    if not sources:
                        raise ValueError(f'{task.tid} 依赖的任务编号 {dep.ref} 不在生成图中')
                else:
                    sources = by_type.get(dep.ref, [])
                    if not sources:
                        raise ValueError(f'{task.tid} 依赖的任务类型 {dep.ref} 不在生成图中')
                for src in sources:
                    if src != node.nid and src not in self.preds[node.nid]:
                        self.preds[node.nid].append(src)
                        self.succs[src].append(node.nid)

    def _fresh(self) -> tuple[dict[str, Robot], EnvState]:
        return copy.deepcopy(self.robots), copy.deepcopy(self.env)

    def _eligible(self, task: TaskNode, role: RobotRole) -> list[Robot]:
        need = role.capability
        return [
            robot for robot in self.robots.values()
            if robot.rtype == role.rtype and robot.capability.get(task.ttype, 0.0) >= need
        ]

    def _destination(self, task: TaskNode, rtype: str):
        for effect in task.post:
            if effect.target == ('robot', rtype) and effect.key == 'pos' and effect.mode == 'set':
                return effect.value
        return None

    def _candidates(
        self,
        task: TaskNode,
        role: RobotRole,
        robots: dict[str, Robot],
        partners: list[str],
        incumbent: str | None,
    ) -> list[Robot]:
        """保留最近的一半候选，再从其余合格机器人里随机补足，避免总是同一批车。"""
        pool = [robot for robot in self._eligible(task, role) if robot.rid not in partners]
        dest = self._destination(task, role.key())
        if dest is None and partners:
            dest = robots[partners[0]].state.pos

        def rank(robot: Robot):
            dist = 0.0 if dest is None else move_distance(robot.state.pos, dest)
            return (dist, -robot.capability.get(task.ttype, 0.0), robot.rid)

        pool.sort(key=rank)
        if len(pool) <= self.robot_limit:
            chosen = list(pool)
        else:
            head = max(1, self.robot_limit // 2)
            rest = pool[head:]
            self.rng.shuffle(rest)
            chosen = pool[:head] + rest[: self.robot_limit - head]
        if incumbent is not None:
            keep = next((robot for robot in pool if robot.rid == incumbent), None)
            if keep is not None and all(robot.rid != incumbent for robot in chosen):
                chosen.append(keep)
        return chosen

    def _move(self, robots: dict[str, Robot], rid: str, dest: tuple[float, float]) -> float:
        robot = robots[rid]
        dist = move_distance(robot.state.pos, dest)
        if dist <= 1e-9:
            return 0.0
        robot.state.pos = (float(dest[0]), float(dest[1]))
        robot.state.bat -= robot.lam_b * dist
        if isinstance(robot.state, TransportRobot):
            robot.state.dis += dist
        if robot.state.bat < -1e-9:
            raise _Reject
        return dist

    def _approach(self, robots: dict[str, Robot], task: TaskNode, assignment: dict[str, str]) -> float:
        travel = 0.0
        for effect in task.post:
            if effect.target[0] == 'robot' and effect.key == 'pos' and effect.mode == 'set':
                travel += self._move(robots, assignment[effect.target[1]], effect.value)
        for cond in task.pre:
            if len(cond.who) != 2 or cond.key != 'dist' or cond.op not in ('<', '<='):
                continue
            left, right = assignment[cond.who[0]], assignment[cond.who[1]]
            gap = move_distance(robots[left].state.pos, robots[right].state.pos)
            if _compare(gap, cond.op, cond.value):
                continue
            p = robots[left].state.pos
            q = robots[right].state.pos
            meet = ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
            travel += self._move(robots, left, meet)
            travel += self._move(robots, right, meet)
        return travel

    def _read(self, robots: dict[str, Robot], env: EnvState, cond: PreCondition, assignment: dict[str, str]) -> float:
        if len(cond.who) == 2:
            if cond.key != 'dist':
                raise _Reject
            left, right = assignment[cond.who[0]], assignment[cond.who[1]]
            return move_distance(robots[left].state.pos, robots[right].state.pos)
        return float(getattr(robots[assignment[cond.who[0]]].state, cond.key))

    def _write(self, robots: dict[str, Robot], env: EnvState, effect: PostEffect, assignment: dict[str, str]) -> None:
        if effect.target[0] == 'env':
            host = env
        else:
            host = robots[assignment[effect.target[1]]].state
        if effect.mode == 'set':
            setattr(host, effect.key, copy.deepcopy(effect.value))
        elif effect.mode == 'add':
            setattr(host, effect.key, getattr(host, effect.key) + effect.value)
        else:
            current = getattr(host, effect.key)
            if not isinstance(current, dict):
                raise _Reject
            current.update(copy.deepcopy(effect.value))
        if effect.target[0] == 'robot' and effect.key == 'bat':
            if robots[assignment[effect.target[1]]].state.bat < -1e-9:
                raise _Reject
        if effect.target[0] == 'robot' and effect.key == 'full':
            full = robots[assignment[effect.target[1]]].state.full
            if not 0.0 <= full <= 1.0:
                raise _Reject

    def _execute(self, robots: dict[str, Robot], env: EnvState, task: TaskNode, assignment: dict[str, str]) -> float:
        travel = self._approach(robots, task, assignment)
        for cond in task.pre:
            if not _compare(self._read(robots, env, cond, assignment), cond.op, cond.value):
                raise _Reject
        for effect in task.post:
            self._write(robots, env, effect, assignment)
        return travel

    def _replay(self, order: list[str], assign: dict[str, dict[str, str]], complete: bool) -> tuple[bool, float, dict[str, float]]:
        """重放一段顺序。complete 为真时，顺序必须覆盖全部基元。"""
        seen = set(order)
        if len(seen) != len(order):
            return False, math.inf, {}
        if complete and seen != set(self.by_id):
            return False, math.inf, {}
        placed = set()
        for nid in order:
            if any(pred not in placed for pred in self.preds[nid]):
                return False, math.inf, {}
            placed.add(nid)
        robots, env = self._fresh()
        shares: dict[str, float] = {}
        total = 0.0
        try:
            for nid in order:
                task = self.by_id[nid].task
                assert task is not None
                crew = assign.get(nid)
                expected = [role.key() for role in task.roles]
                if crew is None or set(crew) != set(expected):
                    return False, math.inf, {}
                seen_robots: set[str] = set()
                for role in task.roles:
                    robot = self.robots.get(crew[role.key()])
                    if robot is None or robot.rtype != role.rtype or robot.rid in seen_robots:
                        return False, math.inf, {}
                    seen_robots.add(robot.rid)
                    if robot.capability.get(task.ttype, 0.0) < role.capability:
                        return False, math.inf, {}
                shares[nid] = self._execute(robots, env, task, crew)
                total += shares[nid]
        except _Reject:
            return False, math.inf, {}
        return True, total, shares

    def evaluate(self, order: tuple[str, ...] | list[str], assign: dict[str, dict[str, str]]) -> tuple[bool, float, dict[str, float]]:
        """按顺序重放完整方案。返回是否可行、总行程、每个基元的行程。"""
        return self._replay(list(order), assign, True)

    def _ready(self, done: set[str]) -> list[str]:
        return [
            nid for nid in self.by_id
            if nid not in done and all(pred in done for pred in self.preds[nid])
        ]

    def _expand(self, nid: str, robots: dict[str, Robot], incumbent: dict[str, str] | None):
        task = self.by_id[nid].task
        assert task is not None
        groups = []
        chosen: list[str] = []
        for role in task.roles:
            keep = None if incumbent is None else incumbent.get(role.key())
            found = self._candidates(task, role, robots, chosen, keep)
            if not found:
                return
            groups.append(found)
            chosen.append(found[0].rid)
        if not groups:
            yield {}
            return
        for combo in itertools.product(*groups):
            if len({robot.rid for robot in combo}) != len(combo):
                continue
            yield {role.key(): robot.rid for role, robot in zip(task.roles, combo)}

    def search(self) -> TaskPlan | None:
        """束搜索。找到的最好可行方案写入 best；没有可行方案时返回 None。"""
        beam = [([], {}, 0.0)]
        complete: list[tuple[list[str], dict[str, dict[str, str]], float]] = []
        width = len(self.by_id)
        for _ in range(width):
            pool = []
            for order, assign, travel in beam:
                if len(order) == width:
                    complete.append((order, assign, travel))
                    continue
                done = set(order)
                for nid in self._ready(done):
                    robots, _env = self._fresh()
                    try:
                        for done_id in order:
                            task = self.by_id[done_id].task
                            assert task is not None
                            self._execute(robots, _env, task, assign[done_id])
                    except _Reject:
                        continue
                    for crew in self._expand(nid, robots, None):
                        trial_robots, trial_env = copy.deepcopy(robots), copy.deepcopy(_env)
                        task = self.by_id[nid].task
                        assert task is not None
                        try:
                            extra = self._execute(trial_robots, trial_env, task, crew)
                        except _Reject:
                            continue
                        pool.append((order + [nid], {**assign, nid: crew}, travel + extra))
            if not pool:
                break
            pool.sort(key=lambda item: (item[2], item[0]))
            beam = pool[: self.beam_width]
        complete.extend(item for item in beam if len(item[0]) == width)
        if not complete:
            return None
        order, assign, travel = min(complete, key=lambda item: (item[2], item[0]))
        plan = self._wrap(order, assign, travel)
        self.best = plan
        return plan

    def _wrap(self, order, assign, travel: float) -> TaskPlan:
        labels = {}
        for nid in order:
            task = self.by_id[nid].task
            assert task is not None
            labels[nid] = f'{task.ttype}  {task.tid}'
        return TaskPlan(tuple(order), copy.deepcopy(assign), travel, True, labels)

    def _slots(self, order: list[str], nid: str) -> list[int]:
        index = {name: pos for pos, name in enumerate(order)}
        earliest = 0
        latest = len(order)
        for pred in self.preds[nid]:
            if pred in index:
                earliest = max(earliest, index[pred] + 1)
        for succ in self.succs[nid]:
            if succ in index:
                latest = min(latest, index[succ])
        if earliest > latest:
            return []
        return list(range(earliest, latest + 1))

    def _place(self, order: list[str], assign: dict[str, dict[str, str]], nid: str, incumbent: dict[str, str] | None):
        """把一个基元插回某个合法位置，返回插入后的新方案及其行程。插不回去则无结果。"""
        found = []
        for slot in self._slots(order, nid):
            robots, env = self._fresh()
            try:
                for done_id in order[:slot]:
                    task = self.by_id[done_id].task
                    assert task is not None
                    self._execute(robots, env, task, assign[done_id])
            except _Reject:
                continue
            for crew in self._expand(nid, robots, incumbent):
                trial_order = order[:slot] + [nid] + order[slot:]
                trial_assign = {**assign, nid: crew}
                ok, travel, _shares = self._replay(trial_order, trial_assign, False)
                if ok:
                    found.append((travel, trial_order, trial_assign))
        found.sort(key=lambda item: item[0])
        return found

    def _remove(self, order: list[str], assign: dict[str, dict[str, str]], victims: list[str]):
        gone = set(victims)
        kept = [nid for nid in order if nid not in gone]
        crew = {nid: assign[nid] for nid in kept}
        return kept, crew

    def _destroy(self, order: list[str], shares: dict[str, float], kind: str) -> list[str]:
        count = len(order)
        k = max(1, min(count, int(round(count * self.destroy_ratio))))
        if kind == 'worst':
            ranked = sorted(order, key=lambda nid: shares.get(nid, 0.0), reverse=True)
            return ranked[:k]
        if kind == 'chain':
            start = self.rng.choice(order)
            victims = [start]
            frontier = list(self.succs[start])
            while frontier and len(victims) < k:
                nxt = frontier.pop(0)
                if nxt in order and nxt not in victims:
                    victims.append(nxt)
                    frontier.extend(self.succs[nxt])
            return victims
        return self.rng.sample(order, k)

    def _repair(self, order: list[str], assign: dict[str, dict[str, str]], pending: list[str], kind: str):
        left = list(pending)
        while left:
            ready = [nid for nid in left if all(pred not in left for pred in self.preds[nid])]
            if not ready:
                return None
            ranked = []
            for nid in ready:
                options = self._place(order, assign, nid, None)
                if not options:
                    return None
                best = options[0][0]
                second = options[1][0] if len(options) > 1 else best
                ranked.append((second - best, best, nid, options[0]))
            if kind == 'regret':
                ranked.sort(key=lambda item: (-item[0], item[1]))
            else:
                ranked.sort(key=lambda item: item[1])
            _regret, _best, nid, chosen = ranked[0]
            _travel, order, assign = chosen
            left.remove(nid)
        ok, travel, shares = self.evaluate(order, assign)
        if not ok:
            return None
        return order, assign, travel, shares

    def optimize(self, seed: TaskPlan) -> TaskPlan:
        """从一可行方案出发做自适应大邻域搜索。更优的方案必接受，更差的方案按温度随机接受。"""
        ok, travel, shares = self.evaluate(seed.order, seed.assign)
        if not ok:
            raise ValueError('优化的起点必须是可行方案')
        current = (list(seed.order), copy.deepcopy(seed.assign), travel, shares)
        best = self._wrap(current[0], current[1], travel)
        weights = {'worst': 1.0, 'chain': 1.0, 'random': 1.0, 'greedy': 1.0, 'regret': 1.0}
        scores = {name: 0.0 for name in weights}
        used = {name: 0 for name in weights}
        temperature = self.T0
        for step in range(self.iters):
            destroy = self._pick(('worst', 'chain', 'random'), weights)
            repair = self._pick(('greedy', 'regret'), weights)
            order, assign, travel, shares = current
            victims = self._destroy(order, shares, destroy)
            kept_order, kept_assign = self._remove(order, assign, victims)
            trial = self._repair(kept_order, kept_assign, victims, repair)
            used[destroy] += 1
            used[repair] += 1
            reward = 0.0
            if trial is not None:
                trial_order, trial_assign, trial_travel, trial_shares = trial
                delta = trial_travel - travel
                if delta <= 1e-9 or self.rng.random() < math.exp(-delta / max(temperature, 1e-6)):
                    current = (trial_order, trial_assign, trial_travel, trial_shares)
                    reward = 1.0
                    if trial_travel < travel - 1e-9:
                        reward = 3.0
                    if trial_travel < best.travel - 1e-9:
                        best = self._wrap(trial_order, trial_assign, trial_travel)
                        reward = 5.0
            scores[destroy] += reward
            scores[repair] += reward
            if (step + 1) % 20 == 0:
                for name in weights:
                    average = scores[name] / used[name] if used[name] else 0.0
                    weights[name] = 0.8 * weights[name] + 0.2 * average
                    scores[name] = 0.0
                    used[name] = 0
            temperature *= self.alpha
        self.best = best
        return best

    def _pick(self, names: tuple[str, ...], weights: dict[str, float]) -> str:
        total = sum(weights[name] for name in names)
        draw = self.rng.random() * total
        for name in names:
            draw -= weights[name]
            if draw <= 0:
                return name
        return names[-1]

    def solve(self) -> TaskPlan:
        """先搜索一个可行方案，再优化行程。搜索失败时返回不可行方案。"""
        seed = self.search()
        if seed is None:
            missing = TaskPlan((), {}, math.inf, False)
            self.best = missing
            return missing
        return self.optimize(seed)

    def text(self) -> str:
        if self.best is None:
            return '任务求解  （尚未运行）'
        return self.best.text()


# ============================================================
# 6. 由表格装配一个场景
#    应用文件只填表。同一类任务的前置、后置和执行图由类型名生成。
# ============================================================

def _grid_key(point: tuple[float, float]) -> str:
    """把平面坐标写成环境字典用的 'x,y,0'。整数不带小数点。"""
    def num(value: float) -> str:
        return str(int(value)) if float(value).is_integer() else str(value)
    return f'{num(point[0])},{num(point[1])},0'


def _as_point(token, points: dict[str, tuple[float, float]]) -> tuple[float, float]:
    """落点名，或直接写的 (x, y)。"""
    if isinstance(token, str):
        if token not in points:
            raise KeyError(f'落点 {token} 没有在落点表里')
        return points[token]
    if isinstance(token, (tuple, list)) and len(token) == 2:
        return (float(token[0]), float(token[1]))
    raise TypeError(f'无法当作平面点: {token}')


def _task_kind(ttype: str) -> str:
    for name in ('unload', 'load', 'grasp', 'place', 'dig'):
        if ttype.startswith(name):
            return name
    raise ValueError(f'{ttype} 的类型须以 dig、load、unload、grasp 或 place 开头')


def _task_object(ttype: str) -> str:
    letter = ttype.rsplit('_', 1)[-1]
    if len(letter) != 1 or not letter.isalpha():
        raise ValueError(f'{ttype} 的物体须写在最后一个下划线后，如 dig_a')
    return letter.upper()


@dataclass
class CaseBundle:
    """compile_case 的产物。应用文件把它拆成原来的模块级名字。"""
    points: dict[str, tuple[float, float]]
    robots: dict[str, Robot]
    env: EnvState
    actions: dict
    facilities: dict[str, Facility]
    comm_links: dict[str, CommLink]
    tasks: dict[str, TaskNode]
    tasks_by_type: dict[str, TaskNode]
    operad: TaskOperad
    trees: dict
    translator: TaskTranslator
    translated: dict
    solver: TaskSolver
    plan: TaskPlan


def compile_case(
    sites: list[tuple],
    robots: list[tuple],
    caps: dict[str, dict[str, float]],
    actions: list[tuple],
    facilities: list[tuple],
    links: list[tuple],
    tasks: list[tuple],
    generators: list[tuple],
    seed: int = 1,
) -> CaseBundle:
    """
    把应用文件里的表装配成可求解的场景。

    sites      (名字, x, y, 材质或 None)
    robots     (编号, 机型, x, y, 朝向角度, 电量)
    caps       编号 → {任务类型: 能力}
    actions    (名字, 种类, 左参数, 右参数)。点可以写落点名。
    facilities (名字, 种类, 可靠性, 落点名元组)
    links      (名字, 可靠性, 最少同时在场人数)
    tasks      (编号, 类型, 可靠性, 结束落点, 电量门槛, 耗电,
                依赖编号元组, 角色元组, 要清空的落点或 None)
               角色是 (机型, 槽, 能力阈值)。多名角色时两两距离须 ≤ 1。
               类型前缀决定挖掘、装载、卸载、抓取、放置的状态写法。
               放置把 module_物体 写到名为 stack 的落点。
    generators (名字, 步骤元组, 依赖)。依赖 None 表示连成链，
               () 表示并行，也可以写去掉移动后的下标对。
    """
    points: dict[str, tuple[float, float]] = {}
    mat: dict[str, str] = {}
    for name, x, y, material in sites:
        if name in points:
            raise ValueError(f'落点 {name} 重复')
        points[name] = (float(x), float(y))
        if material:
            mat[_grid_key(points[name])] = material
    if 'stack' not in points:
        raise ValueError('落点表需要名为 stack 的建造点，放置写到这里')

    robot_map: dict[str, Robot] = {}
    for rid, rtype, x, y, degree, bat in robots:
        if rid in robot_map:
            raise ValueError(f'机器人 {rid} 重复')
        pos = (float(x), float(y))
        theta = math.radians(float(degree))
        if rtype == 'transport':
            state: RobotState = TransportRobot(pos, theta, float(bat))
        elif rtype == 'arm':
            state = ArmRobot(pos, theta, float(bat))
        else:
            raise ValueError(f'{rid} 的机型只能是 transport 或 arm')
        robot_map[rid] = Robot(rid, rtype, state, capability=dict(caps.get(rid, {})))

    action_map = {
        name: _compile_action(name, kind, left, right, points)
        for name, kind, left, right in actions
    }
    facility_map = {
        name: Facility(name, tuple(where), float(reliability), kind=kind)
        for name, kind, reliability, where in facilities
    }
    link_map = {
        name: CommLink(name, float(reliability), min_crew=int(min_crew))
        for name, reliability, min_crew in links
    }

    task_map: dict[str, TaskNode] = {}
    for row in tasks:
        tid, ttype, reliability, site, bat_need, bat_cost, dep_ids, role_rows, clear = row
        if tid in task_map:
            raise ValueError(f'任务编号 {tid} 重复')
        node = _compile_task(
            tid, ttype, float(reliability), site, float(bat_need), float(bat_cost),
            tuple(dep_ids), tuple(role_rows), clear, points,
        )
        task_map[tid] = node
    by_type = {node.ttype: node for node in task_map.values()}
    if len(by_type) != len(task_map):
        raise ValueError('任务类型重复')

    operad = TaskOperad()
    for node in task_map.values():
        operad.add_primitive(node.ttype)
    for name, steps, deps in generators:
        if deps is None:
            operad.add_generator(name, list(steps))
        else:
            operad.add_generator(name, list(steps), deps=[tuple(pair) for pair in deps])
    trees = {
        name: operad.decompose(name, by_type)
        for name, _steps, _deps in generators
    }

    translator = TaskTranslator()
    translated = {}
    for node in task_map.values():
        clear = _clear_of(node.ttype, tasks)
        item = _compile_translation(node.ttype, clear, points)
        translator.add(item)
        translated[node.ttype] = translator.translate(node)

    env = EnvState(occ={}, mat=mat, sigma={})
    solver = TaskSolver(robot_map, env, trees['build'], seed=seed)
    return CaseBundle(
        points=points,
        robots=robot_map,
        env=env,
        actions=action_map,
        facilities=facility_map,
        comm_links=link_map,
        tasks=task_map,
        tasks_by_type=by_type,
        operad=operad,
        trees=trees,
        translator=translator,
        translated=translated,
        solver=solver,
        plan=solver.solve(),
    )


def _clear_of(ttype: str, tasks: list[tuple]):
    for row in tasks:
        if row[1] == ttype:
            return row[8]
    return None


def _compile_action(name: str, kind: str, left, right, points: dict[str, tuple[float, float]]):
    if kind == 'a1':
        return A1(float(left))
    if kind == 'a2':
        return A2(_as_point(left, points), _as_point(right, points))
    if kind == 'a3':
        return A3(dict(left))
    if kind == 'm1':
        return M1(tuple(left), tuple(right))
    if kind == 'm2':
        return M2(str(left))
    if kind == 't1':
        return T1(move_distance(_as_point(left, points), _as_point(right, points)))
    if kind == 't2':
        return T2(float(left))
    raise ValueError(f'{name} 的动作种类只能是 a1、a2、a3、m1、m2、t1、t2')


def _compile_task(
    tid: str,
    ttype: str,
    reliability: float,
    site: str,
    bat_need: float,
    bat_cost: float,
    dep_ids: tuple[str, ...],
    role_rows: tuple,
    clear,
    points: dict[str, tuple[float, float]],
) -> TaskNode:
    kind = _task_kind(ttype)
    obj = _task_object(ttype)
    if site not in points:
        raise KeyError(f'{tid} 的落点 {site} 没有定义')
    roles = [
        RobotRole(rtype, float(threshold), slot=slot)
        for rtype, slot, threshold in role_rows
    ]
    keys = [role.key() for role in roles]
    pre: list[PreCondition] = []
    if len(keys) >= 2:
        pre.extend(
            PreCondition((left, right), 'dist', '<=', 1.0)
            for left, right in itertools.combinations(keys, 2)
        )
    for role in roles:
        if kind == 'load' and role.rtype == 'transport':
            pre.append(PreCondition((role.key(),), 'full', '<=', 0.0))
        elif kind == 'unload' and role.rtype == 'transport':
            pre.append(PreCondition((role.key(),), 'full', '>=', 1.0))
    for role in roles:
        pre.append(PreCondition((role.key(),), 'bat', '>=', bat_need))

    point = points[site]
    post: list[PostEffect] = [
        PostEffect(('robot', role.key()), 'pos', point) for role in roles
    ]
    arms = [role for role in roles if role.rtype == 'arm']
    trucks = [role for role in roles if role.rtype == 'transport']
    if kind in ('dig', 'grasp', 'place') and len(arms) != 1:
        raise ValueError(f'{tid} 需要且只需要一台机械臂')
    if kind in ('load', 'unload') and (not trucks or len(arms) != 1):
        raise ValueError(f'{tid} 需要一台机械臂和至少一台运输车')
    if kind == 'load':
        post.extend(PostEffect(('robot', role.key()), 'full', 1.0) for role in trucks)
        post.append(PostEffect(('robot', arms[0].key()), 'grip', 0.0))
    elif kind == 'unload':
        post.extend(PostEffect(('robot', role.key()), 'full', 0.0) for role in trucks)
        post.append(PostEffect(('robot', arms[0].key()), 'grip', 0.0))
    elif kind == 'place':
        stack_key = _grid_key(points['stack'])
        material = {stack_key: f'module_{obj}'}
        if clear:
            material[_grid_key(points[clear])] = 'empty'
        post.append(PostEffect(('env',), 'mat', material, 'merge'))
        post.append(PostEffect(('env',), 'occ', {stack_key: 1}, 'merge'))
        post.append(PostEffect(('robot', arms[0].key()), 'grip', 0.0))
    else:
        post.append(PostEffect(('robot', arms[0].key()), 'grip', 1.0))
    post.extend(
        PostEffect(('robot', role.key()), 'bat', -bat_cost, 'add') for role in roles
    )
    if kind == 'dig':
        post.append(PostEffect(('robot', arms[0].key()), 'belief', {'holding': f'raw_{obj}'}, 'merge'))
    elif kind == 'grasp':
        post.append(PostEffect(('robot', arms[0].key()), 'belief', {'holding': f'module_{obj}'}, 'merge'))
    elif kind == 'load':
        post.extend(
            PostEffect(('robot', role.key()), 'belief', {'cargo': obj}, 'merge') for role in trucks
        )
    elif kind == 'unload':
        post.extend(
            PostEffect(('robot', role.key()), 'belief', {'cargo': None}, 'merge') for role in trucks
        )
    else:
        post.append(PostEffect(('robot', arms[0].key()), 'belief', {f'placed_{obj}': True}, 'merge'))
    return TaskNode(
        tid=tid,
        ttype=ttype,
        roles=roles,
        pre=pre,
        post=post,
        deps=[Dependency(ref, 'id') for ref in dep_ids],
        reliability=reliability,
    )


def _compile_translation(ttype: str, clear, points: dict[str, tuple[float, float]]) -> TaskTranslation:
    kind = _task_kind(ttype)
    obj = _task_object(ttype)
    built = ActBuilder()
    env: tuple[PostEffect, ...] = ()
    if kind == 'dig':
        built.seq('arm', ('m1', 'a3', 'm2'))
    elif kind in ('load', 'unload'):
        (carry,) = built.seq('transport', ('t2',))
        _reach, grip = built.seq('arm', ('m1', 'm2'))
        built.link(grip, carry)
    elif kind == 'grasp':
        built.seq('arm', ('a3', 'm1', 'm2'))
    else:
        built.seq('arm', ('m1', 'm2'))
        stack_key = _grid_key(points['stack'])
        env = (
            PostEffect(('env',), 'mat', {stack_key: f'module_{obj}'}, 'merge'),
            PostEffect(('env',), 'occ', {stack_key: 1}, 'merge'),
        )
        if clear:
            env = env + (
                PostEffect(('env',), 'mat', {_grid_key(points[clear]): 'empty'}, 'merge'),
            )
    return TaskTranslation(ttype, built.graph(), env=env)
