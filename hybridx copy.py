"""
HybridX 分层范畴框架（重写）

本文件只放类型和算法，相当于头文件。
具体的机器人、环境和任务写在 hybridx_apply.py。
0.1 与 0.2 是数学工具，不在应用文件里实例化。
机器人与环境上的态射在第 1.2 节定义。
任务的分解与合成在第 3.1 节定义。
基元任务到机器人动作的翻译在第 4 节定义。
对照论文第二章逐步实现，旧脚本 solute.py 先不动。

已完成：
  1. 态射、复合、路径可靠性、多路径可靠性、设施增益

随后再写：
  2. 状态层：机器人动作与环境变化，移动单独标出
  3. 任务范畴：operad 分解与依赖偏序
  4. 函子 F_R、F_E，以及交换图
  5. 故障恢复与架构可靠性
"""
from __future__ import annotations
import builtins
import math
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
#     数学工具。具体的图不在应用文件里填写。
# ============================================================




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
    框架不计算新信念。content 由通讯层随后填入，执行时并入当前信念。
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
# 2. 任务节点
#    一个节点描述一件任务要谁来做、动手前必须成立什么、做完后留下什么。
# ============================================================

@dataclass
class RobotRole(Showable):
    """该任务需要的一类机器人。同一任务里每种类型至多一台。"""
    rtype: str                 # 'transport' 运输车 | 'arm' 机械臂
    capability: float          # 该机型完成此任务的能力，取值 [0, 1]

    def __post_init__(self):
        if not 0.0 <= self.capability <= 1.0:
            raise ValueError(f'{self.rtype} 在任务上的能力必须在 [0, 1] 内')

    def text(self) -> str:
        kind = {'transport': '运输车', 'arm': '机械臂'}.get(self.rtype, self.rtype)
        return f'{kind}  能力 {self.capability:.2f}'


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

    def __post_init__(self):
        rtypes = [role.rtype for role in self.roles]
        if len(rtypes) != len(set(rtypes)):
            raise ValueError(f'任务 {self.tid} 中每种机器人类型至多一台')
        known = set(rtypes)
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

        lines = [f'{self.tid}  [{self.ttype}]']
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
#       nav        二维空间内的导航运输
#       roadtrans  一维道路上的长途运输
#     分析可靠性、效率时，相邻两个基元之间的移动距离由
#     前一任务的终位置和后一任务的始位置计算，不在树上单独存放。
#
#     carve 采集原料：
#       move → dig → move
#       图中只留下 dig
#     deliver 转运：
#       load → move → roadtrans → move → unload
#       图中只留下 load → unload
#     transport 转运：
#       load → move → nav → move → unload
#       图中只留下 load → unload
#     assemble 拼装：
#       move → move_ang → grasp → move_ang → move → move_ang → place
#       图中只留下 grasp → place
# ============================================================

# 这些类型是移动，生成树里不保留
MOVE_TYPES = frozenset({'move', 'move_ang', 'nav', 'roadtrans'})


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
