"""
HybridX：面向多机器人任务分配的分层范畴框架

核心模块：
  1. 范畴与函子（M, R, E, T）—— 机器人、环境、任务三类状态对象
  2. 基于 Operad 的任务分解 —— 把复合任务递归拆成原子任务
  3. 基于距离的状态转移 —— 移动代价、电量消耗、可靠性衰减
  4. ALNS 求解器 + 模拟退火接受准则
  5. 交接位置优化
  6. 交换图验证与方案可靠性评估
"""
from __future__ import annotations
import math, random, copy, time
from dataclasses import dataclass, field
from typing import Optional
from collections import defaultdict


# ============================================================
# 1. 范畴与状态对象
#    R：机器人状态层；E：环境状态层
# ============================================================

@dataclass
class RobotState:
    """机器人状态层 R 中的单个状态对象。"""
    pos: tuple[float, float]   # 平面位置 (x, y)，单位：米
    theta: float               # 朝向角，单位：弧度
    bat: float                 # 剩余电量，取值范围 [0, 1]
    # 运输机器人专用：装载量与累计行驶距离
    full: float = 0.0          # 当前装载量（0 空载，1 满载）
    dis: float = 0.0           # 累计行驶距离
    # 机械臂专用：三关节角度
    ang: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 可选的信念分布（感知不确定性）
    belief: dict = field(default_factory=dict)


@dataclass
class EnvState:
    """环境状态层 E 中的单个状态对象。"""
    occ: dict = field(default_factory=dict)      # 栅格占用：(x,y,z) -> 0/1
    mat: dict = field(default_factory=dict)      # 栅格材质：(x,y,z) -> 材料名
    sigma: dict = field(default_factory=dict)    # 结构化程度向量，用于估计交接可靠性


@dataclass
class Robot:
    """分类层 R 中的单个机器人对象（能力 + 可靠性参数）。"""
    rid: str
    rtype: str          # 机型：'transport' 运输车 | 'arm' 机械臂
    state: RobotState
    # 能力向量：{任务类型: 能力值}，用于判断该机器人能否执行某类任务
    capability: dict[str, float] = field(default_factory=dict)
    # 可靠性参数：移动距离越长，单次动作可靠性越低
    r0: float = 0.99           # 零距离时的基准可靠性
    lam_r: float = 0.002       # 可靠性随距离的衰减系数（每米）
    lam_b: float = 0.0005      # 电量随距离的消耗系数（每米）


# ============================================================
# 2. 任务范畴 T（基于 Operad 的分解）
#    复合任务由生成元拆成若干输入子任务，子任务之间有偏序依赖
# ============================================================

@dataclass
class TaskNode:
    """原子任务或复合任务节点，构成分解树。"""
    tid: str
    ttype: str                       # 任务类型，如 load / move / unload / assemble
    pre: dict = field(default_factory=dict)   # 前置条件：对机器人或环境的约束
    post: dict = field(default_factory=dict)  # 后置效果：位置、装载量、环境变化
    children: list[TaskNode] = field(default_factory=list)  # Operad 输入：子任务
    deps: list[str] = field(default_factory=list)  # 偏序依赖 < ：必须先完成的任务 id


class TaskCategory:
    """
    任务范畴 T = (O_r -> Pos)
      - O_r：任务生成元构成的自由非对称着色 Operad
      - Pos：由依赖边导出的偏序 Operad
    generators 记录「若干输入任务 -> 一个输出任务」的分解规则。
    """
    def __init__(self):
        # d_id -> (输入任务 id 列表, 输出任务 id)
        self.generators: dict[str, tuple[list[str], str]] = {}
        self.atomic: set[str] = set()  # 不可再分解的原子任务

    def add_generator(self, d_id: str, inputs: list[str], output: str, deps: list[tuple[str, str]] = None):
        """登记一条分解规则：inputs 组合后得到 output。未登记的输入视为原子任务。"""
        self.generators[d_id] = (inputs, output)
        for i in inputs:
            if i not in self.generators:
                self.atomic.add(i)

    def decompose(self, task: str) -> Optional[TaskNode]:
        """查找能产出 task 的生成元，返回一层子任务；原子任务或未找到时返回 None。"""
        for d_id, (inputs, output) in self.generators.items():
            if output == task:
                children = [TaskNode(tid=i, ttype=i) for i in inputs]
                return TaskNode(tid=task, ttype=task, children=children)
        return None


# ============================================================
# 3. 基于距离的状态转移（简化模型）
#    移动只改变位置、电量和累计里程，可靠性按指数衰减
# ============================================================

def distance(p1, p2):
    """二维欧氏距离。"""
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])


def reliability_of_move(robot: Robot, d: float) -> float:
    """单次移动的可靠性：r(a) = r0 * exp(-lam_r * d)。距离越长越不可靠。"""
    return robot.r0 * math.exp(-robot.lam_r * d)


def battery_cost(robot: Robot, d: float) -> float:
    """移动 d 米的电量消耗，与距离成正比。"""
    return robot.lam_b * d


def state_after_move(robot: Robot, target_pos: tuple[float, float]) -> RobotState:
    """构造移动到 target_pos 之后的新状态（不修改原对象）。"""
    d = distance(robot.state.pos, target_pos)
    new = copy.deepcopy(robot.state)
    new.pos = target_pos
    new.bat = robot.state.bat - battery_cost(robot, d)
    new.dis = robot.state.dis + d
    return new


# ============================================================
# 4. 函子 F_R、F_E：把基本任务翻译成机器人动作流程图
#    翻译表由用户按（任务, 机型）手动填写。
#    流程图只用已定义的机器人态射；kind 为 move 的态射不进入图。
#    行驶距离仍单独计入分配代价，不画进流程图。
# ============================================================

@dataclass
class RobotMorphism:
    """机器人范畴 R 上的态射：用户预先定义好的一个动作。"""
    mid: str
    rtype: str                          # 该动作属于哪一类机器人
    src: str                            # 源状态名
    tgt: str                            # 目标状态名
    kind: str = 'action'                # 'action' 业务动作；'move' 移动，翻译时略去
    reliability: float = 1.0
    bat_cost: float = 0.0               # 执行该动作的电量消耗（不含行驶）
    effects: dict = field(default_factory=dict)  # 动作对状态的改写，如 {'full_set': 1.0}


@dataclass
class ActionFlow:
    """
    某个基本任务在某一机型上的动作流程图。
    节点是状态名，边是非移动态射。移动被略去后，图里可能只剩若干段业务动作。
    """
    task_id: str
    rtype: str
    nodes: list[str] = field(default_factory=list)
    edges: list[tuple[str, str, str]] = field(default_factory=list)  # (源状态, 态射 id, 目标状态)

    def render(self) -> str:
        """把流程图写成「状态 -动作-> 状态」的串联文本。"""
        if not self.edges:
            return f'{self.task_id} @ {self.rtype}: （无业务动作，移动已略去）'
        parts = []
        for i, (src, mid, tgt) in enumerate(self.edges):
            if i == 0:
                parts.append(src)
            parts.append(f'-{mid}->')
            parts.append(tgt)
        return f'{self.task_id} @ {self.rtype}: ' + ' '.join(parts)


class FunctorR:
    """
    F_R: T -> R^→
    每个基本任务不直接对应一次位移，而是按用户手动输入的翻译方法，
    展开成该机型的动作流程图。流程图中的边必须是 define_morphism 登记过的态射。
    """
    def __init__(self, robots: list[Robot]):
        self.robots = robots
        self.morphisms: dict[str, RobotMorphism] = {}
        # (任务 id, 机型) -> 用户写下的态射 id 序列，其中可以包含移动
        self.translations: dict[tuple[str, str], list[str]] = {}

    def define_morphism(self, morphism: RobotMorphism):
        """登记一个机器人态射。翻译时只能引用这里出现过的 id。"""
        if morphism.kind not in ('action', 'move'):
            raise ValueError(f'态射 {morphism.mid} 的 kind 只能是 action 或 move')
        if morphism.mid in self.morphisms:
            raise ValueError(f'态射 {morphism.mid} 已定义')
        self.morphisms[morphism.mid] = morphism

    def set_translation(self, task_id: str, rtype: str, morphism_ids: list[str]):
        """
        手动输入一条翻译：基本任务 task_id 在机型 rtype 上按 morphism_ids 的顺序执行。
        序列里可以写下移动态射，生成流程图时会把它们丢掉。
        未调用本方法的（任务, 机型）视为该机型不能执行此任务。
        """
        for mid in morphism_ids:
            morphism = self.morphisms.get(mid)
            if morphism is None:
                raise KeyError(f'翻译使用了未定义的机器人态射: {mid}')
            if morphism.rtype != rtype:
                raise ValueError(f'态射 {mid} 属于机型 {morphism.rtype}，不能用于 {rtype}')
        self.translations[(task_id, rtype)] = list(morphism_ids)

    def translate(self, task: TaskNode, robot: Robot) -> Optional[ActionFlow]:
        """按手动翻译生成动作流程图。没有该机型的翻译时返回 None。"""
        morphism_ids = self.translations.get((task.tid, robot.rtype))
        if morphism_ids is None:
            return None
        flow = ActionFlow(task_id=task.tid, rtype=robot.rtype)
        for mid in morphism_ids:
            morphism = self.morphisms[mid]
            if morphism.kind == 'move':
                continue  # 流程图中略去移动
            if morphism.src not in flow.nodes:
                flow.nodes.append(morphism.src)
            if morphism.tgt not in flow.nodes:
                flow.nodes.append(morphism.tgt)
            flow.edges.append((morphism.src, morphism.mid, morphism.tgt))
        return flow

    def bind(self, task: TaskNode, robot: Robot) -> Optional[dict]:
        """
        尝试把任务绑定到机器人。
        必须已有该机型的手动翻译，且前置条件、电量都满足。
        返回的可靠性 = 流程图中各动作可靠性之积 × 被略去的行驶可靠性。
        """
        flow = self.translate(task, robot)
        if flow is None:
            return None
        for k, v in task.pre.items():
            if not self._check_pre(robot, k, v):
                return None
        # 没有显式目标点时，视为就地执行，行驶距离为 0
        target = task.post.get('pos', robot.state.pos)
        d = distance(robot.state.pos, target)
        r_flow = 1.0
        b_flow = 0.0
        for _src, mid, _tgt in flow.edges:
            morphism = self.morphisms[mid]
            r_flow *= morphism.reliability
            b_flow += morphism.bat_cost
        b = b_flow + battery_cost(robot, d)
        if b > robot.state.bat:
            return None
        r = r_flow * reliability_of_move(robot, d)
        return {
            'robot': robot.rid,
            'target': target,
            'd': d,
            'r': r,
            'b': b,
            'flow': flow,
        }

    def _check_pre(self, robot: Robot, k, v):
        """检查单条前置条件。未识别的键默认放行。"""
        if k == 'rtype':
            return robot.rtype == v
        if k == 'full_min':
            return robot.state.full >= v
        if k == 'full_max':
            return robot.state.full <= v
        if k == 'bat_min':
            return robot.state.bat >= v
        return True

    def execute(self, task: TaskNode, robot: Robot):
        """
        按流程图改写机器人状态。
        行驶不在图中，若任务带目标位置，仍单独更新位置与行驶电量；
        随后按图中动作的 effects 改写装载量等状态，并扣除动作电量。
        """
        flow = self.translate(task, robot)
        if flow is None:
            return
        target = task.post.get('pos')
        if target is not None:
            robot.state = state_after_move(robot, target)
        applied_full = False
        for _src, mid, _tgt in flow.edges:
            morphism = self.morphisms[mid]
            robot.state.bat -= morphism.bat_cost
            if 'full_set' in morphism.effects:
                robot.state.full = morphism.effects['full_set']
                applied_full = True
        if not applied_full and 'full_set' in task.post:
            robot.state.full = task.post['full_set']


class FunctorE:
    """
    F_E: T -> E^→
    把任务映射为环境变化（材质、占用）。当前只收集后置效果，不立即写回环境。
    """
    def __init__(self, env: EnvState):
        self.env = env

    def bind(self, task: TaskNode, handover_point=None) -> Optional[dict]:
        """提取任务对环境的影响。既不改材质也不改占用时返回 None。"""
        mat = task.post.get('mat')
        occ = task.post.get('occ')
        if mat is None and occ is None:
            return None
        return {'mat': mat, 'occ': occ, 'pos': task.post.get('pos')}


# ============================================================
# 5. 资源感知的 Operad 分解
#    自顶向下递归，直到叶子都是原子任务；中间任何一步无法分解则整棵树失败
# ============================================================

class OperadDecomposer:
    """
    把任务分解成叶子均为原子任务的树。
    若某个子任务找不到生成元且又不是原子任务，说明存在资源缺口，返回 None。
    """
    def __init__(self, task_cat: TaskCategory):
        self.task_cat = task_cat

    def decompose(self, task_id: str) -> Optional[TaskNode]:
        """递归分解。原子任务直接作为叶子返回。"""
        if task_id in self.task_cat.atomic:
            return TaskNode(tid=task_id, ttype=task_id)
        children = []
        for d_id, (inputs, output) in self.task_cat.generators.items():
            if output == task_id:
                for i in inputs:
                    sub = self.decompose(i)
                    if sub is None:
                        return None  # 任一子任务分解失败，整条规则作废
                    children.append(sub)
                return TaskNode(tid=task_id, ttype=task_id, children=children)
        return None  # 既不是原子任务，也没有匹配的生成元

    def leaves(self, node: TaskNode) -> list[TaskNode]:
        """收集分解树的全部叶子（实际需要分配的原子任务）。"""
        if not node.children:
            return [node]
        out = []
        for c in node.children:
            out.extend(self.leaves(c))
        return out


# ============================================================
# 6. 交接位置优化
#    两台机器人交接货物时，在候选点中选总代价最小的点
# ============================================================

class HandoverOptimizer:
    """
    给定一对任务及其机器人，寻找最优交接点，最小化：
      c(p) = λc * (d_i(p) + d_j(p)) - λr * log r_handover(p)
    前一项是双方赶到交接点的路程，后一项惩罚低可靠性的交接位置。
    """
    def __init__(self, candidates: list[tuple[float, float]]):
        self.candidates = candidates
        self.cache = {}  # (起点, 终点) -> 最优交接点，避免重复搜索

    def optimize(self, start_i, end_j, sigma_at, lam_c=1.0, lam_r=10.0):
        """在候选点上穷举。sigma_at 给出各点的结构化程度，越高交接越可靠。"""
        key = (start_i, end_j)
        if key in self.cache:
            return self.cache[key]
        best, best_c = None, float('inf')
        for p in self.candidates:
            d_i = distance(start_i, p)   # 前一台机器人赶到交接点的距离
            d_j = distance(p, end_j)     # 后一台机器人从交接点出发的距离
            r_h = 0.5 + 0.5 * sigma_at.get(p, 0.5)  # 交接可靠性，缺省按 0.5 结构化程度估计
            # log 项在可靠性接近 0 时会发散，因此给一个下界
            c = lam_c * (d_i + d_j) - lam_r * math.log(max(r_h, 1e-6))
            if c < best_c:
                best, best_c = p, c
        self.cache[key] = best
        return best


# ============================================================
# 7. 解的表示
#    一条解 = 任务到机器人的分配 + 每台机器人的执行顺序
# ============================================================

@dataclass
class Assignment:
    """单个任务分配给一台机器人，可选地带一个交接点。"""
    task_id: str
    robot_id: str
    target_pos: tuple[float, float]
    distance: float = 0.0      # 从当前位姿到目标的行驶距离
    reliability: float = 1.0   # 本次分配的可靠性
    bat_cost: float = 0.0      # 本次分配的电量消耗
    handover_from: Optional[tuple[float, float]] = None  # 上游交接点，无交接则为空


@dataclass
class Solution:
    """ALNS 搜索中的一个完整分配方案。"""
    assignments: dict[str, Assignment] = field(default_factory=dict)  # task_id -> 分配
    sequences: dict[str, list[str]] = field(default_factory=lambda: defaultdict(list))  # robot_id -> 任务顺序
    cost: float = 0.0          # 能量函数值，越小越好
    reliability: float = 1.0   # 方案整体可靠性
    violation: float = 0.0     # 约束违反量


# ============================================================
# 8. 能量函数
#    把行驶、电量、交接间隙、交换图不一致都折成一个标量代价
# ============================================================

class Energy:
    def __init__(self, lam_c=1.0, lam_conn=10.0, lam_diag=5.0, lam_bat=2.0):
        self.lam_c = lam_c          # 行驶距离权重
        self.lam_conn = lam_conn    # 跨机器人衔接距离权重
        self.lam_diag = lam_diag    # 交换图（交接位置不一致）权重
        self.lam_bat = lam_bat      # 低电量惩罚权重

    def compute(self, sol: Solution, robots: dict[str, Robot], env: EnvState) -> float:
        """四项代价之和。env 预留给环境项，当前计算未使用。"""
        # 1. 行驶代价：所有分配的距离加权和
        c_travel = sum(self.lam_c * a.distance for a in sol.assignments.values())
        # 2. 电量惩罚：任一机器人剩余电量低于 5% 时线性惩罚
        c_bat = sum(self.lam_bat * max(0, 0.05 - robots[rid].state.bat)
                    for rid in sol.sequences)
        # 3. 衔接残差：相邻任务分给不同机器人时，用两者目标点距离衡量交接间隙
        c_conn = 0.0
        for rid, seq in sol.sequences.items():
            for k in range(len(seq) - 1):
                a_prev = sol.assignments[seq[k]]
                a_next = sol.assignments[seq[k + 1]]
                if a_prev.robot_id != a_next.robot_id:
                    c_conn += self.lam_conn * distance(a_prev.target_pos, a_next.target_pos)
        # 4. 交换图违反：交接来源点与本任务目标点不一致时计入
        c_diag = 0.0
        for a in sol.assignments.values():
            if a.handover_from is not None:
                c_diag += self.lam_diag * distance(a.handover_from, a.target_pos)
        return c_travel + c_bat + c_conn + c_diag


# ============================================================
# 9. ALNS 求解器
#    自适应大邻域搜索：反复破坏再修复，用模拟退火决定是否接受变差的解
# ============================================================

class ALNSSolver:
    """
    Adaptive Large Neighborhood Search，接受准则为模拟退火。
    破坏：随机摘掉 k 个任务。
    修复：把摘掉的任务贪心地插回代价最小的（机器人, 位置）。
    交接位置的优化发生在修复阶段（由 HandoverOptimizer 提供候选）。
    """
    def __init__(self, robots, tasks, functor_r: FunctorR, handover: HandoverOptimizer,
                 energy: Energy, iters=2000, T0=100.0, alpha=0.995):
        self.robots = robots
        self.tasks = tasks  # task_id -> TaskNode
        self.fR = functor_r
        self.ho = handover
        self.E = energy
        self.iters = iters    # 外层迭代次数
        self.T0 = T0          # 模拟退火初始温度
        self.alpha = alpha    # 每轮温度衰减系数，越接近 1 降温越慢

    # -------- 初始解：按任务顺序贪心插入 --------
    def initial(self) -> Solution:
        """逐个任务插入当前最优位置，得到一个可行的起点。插不进去的任务先跳过。"""
        sol = Solution()
        for tid, task in self.tasks.items():
            best = self._best_insertion(sol, task)
            if best is None:
                continue
            self._apply_insertion(sol, tid, best)
        sol.cost = self.E.compute(sol, self.robots, None)
        return sol

    def _best_insertion(self, sol: Solution, task: TaskNode):
        """枚举每台机器人和序列中的每个插入位，返回使能量最小的绑定。"""
        best, best_cost = None, float('inf')
        for rid, robot in self.robots.items():
            b = self.fR.bind(task, robot)
            if b is None:
                continue  # 前置条件或电量不满足，跳过该机器人
            # 插入位从 0 到当前序列长度（含末尾）
            for pos in range(len(sol.sequences[rid]) + 1):
                a = Assignment(task.tid, rid, b['target'], b['d'], b['r'], b['b'])
                # 临时写入再评估，评估后立刻撤销，避免污染当前解
                sol.sequences[rid].insert(pos, task.tid)
                sol.assignments[task.tid] = a
                c = self.E.compute(sol, self.robots, None)
                sol.sequences[rid].remove(task.tid)
                del sol.assignments[task.tid]
                if c < best_cost:
                    best_cost = c
                    best = (rid, pos, a)
        return best

    def _apply_insertion(self, sol: Solution, tid: str, best):
        """把 _best_insertion 选出的 (机器人, 位置, 分配) 正式写入解。"""
        rid, pos, a = best
        sol.assignments[tid] = a
        sol.sequences[rid].insert(pos, tid)

    # -------- 破坏：随机移除 k 个已分配任务 --------
    def _destroy(self, sol: Solution, k: int) -> list[str]:
        removed = random.sample(list(sol.assignments.keys()), k)
        for tid in removed:
            a = sol.assignments.pop(tid)
            sol.sequences[a.robot_id].remove(tid)
        return removed

    # -------- 修复：把移除的任务重新贪心插入 --------
    def _repair(self, sol: Solution, removed: list[str]) -> Solution:
        for tid in removed:
            task = self.tasks[tid]
            best = self._best_insertion(sol, task)
            if best is None:
                continue  # 修复失败则该任务暂时不分配
            self._apply_insertion(sol, tid, best)
        return sol

    # -------- 主循环：破坏-修复 + 模拟退火 --------
    def solve(self) -> Solution:
        """
        每轮从当前解摘掉约 1/4 的任务再插回。
        更优的试探解直接接受；更差的解以 exp(-ΔE / T) 的概率接受，
        温度 T 按 alpha 指数下降，后期逐渐只接受改进。
        """
        cur = self.initial()
        best = copy.deepcopy(cur)
        T = self.T0
        for it in range(self.iters):
            # 至少移除 1 个，最多移除当前已分配任务数的 1/4
            k = random.randint(1, max(1, len(cur.assignments) // 4))
            removed = self._destroy(cur, k)
            # 在副本上修复，当前解 cur 已被破坏，接受失败时本轮改进丢失
            trial = self._repair(copy.deepcopy(cur), removed)
            trial.cost = self.E.compute(trial, self.robots, None)

            # Metropolis 准则：改进必接受，变差则按温度随机接受
            if trial.cost < cur.cost or random.random() < math.exp(-(trial.cost - cur.cost) / max(T, 1e-6)):
                cur = trial
                if cur.cost < best.cost:
                    best = copy.deepcopy(cur)
            T *= self.alpha
        return best


# ============================================================
# 10. 方案可靠性
#     假设各次分配相互独立，总可靠性为各次可靠性的乘积
# ============================================================

def solution_reliability(sol: Solution) -> float:
    R = 1.0
    for a in sol.assignments.values():
        R *= a.reliability
    return R


# ============================================================
# 11. 交换图验证
#     检查交接是否对齐、每台机器人的电量沿序列是否始终非负
# ============================================================

def verify_exchange_diagram(sol: Solution, robots: dict[str, Robot]) -> dict:
    """
    对交换图做两项一致性检查：
      - 前序任务目标点与后序任务起点的距离不超过 eps
      - 每台机器人按自己的任务序列扣电后电量仍非负
    返回各类通过/失败的计数。
    """
    report = {'handover_ok': 0, 'handover_bad': 0, 'bat_ok': 0, 'bat_bad': 0}
    eps = 0.5  # 交接对齐容差，单位：米
    for rid, seq in sol.sequences.items():
        # 沿任务序列累减电量，中途为负即判定该机器人电量不连续
        bat = robots[rid].state.bat
        ok = True
        for tid in seq:
            a = sol.assignments[tid]
            bat -= a.bat_cost
            if bat < 0:
                ok = False
        report['bat_ok' if ok else 'bat_bad'] += 1

    # 仅当相邻任务分属不同机器人时才检查交接距离
    for rid, seq in sol.sequences.items():
        for k in range(len(seq) - 1):
            a1 = sol.assignments[seq[k]]
            a2 = sol.assignments[seq[k + 1]]
            if a1.robot_id != a2.robot_id:
                if distance(a1.target_pos, a2.target_pos) <= eps:
                    report['handover_ok'] += 1
                else:
                    report['handover_bad'] += 1
    return report


# ============================================================
# 12. 示例：跑通「分解 -> 分配 -> 评估」整条流水线
#     场景：运输车把物料送到建筑区，机械臂完成装配
# ============================================================

def demo():
    # ---- 机器人：两台运输车从原点附近出发，两台机械臂位于装配区 ----
    robots = {
        'R1': Robot('R1', 'transport', RobotState((0, 0), 0.0, 1.00)),
        'R2': Robot('R2', 'transport', RobotState((2, 0), 0.0, 1.00)),
        'R3': Robot('R3', 'arm',       RobotState((50, 30), math.radians(90), 0.90)),
        'R4': Robot('R4', 'arm',       RobotState((52, 30), math.radians(90), 0.90)),
    }

    # ---- 原子任务：装载 -> 导航 -> 转运 -> 卸载 -> 装配 ----
    # pre 约束机型与装载量，post 给出目标位置以及装载量/材质的变化
    tasks = {
        'T_load_A':  TaskNode('T_load_A', 'load',  pre={'rtype': 'transport', 'full_max': 0.0},
                                                   post={'pos': (5, 0), 'full_set': 1.0}),
        'T_nav_A':   TaskNode('T_nav_A',  'move',  pre={'rtype': 'transport', 'full_min': 1.0},
                                                   post={'pos': (40, 20)}),
        'T_transit_A': TaskNode('T_transit_A', 'move', pre={'rtype': 'transport'},
                                                     post={'pos': (48, 28)}),
        'T_unload_A': TaskNode('T_unload_A', 'unload', pre={'rtype': 'transport', 'full_min': 1.0},
                                                       post={'pos': (48, 28), 'full_set': 0.0}),
        'T_assemble_AB': TaskNode('T_assemble_AB', 'assemble', pre={'rtype': 'arm'},
                                                              post={'pos': (50, 30), 'mat': 'module'}),
    }

    # ---- 任务范畴：两级生成元 ----
    # d1：四段运输合成一次配送；d2：配送 + 装配合成建造
    cat = TaskCategory()
    cat.add_generator('d1', ['T_load_A', 'T_nav_A', 'T_transit_A', 'T_unload_A'], 'T_deliver_A')
    cat.add_generator('d2', ['T_deliver_A', 'T_assemble_AB'], 'T_build')
    cat.atomic.update(['T_load_A', 'T_nav_A', 'T_transit_A', 'T_unload_A', 'T_assemble_AB'])

    decomposer = OperadDecomposer(cat)
    root = decomposer.decompose('T_build')
    print('Decomposition leaves:', [l.tid for l in decomposer.leaves(root)])

    # ---- 函子：先登记机器人态射，再手动写入每个基本任务的翻译 ----
    fR = FunctorR(list(robots.values()))
    # 运输车动作。kind='move' 的态射可以写进翻译，但不会出现在流程图里
    fR.define_morphism(RobotMorphism('a2_to_load', 'transport', 's_idle', 's_at_pile', kind='move', reliability=0.99))
    fR.define_morphism(RobotMorphism('t1_load_A', 'transport', 's_at_pile', 's_loaded', reliability=0.95, bat_cost=0.01, effects={'full_set': 1.0}))
    fR.define_morphism(RobotMorphism('a2_nav', 'transport', 's_loaded', 's_near', kind='move', reliability=0.98))
    fR.define_morphism(RobotMorphism('a2_transit', 'transport', 's_near', 's_handover', kind='move', reliability=0.85))
    fR.define_morphism(RobotMorphism('t1_unload_A', 'transport', 's_handover', 's_empty', reliability=0.95, bat_cost=0.01, effects={'full_set': 0.0}))
    # 机械臂动作
    fR.define_morphism(RobotMorphism('a2_to_asm', 'arm', 's_home', 's_at_mod', kind='move', reliability=0.97))
    fR.define_morphism(RobotMorphism('m1_pose', 'arm', 's_at_mod', 's_posed', reliability=0.96, bat_cost=0.005))
    fR.define_morphism(RobotMorphism('m2_grasp', 'arm', 's_posed', 's_held', reliability=0.93, bat_cost=0.008))
    fR.define_morphism(RobotMorphism('m2_place', 'arm', 's_held', 's_placed', reliability=0.90, bat_cost=0.008))
    # 每个基本任务、每种机型各写一条翻译。这里同一任务只配了一种能执行它的机型
    fR.set_translation('T_load_A', 'transport', ['a2_to_load', 't1_load_A'])
    fR.set_translation('T_nav_A', 'transport', ['a2_nav'])
    fR.set_translation('T_transit_A', 'transport', ['a2_transit'])
    fR.set_translation('T_unload_A', 'transport', ['t1_unload_A'])
    fR.set_translation('T_assemble_AB', 'arm', ['a2_to_asm', 'm1_pose', 'm2_grasp', 'm2_place'])

    print('\n=== 动作流程图（已略去移动）===')
    shown = set()
    for task in tasks.values():
        for robot in robots.values():
            key = (task.tid, robot.rtype)
            if key in shown:
                continue
            shown.add(key)
            flow = fR.translate(task, robot)
            if flow is not None:
                print(' ', flow.render())

    # ---- 交接候选点：建筑区附近的粗网格，步长 1 米 ----
    candidates = [(x, y) for x in range(44, 54, 1) for y in range(24, 34, 1)]
    ho = HandoverOptimizer(candidates)

    # ---- 用 ALNS 求解分配（演示用 500 轮，默认构造参数是 2000 轮）----
    energy = Energy()
    solver = ALNSSolver(robots, tasks, fR, ho, energy, iters=500)
    sol = solver.solve()

    print('\n=== Solution ===')
    for rid, seq in sol.sequences.items():
        print(f'{rid}: {seq}')
    for tid, a in sol.assignments.items():
        print(f'  {tid}: robot={a.robot_id}, target={a.target_pos}, d={a.distance:.2f}m, r={a.reliability:.3f}')
    print(f'\nCost: {sol.cost:.3f}')
    print(f'Reliability: {solution_reliability(sol):.4f}')
    print('Exchange diagram:', verify_exchange_diagram(sol, robots))


if __name__ == '__main__':
    random.seed(42)  # 固定随机种子，便于复现同一份分配结果
    t0 = time.time()
    demo()
    print(f'\nElapsed: {time.time() - t0:.2f}s')
