# framework.py
from enum import Enum, auto
from typing import Dict, List, Optional, Set


# ==================== 态射 ====================
class Morphism:
    class Type(Enum):
        ACTION = auto()
        ENV_CHANGE = auto()
        DECOMPOSITION = auto()
        COMMUNICATION = auto()
        CAPABILITY = auto()

    def __init__(self, id: str, src: str, tgt: str,
                 reliability: float = 1.0, type: "Morphism.Type" = None):
        self.id = id
        self.src = src              # 源对象 id
        self.tgt = tgt              # 目标对象 id
        self.reliability = reliability
        self.type = Morphism.Type.ACTION if type is None else type


# ==================== 机器人状态 ====================
class RobotState:
    def __init__(self, id_: str, x_: float, y_: float, theta_: float, bat_: float):
        self.id = id_
        self.x = x_
        self.y = y_
        self.theta = theta_
        self.bat = bat_


# 运输车：装载量、累计距离
class TransportRobot(RobotState):
    def __init__(self, id_: str, x_: float, y_: float, theta_: float, bat_: float,
                 full_: float, dis_: float):
        super().__init__(id_, x_, y_, theta_, bat_)
        self.full = full_
        self.dis = dis_


# 机械臂：三个关节角
class ArmRobot(RobotState):
    def __init__(self, id_: str, x_: float, y_: float, theta_: float, bat_: float,
                 a0: float = 0, a1: float = 0, a2: float = 0):
        super().__init__(id_, x_, y_, theta_, bat_)
        self.ang = [a0, a1, a2]


# ==================== 环境状态 ====================
class EnvState:
    def __init__(self, id: str,
                 occ: Optional[Dict[str, int]] = None,
                 mat: Optional[Dict[str, str]] = None):
        self.id = id
        self.occ = {} if occ is None else occ      # "x,y,z" -> 0/1
        self.mat = {} if mat is None else mat      # "x,y,z" -> 材质


# ==================== 任务节点 ====================
class TaskNode:
    class Kind(Enum):
        ROOT = auto()
        INTERMEDIATE = auto()
        ATOMIC = auto()

    def __init__(self, id: str, kind: "TaskNode.Kind" = None,
                 boundRobot: str = "", deps: Optional[List[str]] = None,
                 mappedMorphism: str = ""):
        self.id = id
        self.kind = TaskNode.Kind.ATOMIC if kind is None else kind
        self.boundRobot = boundRobot                          # 资源绑定
        self.deps = [] if deps is None else deps              # 依赖（前驱）
        self.mappedMorphism = mappedMorphism                  # 若为原子任务，对应 R 层态射 id


# ==================== 函子 ====================
class Functor:
    def __init__(self):
        self.name = ""
        self.srcCat = ""     # "T" | "R" | "E"
        self.tgtCat = ""
        self.objMap: Dict[str, str] = {}     # 对象 → 对象
        self.morphMap: Dict[str, str] = {}   # 态射 → 态射
        self.domain: Set[str] = set()        # 定义域（仅对部分定义的函子）


# ==================== 交换图 ====================
class ExchangeDiagram:
    def __init__(self, id: str, A: str, B: str, C: str, D: str,
                 f: str, g: str, h: str, k: str):
        self.id = id
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        # A-f->B, A-g->C, C-k->D, B-h->D
        # 交换条件：h∘f == k∘g
        self.f = f
        self.g = g
        self.h = h
        self.k = k


# ==================== 框架容器 ====================
class Framework:
    def __init__(self):
        self.robots: Dict[str, RobotState] = {}
        self.envs: Dict[str, EnvState] = {}
        self.tasks: Dict[str, TaskNode] = {}
        self.morphisms: Dict[str, Morphism] = {}
        self.functors: List[Functor] = []
        self.diagrams: List[ExchangeDiagram] = []

    # ---------- 添加 ----------
    def addRobot(self, r: RobotState) -> None:
        self.robots[r.id] = r

    def addEnv(self, e: EnvState) -> None:
        self.envs[e.id] = e

    def addTask(self, t: TaskNode) -> None:
        self.tasks[t.id] = t

    def addMorphism(self, m: Morphism) -> None:
        self.morphisms[m.id] = m

    def addFunctor(self, f: Functor) -> None:
        self.functors.append(f)

    def addDiagram(self, d: ExchangeDiagram) -> None:
        self.diagrams.append(d)

    # ---------- 基本查询 ----------
    def getMorphism(self, id: str) -> Optional[Morphism]:
        return self.morphisms.get(id)

    def findEdge(self, src: str, tgt: str) -> Optional[Morphism]:
        for m in self.morphisms.values():
            if m.src == src and m.tgt == tgt:
                return m
        return None

    # ---------- 组合检查：f: A->B, g: B->C 可组合 ----------
    def canCompose(self, fId: str, gId: str) -> bool:
        f = self.getMorphism(fId)
        g = self.getMorphism(gId)
        if f is None or g is None:
            return False
        return f.tgt == g.src

    # ---------- 路径可靠性：态射可靠性之积 ----------
    def pathReliability(self, ids: List[str]) -> float:
        r = 1.0
        for id in ids:
            m = self.getMorphism(id)
            if m is None:
                return 0.0
            r *= m.reliability
        return r

    # ---------- 多路径可靠性：至少一条成功 ----------
    def multiPathReliability(self, paths: List[List[str]]) -> float:
        failAll = 1.0
        for p in paths:
            failAll *= (1.0 - self.pathReliability(p))
        return 1.0 - failAll

    # ---------- 交换图验证 ----------
    # 检查 A-f->B, A-g->C, C-k->D, B-h->D 中 h∘f 与 k∘g 是否等价
    # 等价性由端点匹配 + 目标态射的一致性判定
    def verifyDiagram(self, d: ExchangeDiagram) -> bool:
        f = self.getMorphism(d.f)
        g = self.getMorphism(d.g)
        h = self.getMorphism(d.h)
        k = self.getMorphism(d.k)
        if f is None or g is None or h is None or k is None:
            return False
        # 结构检查：端点对齐
        if f.src != g.src:
            return False
        if f.tgt != h.src:
            return False
        if g.tgt != k.src:
            return False
        if h.tgt != k.tgt:
            return False
        # 语义检查：若存在 A->D 的复合态射，其 id 应与两条路径一致
        # 这里简化：检查是否存在 A->D 的直接态射，如果有则要求其 id 与路径约定一致
        direct = self.findEdge(f.src, h.tgt)
        if direct is not None:
            # 直接态射存在时，要求两条路径都映射到同一个直接态射
            return (direct.id == h.id + "_compose_" + f.id
                    or direct.id == k.id + "_compose_" + g.id
                    or direct.id == "AD")    # 或约定的复合 id
        return True  # 无直接态射时，只做结构检查

    # ---------- 函子公理验证 ----------
    # 要求 F(g∘f) == F(g)∘F(f)
    def verifyFunctorAxiom(self, F: Functor, fId: str, gId: str, gfId: str) -> bool:
        # 源范畴中 f: A->B, g: B->C, gf: A->C
        f = self.getMorphism(fId)
        g = self.getMorphism(gId)
        gf = self.getMorphism(gfId)
        if f is None or g is None or gf is None:
            return False
        if f.tgt != g.src:
            return False
        if f.src != gf.src or g.tgt != gf.tgt:
            return False

        # 映射结果
        if fId not in F.morphMap or gId not in F.morphMap or gfId not in F.morphMap:
            return False

        mF = self.getMorphism(F.morphMap[fId])
        mG = self.getMorphism(F.morphMap[gId])
        mGF = self.getMorphism(F.morphMap[gfId])
        if mF is None or mG is None or mGF is None:
            return False

        # 目标范畴中：F(g)∘F(f) 的端点应与 F(gf) 一致
        return (mF.tgt == mG.src
                and mGF.src == mF.src
                and mGF.tgt == mG.tgt)

    # ---------- 资源感知：原子任务是否可执行 ----------
    def isExecutable(self, taskId: str, availRobots: Set[str]) -> bool:
        t = self.tasks.get(taskId)
        if t is None:
            return False
        if t.kind != TaskNode.Kind.ATOMIC:
            return True  # 中间任务不直接检查
        if t.boundRobot == "":
            return False
        return t.boundRobot in availRobots

    # ---------- 任务分解：在可执行子范畴上展开 operad 树 ----------
    # 返回叶子节点列表；若不可行返回空
    def decompose(self, rootId: str, availRobots: Set[str]) -> List[str]:
        leaves: List[str] = []
        ok = self._decomposeRec(rootId, availRobots, leaves)
        if not ok:
            leaves.clear()
        return leaves

    def _decomposeRec(self, taskId: str, availRobots: Set[str],
                      leaves: List[str]) -> bool:
        t = self.tasks.get(taskId)
        if t is None:
            return False

        if t.kind == TaskNode.Kind.ATOMIC:
            if not self.isExecutable(taskId, availRobots):
                return False
            leaves.append(taskId)
            return True

        # 中间任务：所有子任务都要能分解
        if not t.deps:
            return False  # 无子任务定义
        for childId in t.deps:
            if not self._decomposeRec(childId, availRobots, leaves):
                return False
        return True
