# main.py
from framework import (
    ArmRobot,
    EnvState,
    ExchangeDiagram,
    Framework,
    Functor,
    Morphism,
    TaskNode,
    TransportRobot,
)


def main() -> None:
    F = Framework()

    # ---------- 1. 机器人状态 ----------
    # R1 运输车：装载 A
    F.addRobot(TransportRobot("s1_0", 0, 0, 0, 1.00, 0, 0))
    F.addRobot(TransportRobot("s1_1", 5, 0, 0, 0.99, 0, 0))
    F.addRobot(TransportRobot("s1_2", 5, 0, 0, 0.98, 1.0, 0))
    F.addRobot(TransportRobot("s1_3", 20, 10, 0, 0.95, 1.0, 15.8))
    F.addRobot(TransportRobot("s1_4", 35, 15, 0, 0.92, 1.0, 31.2))
    F.addRobot(TransportRobot("s1_5", 40, 20, 0, 0.91, 1.0, 36.4))
    F.addRobot(TransportRobot("s1_6", 48, 28, 0, 0.88, 1.0, 47.7))
    F.addRobot(TransportRobot("s1_7", 48, 28, 0, 0.87, 0, 47.7))

    # R3 机械臂：组装 A
    F.addRobot(ArmRobot("s3_0", 50, 30, 90, 0.90))
    F.addRobot(ArmRobot("s3_1", 48, 28, 90, 0.88))
    F.addRobot(ArmRobot("s3_2", 48, 28, 90, 0.87))
    F.addRobot(ArmRobot("s3_3", 48, 28, 90, 0.86))
    F.addRobot(ArmRobot("s3_4", 50, 30, 90, 0.85))

    # ---------- 2. 环境状态 ----------
    F.addEnv(EnvState("E_0", {}, {"50,30,0": "regolith"}))
    F.addEnv(EnvState("E_1", {"5,0,0": 1}, {"5,0,0": "module"}))
    F.addEnv(EnvState("E_6", {"48,28,0": 1}, {"48,28,0": "module"}))
    F.addEnv(EnvState("E_8", {"50,30,0": 1}, {"50,30,0": "assembled"}))

    # ---------- 3. R 层态射 ----------
    F.addMorphism(Morphism("a2_s1_0_s1_1", "s1_0", "s1_1", 0.99, Morphism.Type.ACTION))
    F.addMorphism(Morphism("t1_load_A", "s1_1", "s1_2", 0.95, Morphism.Type.ACTION))
    F.addMorphism(Morphism("a2_nav1", "s1_2", "s1_3", 0.98, Morphism.Type.ACTION))
    F.addMorphism(Morphism("a2_nav2", "s1_3", "s1_4", 0.98, Morphism.Type.ACTION))
    F.addMorphism(Morphism("a2_nav3", "s1_4", "s1_5", 0.97, Morphism.Type.ACTION))
    F.addMorphism(Morphism("a2_transit", "s1_5", "s1_6", 0.85, Morphism.Type.ACTION))  # 无摄像头
    F.addMorphism(Morphism("t1_unload_A", "s1_6", "s1_7", 0.95, Morphism.Type.ACTION))

    F.addMorphism(Morphism("a2_s3_0_s3_1", "s3_0", "s3_1", 0.97, Morphism.Type.ACTION))
    F.addMorphism(Morphism("m1_pose", "s3_1", "s3_2", 0.96, Morphism.Type.ACTION))
    F.addMorphism(Morphism("m2_grasp", "s3_2", "s3_3", 0.93, Morphism.Type.ACTION))
    F.addMorphism(Morphism("m2_place", "s3_3", "s3_4", 0.90, Morphism.Type.ACTION))

    # ---------- 4. E 层态射 ----------
    F.addMorphism(Morphism("f_occ_01", "E_0", "E_1", 0.95, Morphism.Type.ENV_CHANGE))
    F.addMorphism(Morphism("f_mat_16", "E_1", "E_6", 0.95, Morphism.Type.ENV_CHANGE))
    F.addMorphism(Morphism("f_mat_68", "E_6", "E_8", 0.90, Morphism.Type.ENV_CHANGE))

    # ---------- 5. T 层任务 ----------
    F.addTask(TaskNode("T_build", TaskNode.Kind.ROOT, "", ["T_transport", "T_assemble"], ""))
    F.addTask(TaskNode("T_transport", TaskNode.Kind.INTERMEDIATE, "",
                       ["T_load_A", "T_nav_A", "T_transit_A", "T_unload_A"], ""))
    F.addTask(TaskNode("T_assemble", TaskNode.Kind.INTERMEDIATE, "", ["T_assemble_AB"], ""))
    F.addTask(TaskNode("T_load_A", TaskNode.Kind.ATOMIC, "R1", [], "t1_load_A"))
    F.addTask(TaskNode("T_nav_A", TaskNode.Kind.ATOMIC, "R1", [], "a2_nav1"))
    F.addTask(TaskNode("T_transit_A", TaskNode.Kind.ATOMIC, "R1", [], "a2_transit"))
    F.addTask(TaskNode("T_unload_A", TaskNode.Kind.ATOMIC, "R1", [], "t1_unload_A"))
    F.addTask(TaskNode("T_assemble_AB", TaskNode.Kind.ATOMIC, "R3", [], "m2_place"))

    # ---------- 6. 函子 F_R: T -> R ----------
    F_R = Functor()
    F_R.name = "F_R"
    F_R.srcCat = "T"
    F_R.tgtCat = "R"
    F_R.morphMap = {
        "T_load_A": "t1_load_A",
        "T_nav_A": "a2_nav1",
        "T_transit_A": "a2_transit",
        "T_unload_A": "t1_unload_A",
        "T_assemble_AB": "m2_place",
    }
    F.addFunctor(F_R)

    # ---------- 7. 函子 F_E: T -> E ----------
    F_E = Functor()
    F_E.name = "F_E"
    F_E.srcCat = "T"
    F_E.tgtCat = "E"
    F_E.morphMap = {
        "T_load_A": "f_occ_01",
        "T_unload_A": "f_mat_16",
        "T_assemble_AB": "f_mat_68",
    }
    F.addFunctor(F_E)

    # ---------- 8. 交换图 ----------
    # 行动一致性：A --F(T_transport)--> B
    #              A --F_percept--> C
    #              C --F_E(T_transport)--> D
    F.addDiagram(ExchangeDiagram(
        "diagram_transport",
        "s1_0", "s1_7",     # A, B
        "E_0", "E_6",       # C, D
        "t1_load_A",        # 占位，实际用复合路径
        "f_occ_01",
        "f_mat_16",
        "t1_unload_A",
    ))

    # ---------- 9. 验证 ----------
    # 路径可靠性
    R_transport = F.pathReliability([
        "a2_s1_0_s1_1", "t1_load_A", "a2_nav1", "a2_nav2",
        "a2_nav3", "a2_transit", "t1_unload_A",
    ])
    print(f"R(T_transport) = {R_transport:.4f}")

    # 有摄像头时的增益
    F.morphisms["a2_transit"].reliability = 0.95
    R_transport_new = F.pathReliability([
        "a2_s1_0_s1_1", "t1_load_A", "a2_nav1", "a2_nav2",
        "a2_nav3", "a2_transit", "t1_unload_A",
    ])
    print(f"R'(T_transport) = {R_transport_new:.4f}")
    print(f"Gain = {(R_transport_new - R_transport):.4f}")

    # 多路径可靠性：无摄像头 / 有摄像头两条路径
    multi = F.multiPathReliability([
        ["a2_transit"],          # 原路径
        ["a2_transit_vis"],      # 摄像头路径（需先添加该态射）
    ])
    # 若摄像头态射未添加，先添加
    F.addMorphism(Morphism("a2_transit_vis", "s1_5", "s1_6", 0.95, Morphism.Type.ACTION))
    multi = F.multiPathReliability([
        ["a2_transit"],
        ["a2_transit_vis"],
    ])
    print(f"Multi-path reliability = {multi:.4f}")

    # 交换图验证
    print("Diagram transport verifies: "
          + ("yes" if F.verifyDiagram(F.diagrams[0]) else "no"))

    # 函子公理：F(gf) == F(g)∘F(f)
    F.addMorphism(Morphism("load_then_nav", "s1_1", "s1_3", 0.93, Morphism.Type.ACTION))
    F.addMorphism(Morphism("t1_load_A_implied", "s1_1", "s1_2", 0.95, Morphism.Type.ACTION))
    print("Functor axiom (load): "
          + ("yes" if F.verifyFunctorAxiom(F_R, "t1_load_A", "a2_nav1", "load_then_nav") else "no"))

    # 资源感知分解
    avail = {"R1", "R2", "R3", "R4"}
    leaves = F.decompose("T_build", avail)
    print(f"Decomposition leaves: {len(leaves)}")
    for leaf in leaves:
        print(f"  {leaf}")

    # 移除 R3 后重新分解
    avail.remove("R3")
    leaves = F.decompose("T_build", avail)
    print(f"Decomposition with R3 removed: {len(leaves)} leaves")
    for leaf in leaves:
        print(f"  {leaf}")


if __name__ == "__main__":
    main()
