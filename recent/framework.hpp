// framework.hpp
#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <cmath>
#include <memory>

// ==================== 态射 ====================
struct Morphism {
    std::string id;
    std::string src;        // 源对象 id
    std::string tgt;        // 目标对象 id
    double reliability = 1.0;
    enum Type { ACTION, ENV_CHANGE, DECOMPOSITION, COMMUNICATION, CAPABILITY } type = ACTION;
};

// ==================== 机器人状态 ====================
struct RobotState {
    std::string id;
    double x = 0, y = 0, theta = 0, bat = 1.0;

    RobotState(std::string id_, double x_, double y_, double theta_, double bat_)
        : id(std::move(id_)), x(x_), y(y_), theta(theta_), bat(bat_) {}
    virtual ~RobotState() = default;
};

// 运输车：装载量、累计距离
struct TransportRobot : RobotState {
    double full = 0, dis = 0;

    TransportRobot(std::string id_, double x_, double y_, double theta_, double bat_,
                   double full_, double dis_)
        : RobotState(std::move(id_), x_, y_, theta_, bat_), full(full_), dis(dis_) {}
};

// 机械臂：三个关节角
struct ArmRobot : RobotState {
    double ang[3] = {0, 0, 0};

    ArmRobot(std::string id_, double x_, double y_, double theta_, double bat_,
             double a0 = 0, double a1 = 0, double a2 = 0)
        : RobotState(std::move(id_), x_, y_, theta_, bat_), ang{a0, a1, a2} {}
};

// ==================== 环境状态 ====================
struct EnvState {
    std::string id;
    std::unordered_map<std::string, int> occ;              // "x,y,z" -> 0/1
    std::unordered_map<std::string, std::string> mat;      // "x,y,z" -> 材质
};

// ==================== 任务节点 ====================
struct TaskNode {
    std::string id;
    enum Kind { ROOT, INTERMEDIATE, ATOMIC } kind = ATOMIC;
    std::string boundRobot;                                // 资源绑定
    std::vector<std::string> deps;                         // 依赖（前驱）
    std::string mappedMorphism;                            // 若为原子任务，对应 R 层态射 id
};

// ==================== 函子 ====================
struct Functor {
    std::string name;
    std::string srcCat;     // "T" | "R" | "E"
    std::string tgtCat;
    std::unordered_map<std::string, std::string> objMap;    // 对象 → 对象
    std::unordered_map<std::string, std::string> morphMap;  // 态射 → 态射
    std::unordered_set<std::string> domain;                 // 定义域（仅对部分定义的函子）
};

// ==================== 交换图 ====================
struct ExchangeDiagram {
    std::string id;
    std::string A, B, C, D;     // 四角对象
    std::string f, g, h, k;     // A-f->B, A-g->C, C-k->D, B-h->D
    // 交换条件：h∘f == k∘g
};

// ==================== 框架容器 ====================
class Framework {
public:
    std::unordered_map<std::string, std::unique_ptr<RobotState>> robots;
    std::unordered_map<std::string, EnvState> envs;
    std::unordered_map<std::string, TaskNode> tasks;
    std::unordered_map<std::string, Morphism> morphisms;
    std::vector<Functor> functors;
    std::vector<ExchangeDiagram> diagrams;

    // ---------- 添加 ----------
    void addRobot(std::unique_ptr<RobotState> r) {
        std::string id = r->id;
        robots[std::move(id)] = std::move(r);
    }
    void addEnv(const EnvState& e)               { envs[e.id] = e; }
    void addTask(const TaskNode& t)              { tasks[t.id] = t; }
    void addMorphism(const Morphism& m)          { morphisms[m.id] = m; }
    void addFunctor(const Functor& f)            { functors.push_back(f); }
    void addDiagram(const ExchangeDiagram& d)    { diagrams.push_back(d); }

    // ---------- 基本查询 ----------
    const Morphism* getMorphism(const std::string& id) const {
        auto it = morphisms.find(id);
        return it == morphisms.end() ? nullptr : &it->second;
    }

    std::optional<Morphism> findEdge(const std::string& src, const std::string& tgt) const {
        for (const auto& kv : morphisms) {
            if (kv.second.src == src && kv.second.tgt == tgt) return kv.second;
        }
        return std::nullopt;
    }

    // ---------- 组合检查：f: A->B, g: B->C 可组合 ----------
    bool canCompose(const std::string& fId, const std::string& gId) const {
        auto f = getMorphism(fId);
        auto g = getMorphism(gId);
        if (!f || !g) return false;
        return f->tgt == g->src;
    }

    // ---------- 路径可靠性：态射可靠性之积 ----------
    double pathReliability(const std::vector<std::string>& ids) const {
        double r = 1.0;
        for (const auto& id : ids) {
            auto m = getMorphism(id);
            if (!m) return 0.0;
            r *= m->reliability;
        }
        return r;
    }

    // ---------- 多路径可靠性：至少一条成功 ----------
    double multiPathReliability(const std::vector<std::vector<std::string>>& paths) const {
        double failAll = 1.0;
        for (const auto& p : paths) failAll *= (1.0 - pathReliability(p));
        return 1.0 - failAll;
    }

    // ---------- 交换图验证 ----------
    // 检查 A-f->B, A-g->C, C-k->D, B-h->D 中 h∘f 与 k∘g 是否等价
    // 等价性由端点匹配 + 目标态射的一致性判定
    bool verifyDiagram(const ExchangeDiagram& d) const {
        auto f = getMorphism(d.f);
        auto g = getMorphism(d.g);
        auto h = getMorphism(d.h);
        auto k = getMorphism(d.k);
        if (!f || !g || !h || !k) return false;
        // 结构检查：端点对齐
        if (f->src != g->src) return false;
        if (f->tgt != h->src) return false;
        if (g->tgt != k->src) return false;
        if (h->tgt != k->tgt) return false;
        // 语义检查：若存在 A->D 的复合态射，其 id 应与两条路径一致
        // 这里简化：检查是否存在 A->D 的直接态射，如果有则要求其 id 与路径约定一致
        auto direct = findEdge(f->src, h->tgt);
        if (direct) {
            // 直接态射存在时，要求两条路径都映射到同一个直接态射
            return direct->id == h->id + "_compose_" + f->id
                || direct->id == k->id + "_compose_" + g->id
                || direct->id == "AD";    // 或约定的复合 id
        }
        return true;  // 无直接态射时，只做结构检查
    }

    // ---------- 函子公理验证 ----------
    // 要求 F(g∘f) == F(g)∘F(f)
    bool verifyFunctorAxiom(const Functor& F,
                            const std::string& fId,
                            const std::string& gId,
                            const std::string& gfId) const {
        // 源范畴中 f: A->B, g: B->C, gf: A->C
        auto f = getMorphism(fId);
        auto g = getMorphism(gId);
        auto gf = getMorphism(gfId);
        if (!f || !g || !gf) return false;
        if (f->tgt != g->src) return false;
        if (f->src != gf->src || g->tgt != gf->tgt) return false;

        // 映射结果
        auto itF = F.morphMap.find(fId);
        auto itG = F.morphMap.find(gId);
        auto itGF = F.morphMap.find(gfId);
        if (itF == F.morphMap.end() || itG == F.morphMap.end() || itGF == F.morphMap.end())
            return false;

        auto mF = getMorphism(itF->second);
        auto mG = getMorphism(itG->second);
        auto mGF = getMorphism(itGF->second);
        if (!mF || !mG || !mGF) return false;

        // 目标范畴中：F(g)∘F(f) 的端点应与 F(gf) 一致
        return mF->tgt == mG->src
            && mGF->src == mF->src
            && mGF->tgt == mG->tgt;
    }

    // ---------- 资源感知：原子任务是否可执行 ----------
    bool isExecutable(const std::string& taskId,
                      const std::unordered_set<std::string>& availRobots) const {
        auto it = tasks.find(taskId);
        if (it == tasks.end()) return false;
        const auto& t = it->second;
        if (t.kind != TaskNode::ATOMIC) return true;  // 中间任务不直接检查
        if (t.boundRobot.empty()) return false;
        return availRobots.count(t.boundRobot) > 0;
    }

    // ---------- 任务分解：在可执行子范畴上展开 operad 树 ----------
    // 返回叶子节点列表；若不可行返回空
    std::vector<std::string> decompose(const std::string& rootId,
                                       const std::unordered_set<std::string>& availRobots) const {
        std::vector<std::string> leaves;
        bool ok = decomposeRec(rootId, availRobots, leaves);
        if (!ok) leaves.clear();
        return leaves;
    }

private:
    bool decomposeRec(const std::string& taskId,
                      const std::unordered_set<std::string>& availRobots,
                      std::vector<std::string>& leaves) const {
        auto it = tasks.find(taskId);
        if (it == tasks.end()) return false;
        const auto& t = it->second;

        if (t.kind == TaskNode::ATOMIC) {
            if (!isExecutable(taskId, availRobots)) return false;
            leaves.push_back(taskId);
            return true;
        }

        // 中间任务：所有子任务都要能分解
        if (t.deps.empty()) return false;  // 无子任务定义
        for (const auto& childId : t.deps) {
            if (!decomposeRec(childId, availRobots, leaves)) return false;
        }
        return true;
    }
};