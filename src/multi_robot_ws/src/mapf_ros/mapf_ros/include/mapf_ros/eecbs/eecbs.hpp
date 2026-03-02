#pragma once

#include <map>
#include <cmath>
#include <limits>
#include <boost/heap/d_ary_heap.hpp>
#include "a_star_epsilon.hpp"

namespace mapf {

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class EECBS {
public:
    EECBS(Environment& environment, float w, double k1)
        : m_env(environment), m_w(w), m_k(k1), m_averageError(0), 
        m_averageDistance(0), m_error_count(0), m_averageH(0), m_h_count(0),
        m_focal_best_fhat(std::numeric_limits<Cost>::max() ) {}

    bool search(const std::vector<State>& initialStates,
                std::vector<PlanResult<State, Action, Cost>>& solution,
                const double& time_tolerance) ;
    //             {
    //     Timer timer;
    //     HighLevelNode start = createStartNode(initialStates, solution);
        
    //     openSet_t open;
    //     focalSet_t focal;
    //     cleanupSet_t cleanup;

    //     auto handle = open.push(start);
    //     start.handle = handle;
    //     focal.push(start);
    //     cleanup.push(start);

    //     solution.clear();
    //     int id = 1;

    //     while (!open.empty()) {
    //         timer.stop();
    //         if (timer.elapsedSeconds() > time_tolerance) {
    //             return false;
    //         }

    //         updateFocalList(open, focal, cleanup);
    //         HighLevelNode P = selectNode(focal, open, cleanup);
    //         m_env.onExpandHighLevelNode(P.cost);

    //         Conflict conflict;
    //         if (!m_env.getFirstConflict(P.solution, conflict)) {
    //             solution = P.solution;
    //             return true;
    //         }

    //         std::map<size_t, Constraints> constraints;
    //         m_env.createConstraintsFromConflict(conflict, constraints);

    //         for (const auto& c : constraints) {
    //             size_t i = c.first;
    //             HighLevelNode newNode = createNewNode(P, i, c.second, id, initialStates);

    //             if (newNode.cost > 0) {  // Indicates successful child creation
    //                 auto handle = open.push(newNode);
    //                 newNode.handle = handle;
    //                 //open.push(newNode);
    //                 if (newNode.fHat <= m_w * open.top().fHat) {
    //                     focal.push(newNode);
    //                 }
    //                 cleanup.push(newNode);
    //             }
    //             ++id;

    //             timer.stop();
    //             if (timer.elapsedSeconds() > time_tolerance) {
    //                 return false;
    //             }
    //         }
    //     }
    //     return false;
    // }

private:
    struct HighLevelNode;

    struct CompareNodesFHat {
        bool operator()(const HighLevelNode& n1, const HighLevelNode& n2) const {
            return n1.fHat > n2.fHat;
        }
    };

    struct CompareFocalHeuristic{

        bool operator()(const HighLevelNode& n1, const HighLevelNode& n2) const {
            if (n1.hc != n2.hc) {
                return n1.hc > n2.hc;
            }
            return n1.fHat > n2.fHat;
        }
    };

    struct CompareNodesLB {
        bool operator()(const HighLevelNode& n1, const HighLevelNode& n2)const {
            return n1.LB > n2.LB;
        }
    };
    
    // using openSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
    // using focalSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<CompareFocalHeuristic>>;
    // using cleanupSet_t = openSet_t;
    using openSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<CompareNodesFHat>>;
    using focalSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<CompareFocalHeuristic>>;
    using cleanupSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<CompareNodesLB>>;

    struct HighLevelNode {
        std::vector<PlanResult<State, Action, Cost>> solution;
        std::vector<Constraints> constraints;
        Cost cost;
        Cost LB;
        int hc;
        Cost fHat;
        int id;

        typename openSet_t::handle_type open_handle;
        typename focalSet_t::handle_type focal_handle;
        typename cleanupSet_t::handle_type cleanup_handle;

        bool in_focal_list;

        HighLevelNode() : cost(0), LB(0), hc(0), fHat(0), id(-1), in_focal_list(false) {}

        bool operator<(const HighLevelNode& n) const {
            return fHat > n.fHat;
        }
    };

    //bool CompareFocalHeuristic::operator()(const HighLevelNode& n1, const HighLevelNode& n2) const;
    // {
    //         if (n1.focalHeuristic != n2.focalHeuristic) {
    //             return n1.focalHeuristic > n2.focalHeuristic;
    //         }
    //         return n1.fHat > n2.fHat;
    // };

    // using openSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
    // using focalSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<CompareFocalHeuristic>>;
    // using cleanupSet_t = openSet_t;

    HighLevelNode createStartNode(const std::vector<State>& initialStates,
                                  const std::vector<PlanResult<State, Action, Cost>>& initialSolution) {
        HighLevelNode start;
        start.solution.resize(initialStates.size());
        start.constraints.resize(initialStates.size());
        start.cost = 0;
        start.LB = 0;
        start.id = 0;
        start.hc = 0;

        for (size_t i = 0; i < initialStates.size(); ++i) {
            if (i < initialSolution.size() && initialSolution[i].states.size() > 1) {
                start.solution[i] = initialSolution[i];
            } else {
                if (!computeInitialSolution(start, i, initialStates[i])) {
                    throw std::runtime_error("Failed to find initial solution for agent " + std::to_string(i));
                }
            }
            start.cost += start.solution[i].cost;
            start.LB += start.solution[i].fmin;
        }

        start.hc = m_env.focalHeuristic(start.solution);
        start.fHat = start.cost + computeHHat(start);
        return start;
    }

    bool computeInitialSolution(HighLevelNode& node, size_t agentIdx, const State& initialState) {
        LowLevelEnvironment llenv(m_env, agentIdx, node.constraints[agentIdx], node.solution);
        LowLevelSearch_t lowLevel(llenv, m_w);
        Timer timer;
        return lowLevel.search(initialState, node.solution[agentIdx], timer, std::numeric_limits<double>::max());
    }

    HighLevelNode createNewNode(const HighLevelNode& parent, size_t agentIdx,
                                const Constraints& newConstraints, int newId,
                                const std::vector<State>& initialStates) {
        HighLevelNode newNode = parent;
        newNode.id = newId;
        newNode.constraints[agentIdx].add(newConstraints);
        newNode.cost -= newNode.solution[agentIdx].cost;
        newNode.LB -= newNode.solution[agentIdx].fmin;

        LowLevelEnvironment llenv(m_env, agentIdx, newNode.constraints[agentIdx], newNode.solution);
        LowLevelSearch_t lowLevel(llenv, m_w);
        Timer timer;
        bool success = lowLevel.search(initialStates[agentIdx], newNode.solution[agentIdx], timer, std::numeric_limits<double>::max());

        if (success) {
            newNode.cost += newNode.solution[agentIdx].cost;
            newNode.LB += newNode.solution[agentIdx].fmin;
            newNode.hc = m_env.focalHeuristic(newNode.solution);
            newNode.fHat = newNode.cost + computeHHat(newNode);
            updateOneStepErrors(parent, newNode);
            return newNode;
        }

        newNode.cost = -1;
        return newNode;
    }

    Cost estimateCostToGo(const HighLevelNode& node) {
        Cost h = 0;
        for (size_t i = 0; i < node.solution.size(); ++i) {
            const auto& s = node.solution[i].states.back().first;
            h += m_env.admissibleHeuristic(s);
        }
        updateAverageH(h);
        return h * (1 + m_averageError);
    }

    Cost computeFHat(const HighLevelNode& node){
        Cost costToGo = estimateCostToGo(node);
        Cost fHat = node.cost + costToGo;
        updateAverageError(fHat - node.cost);
        return fHat;
    }

    //xinjiade
    Cost computeHHat(const HighLevelNode& node){
        return std::abs((node.hc * m_averageError)/(1 - m_averageDistance));
        // Cost remainingCost = 0;
        // for (const auto& sol : node.solution) {
        //     if (!sol.states.empty()) {
        //         State lastState = sol.states.back().first;
        //         remainingCost += m_env.admissibleHeuristic(lastState);
        //     }
        // }
        // return remainingCost * (1 + m_averageError);
    }

    void updateOneStepErrors(const HighLevelNode& parent, const HighLevelNode& child){
        Cost distanceError = child.hc - (parent.hc - 1);
        Cost costError = child.cost - parent.cost;

        m_averageDistance = (m_averageDistance * m_error_count + distanceError) / (m_error_count + 1);
        m_averageError = (m_averageError * m_error_count + costError) / (m_error_count + 1);
        m_error_count++; 
    }

    void updateAverageH(Cost h) {
        m_averageH = (m_averageH * m_h_count + h) / (m_h_count + 1);
        m_h_count++;
    }

    void updateAverageError(Cost error){
        m_averageError = (m_averageError * m_error_count +error) / (m_error_count +1);
        m_error_count++;
    }

    void updateFocalList(openSet_t& open, focalSet_t& focal) {
        Cost bestFHat = open.top().fHat;
        for (auto iter = open.ordered_begin(); iter != open.ordered_end(); ++iter) {
            if (iter->fHat <= m_k * bestFHat && 
                std::find_if(focal.begin(), focal.end(),
                    [&](const HighLevelNode& n) { return n.id == iter->id; }) == focal.end()) {
                focal.push(*iter)  ;
            }
            else {
                break;
            }
        }
    }

    HighLevelNode selectNode(focalSet_t& focal, openSet_t& open, cleanupSet_t& cleanup) {

        HighLevelNode P;

        if (!focal.empty() && focal.top().cost <= m_k * cleanup.top().LB) {
            P = focal.top();
            focal.pop();
            open.erase(P.open_handle);
            cleanup.erase(P.cleanup_handle);
        } else if (!open.empty() && open.top().cost <= m_k * cleanup.top().LB){
            P = open.top();
            open.pop();
            cleanup.erase(P.cleanup_handle);
            if (P.in_focal_list) {
                focal.erase(P.focal_handle);
            }
        } else {
            P = cleanup.top();
            cleanup.pop();
            open.erase(P.open_handle);
            if(P.in_focal_list) {
                focal.erase(P.focal_handle);
            }
        }

        return P;
    }

    struct LowLevelEnvironment {
        LowLevelEnvironment(Environment& env, size_t agentIdx, const Constraints& constraints,
                            const std::vector<PlanResult<State, Action, Cost>>& solution)
            : m_env(env), m_solution(solution) {
            m_env.setLowLevelContext(agentIdx, &constraints);
        }

        Cost admissibleHeuristic(const State& s) { return m_env.admissibleHeuristic(s); }
        Cost focalStateHeuristic(const State& s, Cost gScore) { return m_env.focalStateHeuristic(s, gScore, m_solution); }
        Cost focalTransitionHeuristic(const State& s1, const State& s2, Cost gScoreS1, Cost gScoreS2) {
            return m_env.focalTransitionHeuristic(s1, s2, gScoreS1, gScoreS2, m_solution);
        }
        bool isSolution(const State& s) { return m_env.isSolution(s); }
        void getNeighbors(const State& s, std::vector<Neighbor<State, Action, Cost>>& neighbors) { m_env.getNeighbors(s, neighbors); }
        void onExpandNode(const State& s, Cost fScore, Cost gScore) { m_env.onExpandLowLevelNode(s, fScore, gScore); }
        void onDiscover(const State& s, Cost fScore, Cost gScore) { }

    private:
        Environment& m_env;
        const std::vector<PlanResult<State, Action, Cost>>& m_solution;
    };

    Environment& m_env;
    float m_w;
    double m_k;
    Cost m_averageError;
    Cost m_averageDistance;
    int m_error_count;
    Cost m_averageH;
    int m_h_count;

    Cost m_focal_best_fhat;

    using LowLevelSearch_t = AStarEpsilon<State, Action, Cost, LowLevelEnvironment>;
};

// template<typename State, typename Action, typename Cost, typename Conflict,
//          typename Constraints, typename Environment>
//  bool EECBS<State, Action, Cost,Conflict, Constraints, Environment>::CompareFocalHeuristic::operator()(
//     const HighLevelNode& n1, const HighLevelNode& n2) const
//  {
//     if(n1.focalHeuristic != n2.focalHeuristic){
//         return n1.focalHeuristic > n2.focalHeuristic;
//     }
//     return n1.fHat > n2.fHat;
//  }

template<typename State, typename Action, typename Cost, typename Conflict,
         typename Constraints, typename Environment>
 bool EECBS<State, Action, Cost,Conflict, Constraints, Environment>::search(const std::vector<State>& initialStates,
                std::vector<PlanResult<State, Action, Cost>>& solution,
                const double& time_tolerance) {
        Timer timer;
        HighLevelNode start = createStartNode(initialStates, solution);
        
        openSet_t open;
        focalSet_t focal;
        cleanupSet_t cleanup;
        
        start.in_focal_list = true;
        auto open_h = open.push(start);
        auto focal_h = focal.push(start);
        auto cleanup_h = cleanup.push(start);

        (*open_h).open_handle = open_h;
        (*open_h).focal_handle = focal_h;
        (*open_h).cleanup_handle = cleanup_h;
        (*focal_h).open_handle = open_h;
        (*focal_h).focal_handle = focal_h;
        (*focal_h).cleanup_handle = cleanup_h;
        (*cleanup_h).open_handle = open_h;
        (*cleanup_h).focal_handle = focal_h;
        (*cleanup_h).cleanup_handle = cleanup_h;

        solution.clear();
        int id = 1;

        while (!open.empty()) {
            timer.stop();
            if (timer.elapsedSeconds() > time_tolerance) {
                return false;
            }

            ROS_ERROR("DEBUG: Entering selectNode...");
            HighLevelNode P = selectNode(focal, open, cleanup);
            ROS_ERROR("DEBUG: Exited selectNode successfully with node ID %d", P.id);

            m_env.onExpandHighLevelNode(P.cost);

            Conflict conflict;
            if (!m_env.getFirstConflict(P.solution, conflict)) {
                solution = P.solution;
                return true;
            }

            std::map<size_t, Constraints> constraints;
            m_env.createConstraintsFromConflict(conflict, constraints);

            for (const auto& c : constraints) {
                size_t i = c.first;
                HighLevelNode newNode = createNewNode(P, i, c.second, id, initialStates);

                if (newNode.cost > 0) {  // Indicates successful child creation
                    // 默认新节点不在focal列表
                    newNode.in_focal_list = false; 

                    auto new_open_h = open.push(newNode);
                    auto new_cleanup_h = cleanup.push(newNode);

                    (*new_open_h).open_handle = new_open_h;
                    (*new_open_h).cleanup_handle = new_cleanup_h;
                    (*new_cleanup_h).open_handle = new_open_h;
                    (*new_cleanup_h).cleanup_handle = new_cleanup_h;


                    // 判断是否需要加入focal 列表
                    if (newNode.fHat <= m_k * open.top().fHat) {
                        newNode.in_focal_list = true;
                        auto new_focal_h = focal.push(newNode);
                    
                        // 将focal句柄也保存起来
                        (*new_open_h).focal_handle = new_focal_h;
                        (*new_open_h).in_focal_list = true;
                        (*new_cleanup_h).focal_handle = new_focal_h;
                        (*new_cleanup_h).in_focal_list = true;
                        (*new_focal_h).open_handle = new_open_h;
                        (*new_focal_h).cleanup_handle = new_cleanup_h;
                        (*new_focal_h).focal_handle = new_focal_h;
                        (*new_focal_h).in_focal_list = true;
                    }
                }
                ++id;

                timer.stop();
                if (timer.elapsedSeconds() > time_tolerance) {
                    return false;
                }
            }
        }
        return false;
    }

} // namespace mapf

