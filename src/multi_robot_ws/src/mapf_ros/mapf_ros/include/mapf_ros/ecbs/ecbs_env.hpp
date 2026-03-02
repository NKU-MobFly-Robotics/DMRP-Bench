#pragma once

#ifndef ECBS_ENV_H
#define ECBS_ENV_H

#include "../utils/utility.hpp"

#include "../utils/neighbor.hpp"
#include "../utils/planresult.hpp"

using mapf::Neighbor;
using mapf::PlanResult;

struct State {
  State(int time, int x, int y) : time(time), x(x), y(y) {}

  bool operator==(const State &s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State &s) const { return x == s.x && y == s.y; }

  friend std::ostream &operator<<(std::ostream &os, const State &s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
};

namespace std {
template <> struct hash<State> {
  size_t operator()(const State &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

///
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
  UpLeft,    // Added
  UpRight,   // Added
  DownLeft,  // Added
  DownRight,  // Added
};

std::ostream &operator<<(std::ostream &os, const Action &a) {
  switch (a) {
  case Action::Up:
    os << "Up";
    break;
  case Action::Down:
    os << "Down";
    break;
  case Action::Left:
    os << "Left";
    break;
  case Action::Right:
    os << "Right";
    break;
  case Action::Wait:
    os << "Wait";
    break;
  case Action::UpLeft: os << "UpLeft"; break;
  case Action::UpRight: os << "UpRight"; break;
  case Action::DownLeft: os << "DownLeft"; break;
  case Action::DownRight: os << "DownRight"; break;
  }
  return os;
}

///

struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  friend std::ostream &operator<<(std::ostream &os, const Conflict &c) {
    switch (c.type) {
    case Vertex:
      return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
    case Edge:
      return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                << "," << c.y2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint &other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint &other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream &operator<<(std::ostream &os, const VertexConstraint &c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};

namespace std {
template <> struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint &other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint &other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream &operator<<(std::ostream &os, const EdgeConstraint &c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};

namespace std {
template <> struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
} // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints &other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints &other) const {
    for (const auto &vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }
    for (const auto &ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        return true;
      }
    }
    return false;
  }

  friend std::ostream &operator<<(std::ostream &os, const Constraints &c) {
    for (const auto &vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto &ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location &other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location &other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream &operator<<(std::ostream &os, const Location &c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <> struct hash<Location> {
  size_t operator()(const Location &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

///
class Environment {
public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals, bool disappearAtGoal = false)
      : m_dimx(dimx), m_dimy(dimy), m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)), m_agentIdx(0), m_constraints(nullptr),
        m_lastGoalConstraint(-1), m_highLevelExpanded(0), m_lowLevelExpanded(0),
        m_disappearAtGoal(disappearAtGoal) {}

  Environment(const Environment &) = delete;
  Environment &operator=(const Environment &) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints *constraints) {
    assert(constraints); // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto &vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  // int admissibleHeuristic(const State &s) {
  //   return std::abs(s.x - m_goals[m_agentIdx].x) +
  //          std::abs(s.y - m_goals[m_agentIdx].y);
  // }

  float admissibleHeuristic(const State &s) {
  int dx = std::abs(s.x - m_goals[m_agentIdx].x);
  int dy = std::abs(s.y - m_goals[m_agentIdx].y);
  // Chebyshev distance for 8-connectivity (max of dx, dy for orthogonal, scaled for diagonal)
  return static_cast<float>(std::max(dx, dy)) * 1.0f + 
         static_cast<float>(std::min(dx, dy)) * (1.414f - 1.0f);
  }
  
  State getGoalState() const {
        return State(0, m_goals[m_agentIdx].x, m_goals[m_agentIdx].y);
  }
  // low-level
  float focalStateHeuristic(
      const State &s, int /*gScore*/,
      const std::vector<PlanResult<State, Action, float>> &solution) {
    float numConflicts = 0.0f;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State state2 = getState(i, solution, s.time);
        if (s.equalExceptTime(state2)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // low-level
  float focalTransitionHeuristic(
      const State &s1a, const State &s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State, Action, float>> &solution) {
    float numConflicts = 0.0f;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // Count all conflicts
  float focalHeuristic(
      const std::vector<PlanResult<State, Action, float>> &solution) {
    float numConflicts = 0.0f;

    int max_t = 0;
    for (const auto &sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            ++numConflicts;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }

  bool isSolution(const State &s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State &s,
                    std::vector<Neighbor<State, Action, float>> &neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Wait, 1.0f));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Left, 1.0f));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Right, 1.0f));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, float>(n, Action::Up, 1.0f));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Down, 1.0f));
      }
    }
    // Diagonal movements (cost = sqrt(2))
  const float sqrt2 = 1.414f;
  {
    State n(s.time + 1, s.x - 1, s.y + 1);
    State check1(s.time + 1, s.x - 1, s.y); // Left
    State check2(s.time + 1, s.x, s.y + 1); // Up
    if (stateValid(n) && transitionValid(s, n) && stateValid(check1) && stateValid(check2)) {
      neighbors.emplace_back(Neighbor<State, Action, float>(n, Action::UpLeft, sqrt2));
    }
  }
  {
    State n(s.time + 1, s.x + 1, s.y + 1);
    State check1(s.time + 1, s.x + 1, s.y); // Right
    State check2(s.time + 1, s.x, s.y + 1); // Up
    if (stateValid(n) && transitionValid(s, n) && stateValid(check1) && stateValid(check2)) {
      neighbors.emplace_back(Neighbor<State, Action, float>(n, Action::UpRight, sqrt2));
    }
  }
  {
    State n(s.time + 1, s.x - 1, s.y - 1);
    State check1(s.time + 1, s.x - 1, s.y); // Left
    State check2(s.time + 1, s.x, s.y - 1); // Down
    if (stateValid(n) && transitionValid(s, n) && stateValid(check1) && stateValid(check2)) {
      neighbors.emplace_back(Neighbor<State, Action, float>(n, Action::DownLeft, sqrt2));
    }
  }
  {
    State n(s.time + 1, s.x + 1, s.y - 1);
    State check1(s.time + 1, s.x + 1, s.y); // Right
    State check2(s.time + 1, s.x, s.y - 1); // Down
    if (stateValid(n) && transitionValid(s, n) && stateValid(check1) && stateValid(check2)) {
      neighbors.emplace_back(Neighbor<State, Action, float>(n, Action::DownRight, sqrt2));
    }
  }
  }


  bool getFirstConflict(const std::vector<PlanResult<State, Action, float>> &solution, Conflict &result) {
  int max_t = 0;
  for (const auto &sol : solution) {
    max_t = std::max<int>(max_t, sol.states.size() - 1);
  }

  for (int t = 0; t <= max_t; ++t) {
    // Vertex collisions
    for (size_t i = 0; i < solution.size(); ++i) {
      State state1 = getState(i, solution, t);
      for (size_t j = i + 1; j < solution.size(); ++j) {
        State state2 = getState(j, solution, t);
        if (state1.equalExceptTime(state2)) {
          result.time = t;
          result.agent1 = i;
          result.agent2 = j;
          result.type = Conflict::Vertex;
          result.x1 = state1.x;
          result.y1 = state1.y;
          return true;
        }
      }
    }
    // Edge collisions (swap)
    for (size_t i = 0; i < solution.size(); ++i) {
      State state1a = getState(i, solution, t);
      State state1b = getState(i, solution, t + 1);
      for (size_t j = i + 1; j < solution.size(); ++j) {
        State state2a = getState(j, solution, t);
        State state2b = getState(j, solution, t + 1);
        if (state1a.equalExceptTime(state2b) && state1b.equalExceptTime(state2a)) {
          result.time = t;
          result.agent1 = i;
          result.agent2 = j;
          result.type = Conflict::Edge;
          result.x1 = state1a.x;
          result.y1 = state1a.y;
          result.x2 = state1b.x;
          result.y2 = state1b.y;
          return true;
        }
      }
    }
    // Diagonal collisions
    for (size_t i = 0; i < solution.size(); ++i) {
      State state1a = getState(i, solution, t);
      State state1b = getState(i, solution, t + 1);
      int dx1 = state1b.x - state1a.x;
      int dy1 = state1b.y - state1a.y;
      bool isDiagonal1 = (std::abs(dx1) == 1 && std::abs(dy1) == 1);
      
      for (size_t j = i + 1; j < solution.size(); ++j) {
        State state2a = getState(j, solution, t);
        State state2b = getState(j, solution, t + 1);
        int dx2 = state2b.x - state2a.x;
        int dy2 = state2b.y - state2a.y;
        bool isDiagonal2 = (std::abs(dx2) == 1 && std::abs(dy2) == 1);

        if (isDiagonal1) {
          State mid1(t + 1, state1a.x + dx1, state1a.y); // e.g., (x+1, y)
          State mid2(t + 1, state1a.x, state1a.y + dy1); // e.g., (x, y+1)
          if (state2a.equalExceptTime(mid1) || state2a.equalExceptTime(mid2) || 
              state2b.equalExceptTime(mid1) || state2b.equalExceptTime(mid2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
        if (isDiagonal2) {
          State mid1(t + 1, state2a.x + dx2, state2a.y);
          State mid2(t + 1, state2a.x, state2a.y + dy2);
          if (state1b.equalExceptTime(mid1) || state1b.equalExceptTime(mid2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }
  }
  return false;
}


void createConstraintsFromConflict(const Conflict &conflict, std::map<size_t, Constraints> &constraints) {
  if (conflict.type == Conflict::Vertex) {
    Constraints c1;
    c1.vertexConstraints.emplace(VertexConstraint(conflict.time, conflict.x1, conflict.y1));
    constraints[conflict.agent1] = c1;
    constraints[conflict.agent2] = c1;
  } else if (conflict.type == Conflict::Edge) {
    Constraints c1, c2;
    int dx = conflict.x2 - conflict.x1;
    int dy = conflict.y2 - conflict.y1;
    bool isDiagonal = (std::abs(dx) == 1 && std::abs(dy) == 1);

    if (isDiagonal) {
      c1.edgeConstraints.emplace(EdgeConstraint(conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;

      // Conservatively block the intermediate point most likely to conflict
      // Assume conflict detected by getFirstConflict involves mid1 (e.g., (x1+dx, y1))
      c2.vertexConstraints.emplace(VertexConstraint(conflict.time + 1, conflict.x1 + dx, conflict.y1));
      constraints[conflict.agent2] = c2;
    } else {
      c1.edgeConstraints.emplace(EdgeConstraint(conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      c2.edgeConstraints.emplace(EdgeConstraint(conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }
}

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, float>> &solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    if (m_disappearAtGoal) {
      // This is a trick to avoid changing the rest of the code significantly
      // After an agent disappeared, put it at a unique but invalid position
      // This will cause all calls to equalExceptTime(.) to return false.
      return State(-1, -1 * (agentIdx + 1), -1);
    }
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State &s) {
    assert(m_constraints);
    const auto &con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State &s1, const State &s2) {
    assert(m_constraints);
    const auto &con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }

private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  size_t m_agentIdx;
  const Constraints *m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_disappearAtGoal;
};

#endif
