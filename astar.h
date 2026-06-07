#pragma once

#include "types.h"
#include "map3d.h"
#include <unordered_map>
#include <queue>
#include <chrono>

class AStar {
public:
    AStar(const SearchConfig& config);

    // Main entry point: find path from start to goal(s)
    // If visit_all_goals is true, visits all goals in greedy nearest-neighbor order.
    // Otherwise, finds path to the single nearest goal.
    SearchResult search(const Map3D& map, ProgressCallback progress_cb = nullptr);

private:
    // Single A* search from start to a specific goal
    // Returns the path segment and updates came_from for reconstruction
    bool searchSingle(
        const Map3D& map,
        const Pos3D& segment_start,
        const Pos3D& segment_goal,
        std::vector<Pos3D>& path_out,
        int& nodes_explored,
        ProgressCallback progress_cb,
        int segment_idx,
        int total_segments
    );

    // Find the nearest goal from a position using heuristic as lower bound
    int findNearestGoal(const Pos3D& from, const std::vector<Pos3D>& goals,
                        const std::vector<bool>& visited, const Map3D& map) const;

    SearchConfig config_;
};
