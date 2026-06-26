#include "astar.h"
#include <iostream>
#include <cmath>

AStar::AStar(const SearchConfig& config) : config_(config) {}

SearchResult AStar::search(const Map3D& map, ProgressCallback progress_cb) {
    SearchResult result;
    auto start_time = std::chrono::high_resolution_clock::now();

    const Pos3D& start = map.getStart();
    const auto& all_goals = map.getGoals();

    if (all_goals.empty()) {
        result.success = false;
        return result;
    }

    std::vector<bool> visited_goals(all_goals.size(), false);
    int goals_remaining = config_.visit_all_goals ? static_cast<int>(all_goals.size()) : 1;

    Pos3D current_pos = start;
    int total_explored = 0;
    int total_visited = 0;

    for (int g = 0; g < goals_remaining; ++g) {
        // Find the nearest unvisited goal
        int nearest_idx = findNearestGoal(current_pos, all_goals, visited_goals, map);
        if (nearest_idx < 0) break;

        std::vector<Pos3D> segment;
        int segment_explored = 0;
        int segment_visited = 0;

        bool found = searchSingle(
            map, current_pos, all_goals[nearest_idx],
            segment, segment_explored, segment_visited,
            progress_cb, g, goals_remaining
        );

        total_explored += segment_explored;
        total_visited += segment_visited;

        if (!found) {
            std::cerr << "  Warning: Could not find path to goal " << nearest_idx
                      << " at (" << all_goals[nearest_idx].x << ", "
                      << all_goals[nearest_idx].y << ", "
                      << all_goals[nearest_idx].z << ")\n";
            // Try next goal
            visited_goals[nearest_idx] = true;
            continue;
        }

        // Remove the last position of previous segment (it's the same as first of this segment)
        if (!result.path.empty() && !segment.empty()) {
            result.path.pop_back();
        }

        result.path.insert(result.path.end(), segment.begin(), segment.end());
        result.segments.push_back(std::move(segment));
        visited_goals[nearest_idx] = true;
        current_pos = all_goals[nearest_idx];
    }

    // Calculate total cost
    result.total_cost = 0.0f;
    for (size_t i = 1; i < result.path.size(); ++i) {
        result.total_cost += map.getMoveCost(result.path[i - 1], result.path[i]);
    }

    result.nodes_explored = total_explored;
    result.nodes_visited = total_visited;
    // Success if we found at least one path segment (partial success is still useful)
    result.success = !result.segments.empty();

    // If visit_all_goals mode, also verify all goals were reached
    if (result.success && config_.visit_all_goals) {
        result.success = (result.segments.size() == all_goals.size());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    return result;
}

bool AStar::searchSingle(
    const Map3D& map,
    const Pos3D& segment_start,
    const Pos3D& segment_goal,
    std::vector<Pos3D>& path_out,
    int& nodes_explored,
    int& nodes_visited,
    ProgressCallback progress_cb,
    int segment_idx,
    int total_segments) {

    float layer_penalty = map.getConfig().layer_penalty;

    // Open set: priority queue (min-heap by f-cost)
    AStarNodeComparator cmp(config_.weight);
    std::priority_queue<AStarNode, std::vector<AStarNode>, AStarNodeComparator> open_set(cmp);

    // g_score: cost from start to node (sparse, only for visited nodes)
    std::unordered_map<uint64_t, float> g_score;

    // came_from: reconstruct path
    std::unordered_map<uint64_t, uint64_t> came_from;

    // closed set (we use g_score.find() == end to mean "not visited")

    uint64_t start_key = segment_start.key();
    uint64_t goal_key = segment_goal.key();

    // If start == goal
    if (segment_start == segment_goal) {
        path_out.push_back(segment_start);
        nodes_explored = 0;
        nodes_visited = 0;
        return true;
    }

    // Push start node
    float h_start = segment_start.manhattanTo(segment_goal, layer_penalty);
    g_score[start_key] = 0.0f;
    open_set.push({segment_start, 0.0f, h_start, 0});

    int explored = 0;
    int visited = 1;  // start node added to open set
    SearchProgress progress;
    progress.segment_index = segment_idx;
    progress.total_segments = total_segments;

    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        uint64_t current_key = current.pos.key();

        // Skip if we've found a better path to this node
        auto g_it = g_score.find(current_key);
        if (g_it != g_score.end() && current.g_cost > g_it->second) {
            continue;
        }

        ++explored;

        // Goal check
        if (current.pos == segment_goal) {
            nodes_explored = explored;
            nodes_visited = visited;
            path_out = reconstructPath(start_key, goal_key, came_from);
            return true;
        }

        // Progress callback
        if (progress_cb && explored % config_.progress_interval == 0) {
            progress.nodes_explored = explored;
            progress.open_set_size = static_cast<int>(open_set.size());
            progress.current_best_f = current.g_cost + config_.weight * current.h_cost;
            progress.current_best_g = current.g_cost;
            progress.current_node = current.pos;
            progress_cb(progress);
        }

        // Expand neighbors
        auto neighbors = map.getNeighbors(current.pos);
        for (const auto& [npos, move_cost] : neighbors) {
            float tentative_g = current.g_cost + move_cost;

            uint64_t nkey = npos.key();
            auto ng_it = g_score.find(nkey);
            float old_g = (ng_it != g_score.end()) ? ng_it->second
                                                    : std::numeric_limits<float>::infinity();

            if (tentative_g < old_g) {
                // This path is better
                g_score[nkey] = tentative_g;
                came_from[nkey] = current_key;

                float h = npos.manhattanTo(segment_goal, layer_penalty);
                open_set.push({npos, tentative_g, h, current_key});
                ++visited;
            }
        }
    }

    nodes_explored = explored;
    nodes_visited = visited;
    return false; // No path found
}

bool AStar::searchPath(const Map3D& map, const Pos3D& from, const Pos3D& to,
                       std::vector<Pos3D>& path_out) {
    int explored = 0, visited = 0;
    return searchSingle(map, from, to, path_out, explored, visited, nullptr, 0, 1);
}

int AStar::findNearestGoal(const Pos3D& from, const std::vector<Pos3D>& goals,
                            const std::vector<bool>& visited, const Map3D& map) const {
    int best_idx = -1;
    float best_dist = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < goals.size(); ++i) {
        if (visited[i]) continue;
        float d = from.manhattanTo(goals[i], map.getConfig().layer_penalty);
        if (d < best_dist) {
            best_dist = d;
            best_idx = static_cast<int>(i);
        }
    }
    return best_idx;
}
