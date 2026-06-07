#pragma once

#include <cstdint>
#include <cmath>
#include <limits>
#include <vector>
#include <functional>

// ============================================================
// Cell types in the 3D map
// ============================================================
enum class CellType : uint8_t {
    FREE = 0,              // Traversable at normal cost
    BLOCKING_OBSTACLE = 1, // Impassable, must keep distance
    COSTLY_OBSTACLE = 2,   // Traversable but at high cost
    DANGER_ZONE = 3        // Buffer zone around blocking obstacles
};

// ============================================================
// 3D position with utility functions
// ============================================================
struct Pos3D {
    int x, y, z;

    Pos3D() : x(0), y(0), z(0) {}
    Pos3D(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}

    bool operator==(const Pos3D& o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator!=(const Pos3D& o) const { return !(*this == o); }

    // Pack into a 64-bit key for hash maps
    uint64_t key() const {
        return (static_cast<uint64_t>(z) << 40) |
               (static_cast<uint64_t>(y) << 20) |
               static_cast<uint64_t>(x);
    }

    // Manhattan distance in 3D (x,y use unit cost, z uses layer penalty)
    float manhattanTo(const Pos3D& o, float layer_penalty) const {
        return std::abs(x - o.x) + std::abs(y - o.y) + layer_penalty * std::abs(z - o.z);
    }

    // Euclidean distance in 3D (z scaled by layer penalty)
    float distanceTo(const Pos3D& o, float layer_penalty) const {
        float dz = layer_penalty * (z - o.z);
        return std::sqrt(static_cast<float>((x - o.x) * (x - o.x) +
                                            (y - o.y) * (y - o.y)) + dz * dz);
    }

    // Chebyshev distance (octile), useful as a tighter heuristic
    float chebyshevTo(const Pos3D& o, float layer_penalty) const {
        float dxy = std::max(std::abs(x - o.x), std::abs(y - o.y));
        float dz = layer_penalty * std::abs(z - o.z);
        return std::max(dxy, dz);
    }
};

// ============================================================
// A node in the A* open set
// ============================================================
struct AStarNode {
    Pos3D pos;
    float g_cost;   // Cost from start to this node
    float h_cost;   // Heuristic cost to goal
    uint64_t parent_key; // Key of parent node (0 means none)

    float fCost(float weight) const {
        return g_cost + weight * h_cost;
    }
};

// Comparator for priority queue (min-heap), takes weight into account
struct AStarNodeComparator {
    float weight;
    explicit AStarNodeComparator(float w) : weight(w) {}
    bool operator()(const AStarNode& a, const AStarNode& b) const {
        float fa = a.g_cost + weight * a.h_cost;
        float fb = b.g_cost + weight * b.h_cost;
        if (fa != fb) return fa > fb;  // Min-heap: larger f pops later
        return a.h_cost > b.h_cost;     // Tiebreak: prefer lower h
    }
};

// ============================================================
// Search result
// ============================================================
struct SearchResult {
    std::vector<Pos3D> path;       // Complete path (all segments concatenated)
    std::vector<std::vector<Pos3D>> segments; // Individual goal-to-goal segments
    int nodes_explored = 0;        // Total nodes expanded
    int nodes_visited = 0;         // Total nodes added to open set
    float total_cost = 0.0f;       // Total path cost
    double time_ms = 0.0;          // Search time in milliseconds
    bool success = false;          // Whether a complete path was found
};

// ============================================================
// Map configuration
// ============================================================
struct MapConfig {
    int width = 1000;
    int height = 1000;
    int depth = 10;

    float layer_penalty = 5.0f;     // Cost multiplier for changing layers
    float costly_obstacle_cost = 10.0f; // Cost multiplier on costly obstacles
    float danger_zone_cost = 100.0f;    // Cost multiplier in danger zones
    float free_cost = 1.0f;         // Base movement cost on free cells

    float blocking_density = 0.08f; // Fraction of map covered by blocking obstacles
    float costly_density = 0.05f;   // Fraction of map covered by costly obstacles
    int obstacle_cluster_size = 30; // Average size of obstacle clusters

    int num_goals = 3;              // Number of goal positions
    unsigned random_seed = 42;      // Random seed for reproducibility
};

// ============================================================
// Search configuration
// ============================================================
struct SearchConfig {
    float weight = 2.0f;           // Weight for weighted A* (>1 = faster, less optimal)
    int progress_interval = 1000;  // Callback invoked every N node expansions
    bool visit_all_goals = true;   // If true, visit all goals sequentially; else go to nearest
};

// ============================================================
// Progress callback type
// ============================================================
struct SearchProgress {
    int nodes_explored = 0;
    int open_set_size = 0;
    float current_best_f = 0.0f;
    float current_best_g = 0.0f;
    Pos3D current_node;
    int segment_index = 0;
    int total_segments = 0;
};

using ProgressCallback = std::function<void(const SearchProgress&)>;

// ============================================================
// 6 possible movement directions in 3D grid
// ============================================================
constexpr int DX[6] = { 1, -1,  0,  0,  0,  0};
constexpr int DY[6] = { 0,  0,  1, -1,  0,  0};
constexpr int DZ[6] = { 0,  0,  0,  0,  1, -1};

// Direction names for debugging
constexpr const char* DIR_NAMES[6] = {"E", "W", "S", "N", "Up", "Dn"};

// ============================================================
// Utility: reconstruct path from came_from map
// ============================================================
inline std::vector<Pos3D> reconstructPath(
    uint64_t start_key, uint64_t goal_key,
    const std::unordered_map<uint64_t, uint64_t>& came_from) {

    std::vector<Pos3D> path;
    uint64_t current = goal_key;

    while (current != start_key) {
        int z = static_cast<int>(current >> 40);
        int y = static_cast<int>((current >> 20) & 0xFFFFF);
        int x = static_cast<int>(current & 0xFFFFF);
        path.push_back(Pos3D(x, y, z));

        auto it = came_from.find(current);
        if (it == came_from.end()) break;
        current = it->second;
    }
    // Add start position
    {
        int z = static_cast<int>(start_key >> 40);
        int y = static_cast<int>((start_key >> 20) & 0xFFFFF);
        int x = static_cast<int>(start_key & 0xFFFFF);
        path.push_back(Pos3D(x, y, z));
    }
    std::reverse(path.begin(), path.end());
    return path;
}
