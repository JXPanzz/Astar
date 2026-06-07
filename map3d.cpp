#include "map3d.h"
#include <algorithm>
#include <cstring>
#include <iostream>

Map3D::Map3D(const MapConfig& config)
    : config_(config)
{
    cells_.resize(totalCells(), static_cast<uint8_t>(CellType::FREE));
}

void Map3D::generate() {
    std::mt19937 rng(config_.random_seed);

    std::cout << "Generating " << config_.width << "x" << config_.height
              << "x" << config_.depth << " 3D map...\n";

    // 1. Generate blocking obstacles (impassable walls)
    generateBlockingObstacles(rng);
    std::cout << "  Blocking obstacles placed.\n";

    // 2. Create danger zones around blocking obstacles (must keep distance)
    generateDangerZones();
    std::cout << "  Danger zones created.\n";

    // 3. Generate costly obstacles (traversable but expensive)
    generateCostlyObstacles(rng);
    std::cout << "  Costly obstacles placed.\n";

    // 4. Place start and goal positions
    placeStartAndGoals(rng);
    std::cout << "  Start and " << goals_.size() << " goals placed.\n";

    // Stats
    size_t blocking = 0, costly = 0, danger = 0, free_cnt = 0;
    for (auto c : cells_) {
        switch (static_cast<CellType>(c)) {
            case CellType::BLOCKING_OBSTACLE: blocking++; break;
            case CellType::COSTLY_OBSTACLE:   costly++;   break;
            case CellType::DANGER_ZONE:       danger++;   break;
            default:                          free_cnt++; break;
        }
    }
    float pct_block = 100.0f * blocking / totalCells();
    float pct_costly = 100.0f * costly / totalCells();
    float pct_danger = 100.0f * danger / totalCells();
    float pct_free = 100.0f * free_cnt / totalCells();

    std::cout << "  Map composition: "
              << "Free=" << pct_free << "%, "
              << "Block=" << pct_block << "%, "
              << "Costly=" << pct_costly << "%, "
              << "Danger=" << pct_danger << "%\n";
}

CellType Map3D::getCell(int x, int y, int z) const {
    if (!isInBounds(x, y, z)) return CellType::BLOCKING_OBSTACLE;
    return static_cast<CellType>(cells_[index(x, y, z)]);
}

bool Map3D::isInBounds(int x, int y, int z) const {
    return x >= 0 && x < config_.width &&
           y >= 0 && y < config_.height &&
           z >= 0 && z < config_.depth;
}

bool Map3D::isTraversable(int x, int y, int z) const {
    if (!isInBounds(x, y, z)) return false;
    CellType t = static_cast<CellType>(cells_[index(x, y, z)]);
    return t != CellType::BLOCKING_OBSTACLE;
}

float Map3D::getMoveCost(const Pos3D& from, const Pos3D& to) const {
    if (!isInBounds(to)) return std::numeric_limits<float>::infinity();

    CellType cell_type = static_cast<CellType>(cells_[index(to.x, to.y, to.z)]);

    // Impassable
    if (cell_type == CellType::BLOCKING_OBSTACLE) {
        return std::numeric_limits<float>::infinity();
    }

    float base_cost;
    switch (cell_type) {
        case CellType::FREE:            base_cost = config_.free_cost; break;
        case CellType::COSTLY_OBSTACLE: base_cost = config_.costly_obstacle_cost; break;
        case CellType::DANGER_ZONE:     base_cost = config_.danger_zone_cost; break;
        default:                        base_cost = config_.free_cost; break;
    }

    // Add layer-change penalty if z differs
    if (from.z != to.z) {
        base_cost += config_.layer_penalty;
    }

    return base_cost;
}

std::vector<std::pair<Pos3D, float>> Map3D::getNeighbors(const Pos3D& pos) const {
    std::vector<std::pair<Pos3D, float>> neighbors;
    neighbors.reserve(6);

    for (int d = 0; d < 6; ++d) {
        Pos3D npos(pos.x + DX[d], pos.y + DY[d], pos.z + DZ[d]);
        if (!isInBounds(npos)) continue;
        if (!isTraversable(npos)) continue;

        float cost = getMoveCost(pos, npos);
        if (std::isinf(cost)) continue;

        neighbors.emplace_back(npos, cost);
    }
    return neighbors;
}

// ---- Private generation methods -------------------------------------------

void Map3D::generateBlockingObstacles(std::mt19937& rng) {
    int target_cells = static_cast<int>(totalCells() * config_.blocking_density);
    int placed = 0;

    std::uniform_int_distribution<int> wx(0, config_.width - 1);
    std::uniform_int_distribution<int> wy(0, config_.height - 1);
    std::uniform_int_distribution<int> wz(0, config_.depth - 1);
    std::uniform_int_distribution<int> cluster_size(5, config_.obstacle_cluster_size);

    while (placed < target_cells) {
        // Pick a random seed point
        int cx = wx(rng), cy = wy(rng), cz = wz(rng);
        int size = cluster_size(rng);

        // Grow a cluster using random walk
        int cluster_x = cx, cluster_y = cy, cluster_z = cz;
        std::uniform_int_distribution<int> step(-2, 2);
        std::uniform_int_distribution<int> layer_step(-1, 1);

        for (int i = 0; i < size && placed < target_cells; ++i) {
            cluster_x = std::clamp(cluster_x + step(rng), 0, config_.width - 1);
            cluster_y = std::clamp(cluster_y + step(rng), 0, config_.height - 1);
            cluster_z = std::clamp(cluster_z + layer_step(rng), 0, config_.depth - 1);

            size_t idx = index(cluster_x, cluster_y, cluster_z);
            if (cells_[idx] == static_cast<uint8_t>(CellType::FREE)) {
                cells_[idx] = static_cast<uint8_t>(CellType::BLOCKING_OBSTACLE);
                ++placed;
            }
        }
    }
}

void Map3D::generateDangerZones() {
    // For each blocking obstacle cell, mark adjacent FREE cells as DANGER_ZONE.
    // Uses only the 6 movement directions (N,S,E,W,Up,Down) since those are
    // the only directions the path can approach from.
    std::vector<uint8_t> new_cells = cells_; // Work on a copy

    for (int z = 0; z < config_.depth; ++z) {
        for (int y = 0; y < config_.height; ++y) {
            for (int x = 0; x < config_.width; ++x) {
                if (cells_[index(x, y, z)] != static_cast<uint8_t>(CellType::BLOCKING_OBSTACLE))
                    continue;

                // Mark 6-directional neighbors as danger zones
                for (int d = 0; d < 6; ++d) {
                    int nx = x + DX[d], ny = y + DY[d], nz = z + DZ[d];
                    if (!isInBounds(nx, ny, nz)) continue;
                    size_t nidx = index(nx, ny, nz);
                    if (new_cells[nidx] == static_cast<uint8_t>(CellType::FREE)) {
                        new_cells[nidx] = static_cast<uint8_t>(CellType::DANGER_ZONE);
                    }
                }
            }
        }
    }
    cells_ = std::move(new_cells);
}

void Map3D::generateCostlyObstacles(std::mt19937& rng) {
    int target_cells = static_cast<int>(totalCells() * config_.costly_density);
    int placed = 0;

    std::uniform_int_distribution<int> wx(0, config_.width - 1);
    std::uniform_int_distribution<int> wy(0, config_.height - 1);
    std::uniform_int_distribution<int> wz(0, config_.depth - 1);

    // Place costly obstacles only on FREE cells (not on blocking or danger zones)
    while (placed < target_cells) {
        int attempts = 0;
        while (attempts++ < 100) {
            int rx = wx(rng), ry = wy(rng), rz = wz(rng);
            size_t idx = index(rx, ry, rz);
            if (cells_[idx] == static_cast<uint8_t>(CellType::FREE)) {
                cells_[idx] = static_cast<uint8_t>(CellType::COSTLY_OBSTACLE);
                ++placed;
                break;
            }
        }
        if (attempts >= 100) break; // No more free cells available
    }
}

void Map3D::placeStartAndGoals(std::mt19937& rng) {
    std::uniform_int_distribution<int> wx(0, config_.width - 1);
    std::uniform_int_distribution<int> wy(0, config_.height - 1);
    std::uniform_int_distribution<int> wz(0, config_.depth - 1);

    // Place start in a free cell
    while (true) {
        Pos3D s(wx(rng), wy(rng), wz(rng));
        if (getCell(s) == CellType::FREE) {
            start_ = s;
            break;
        }
    }

    // Place goals in free cells on different layers if possible
    goals_.clear();
    std::uniform_int_distribution<int> gz(0, config_.depth - 1);

    for (int i = 0; i < config_.num_goals; ++i) {
        int attempts = 0;
        while (attempts++ < 10000) {
            Pos3D g(wx(rng), wy(rng), gz(rng));
            if (getCell(g) == CellType::FREE && g != start_) {
                // Check it's not too close to start or other goals
                bool too_close = false;
                if (g.manhattanTo(start_, 1.0f) < config_.width * 0.1f) too_close = true;
                for (const auto& og : goals_) {
                    if (g.manhattanTo(og, 1.0f) < config_.width * 0.1f) too_close = true;
                }
                if (!too_close) {
                    goals_.push_back(g);
                    break;
                }
            }
        }
        if (attempts >= 10000) {
            // Fallback: just place anywhere free
            while (true) {
                Pos3D g(wx(rng), wy(rng), gz(rng));
                if (getCell(g) == CellType::FREE && g != start_) {
                    goals_.push_back(g);
                    break;
                }
            }
        }
    }

    std::cout << "  Start: (" << start_.x << ", " << start_.y << ", " << start_.z << ")\n";
    for (size_t i = 0; i < goals_.size(); ++i) {
        std::cout << "  Goal " << i << ":  (" << goals_[i].x << ", "
                  << goals_[i].y << ", " << goals_[i].z << ")\n";
    }
}

bool Map3D::hasClearPath(const Pos3D& a, const Pos3D& b) const {
    // Simple diagonal check using Bresenham-like line
    int dx = std::abs(b.x - a.x), dy = std::abs(b.y - a.y), dz = std::abs(b.z - a.z);
    int steps = std::max({dx, dy, dz});
    if (steps == 0) return true;

    for (int i = 0; i <= steps; ++i) {
        float t = static_cast<float>(i) / steps;
        int x = a.x + static_cast<int>((b.x - a.x) * t + 0.5f);
        int y = a.y + static_cast<int>((b.y - a.y) * t + 0.5f);
        int z = a.z + static_cast<int>((b.z - a.z) * t + 0.5f);
        if (!isTraversable(x, y, z)) return false;
    }
    return true;
}
