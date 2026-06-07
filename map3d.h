#pragma once

#include "types.h"
#include <vector>
#include <random>
#include <cstdint>

class Map3D {
public:
    explicit Map3D(const MapConfig& config);
    ~Map3D() = default;

    // Generate the map: obstacles, start, goals
    void generate();

    // ---- Accessors ----
    CellType getCell(int x, int y, int z) const;
    CellType getCell(const Pos3D& p) const { return getCell(p.x, p.y, p.z); }

    float getMoveCost(const Pos3D& from, const Pos3D& to) const;

    bool isInBounds(int x, int y, int z) const;
    bool isInBounds(const Pos3D& p) const { return isInBounds(p.x, p.y, p.z); }

    bool isTraversable(int x, int y, int z) const;
    bool isTraversable(const Pos3D& p) const { return isTraversable(p.x, p.y, p.z); }

    // Get valid neighboring positions with their movement costs
    // Returns vector of (position, cost) pairs
    std::vector<std::pair<Pos3D, float>> getNeighbors(const Pos3D& pos) const;

    // ---- Map dimensions ----
    int width()  const { return config_.width; }
    int height() const { return config_.height; }
    int depth()  const { return config_.depth; }
    size_t totalCells() const { return static_cast<size_t>(config_.width) *
                                       config_.height * config_.depth; }

    // ---- Start and goals ----
    const Pos3D& getStart() const { return start_; }
    const std::vector<Pos3D>& getGoals() const { return goals_; }

    // ---- Config ----
    const MapConfig& getConfig() const { return config_; }

private:
    void generateBlockingObstacles(std::mt19937& rng);
    void generateDangerZones();
    void generateCostlyObstacles(std::mt19937& rng);
    void placeStartAndGoals(std::mt19937& rng);
    bool hasClearPath(const Pos3D& a, const Pos3D& b) const;

    size_t index(int x, int y, int z) const {
        return static_cast<size_t>(z) * (static_cast<size_t>(config_.width) * config_.height) +
               static_cast<size_t>(y) * config_.width + static_cast<size_t>(x);
    }

    MapConfig config_;
    std::vector<uint8_t> cells_; // Flat 3D array of CellType values

    Pos3D start_;
    std::vector<Pos3D> goals_;
};
