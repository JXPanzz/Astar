#pragma once

#include "types.h"
#include "map3d.h"
#include <unordered_set>
#include <termios.h>

class Visualizer {
public:
    Visualizer();
    ~Visualizer();

    // Initialize terminal for raw mode
    void init();
    void cleanup();

    // Show a progress update during search (prints to stderr so it doesn't
    // interfere with the main display)
    void showProgress(const SearchProgress& progress);

    // Animate the final path on the map
    void animatePath(const Map3D& map, const SearchResult& result);

    // Interactive exploration mode after search
    void explore(const Map3D& map, const SearchResult& result);

private:
    void clearScreen();
    void moveCursor(int row, int col);
    void setColor(int fg, int bg = -1);
    void resetColor();
    void hideCursor();
    void showCursor();

    // Render a 2D slice of the map at the given layer
    void renderLayer(const Map3D& map, int layer,
                     const std::vector<Pos3D>& path,
                     const std::unordered_set<uint64_t>& path_set,
                     int viewport_cx, int viewport_cy,
                     int term_w, int term_h);

    // Get cell display character and color
    char cellChar(CellType t) const;
    int cellColor(CellType t) const;

    // Draw a single cell at screen position
    void drawCell(int screen_row, int screen_col, CellType type,
                  bool is_path, bool is_start, bool is_goal);

    // Non-blocking check for keypress
    int getKeyNonBlocking();

    // Statistics display
    void renderStats(const SearchResult& result, int current_layer);

    struct termios orig_termios_;
    bool initialized_ = false;
    int term_rows_ = 24;
    int term_cols_ = 80;
};

// Build a set of path keys for fast lookup
inline std::unordered_set<uint64_t> buildPathSet(const std::vector<Pos3D>& path) {
    std::unordered_set<uint64_t> s;
    s.reserve(path.size());
    for (const auto& p : path) s.insert(p.key());
    return s;
}
