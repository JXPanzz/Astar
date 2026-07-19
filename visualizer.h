#pragma once

#include "types.h"
#include "map3d.h"
#include <unordered_set>
#include <termios.h>
#include <string>
#include <vector>

// ============================================================
// Input event types
// ============================================================
enum class InputType { NONE, KEY, MOUSE_CLICK };

struct InputEvent {
    InputType type = InputType::NONE;
    int key = 0;           // For KEY: ASCII char or special key code
    int mouse_x = 0;       // For MOUSE_CLICK: 1-based column
    int mouse_y = 0;       // For MOUSE_CLICK: 1-based row
    int mouse_button = 0;  // 0=left, 1=middle, 2=right
};

// Special key codes (outside ASCII range)
constexpr int KEY_UP    = 0x100;
constexpr int KEY_DOWN  = 0x101;
constexpr int KEY_RIGHT = 0x102;
constexpr int KEY_LEFT  = 0x103;
constexpr int KEY_HOME  = 0x104;

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
    // ---- Mouse support ----
    void enableMouse();
    void disableMouse();

    // ---- Input parsing ----
    InputEvent readInput();
    InputEvent parseSGRMouse(const unsigned char* data, ssize_t len);
    InputEvent parseArrowKey(unsigned char c2, unsigned char c3);

    // ---- Coordinate conversion ----
    bool screenToMap(int screen_col, int screen_row,
                     int vp_x, int vp_y, int current_layer,
                     const Map3D& map, Pos3D& out) const;

    // ---- Mouse click handler ----
    void handleMouseClick(const InputEvent& event,
                          const Map3D& map,
                          std::vector<Pos3D>& current_path,
                          std::unordered_set<uint64_t>& current_path_set,
                          Pos3D& current_position,
                          float& cumulative_cost,
                          int vp_x, int vp_y, int current_layer,
                          std::string& status_msg,
                          int& status_timer);

    // ---- Frame buffer rendering (flicker reduction) ----
    void renderFrame();

    // ---- Legacy rendering (used by animatePath) ----
    void clearScreen();
    void moveCursor(int row, int col);
    void setColor(int fg, int bg = -1);
    void resetColor();
    void hideCursor();
    void showCursor();

    // Render a 2D slice of the map into the frame buffer
    void renderLayer(const Map3D& map, int layer,
                     const std::unordered_set<uint64_t>& path_set,
                     const std::unordered_set<uint64_t>& goal_set,
                     int vp_cx, int vp_cy,
                     int term_w, int term_h);

    // Statistics display into the frame buffer
    void renderStats(const SearchResult& result, int current_layer);

    // Status message bar
    void renderStatus(const std::string& msg);

    // Non-blocking check for keypress (used only by animatePath for simple keys)
    int getKeyNonBlocking();

    struct termios orig_termios_;
    bool initialized_ = false;
    bool mouse_enabled_ = false;
    int term_rows_ = 30;
    int term_cols_ = 90;

};

// Build a set of path keys for fast lookup
inline std::unordered_set<uint64_t> buildPathSet(const std::vector<Pos3D>& path) {
    std::unordered_set<uint64_t> s;
    s.reserve(path.size());
    for (const auto& p : path) s.insert(p.key());
    return s;
}
