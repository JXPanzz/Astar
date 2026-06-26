#include "visualizer.h"
#include "astar.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

Visualizer::Visualizer() {}

Visualizer::~Visualizer() {
    if (initialized_) cleanup();
}

void Visualizer::init() {
    // Save original terminal settings
    tcgetattr(STDIN_FILENO, &orig_termios_);
    struct termios raw = orig_termios_;
    raw.c_lflag &= ~(ECHO | ICANON); // No echo, no canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    initialized_ = true;
    hideCursor();
    enableMouse();

    term_rows_ = 30;
    term_cols_ = 90;
}

void Visualizer::cleanup() {
    if (initialized_) {
        disableMouse();
        resetColor();
        showCursor();
        clearScreen();
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        // Restore blocking mode
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
        initialized_ = false;
    }
}

// ============================================================
// Mouse
// ============================================================

void Visualizer::enableMouse() {
    if (mouse_enabled_) return;
    std::cout << "\033[?1000h" << std::flush;  // SGR mouse tracking (press only)
    mouse_enabled_ = true;
}

void Visualizer::disableMouse() {
    if (!mouse_enabled_) return;
    std::cout << "\033[?1000l" << std::flush;
    mouse_enabled_ = false;
}

// ============================================================
// Input parsing
// ============================================================

InputEvent Visualizer::readInput() {
    unsigned char buf[32];
    ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    if (n <= 0) return {InputType::NONE};

    // Single-byte: plain ASCII key
    if (buf[0] != 27) {
        return {InputType::KEY, static_cast<int>(buf[0])};
    }

    // Bare ESC (no more bytes available)
    if (n == 1) return {InputType::KEY, 27};

    // ESC [ <  —  SGR mouse event
    if (n >= 3 && buf[1] == '[' && buf[2] == '<') {
        return parseSGRMouse(buf + 3, n - 3);
    }

    // ESC [ …  —  arrow keys, Home, etc.
    if (n >= 3 && buf[1] == '[') {
        return parseArrowKey(buf[2], (n > 3 ? buf[3] : 0));
    }

    // ESC O …  —  older terminal arrow/Home style
    if (n >= 3 && buf[1] == 'O') {
        return parseArrowKey(buf[2], (n > 3 ? buf[3] : 0));
    }

    return {InputType::NONE};
}

InputEvent Visualizer::parseSGRMouse(const unsigned char* data, ssize_t len) {
    // Expected format: btn;x;yM (press) or btn;x;ym (release)
    // Find the terminator
    ssize_t end = 0;
    while (end < len && data[end] != 'M' && data[end] != 'm') ++end;
    if (end >= len) return {InputType::NONE};  // incomplete

    bool is_release = (data[end] == 'm');
    if (is_release) return {InputType::NONE};  // ignore releases

    // Parse btn;x;y
    int btn = 0, x = 0, y = 0;
    char payload[32];
    ssize_t plen = std::min(end, static_cast<ssize_t>(sizeof(payload) - 1));
    memcpy(payload, data, plen);
    payload[plen] = '\0';

    if (sscanf(payload, "%d;%d;%d", &btn, &x, &y) < 3) return {InputType::NONE};

    // Only handle left-button press (btn 0)
    if (btn != 0) return {InputType::NONE};

    InputEvent ev;
    ev.type = InputType::MOUSE_CLICK;
    ev.mouse_x = x;
    ev.mouse_y = y;
    ev.mouse_button = btn;
    return ev;
}

InputEvent Visualizer::parseArrowKey(unsigned char c2, unsigned char c3) {
    InputEvent ev;
    ev.type = InputType::KEY;

    switch (c2) {
        case 'A': ev.key = KEY_UP;    break;
        case 'B': ev.key = KEY_DOWN;  break;
        case 'C': ev.key = KEY_RIGHT; break;
        case 'D': ev.key = KEY_LEFT;  break;
        case 'H': ev.key = KEY_HOME;  break;
        case '1': // [1~  or  [1;…  —  Home on some terminals
            if (c3 == '~') ev.key = KEY_HOME;
            break;
        default:  ev.key = 0; break;
    }
    return ev;
}

// ============================================================
// Coordinate conversion
// ============================================================

bool Visualizer::screenToMap(int screen_col, int screen_row,
                              int vp_x, int vp_y, int current_layer,
                              const Map3D& map, Pos3D& out) const {
    // Map rendering in explore() starts at screen row 4, column 2
    // drawCell is called with (dy + 4, dx + 2) where dy∈[0,19], dx∈[0,69]
    int map_dx = screen_col - 2;
    int map_dy = screen_row - 4;

    // Viewport bounds (matching explore() render loop: dy<20, dx<70)
    if (map_dx < 0 || map_dx >= 70) return false;
    if (map_dy < 0 || map_dy >= 20) return false;

    int mx = vp_x + map_dx;
    int my = vp_y + map_dy;

    if (mx < 0 || mx >= map.width())  return false;
    if (my < 0 || my >= map.height()) return false;

    out = Pos3D(mx, my, current_layer);
    return true;
}

// ============================================================
// Mouse click handler
// ============================================================

void Visualizer::handleMouseClick(const InputEvent& event,
                                   const Map3D& map,
                                   std::vector<Pos3D>& current_path,
                                   std::unordered_set<uint64_t>& current_path_set,
                                   Pos3D& current_position,
                                   float& cumulative_cost,
                                   int vp_x, int vp_y, int current_layer,
                                   std::string& status_msg) {
    Pos3D clicked;
    if (!screenToMap(event.mouse_x, event.mouse_y,
                     vp_x, vp_y, current_layer, map, clicked)) {
        status_msg = "  Click outside map area";
        return;
    }

    // Validate: must be on the current layer and traversable
    if (!map.isTraversable(clicked)) {
        std::ostringstream oss;
        oss << "  Cannot traverse (" << clicked.x << "," << clicked.y
            << "," << clicked.z << ") — obstacle or danger zone";
        status_msg = oss.str();
        return;
    }

    // Don't search to current position
    if (clicked == current_position) {
        status_msg = "  Already at this position";
        return;
    }

    // Run A* search for the new segment
    status_msg = "  Searching to (" + std::to_string(clicked.x) + ","
               + std::to_string(clicked.y) + "," + std::to_string(clicked.z) + ")...";

    // Flush the status message now so the user sees it during search
    renderFrame();
    std::cout << std::flush;

    SearchConfig sc;
    sc.weight = 2.0f;
    sc.visit_all_goals = false;
    AStar astar(sc);

    std::vector<Pos3D> segment;
    if (!astar.searchPath(map, current_position, clicked, segment)) {
        std::ostringstream oss;
        oss << "  No path to (" << clicked.x << "," << clicked.y
            << "," << clicked.z << ")";
        status_msg = oss.str();
        return;
    }

    // Append segment to current path, deduplicating the junction point
    if (!current_path.empty() && !segment.empty() &&
        current_path.back() == segment.front()) {
        current_path.pop_back();
    }

    current_path.insert(current_path.end(), segment.begin(), segment.end());

    // Rebuild path set
    current_path_set.clear();
    for (const auto& p : current_path) current_path_set.insert(p.key());

    // Accumulate cost
    for (size_t i = 1; i < segment.size(); ++i) {
        cumulative_cost += map.getMoveCost(segment[i - 1], segment[i]);
    }

    current_position = clicked;

    std::ostringstream oss;
    oss << "  Path extended! +" << segment.size() << " cells to ("
        << clicked.x << "," << clicked.y << "," << clicked.z
        << ")  |  Total: " << current_path.size() << " cells";
    status_msg = oss.str();
}

// ============================================================
// Frame buffer rendering
// ============================================================

// REVISED APPROACH: Direct ostringstream construction.
// We build the entire frame in a single ostringstream with embedded escapes,
// then write it in one shot. This eliminates flicker by avoiding clearScreen()
// between frames (we always overwrite every cell).

static std::ostringstream g_frame;  // global frame builder

static void frameBegin() {
    g_frame.str("");
    g_frame.clear();
    // Move to home — no clear needed since we overwrite every cell position
    g_frame << "\033[H";
}

static void frameWrite(int row, int col, const std::string& s) {
    // row, col are 1-based for ANSI cursor positioning
    g_frame << "\033[" << row << ";" << col << "H" << s;
}

// ---- Color map for ANSI 256-color mode ----
static int cellColor(CellType t) {
    switch (t) {
        case CellType::FREE:             return 255; // light gray
        case CellType::BLOCKING_OBSTACLE: return 196; // red
        case CellType::COSTLY_OBSTACLE:   return 220; // gold/yellow
        case CellType::DANGER_ZONE:       return 201; // magenta
        default:                          return 255;
    }
}

static const char* cellSymbol(CellType t) {
    switch (t) {
        case CellType::FREE:             return "·";
        case CellType::BLOCKING_OBSTACLE: return "█";
        case CellType::COSTLY_OBSTACLE:   return "▓";
        case CellType::DANGER_ZONE:       return "▒";
        default:                          return " ";
    }
}

static void frameDrawCell(int row, int col, CellType type,
                           bool is_path, bool is_start, bool is_goal,
                           bool highlight) {
    if (is_start) {
        g_frame << "\033[" << row << ";" << col << "H"
                << "\033[38;5;46m" << "S" << "\033[0m";
    } else if (is_goal) {
        g_frame << "\033[" << row << ";" << col << "H"
                << "\033[38;5;33m" << "G" << "\033[0m";
    } else if (is_path) {
        g_frame << "\033[" << row << ";" << col << "H"
                << "\033[38;5;226;48;5;22m" << "*" << "\033[0m";
    } else if (highlight) {
        g_frame << "\033[" << row << ";" << col << "H"
                << "\033[48;5;51m" << " " << "\033[0m";
    } else {
        int c = cellColor(type);
        g_frame << "\033[" << row << ";" << col << "H"
                << "\033[38;5;" << c << "m" << cellSymbol(type) << "\033[0m";
    }
}

void Visualizer::renderFrame() {
    std::cout << g_frame.str() << std::flush;
}

// ============================================================
// Legacy terminal helpers (used by animatePath)
// ============================================================

void Visualizer::clearScreen() {
    std::cout << "\033[2J\033[H" << std::flush;
}

void Visualizer::moveCursor(int row, int col) {
    std::cout << "\033[" << row << ";" << col << "H" << std::flush;
}

void Visualizer::setColor(int fg, int bg) {
    if (fg >= 0) std::cout << "\033[38;5;" << fg << "m";
    if (bg >= 0) std::cout << "\033[48;5;" << bg << "m";
}

void Visualizer::resetColor() {
    std::cout << "\033[0m";
}

void Visualizer::hideCursor() {
    std::cout << "\033[?25l" << std::flush;
}

void Visualizer::showCursor() {
    std::cout << "\033[?25h" << std::flush;
}

int Visualizer::getKeyNonBlocking() {
    char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n > 0) return static_cast<int>(c);
    return -1;
}

// ============================================================
// Layer rendering (into ostringstream frame buffer)
// ============================================================

void Visualizer::renderLayer(const Map3D& map, int layer,
                              const std::unordered_set<uint64_t>& path_set,
                              const std::unordered_set<uint64_t>& goal_set,
                              int vp_cx, int vp_cy,
                              int /*term_w*/, int /*term_h*/) {
    int map_w = map.width();
    int map_h = map.height();
    int half_w = 35;
    int half_h = 10;

    int vp_x = std::max(0, vp_cx - half_w);
    int vp_y = std::max(0, vp_cy - half_h);

    const Pos3D& start = map.getStart();

    for (int dy = 0; dy < 20; ++dy) {
        for (int dx = 0; dx < 70; ++dx) {
            int mx = vp_x + dx;
            int my = vp_y + dy;
            if (mx >= map_w || my >= map_h) continue;

            Pos3D p(mx, my, layer);
            CellType ct = map.getCell(p);
            bool on_path = path_set.count(p.key()) > 0;
            bool is_start = (p == start);
            bool is_goal = goal_set.count(p.key()) > 0;

            int sr = dy + 4;  // screen row (1-based)
            int sc = dx + 2;  // screen col (1-based)
            frameDrawCell(sr, sc, ct, on_path, is_start, is_goal, false);
        }
    }
}

// ============================================================
// Statistics panel (into ostringstream frame buffer)
// ============================================================

void Visualizer::renderStats(const SearchResult& result, int current_layer) {
    int r = 3, c = 2;

    frameWrite(r++, c, "╔══════════════════════════════════════════╗");
    frameWrite(r++, c, "║  A* 3D Pathfinding — Explore Mode        ║");
    frameWrite(r++, c, "╠══════════════════════════════════════════╣");

    std::ostringstream oss;
    oss << "║  Explored: " << std::setw(8) << result.nodes_explored
        << "  Visited: " << std::setw(8) << result.nodes_visited << "  ║";
    frameWrite(r++, c, oss.str());

    oss.str(""); oss << "║  Path:     " << std::setw(8) << result.path.size()
                     << " cells  Cost: " << std::setw(9) << std::fixed
                     << std::setprecision(1) << result.total_cost << "  ║";
    frameWrite(r++, c, oss.str());

    oss.str(""); oss << "║  Time:     " << std::setw(8) << std::fixed
                     << std::setprecision(2) << result.time_ms << " ms"
                     << "  Segments: " << std::setw(4) << result.segments.size() << "  ║";
    frameWrite(r++, c, oss.str());

    frameWrite(r++, c, "╠══════════════════════════════════════════╣");
    oss.str(""); oss << "║  Layer: " << std::setw(3) << (current_layer + 1) << "/"
                     << std::setw(3) << 10
                     << "                            ║";
    frameWrite(r++, c, oss.str());
    frameWrite(r++, c, "╚══════════════════════════════════════════╝");
}

// ============================================================
// Status message
// ============================================================

void Visualizer::renderStatus(const std::string& msg) {
    // Status bar at row 27
    // Clear the line first by writing spaces
    std::string clear(term_cols_ - 1, ' ');
    frameWrite(27, 1, clear);
    if (!msg.empty()) {
        frameWrite(27, 2, msg);
    }
}

// ============================================================
// Progress callback
// ============================================================

void Visualizer::showProgress(const SearchProgress& progress) {
    std::ostringstream oss;
    oss << "\r  Segment " << (progress.segment_index + 1) << "/"
        << progress.total_segments
        << " | Explored: " << progress.nodes_explored
        << " | Open: " << progress.open_set_size
        << " | f=" << std::fixed << std::setprecision(1) << progress.current_best_f
        << " | g=" << progress.current_best_g
        << " | at (" << progress.current_node.x << ","
        << progress.current_node.y << ","
        << progress.current_node.z << ")   " << std::flush;
    std::cerr << oss.str();
}

// ============================================================
// Path animation (legacy rendering — terminal writes directly)
// ============================================================

void Visualizer::animatePath(const Map3D& map, const SearchResult& result) {
    if (!initialized_) init();

    clearScreen();

    if (result.path.empty()) {
        std::cout << "No path found!\n";
        return;
    }

    auto path_set = buildPathSet(result.path);
    const Pos3D& start = map.getStart();
    const auto& goals = map.getGoals();

    // Build goal key set
    std::unordered_set<uint64_t> goal_keys;
    for (const auto& g : goals) goal_keys.insert(g.key());

    // Show the path being drawn step by step
    std::unordered_set<uint64_t> drawn_path;
    int current_layer = result.path[0].z;

    // Display header
    std::cout << "\033[1;1H\033[37m╔══════════════════════════════════════════════════════╗";
    std::cout << "\033[2;1H║  A* 3D PATHFINDING — Animated Path                   ║";
    std::cout << "\033[3;1H╚══════════════════════════════════════════════════════╝\033[0m";

    // Animate drawing the path
    for (size_t i = 0; i < result.path.size(); ++i) {
        const auto& p = result.path[i];
        drawn_path.insert(p.key());

        // Switch layer display if needed
        if (p.z != current_layer) {
            current_layer = p.z;
            clearScreen();
        }

        // Only show a window around the current path position
        int half_w = 30, half_h = 12;
        int vp_x = std::max(0, p.x - half_w);
        int vp_y = std::max(0, p.y - half_h);

        // Redraw visible portion of the layer
        for (int dy = 0; dy < 25; ++dy) {
            for (int dx = 0; dx < 60; ++dx) {
                int mx = vp_x + dx;
                int my = vp_y + dy;
                if (mx >= map.width() || my >= map.height()) continue;

                Pos3D mp(mx, my, current_layer);
                CellType ct = map.getCell(mp);
                bool is_path = drawn_path.count(mp.key()) > 0;
                bool is_start = (mp == start);
                bool is_goal = goal_keys.count(mp.key()) > 0;
                bool highlight = (mp == p); // current drawing position

                frameDrawCell(dy + 4, dx + 1, ct, is_path, is_start, is_goal, highlight);
            }
        }

        // Status line
        moveCursor(29, 1);
        setColor(252);
        std::cout << "Layer: " << (current_layer + 1) << "/" << map.depth()
                  << " | Step: " << (i + 1) << "/" << result.path.size()
                  << " | Pos: (" << p.x << ", " << p.y << ", " << p.z << ")";
        resetColor();

        // Render the frame
        std::cout << g_frame.str() << std::flush;
        g_frame.str("");
        g_frame.clear();

        // Delay for animation
        usleep(20000); // 20ms per step

        // Check for skip key
        int key = getKeyNonBlocking();
        if (key == 'q' || key == 'Q') break;
        if (key == ' ') {
            // Skip animation, draw rest instantly
            for (size_t j = i + 1; j < result.path.size(); ++j) {
                drawn_path.insert(result.path[j].key());
            }
            break;
        }
    }

    // Final display: show complete path
    clearScreen();
    current_layer = result.path[0].z;
    int half_w = 35, half_h = 12;
    int cx = result.path[result.path.size() / 2].x;
    int cy = result.path[result.path.size() / 2].y;

    for (int dy = 0; dy < 25; ++dy) {
        for (int dx = 0; dx < 65; ++dx) {
            int mx = cx - half_w + dx;
            int my = cy - half_h + dy;
            if (mx < 0 || mx >= map.width() || my < 0 || my >= map.height()) continue;

            Pos3D mp(mx, my, current_layer);
            CellType ct = map.getCell(mp);
            bool is_path = path_set.count(mp.key()) > 0;
            bool is_start = (mp == start);
            bool is_goal = goal_keys.count(mp.key()) > 0;

            frameDrawCell(dy + 5, dx + 2, ct, is_path, is_start, is_goal, false);
        }
    }

    // Draw stats into the frame
    int r = 3, c = 2;
    frameWrite(r++, c, "╔══════════════════════════════════════════╗");
    frameWrite(r++, c, "║  A* 3D Pathfinding Demo                  ║");
    frameWrite(r++, c, "╠══════════════════════════════════════════╣");
    std::ostringstream oss;
    oss << "║  Nodes Explored: " << std::setw(10) << result.nodes_explored << "              ║";
    frameWrite(r++, c, oss.str());
    oss.str(""); oss << "║  Path Length:    " << std::setw(10) << result.path.size() << " cells          ║";
    frameWrite(r++, c, oss.str());
    oss.str(""); oss << "║  Total Cost:     " << std::setw(10) << std::fixed << std::setprecision(1)
              << result.total_cost << "              ║";
    frameWrite(r++, c, oss.str());
    oss.str(""); oss << "║  Time:           " << std::setw(10) << std::fixed << std::setprecision(2)
              << result.time_ms << " ms          ║";
    frameWrite(r++, c, oss.str());
    oss.str(""); oss << "║  Segments:       " << std::setw(10) << result.segments.size() << "              ║";
    frameWrite(r++, c, oss.str());
    frameWrite(r++, c, "╚══════════════════════════════════════════╝");

    frameWrite(32, 2, "Press ENTER to explore interactively, or 'q' to quit...");

    std::cout << g_frame.str() << std::flush;
}

// ============================================================
// Explore mode (buffered rendering + mouse support)
// ============================================================

void Visualizer::explore(const Map3D& map, const SearchResult& result) {
    if (!initialized_) init();

    // ---- Mutable path state ----
    std::vector<Pos3D> current_path = result.path;
    std::unordered_set<uint64_t> current_path_set = buildPathSet(current_path);
    Pos3D current_position = result.path.empty()
        ? map.getStart()
        : result.path.back();
    float cumulative_cost = result.total_cost;

    // ---- Goal set ----
    std::unordered_set<uint64_t> goal_keys;
    for (const auto& g : map.getGoals()) goal_keys.insert(g.key());

    // ---- Original path backup (for 'r' reset) ----
    const std::vector<Pos3D> original_path = result.path;
    const std::unordered_set<uint64_t> original_path_set = buildPathSet(original_path);
    const float original_cost = result.total_cost;
    const Pos3D original_position = result.path.empty()
        ? map.getStart()
        : result.path.back();

    // ---- Viewport state ----
    int current_layer = result.path.empty() ? 0 : result.path[0].z;
    int vp_cx = result.path.empty() ? map.width() / 2 : result.path[0].x;
    int vp_cy = result.path.empty() ? map.height() / 2 : result.path[0].y;

    std::string status_msg;
    bool running = true;

    while (running) {
        // Build frame
        frameBegin();

        // Header
        frameWrite(1, 2, "\033[1m╔══════════════════════════════════════════════════════════════╗\033[0m");
        frameWrite(2, 2, "\033[1m║  A* 3D PATHFINDING — Explore Mode                           ║\033[0m");
        frameWrite(3, 2, "\033[1m╚══════════════════════════════════════════════════════════════╝\033[0m");

        // Render map layer
        renderLayer(map, current_layer, current_path_set, goal_keys,
                    vp_cx, vp_cy, term_cols_, term_rows_);

        // Status bar
        std::ostringstream bar;
        bar << "Layer:" << std::setw(3) << (current_layer + 1) << "/" << map.depth()
            << " | Center:(" << vp_cx << "," << vp_cy << ")"
            << " | Path:" << current_path.size() << " cells"
            << " | Cost:" << std::fixed << std::setprecision(1) << cumulative_cost;
        frameWrite(25, 2, bar.str());

        // Help bar
        frameWrite(26, 2, "[1-9]Layer [Arrows/WASD/HJKL]Pan [Home]Start [0]Cycle [Click]SetGoal [r]Reset [q]Quit");

        // Status message
        renderStatus(status_msg);
        // Clear status after showing it (it will reappear if handleMouseClick sets it again)
        if (!status_msg.empty()) {
            // Keep it for one more frame, then clear
        }

        // Render everything
        renderFrame();

        // Clear status for next frame (unless re-set by a new click)
        status_msg.clear();

        // Poll for input
        usleep(30000); // 30ms
        InputEvent event = readInput();

        if (event.type == InputType::KEY) {
            switch (event.key) {
                case 'q': case 'Q':
                    running = false;
                    break;
                case '1': case '2': case '3': case '4': case '5':
                case '6': case '7': case '8': case '9':
                    if (event.key - '1' < map.depth())
                        current_layer = event.key - '1';
                    break;
                case '0':
                    current_layer = (current_layer + 1) % map.depth();
                    break;
                case 'w': case 'k': case KEY_UP:
                    vp_cy = std::max(10, vp_cy - 3);
                    break;
                case 's': case 'j': case KEY_DOWN:
                    vp_cy = std::min(map.height() - 11, vp_cy + 3);
                    break;
                case 'a': case 'h': case KEY_LEFT:
                    vp_cx = std::max(35, vp_cx - 5);
                    break;
                case 'd': case 'l': case KEY_RIGHT:
                    vp_cx = std::min(map.width() - 36, vp_cx + 5);
                    break;
                case KEY_HOME:
                    vp_cx = map.getStart().x;
                    vp_cy = map.getStart().y;
                    current_layer = map.getStart().z;
                    break;
                case 'r': case 'R':
                    // Reset path to original search result
                    current_path = original_path;
                    current_path_set = original_path_set;
                    current_position = original_position;
                    cumulative_cost = original_cost;
                    status_msg = "  Path reset to original search result";
                    break;
                default:
                    break;
            }
        } else if (event.type == InputType::MOUSE_CLICK) {
            int vp_x = std::max(0, vp_cx - 35);
            int vp_y = std::max(0, vp_cy - 10);
            handleMouseClick(event, map,
                             current_path, current_path_set,
                             current_position, cumulative_cost,
                             vp_x, vp_y,
                             current_layer, status_msg);
        }
    }
}
