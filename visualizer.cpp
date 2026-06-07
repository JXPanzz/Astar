#include "visualizer.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>
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
    clearScreen();

    // Detect terminal size (or use defaults)
    // We'll use standard 80x24 as fallback
    term_rows_ = 30;
    term_cols_ = 90;
}

void Visualizer::cleanup() {
    if (initialized_) {
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

void Visualizer::clearScreen() {
    std::cout << "\033[2J\033[H" << std::flush;
}

void Visualizer::moveCursor(int row, int col) {
    std::cout << "\033[" << row << ";" << col << "H" << std::flush;
}

void Visualizer::setColor(int fg, int bg) {
    // ANSI 256-color mode foreground
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

// ---- Color map for ANSI 256-color mode ----
static int colorForCell(CellType t) {
    switch (t) {
        case CellType::FREE:            return 255; // light gray
        case CellType::BLOCKING_OBSTACLE: return 196; // red
        case CellType::COSTLY_OBSTACLE: return 220;  // gold/yellow
        case CellType::DANGER_ZONE:     return 201;  // magenta
        default:                        return 255;
    }
}

static const char* symbolForCell(CellType t) {
    switch (t) {
        case CellType::FREE:             return "·";
        case CellType::BLOCKING_OBSTACLE: return "█";
        case CellType::COSTLY_OBSTACLE:  return "▓";
        case CellType::DANGER_ZONE:      return "▒";
        default:                         return " ";
    }
}

void Visualizer::drawCell(int sr, int sc, CellType type,
                           bool is_path, bool is_start, bool is_goal) {
    moveCursor(sr, sc);

    if (is_start) {
        setColor(46);  // bright green
        std::cout << "S";
    } else if (is_goal) {
        setColor(33);  // bright blue
        std::cout << "G";
    } else if (is_path) {
        setColor(226, 22); // bright yellow on dark green
        std::cout << "*";
    } else {
        setColor(colorForCell(type));
        std::cout << symbolForCell(type);
    }
    resetColor();
}

void Visualizer::showProgress(const SearchProgress& progress) {
    // Simple text-based progress output
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

void Visualizer::renderLayer(const Map3D& map, int layer,
                              const std::vector<Pos3D>& /*path*/,
                              const std::unordered_set<uint64_t>& path_set,
                              int vp_cx, int vp_cy,
                              int term_w, int term_h) {
    int map_w = map.width();
    int map_h = map.height();

    // Calculate viewport: center on start/path center, bounded by map
    int half_w = (term_w - 4) / 2;
    int half_h = (term_h - 6) / 2; // Leave room for header and stats

    int vp_x = std::clamp(vp_cx - half_w, 0, map_w - (term_w - 4));
    int vp_y = std::clamp(vp_cy - half_h, 0, map_h - (term_h - 6));
    int vp_w = std::min(term_w - 4, map_w);
    int vp_h = std::min(term_h - 6, map_h);

    // Ensure vp_x, vp_y are valid
    vp_x = std::max(0, std::min(vp_x, map_w - vp_w));
    vp_y = std::max(0, std::min(vp_y, map_h - vp_h));

    const Pos3D& start = map.getStart();
    const auto& goals = map.getGoals();

    // Build goal key set
    std::unordered_set<uint64_t> goal_keys;
    for (const auto& g : goals) goal_keys.insert(g.key());

    for (int dy = 0; dy < vp_h && dy < term_h - 6; ++dy) {
        for (int dx = 0; dx < vp_w && dx < term_w - 4; ++dx) {
            int mx = vp_x + dx;
            int my = vp_y + dy;

            Pos3D p(mx, my, layer);
            CellType ct = map.getCell(p);
            bool on_path = path_set.count(p.key()) > 0;
            bool is_start = (p == start);
            bool is_goal = goal_keys.count(p.key()) > 0;

            drawCell(dy + 3, dx + 2, ct, on_path, is_start, is_goal);
        }
    }
}

void Visualizer::renderStats(const SearchResult& result, int current_layer) {
    int row = 3;
    int col = 2;

    moveCursor(row++, col);
    setColor(255);
    std::cout << "╔══════════════════════════════════════════╗";
    moveCursor(row++, col);
    std::cout << "║  A* 3D Pathfinding Demo                  ║";
    moveCursor(row++, col);
    std::cout << "╠══════════════════════════════════════════╣";

    std::ostringstream oss;
    oss << "║  Nodes Explored: " << std::setw(10) << result.nodes_explored << "              ║";
    moveCursor(row++, col); std::cout << oss.str();

    oss.str(""); oss << "║  Path Length:    " << std::setw(10) << result.path.size() << " cells          ║";
    moveCursor(row++, col); std::cout << oss.str();

    oss.str(""); oss << "║  Total Cost:     " << std::setw(10) << std::fixed << std::setprecision(1)
              << result.total_cost << "              ║";
    moveCursor(row++, col); std::cout << oss.str();

    oss.str(""); oss << "║  Time:           " << std::setw(10) << std::fixed << std::setprecision(2)
              << result.time_ms << " ms          ║";
    moveCursor(row++, col); std::cout << oss.str();

    oss.str(""); oss << "║  Segments:       " << std::setw(10) << result.segments.size() << "              ║";
    moveCursor(row++, col); std::cout << oss.str();

    moveCursor(row++, col);
    std::cout << "╠══════════════════════════════════════════╣";

    oss.str(""); oss << "║  Layer: " << std::setw(3) << (current_layer + 1) << "/"
              << std::setw(3) << 10 << "                          ║";
    moveCursor(row++, col); std::cout << oss.str();

    moveCursor(row++, col);
    std::cout << "╚══════════════════════════════════════════╝";
    resetColor();
}

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
            clearScreen(); // Clear and redraw for new layer
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
                bool is_goal = false;
                for (const auto& g : goals) if (mp == g) { is_goal = true; break; }

                drawCell(dy + 4, dx + 1, ct, is_path, is_start, is_goal);
            }
        }

        // Status line
        moveCursor(29, 1);
        setColor(252);
        std::cout << "Layer: " << (current_layer + 1) << "/" << map.depth()
                  << " | Step: " << (i + 1) << "/" << result.path.size()
                  << " | Pos: (" << p.x << ", " << p.y << ", " << p.z << ")"
                  << " | Cost so far: " << std::fixed << std::setprecision(1)
                  << (i > 0 ? result.total_cost * i / result.path.size() : 0.0f);
        resetColor();

        // Delay for animation
        usleep(30000); // 30ms per step

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
            bool is_goal = false;
            for (const auto& g : goals) if (mp == g) { is_goal = true; break; }

            drawCell(dy + 5, dx + 2, ct, is_path, is_start, is_goal);
        }
    }

    renderStats(result, current_layer);

    moveCursor(33, 2);
    setColor(252);
    std::cout << "Press ENTER to explore interactively, or 'q' to quit...";
    resetColor();
}

void Visualizer::explore(const Map3D& map, const SearchResult& result) {
    if (!initialized_) init();

    auto path_set = buildPathSet(result.path);
    const Pos3D& start = map.getStart();
    const auto& goals = map.getGoals();
    std::unordered_set<uint64_t> goal_keys;
    for (const auto& g : goals) goal_keys.insert(g.key());

    int current_layer = result.path.empty() ? 0 : result.path[0].z;
    int vp_cx = result.path.empty() ? map.width() / 2 : result.path[0].x;
    int vp_cy = result.path.empty() ? map.height() / 2 : result.path[0].y;

    int half_w = 35, half_h = 10;
    bool running = true;

    while (running) {
        clearScreen();

        // Header
        moveCursor(1, 2);
        setColor(255);
        std::cout << "╔══════════════════════════════════════════════════════════════╗";
        moveCursor(2, 2);
        std::cout << "║  A* 3D PATHFINDING — Explore Mode                           ║";
        moveCursor(3, 2);
        std::cout << "╚══════════════════════════════════════════════════════════════╝";
        resetColor();

        // Render map
        int vp_x = std::max(0, vp_cx - half_w);
        int vp_y = std::max(0, vp_cy - half_h);

        for (int dy = 0; dy < 20; ++dy) {
            for (int dx = 0; dx < 70; ++dx) {
                int mx = vp_x + dx;
                int my = vp_y + dy;
                if (mx >= map.width() || my >= map.height()) continue;

                Pos3D mp(mx, my, current_layer);
                CellType ct = map.getCell(mp);
                bool is_path = path_set.count(mp.key()) > 0;
                bool is_start = (mp == start);
                bool is_goal = goal_keys.count(mp.key()) > 0;

                drawCell(dy + 4, dx + 2, ct, is_path, is_start, is_goal);
            }
        }

        // Stats and info bar
        moveCursor(25, 2);
        setColor(252);
        std::cout << "Layer:" << std::setw(3) << (current_layer + 1) << "/" << map.depth()
                  << " | Center:(" << vp_cx << "," << vp_cy << ")"
                  << " | Path:" << result.path.size() << " cells"
                  << " | Cost:" << std::fixed << std::setprecision(1) << result.total_cost
                  << " | Time:" << result.time_ms << "ms";
        resetColor();

        moveCursor(26, 2);
        std::cout << "[1-9]Switch Layer  [Arrows]Pan  [WASD]Pan  [Home]Go to Start  [0]Overview  [q]Quit";

        // Wait for input
        usleep(50000);
        int key = getKeyNonBlocking();

        switch (key) {
            case 'q': case 'Q': running = false; break;
            case '1': case '2': case '3': case '4': case '5':
            case '6': case '7': case '8': case '9':
                if (key - '1' < map.depth()) current_layer = key - '1';
                break;
            case '0': { // Overview: show layers 0-9 as thumbnails
                // Simplified: just cycle through all layers
                current_layer = (current_layer + 1) % map.depth();
                break;
            }
            case 'h': vp_cx = std::max(half_w, vp_cx - 5); break;
            case 'l': vp_cx = std::min(map.width() - half_w - 1, vp_cx + 5); break;
            case 'k': vp_cy = std::max(half_h, vp_cy - 3); break;
            case 'j': vp_cy = std::min(map.height() - half_h - 1, vp_cy + 3); break;
            default: break;
        }

        // Arrow keys come as ESC sequences
        if (key == 27) {
            int k2 = getKeyNonBlocking();
            if (k2 == '[' || k2 == 'O') {
                int k3 = getKeyNonBlocking();
                switch (k3) {
                    case 'A': vp_cy = std::max(half_h, vp_cy - 3); break;      // Up
                    case 'B': vp_cy = std::min(map.height() - half_h - 1, vp_cy + 3); break; // Down
                    case 'C': vp_cx = std::min(map.width() - half_w - 1, vp_cx + 5); break;  // Right
                    case 'D': vp_cx = std::max(half_w, vp_cx - 5); break;                    // Left
                    case 'H': // Home
                        vp_cx = start.x;
                        vp_cy = start.y;
                        current_layer = start.z;
                        break;
                }
            }
        }
    }
}

int Visualizer::getKeyNonBlocking() {
    char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n > 0) return static_cast<int>(c);
    return -1;
}
