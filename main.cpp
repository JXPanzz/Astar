#include "types.h"
#include "map3d.h"
#include "astar.h"
#include "visualizer.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <csignal>

// Global visualizer for signal handler cleanup
static Visualizer* g_viz = nullptr;

void signalHandler(int /*sig*/) {
    if (g_viz) {
        g_viz->cleanup();
        g_viz = nullptr;
    }
    std::cout << "\nInterrupted. Goodbye!\n";
    exit(0);
}

void printUsage(const char* prog) {
    std::cout << "A* 3D Pathfinding Demo\n\n"
              << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  -w  <int>    Map width         (default: 1000)\n"
              << "  -h  <int>    Map height        (default: 1000)\n"
              << "  -d  <int>    Map depth/layers  (default: 10)\n"
              << "  -g  <int>    Number of goals   (default: 3)\n"
              << "  -b  <float>  Blocking density  (default: 0.08)\n"
              << "  -c  <float>  Costly density    (default: 0.05)\n"
              << "  -s  <int>    Random seed       (default: 42)\n"
              << "  -W  <float>  A* weight (>=1)   (default: 2.0)\n"
              << "  -L  <float>  Layer penalty     (default: 5.0)\n"
              << "  -C  <float>  Costly obstacle multiplier (default: 10.0)\n"
              << "  -D  <float>  Danger zone multiplier     (default: 100.0)\n"
              << "  --demo       Demo mode: small map for visualization\n"
              << "  --text       Text-only mode (no terminal graphics)\n"
              << "  --visit-one  Only go to nearest goal (don't visit all)\n"
              << "  --help       Show this help\n\n"
              << "Interactive Controls (in visual mode):\n"
              << "  1-9          Switch to layer 1-9\n"
              << "  Arrow keys   Pan the viewport\n"
              << "  WASD / HJKL  Pan the viewport\n"
              << "  Home         Jump to start position\n"
              << "  0            Cycle through layers\n"
              << "  Space        Skip path animation\n"
              << "  Mouse click  Set new intermediate goal & re-search\n"
              << "  r            Reset path to original search result\n"
              << "  q            Quit\n"
              << std::endl;
}

int main(int argc, char** argv) {
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // ---- Parse command line ----
    MapConfig map_cfg;
    SearchConfig search_cfg;
    bool demo_mode = false;
    bool text_mode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help") { printUsage(argv[0]); return 0; }
        else if (arg == "--demo") { demo_mode = true; }
        else if (arg == "--text") { text_mode = true; }
        else if (arg == "--visit-one") { search_cfg.visit_all_goals = false; }
        else if (arg == "-w" && i + 1 < argc) map_cfg.width = std::atoi(argv[++i]);
        else if (arg == "-h" && i + 1 < argc) map_cfg.height = std::atoi(argv[++i]);
        else if (arg == "-d" && i + 1 < argc) map_cfg.depth = std::atoi(argv[++i]);
        else if (arg == "-g" && i + 1 < argc) map_cfg.num_goals = std::atoi(argv[++i]);
        else if (arg == "-b" && i + 1 < argc) map_cfg.blocking_density = std::atof(argv[++i]);
        else if (arg == "-c" && i + 1 < argc) map_cfg.costly_density = std::atof(argv[++i]);
        else if (arg == "-s" && i + 1 < argc) map_cfg.random_seed = std::atoi(argv[++i]);
        else if (arg == "-W" && i + 1 < argc) search_cfg.weight = std::atof(argv[++i]);
        else if (arg == "-L" && i + 1 < argc) map_cfg.layer_penalty = std::atof(argv[++i]);
        else if (arg == "-C" && i + 1 < argc) map_cfg.costly_obstacle_cost = std::atof(argv[++i]);
        else if (arg == "-D" && i + 1 < argc) map_cfg.danger_zone_cost = std::atof(argv[++i]);
    }

    // Demo mode: smaller map for visualization
    if (demo_mode) {
        map_cfg.width = 80;
        map_cfg.height = 50;
        map_cfg.depth = 5;
        map_cfg.num_goals = 3;
        map_cfg.blocking_density = 0.10f;
        map_cfg.costly_density = 0.06f;
        map_cfg.obstacle_cluster_size = 15;
    }

    // Validate
    if (map_cfg.width < 2 || map_cfg.height < 2 || map_cfg.depth < 1) {
        std::cerr << "Error: Invalid map dimensions.\n";
        return 1;
    }
    if (map_cfg.num_goals < 1) {
        std::cerr << "Error: Need at least 1 goal.\n";
        return 1;
    }
    if (search_cfg.weight < 1.0f) {
        std::cerr << "Error: A* weight must be >= 1.0.\n";
        return 1;
    }

    // ---- Banner ----
    std::cout << "\n╔══════════════════════════════════════════╗\n"
              << "║     A* 3D PATHFINDING DEMO               ║\n"
              << "╚══════════════════════════════════════════╝\n\n";
    std::cout << "Configuration:\n"
              << "  Map size:    " << map_cfg.width << " × " << map_cfg.height
              << " × " << map_cfg.depth
              << "  (" << (map_cfg.width * map_cfg.height * map_cfg.depth / 1000000.0) << "M cells)\n"
              << "  Goals:       " << map_cfg.num_goals << "\n"
              << "  A* weight:   " << search_cfg.weight << "\n"
              << "  Layer cost:  " << map_cfg.layer_penalty << "×\n"
              << "  Costly cost: " << map_cfg.costly_obstacle_cost << "×\n"
              << "  Danger cost: " << map_cfg.danger_zone_cost << "×\n"
              << "  Seed:        " << map_cfg.random_seed << "\n"
              << "  Visit all:   " << (search_cfg.visit_all_goals ? "yes" : "no (nearest only)") << "\n\n";

    // ---- Generate map ----
    Map3D map(map_cfg);
    map.generate();
    std::cout << "\n";

    // ---- Run A* search ----
    std::cout << "Searching...\n";

    AStar astar(search_cfg);

    // Create visualizer if not text mode
    Visualizer viz;
    g_viz = &viz;

    ProgressCallback progress_cb = nullptr;
    if (!text_mode) {
        // Pass progress updates to visualizer
        progress_cb = [&viz](const SearchProgress& p) {
            viz.showProgress(p);
        };
    }

    SearchResult result = astar.search(map, progress_cb);

    std::cout << "\n\n";
    std::cout << "═══════════════════════════════════════════\n";
    std::cout << "Search Results:\n";
    std::cout << "  Success:       " << (result.success ? "YES" : "PARTIAL/FAILED") << "\n";
    std::cout << "  Nodes explored: " << result.nodes_explored << "\n";
    std::cout << "  Nodes visited:  " << result.nodes_visited << "\n";
    std::cout << "  Path length:    " << result.path.size() << " cells\n";
    std::cout << "  Total cost:    " << std::fixed << std::setprecision(1) << result.total_cost << "\n";
    std::cout << "  Time:          " << std::fixed << std::setprecision(2) << result.time_ms << " ms\n";
    std::cout << "  Segments:      " << result.segments.size() << "\n";

    for (size_t i = 0; i < result.segments.size(); ++i) {
        std::cout << "    Segment " << i << ": " << result.segments[i].size() << " cells";
        if (!result.segments[i].empty()) {
            const auto& first = result.segments[i].front();
            const auto& last = result.segments[i].back();
            std::cout << "  [(" << first.x << "," << first.y << "," << first.z
                      << ") → (" << last.x << "," << last.y << "," << last.z << ")]";
        }
        std::cout << "\n";
    }
    std::cout << "═══════════════════════════════════════════\n\n";

    // ---- Visualization ----
    if (!text_mode && result.success && !result.path.empty()) {
        std::cout << "Press ENTER to view animated path, or 'q' to quit...\n";
        char c = std::cin.get();
        if (c != 'q' && c != 'Q') {
            viz.init();
            viz.animatePath(map, result);

            std::cout << "\nPress ENTER to explore, or 'q' to quit...\n";
            c = std::cin.get();
            if (c != 'q' && c != 'Q') {
                viz.explore(map, result);
            }
            viz.cleanup();
        }
    } else if (!text_mode) {
        std::cout << "No valid path to visualize.\n";
    }

    g_viz = nullptr;
    std::cout << "\nDone!\n";
    return 0;
}
