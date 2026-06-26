CXX      := g++

# Resolve Makefile directory so we can build from any working directory
MAKEFILE_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
VPATH      := $(MAKEFILE_DIR)

SDK_PATH  := $(shell xcrun --show-sdk-path 2>/dev/null)
CXXFLAGS := -std=c++23 -Wall -Wextra -O2 -march=native
ifneq ($(SDK_PATH),)
CXXFLAGS += -isysroot $(SDK_PATH) -I$(SDK_PATH)/usr/include/c++/v1
endif
LDFLAGS  :=

TARGET   := astar_demo
SRCS     := main.cpp map3d.cpp astar.cpp visualizer.cpp
BUILD_DIR := $(MAKEFILE_DIR)build

OBJS     := $(addprefix $(BUILD_DIR)/, $(SRCS:.cpp=.o))
DEPS     := $(addprefix $(BUILD_DIR)/, $(SRCS:.cpp=.d))

.PHONY: all clean run demo full test help

all: $(MAKEFILE_DIR)$(TARGET)

$(MAKEFILE_DIR)$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

-include $(DEPS)

clean:
	rm -rf $(BUILD_DIR) $(MAKEFILE_DIR)$(TARGET)

# Run with default settings (1000x1000x10 full map, text mode)
run: $(MAKEFILE_DIR)$(TARGET)
	$(MAKEFILE_DIR)$(TARGET) --text

# Run in demo mode with full visualization
demo: $(MAKEFILE_DIR)$(TARGET)
	$(MAKEFILE_DIR)$(TARGET) --demo

# Run with text mode on full map
full: $(MAKEFILE_DIR)$(TARGET)
	$(MAKEFILE_DIR)$(TARGET) --text -w 1000 -h 1000 -d 10 -g 3 -s 42

# Quick test with very small map
test: $(MAKEFILE_DIR)$(TARGET)
	$(MAKEFILE_DIR)$(TARGET) --text -w 50 -h 50 -d 3 -g 2 -b 0.05 -c 0.03

help:
	@echo "Targets:"
	@echo "  all    - Build the project"
	@echo "  run    - Run with default settings (text mode)"
	@echo "  demo   - Run in demo mode (small map, full visualization)"
	@echo "  full   - Run on 1000x1000x10 full map (text mode)"
	@echo "  test   - Quick test with 50x50x3 map"
	@echo "  clean  - Remove build artifacts"
	@echo ""
	@echo "For interactive visualization, run: $(MAKEFILE_DIR)$(TARGET) --demo"
