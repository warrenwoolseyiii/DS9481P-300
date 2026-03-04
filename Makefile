BUILD_DIR := build
EXAMPLES_BUILD_DIR := examples/c/build

CMAKE_FLAGS ?=

.PHONY: all configure build examples clean

all: build

configure:
	@mkdir -p $(BUILD_DIR)
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)

build: configure
	cmake --build $(BUILD_DIR)

examples: build
	@mkdir -p $(EXAMPLES_BUILD_DIR)
	cmake -S examples/c -B $(EXAMPLES_BUILD_DIR)
	cmake --build $(EXAMPLES_BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR) $(EXAMPLES_BUILD_DIR)
