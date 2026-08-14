override CC := /usr/bin/gcc
INCLUDE_DIRS = -I. -I../hardware_adapter

# Debug flags (DEFAULT)
CFLAGS = -Wall -Wextra -g3 -O0 -std=c11 -pthread -DDEBUG $(INCLUDE_DIRS)
LDFLAGS = -pthread -lzmq -lm -g3

# Release flags (optimized)
RELEASE_CFLAGS = -Wall -Wextra -O2 -std=c11 -pthread $(INCLUDE_DIRS)
RELEASE_LDFLAGS = -pthread -lzmq -lm

# Directories
OBJ_DIR = obj
OBJ_DIR_RELEASE = obj_release

# Source files
SRC = simpleMavlinkTest.c
ZMQ_WRAPPER_SRC = ../hardware_adapter/zmq_wrapper.c
OBJ = $(SRC:.c=$(OBJ_DIR)/.o) $(ZMQ_WRAPPER_SRC:.c=$(OBJ_DIR)/.o)
OBJ_RELEASE = $(SRC:.c=$(OBJ_DIR_RELEASE)/.o) $(ZMQ_WRAPPER_SRC:.c=$(OBJ_DIR_RELEASE)/.o)
TARGET = simpleMavlinkTest
TARGET_RELEASE = simpleMavlinkTest_release

# Default target (debug)
all: debug

# Debug build (default)
debug: $(TARGET)

# Release build
release: $(TARGET_RELEASE)

# Create directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR_RELEASE):
	mkdir -p $(OBJ_DIR_RELEASE)

# Build debug target
$(TARGET): $(OBJ_DIR)/simpleMavlinkTest.o $(OBJ_DIR)/zmq_wrapper.o | $(OBJ_DIR)
	$(CC) $(OBJ_DIR)/simpleMavlinkTest.o $(OBJ_DIR)/zmq_wrapper.o -o $@ $(LDFLAGS)
	@echo "Debug build complete: $@"

# Build release target
$(TARGET_RELEASE): $(OBJ_DIR_RELEASE)/simpleMavlinkTest.o $(OBJ_DIR_RELEASE)/zmq_wrapper.o | $(OBJ_DIR_RELEASE)
	$(CC) $(OBJ_DIR_RELEASE)/simpleMavlinkTest.o $(OBJ_DIR_RELEASE)/zmq_wrapper.o -o $@ $(RELEASE_LDFLAGS)
	@echo "Release build complete: $@"

# Compile debug source files
$(OBJ_DIR)/simpleMavlinkTest.o: simpleMavlinkTest.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/zmq_wrapper.o: ../hardware_adapter/zmq_wrapper.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Compile release source files
$(OBJ_DIR_RELEASE)/simpleMavlinkTest.o: simpleMavlinkTest.c | $(OBJ_DIR_RELEASE)
	$(CC) $(RELEASE_CFLAGS) -c $< -o $@

$(OBJ_DIR_RELEASE)/zmq_wrapper.o: ../hardware_adapter/zmq_wrapper.c | $(OBJ_DIR_RELEASE)
	$(CC) $(RELEASE_CFLAGS) -c $< -o $@

# Clean
clean:
	rm -rf $(OBJ_DIR) $(OBJ_DIR_RELEASE) $(TARGET) $(TARGET_RELEASE)
	@echo "Clean complete"

# Help
help:
	@echo "Available targets:"
	@echo "  all     - Build simpleMavlinkTest (debug, default)"
	@echo "  debug   - Build simpleMavlinkTest with debug symbols (same as 'all')"
	@echo "  release - Build simpleMavlinkTest optimized (no debug symbols)"
	@echo "  clean   - Remove build artifacts"
	@echo "  help    - Show this help message"

.PHONY: all debug release clean help
