#!/bin/bash

set -eo pipefail

CLANGD=clang-format-15
if ! command -v $CLANGD &>/dev/null; then
	echo "Error: $CLANGD is not installed. Please install it to format the code."
	echo "Or if you have a different version of clang-format installed, please update."
	exit 1
fi

echo "Formatting source files..."

IGNORED_DIRECTORIES=(
	"apriltag_detection"
	"realsense2_camera"
	"realsense2_camera_msgs"
	"wit_ros2_imu"
)

# Build the grouped prune expression used by find.
FIND_EXPRESSION=(./src)
if ((${#IGNORED_DIRECTORIES[@]} > 0)); then
	FIND_EXPRESSION+=(\()
	for directory in "${IGNORED_DIRECTORIES[@]}"; do
		FIND_EXPRESSION+=(-path "./src/$directory" -o)
	done
	unset 'FIND_EXPRESSION[-1]'
	FIND_EXPRESSION+=(\) -prune -o)
fi
FIND_EXPRESSION+=(
	-type f
	\(
	-name "*.c" -o
	-name "*.h" -o
	-name "*.cpp" -o
	-name "*.hpp" -o
	-name "*.cc" -o
	-name "*.hh"
	\)
)

# Run formatting three times to ensure that all formatting is applied correctly
for i in {1..3}; do
	echo "Running formatting pass $i..."
	find "${FIND_EXPRESSION[@]}" -exec "$CLANGD" -i {} +
done

echo "Formatting complete."
