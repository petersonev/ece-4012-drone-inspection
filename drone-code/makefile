PARROT_SDK_DIR = external/parrot_sdk
BUILD_SCRIPT = $(PARROT_SDK_DIR)/build.sh
OUTPUT_DIR = $(PARROT_SDK_DIR)/out/arsdk-native/staging/usr/bin

UNAME_S = $(shell uname -s)
WRAPPER = native-wrapper.sh
ifeq ($(UNAME_S),Darwin)
	WRAPPER = native-darwin-wrapper.sh
endif

all:
	mkdir -p build/
	cd build/ && cmake .. && make

run: all
	mkdir -p logs
	./build/flight_control/drone_inspection

run-gdb: all
	mkdir -p logs
	gdb ./build/flight_control/drone_inspection

clean:
	rm -rf build/

test-path-planner: all
	./build/path_planning/path_planner_test path_planning/test_data/matlab_out.csv

# SDK Specific Commands
setup:
	# May want to dump this into a setup script eventually
	rm -rf external/
	mkdir -p external
	mkdir -p ${PARROT_SDK_DIR}
	cd ${PARROT_SDK_DIR} && repo init -u https://github.com/Parrot-Developers/arsdk_manifests.git -m release.xml && repo sync
	$(PARROT_SDK_DIR)/build.sh -p arsdk-native -t build-sdk -j

clean-sdk:
	cd $(PARROT_SDK_DIR) && repo forall -vc "git reset --hard"

STYLE_EXCLUDE_DIRS=build \
				   external
# automatically format code according to our style config defined in .clang-format
pretty:
	@stylize --clang_style=file --yapf_style=.style.yapf --exclude_dirs $(STYLE_EXCLUDE_DIRS)
