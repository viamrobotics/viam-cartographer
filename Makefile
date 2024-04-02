BUILD_CHANNEL?=local
BUILD_DIR = build/$(shell uname -s)-$(shell uname -m)
BIN_OUTPUT_PATH = bin/$(shell uname -s)-$(shell uname -m)
TOOL_BIN := $(shell pwd)/bin/tools/$(shell uname -s)-$(shell uname -m)
GIT_REVISION := $(shell git rev-parse HEAD | tr -d '\n')
TAG_VERSION ?= $(shell git tag --points-at | sort -Vr | head -n1)
GO_BUILD_LDFLAGS := -ldflags "-X 'main.Version=${TAG_VERSION}' -X 'main.GitRevision=${GIT_REVISION}'"
CGO_BUILD_LDFLAGS := -L$(shell pwd)/viam-cartographer/$(BUILD_DIR) -L$(shell pwd)/viam-cartographer/$(BUILD_DIR)/cartographer
SHELL := /usr/bin/env bash
export PATH := $(TOOL_BIN):$(PATH)
export GOBIN := $(TOOL_BIN)

ifneq (, $(shell which brew))
	EXTRA_CMAKE_FLAGS := -DCMAKE_PREFIX_PATH=$(shell brew --prefix) -DQt5_DIR=$(shell brew --prefix qt5)/lib/cmake/Qt5
	export PKG_CONFIG_PATH := $(shell brew --prefix openssl@3)/lib/pkgconfig
endif

default: build

artifact-pull: $(TOOL_BIN)/artifact
	artifact pull

$(TOOL_BIN)/artifact:
	 go install go.viam.com/utils/artifact/cmd/artifact

clean:
	rm -rf grpc $(BIN_OUTPUT_PATH) viam-cartographer/$(BUILD_DIR)

clean-all:
	git clean -fxd
	cd viam-cartographer/cartographer && git checkout . && git clean -fxd

ensure-submodule-initialized:
	@if [ ! -d "viam-cartographer/cartographer/cartographer" ]; then \
		echo "Submodule was not found. Initializing..."; \
		git submodule update --init; \
	else \
		echo "Submodule found successfully"; \
	fi
	grep -q viam-patched viam-cartographer/cartographer/CMakeLists.txt || \
	(cd viam-cartographer/cartographer && git checkout . && git apply ../cartographer_patches/carto.patch)

lint-cpp:
	find . -type f -not -path \
		-and ! -path '*viam-cartographer/cartographer*' \
		-and ! -path '*viam-cartographer/build*' \
		-and ! -path '*api*' \
		-and ! -path '*grpc*' \
		-and \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cc' \) \
		| xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}"

lint-go: $(TOOL_BIN)/combined $(TOOL_BIN)/golangci-lint $(TOOL_BIN)/actionlint
	go vet -vettool=$(TOOL_BIN)/combined ./...
	GOGC=50 golangci-lint run -v --fix --config=./etc/golangci.yaml
	actionlint

$(TOOL_BIN)/combined $(TOOL_BIN)/golangci-lint $(TOOL_BIN)/actionlint:
	go install \
		github.com/edaniels/golinters/cmd/combined \
		github.com/golangci/golangci-lint/cmd/golangci-lint \
		github.com/rhysd/actionlint/cmd/actionlint

lint: ensure-submodule-initialized lint-cpp lint-go

setup: install-dependencies ensure-submodule-initialized artifact-pull

install-dependencies:
ifneq (, $(shell which brew))
	brew update
	brew install abseil boost viamrobotics/brews/suite-sparse@7.1 viamrobotics/brews/ceres-solver@2.1 protobuf ninja cairo googletest lua@5.3 pkg-config cmake go@1.20 grpc clang-format 
	brew link lua@5.3
	brew install openssl@3 eigen gflags glog sphinx-doc pcl viamrobotics/brews/nlopt-static
else ifneq (, $(shell which apt-get))
	$(warning  "Installing cartographer external dependencies via APT.")
	$(warning "Packages may be too old to work with this project.")
	sudo apt-get update
	sudo apt-get install -y cmake ninja-build libgmock-dev libboost-iostreams-dev liblua5.3-dev libcairo2-dev python3-sphinx libnlopt-dev \
		libabsl-dev libceres-dev libprotobuf-dev protobuf-compiler protobuf-compiler-grpc libpcl-dev libgrpc-dev libgrpc++-dev clang-format
else
	$(error "Unsupported system. Only apt and brew currently supported.")
endif

build: cartographer-module

viam-cartographer/build/unit_tests: ensure-submodule-initialized
	cd viam-cartographer && cmake -B$(BUILD_DIR) -G Ninja ${EXTRA_CMAKE_FLAGS} && cmake --build $(BUILD_DIR)

cartographer-module: viam-cartographer/build/unit_tests
	rm -f $(BIN_OUTPUT_PATH)/cartographer-module && mkdir -p bin
# Newer versions of abseil require extra ld flags in our module, so this ugly thing.
# It's expected that if NOT using brew, a prebuilt environment (like canon) is in use with the older abseil installed.
	absl_version=$$(brew list --versions abseil 2>/dev/null | head -n1 | grep -oE '[0-9]{8}' || echo 20010101); \
	export CGO_LDFLAGS="$$CGO_LDFLAGS $(CGO_BUILD_LDFLAGS)"; \
	test "$$absl_version" -gt "20230801" && export CGO_LDFLAGS="$$CGO_LDFLAGS -labsl_log_internal_message -labsl_log_internal_check_op" || true; \
	go build $(GO_BUILD_LDFLAGS) -tags osusergo,netgo -o $(BIN_OUTPUT_PATH)/cartographer-module module/main.go

# Ideally build-asan would be added to build-debug, but can't yet 
# as these options they fail on arm64 linux. This is b/c that 
# platform currently uses gcc as opposed to clang & gcc doesn't
# support using asan in this way (more work would be needed to get it to work there).
# Ticket: https://viam.atlassian.net/browse/RSDK-3538 is the ticket to 
# add full asan support
build-asan: EXTRA_CMAKE_FLAGS += -DCMAKE_CXXFLAGS="-fno-omit-frame-pointer -fsanitize=address -fsanitize-address-use-after-scope -O1" -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"
build-asan: export ASAN_OPTIONS= detect_leaks=1 detect_stack_use_after_return=true
build-asan: build-debug

build-debug: EXTRA_CMAKE_FLAGS += -DCMAKE_BUILD_TYPE=Debug -DFORCE_DEBUG_BUILD=True
build-debug: build

# Linux only
setup-cpp-debug: 
	sudo apt install -y valgrind gdb

test-cpp:
	viam-cartographer/$(BUILD_DIR)/unit_tests -p -l all

# Linux only
test-cpp-valgrind: build-debug
	valgrind --error-exitcode=1 --leak-check=full -s viam-cartographer/$(BUILD_DIR)/unit_tests -p -l all

# Linux only
test-cpp-gdb: build-debug
	gdb --batch --ex run --ex bt --ex q --args viam-cartographer/$(BUILD_DIR)/unit_tests -p -l all

test-cpp-asan: build-asan
	viam-cartographer/$(BUILD_DIR)/unit_tests -p -l all

test-go:
	absl_version=$$(brew list --versions abseil 2>/dev/null | head -n1 | grep -oE '[0-9]{8}' || echo 20010101); \
	export CGO_LDFLAGS="$$CGO_LDFLAGS $(CGO_BUILD_LDFLAGS)"; \
	test "$$absl_version" -gt "20230801" && export CGO_LDFLAGS="$$CGO_LDFLAGS -labsl_log_internal_message -labsl_log_internal_check_op" || true; \
	go test -race ./...

test: test-cpp test-go

install-lua-files:
	sudo mkdir -p /usr/local/share/cartographer/lua_files/
	sudo cp viam-cartographer/lua_files/* /usr/local/share/cartographer/lua_files/

install: install-lua-files
	sudo rm -f /usr/local/bin/cartographer-module
	sudo cp $(BIN_OUTPUT_PATH)/cartographer-module /usr/local/bin/cartographer-module

include *.make
