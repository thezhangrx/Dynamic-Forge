# ===========================================================================
# core/board/ —— PS 侧（板上）代码的构建
#
# 目标平台：**复旦微悟净 PSOC Lite**，PS 是 4×Cortex-A7，跑 **Ubuntu 20.04 aarch64**。
#
# 语言为什么是 C++17 而不是 C：
#   1. 厂商整套 PS 侧栈就是 C++17 —— 见
#      `fpai_demo_app/examples/1_single_input+ai/PLin+SingleNet+HDMI/CMakeLists.txt`
#      的 `set(CMAKE_CXX_STANDARD 17)`，示例源码是 `adder_demo.cpp`；
#   2. 我们要用的 `icraft-xrt`（`Device::Open` / `defaultRegRegion()` /
#      `defaultMemRegion()`）**只有 C++ 接口**；
#   3. 项目自己的 `core/cpu/docs/CPP_PORT.md` 已经规定决策要移植到 C++。
#   三者方向一致，所以不纠结。
#
# 构建（在**主机**上交叉编译，不要在板子上编 —— 厂商 README 明确写了）：
#
#   cmake -S core/board -B build/board -DCMAKE_TOOLCHAIN_FILE=... -DICRAFT_SYSROOT=/path
#   cmake --build build/board -j$(nproc)
#
# `ICRAFT_SYSROOT` 指向解开的 icraft deb 前缀（`usr/{include,lib,cmake}`）。
# 交给板子时把可执行文件拷过去即可（aarch64 静态链或带上 .so）。
# ===========================================================================
cmake_minimum_required(VERSION 3.16)
project(vision_board CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 本机也能编的目标：线协议解码 + 驱动逻辑（都不含 icraft）
#
# 用途：
#   * `tests/test_vision_driver.cpp` —— 把"写 start → 等 busy → 按 FLEN 读窗口
#     → 拼字节"这条**顺序**测干净；
#   * `tests/geometry_frame_dump.cpp` —— 把几何帧字节解成文本，给 Python 侧的
#     `tests/test_geometry_frame_python.py` 对账（线协议有三个实现，这是第三个）。
#     **不用等板子。**
# ---------------------------------------------------------------------------
add_library(vision_driver_core STATIC
    vision_driver.cpp
    geometry_frame.cpp
)
target_include_directories(vision_driver_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 决策：`core/cpu/gap_avoid.py` 的 C++ 移植（`core/cpu/cpp/`）。
# 板上 `main.cpp` 要用它；本机测试也用它。
set(GAP_AVOID_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../cpu/cpp")
add_library(gap_avoid STATIC "${GAP_AVOID_DIR}/gap_avoid.cpp")
target_include_directories(gap_avoid PUBLIC "${GAP_AVOID_DIR}")
# -ffp-contract=off 是**必须**的：GCC 默认会把 a*b+c 收缩成 FMA，改变舍入，
# 于是 tier 会在判据边界上与 Python 分家。见 core/cpu/cpp/README.md。
target_compile_options(gap_avoid PRIVATE -ffp-contract=off)

option(VISION_BOARD_BUILD_TESTS "构建本机自测" ON)
if(VISION_BOARD_BUILD_TESTS)
    enable_testing()

    add_executable(test_vision_driver tests/test_vision_driver.cpp)
    target_link_libraries(test_vision_driver PRIVATE vision_driver_core)
    add_test(NAME vision_driver COMMAND test_vision_driver)

    # 给 Python 对账用的解码器（`core/board/tests/test_geometry_frame_python.py`）
    add_executable(geometry_frame_dump tests/geometry_frame_dump.cpp)
    target_link_libraries(geometry_frame_dump PRIVATE vision_driver_core)
endif()

# ---------------------------------------------------------------------------
# 板上目标：需要 icraft SDK
#
# 只有开了 `VISION_BOARD_WITH_ICRAFT` 才编 —— PC 上没有 aarch64 的 icraft
# 运行时，默认不编，保证 `cmake -S core/board -B build` 在任何机器上都能过。
# ---------------------------------------------------------------------------
option(VISION_BOARD_WITH_ICRAFT "构建需要 icraft SDK 的板上目标" OFF)

if(VISION_BOARD_WITH_ICRAFT)
    # icraft 的 CMake 包：解开的 deb 里是 usr/cmake/icraft-*-config.cmake
    if(NOT DEFINED ICRAFT_SYSROOT)
        message(FATAL_ERROR
            "开了 VISION_BOARD_WITH_ICRAFT 就必须给 -DICRAFT_SYSROOT=<解开的 deb 前缀>\n"
            "  该前缀下应有 usr/cmake/icraft-zg330backend-config.cmake")
    endif()
    list(APPEND CMAKE_PREFIX_PATH "${ICRAFT_SYSROOT}/usr")
    include_directories("${ICRAFT_SYSROOT}/usr/include")

    # 厂商示例用的三个包名（见它的 CMakeLists）
    find_package(icraft-zg330backend REQUIRED)
    find_package(Icraft-HostBackend REQUIRED)

    add_executable(vision_board_app
        vision_driver_icraft.cpp
        main.cpp
    )
    target_link_libraries(vision_board_app PRIVATE
        vision_driver_core
        gap_avoid
        Icraft::HostBackend
        Icraft::ZG330Backend
    )
endif()
