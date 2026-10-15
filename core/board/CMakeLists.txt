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
project(vision_board C CXX)   # C 也要：C 参考模型（黄金模型）是 .c

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
# 板上目标（**默认就编，不需要 icraft**）：直接 /dev/mem 访问 PL 的寄存器窗口
#
# 为什么这是板上 bring-up 的主路径：
#   * 我们这块板子上 PL 的寄存器窗口（0x43C00000，见 vision_regs.hpp 的
#     kRegBase）用 /dev/mem 就能读写 —— core/board/tools/ 已在真硅上验过；
#   * icraft 的 defaultRegRegion() 是**另一个窗口**（0x400C0000，NPU 那边），
#     而且链它还要交叉编译 + 解 deb 做 sysroot；
#   * 板子自带 g++/cmake，所以"在板上直接编"最短，也少一个依赖。
#
# 本机（x86_64）也能编：/dev/mem 打不开是**运行期**的事，编译期无依赖。
# ---------------------------------------------------------------------------
# 离散动作量化：`core/bullet_sim/core/actions.py` 的 DISCRETE_*
# 在 C++ 侧的镜像（见 action_codec.hpp）。板上 main 用它，本机差分测试也用它。
add_library(action_codec STATIC action_codec.cpp)
target_include_directories(action_codec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# -ffp-contract=off 与 gap_avoid **同一个理由，而且这里已经被实测抓到过**：
# GCC 默认把 `dx*wx + dy*wy` 收缩成 FMA，改变最后一位舍入，于是在 45° 边界上
# （相邻两个方向的点积本来就相等）C++ 与 Python 会**破不同的方向** ——
# 实测 527 组速度里有 2 组分家（C++ 选对角线、Python 选轴向）。
# 见 core/board/tests/test_action_codec_matches_python.py。
target_compile_options(action_codec PRIVATE -ffp-contract=off)

# C 参考模型（黄金模型）：PS 侧跑真图（真摄像头 PPM）时用它产出几何帧，
# 同时也是 PL 那侧的参照。VR_NO_MAIN 去掉它自带的 main。
set(REF_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../fpga/ref")
add_library(vision_ref STATIC "${REF_DIR}/vision_ref.c")
target_include_directories(vision_ref PUBLIC "${REF_DIR}")
target_compile_definitions(vision_ref PRIVATE VR_NO_MAIN)

# std::thread（StdinPump 的读线程）要显式链 pthread
find_package(Threads REQUIRED)

add_executable(vision_board_devmem
    vision_driver_devmem.cpp
    vision_driver_factory.cpp
    ppm_reader.cpp
    rgb_source.cpp
    main.cpp
)
target_link_libraries(vision_board_devmem PRIVATE
    vision_driver_core gap_avoid action_codec vision_ref Threads::Threads)

# ---------------------------------------------------------------------------
# 板上目标（可选）：需要 icraft SDK
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
        vision_driver_factory.cpp
        main.cpp
    )
    # 让工厂把 icraft 分支编进来（同时 devmem 分支照样可用，按 URL 选）
    target_compile_definitions(vision_board_app PRIVATE VISION_BOARD_HAVE_ICRAFT)
    target_link_libraries(vision_board_app PRIVATE
        vision_driver_core
        gap_avoid
        action_codec
        Icraft::HostBackend
        Icraft::ZG330Backend
    )
endif()
