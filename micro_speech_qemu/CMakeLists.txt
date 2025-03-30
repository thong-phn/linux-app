cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(micro_speech)

# These samples use local static initialization. Since Zephyr doesn't support the
# C++ ABI for thread-safe initialization of local statics and the constructors don't
# appear to require thread safety, we turn it off in the C++ compiler.
set(NO_THREADSAFE_STATICS $<TARGET_PROPERTY:compiler-cpp,no_threadsafe_statics>)
zephyr_compile_options($<$<COMPILE_LANGUAGE:CXX>:${NO_THREADSAFE_STATICS}>)

file(
    GLOB 
    app_sources 
    src/*
    # ${ZEPHYR_TFLITE_MICRO_MODULE_DIR}/signal/micro/kernels/xtensa/*
    # ${ZEPHYR_TFLITE_MICRO_MODULE_DIR}/signal/micro/kernels/*

    )

target_include_directories(app PRIVATE
    ${ZEPHYR_TFLITE_MICRO_MODULE_DIR}/signal
    ${ZEPHYR_TFLITE_MICRO_MODULE_DIR}/signal/micro/kernels/
    ${ZEPHYR_TFLITE_MICRO_MODULE_DIR}/signal/src)

target_sources(
    app 
    PRIVATE 
    ${app_sources}
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/xtensa/fft_auto_scale_kernel.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/xtensa/filter_bank_square_root.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/xtensa/xtensa_square_root.S

    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/fft_auto_scale.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/max_abs.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/msb_32.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/energy.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/filter_bank.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/pcan_argc_fixed.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/filter_bank_log.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/log.cc

    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/src/filter_bank_spectral_subtraction.cc


    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/energy.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/fft_auto_scale_common.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/filter_bank_square_root_common.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/filter_bank_spectral_subtraction.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/filter_bank.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/pcan.cc
    /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/filter_bank_log.cc
    
    # /home/thongp/zephyrproject/optional/modules/lib/tflite-micro/signal/micro/kernels/fft_auto_scale_kernel.cc
    # ${ZEPHYR_TFLITE_MICRO_MODULE_DIR}/signal/micro/kernels/xtensa/
    # ${ZEPHYR_TFLITE_MICRO_MODULE_DIR}/signal/micro/kernels/
    )
