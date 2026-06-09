cmake_minimum_required(VERSION 3.27.9)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

project(stm32Plus LANGUAGES C CXX ASM)


#
# set the F4-specific hard float option
#
macro(floatOpt)
    if (STM32PLUS_FLOAT STREQUAL "hard")
                message(STATUS "启用硬件浮点传递模式: hard-float")
                add_compile_definitions(-mfloat-abi=hard)
                add_link_options(-Wl,-mfloat-abi=hard,-mfpu=fpv4-sp-d16)

    endif ()
endmacro()


macro(setFlags cpu libdef FAMILY)
    set(cpuOpt "-mcpu=cortex-${cpu}")
    set(libOpt "-DSTM32PLUS_${libdef}")
    set(fwlibOpt ${FAMILY})

    set(common_flags
            ${cpuOpt}
            -mthumb
            -pipe
            -gdwarf-5
            -ffunction-sections
            -fdata-sections
            -Wall
            -Werror
            -Wdouble-promotion
            -Wformat=2
            -Wformat-truncation=2
            -Wundef
            -fno-common
            -Wno-implicit-fallthrough
#            -fstack-usage
#            -Wconversion
    )

    set(c_flags )

    set(cxx_flags
            -std=gnu++23
            -Wextra
            -pedantic-errors
            -fno-exceptions
            -fno-rtti
            -fno-threadsafe-statics
    )
    add_compile_options(${common_flags})

    add_compile_options(
                        "$<$<COMPILE_LANGUAGE:C>:${c_flags}>"
            "$<$<COMPILE_LANGUAGE:CXX>:${cxx_flags}>"
    )

    add_link_options(
            ${cpuOpt}
            -mthumb
            -Wl,--gc-sections
    )
endmacro()



if (NOT STM32PLUS_MCU)
    message(FATAL_ERROR "could not find STD for family ${FAMILY}")
endif ()


if (STM32PLUS_MCU STREQUAL "f1hd")
    setFlags("m3" "F1_HD" "f1")
elseif (STM32PLUS_MCU STREQUAL "f1cle")
    setFlags("m3" "F1_CL_E" "f1")
elseif (STM32PLUS_MCU STREQUAL "f1mdvl")
    setFlags("m3" "F1_MD_VL")
    #elseif (STM32PLUS_MCU STREQUAL "f1md" "f1")
    #    setFlags("m3" "F1_MD")
elseif ( STM32PLUS_MCU STREQUAL "f407")
    setFlags("m4" "F407" "f4")
            floatOpt()
elseif (STM32PLUS_MCU STREQUAL "f405")
    setFlags("m4" "F405" "f4")
    #        floatOpt()
elseif (STM32PLUS_MCU STREQUAL "f415")
    setFlags("m4" "F415" "f4")
    #        floatOpt()
elseif (STM32PLUS_MCU STREQUAL "f417")
    setFlags("m4" "F417" "f4")
    #        floatOpt()
elseif (STM32PLUS_MCU STREQUAL "f427")
    setFlags("m4" "F427" "f4")
    #        floatOpt()
elseif (STM32PLUS_MCU STREQUAL "f437")
    setFlags("m4" "F437" "f4")
    #        floatOpt()
elseif (STM32PLUS_MCU STREQUAL "f429")
    setFlags("m4" "F429" "f4")
    #        floatOpt()
elseif (STM32PLUS_MCU STREQUAL "f439")
    setFlags("m4" "F439" "f4")
    #        floatOpt()
elseif (STM32PLUS_MCU STREQUAL "f051")
    setFlags("m0" "F0_51" "f4")
elseif (STM32PLUS_MCU STREQUAL "f030")
    setFlags("m0" "F0_30" "f4")
elseif (STM32PLUS_MCU STREQUAL "f042")
    setFlags("m0" "F0_42" "f4")
else ()
    message(FATAL_ERROR "could not find MCU default for family ${STM32PLUS_MCU}")
endif ()



macro(add_glob cur_list)
    file(GLOB_RECURSE __tmp CONFIGURE_DEPENDS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${ARGN})
    list(APPEND ${cur_list} ${__tmp})
endmacro()

macro(add_headers_and_sources prefix common_path)
    add_glob(${prefix}_headers ${common_path}/*.h)
    add_glob(${prefix}_sources ${common_path}/*.cpp ${common_path}/*.c)
endmacro()

macro(add_headers_only prefix common_path)
    add_glob(${prefix}_headers ${common_path}/*.h)
endmacro()

add_headers_and_sources(stm32plus lib/src)

if (DEFINED STM32PLUS_USB)
    add_headers_and_sources(stm32plus lib/usblib/device/core/src)
    add_headers_and_sources(stm32plus lib/usblib/hal/src)
endif ()

if (fwlibOpt STREQUAL "f1")
    add_headers_and_sources(stm32plus lib/fwlib/f1)

elseif (fwlibOpt STREQUAL "f4")
    add_headers_and_sources(stm32plus lib/fwlib/f4)
endif ()

add_library(stm32plus  STATIC  ${stm32plus_headers} ${stm32plus_sources})

target_include_directories(stm32plus
        SYSTEM BEFORE PUBLIC
        "lib"
        "lib/include"
)

if (fwlibOpt STREQUAL "f1")

elseif (fwlibOpt STREQUAL "f4")
    target_include_directories(stm32plus
            SYSTEM BEFORE PUBLIC
            "lib/fwlib/f4/cmsis/Include"
    )
endif ()


if (USER_THREADX)
    target_compile_definitions(stm32plus PUBLIC USER_THREADX=1)
    target_link_libraries(stm32plus PUBLIC threadx)
endif ()


# HSE or HSI
if (STM32PLUS_HSE)
    set(STM32PLUS_OSC_TYPE "e")
    set(STM32PLUS_OSC_def "HSE_VALUE")
    set(STM32PLUS_OSC ${STM32PLUS_HSE})
    target_compile_definitions(stm32plus PUBLIC -DHSE_VALUE=${STM32PLUS_HSE})
elseif (STM32PLUS_HSI)
    set(STM32PLUS_OSC_TYPE "i")
    set(STM32PLUS_OSC_def "HSI_VALUE")
    set(STM32PLUS_OSC ${STM32PLUS_HSI})
else ()
    message(FATAL_ERROR "could not find OSC default")
endif ()

target_compile_definitions(stm32plus PUBLIC ${libOpt})
