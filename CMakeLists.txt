cmake_minimum_required(VERSION 3.10.0)
project(xeno_rtheap VERSION 0.1.0 LANGUAGES C CXX)

add_executable(xeno_posix ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)
set(xenomaiDir /usr/xenomai)
set(xeno_cflags_params "--posix" "--cflags")
# 为编译器添加xenomai编译选项
execute_process(
    COMMAND xeno-config ${xeno_cflags_params}
    WORKING_DIRECTORY ${xenomaiDir}/bin
    RESULT_VARIABLE Rvar
    OUTPUT_VARIABLE xeno_cflags
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
#  将标志字符串分割成列表
string(REPLACE " " ";" xeno_cflags_list "${xeno_cflags}")
message(status "xenomai-config cflags return ${Rvar}")
message(status ${xeno_cflags_list})
set(xeno_ldflags_params "--posix" "--ldflags")
execute_process(
    COMMAND xeno-config ${xeno_ldflags_params}
    WORKING_DIRECTORY ${xenomaiDir}/bin
    RESULT_VARIABLE Rvar
    OUTPUT_VARIABLE xeno_ldflags
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
string(REPLACE " " ";" xeno_ldflags_list "${xeno_ldflags}")
message(status "xenomai-config ldflags return ${Rvar}")
message(status ${xeno_ldflags_list})
target_include_directories(xeno_posix PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
 PRIVATE ${xenomaiDir}/include/cobalt
 PRIVATE ${xenomaiDir}/include/trank
 PRIVATE /usr/local/include
 )
target_sources(xeno_posix PRIVATE
 ${CMAKE_SOURCE_DIR}/source/Nano.cpp
 )
# Compiler and linker options
target_compile_options(xeno_posix  PRIVATE ${xeno_cflags_list})
target_link_options(xeno_posix PRIVATE ${xeno_ldflags_list})
# 也可以使用find_library来查找库文件
# find_library(ETHERCAT_STATIC_LIB 
#     NAMES libethercat.a
#     PATHS /usr/local/lib
# )

find_library(ETHERCAT_SHARED_LIB 
    NAMES ethercat
    PATHS /usr/local/lib
)
target_link_libraries(xeno_posix PRIVATE ${ETHERCAT_SHARED_LIB})


add_executable(nano_thread ${CMAKE_CURRENT_SOURCE_DIR}/main2.cpp)
target_include_directories(nano_thread PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
 PRIVATE /usr/local/include
 )
 target_sources(nano_thread PRIVATE
 ${CMAKE_SOURCE_DIR}/source/Nano.cpp
 )
 find_package(Threads REQUIRED)
target_link_libraries(nano_thread PRIVATE Threads::Threads)
