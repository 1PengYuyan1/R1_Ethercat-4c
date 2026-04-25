cmake_minimum_required(VERSION 3.10)
project(linkx_soem_demo C CXX)

# 设置 C 标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 自动搜集所有的源文件 (GLOB_RECURSE 会自动遍历 src 下的所有多级子目录)
file(GLOB_RECURSE SRC_FILES "src/*.c" "src/*.cpp")

# 生成可执行程序
add_executable(${PROJECT_NAME} ${SRC_FILES})

# add_executable(linkx_soem_demo 
#     src/main.cpp   
#     src/5_Task/task/task.cpp
# )

# 指定头文件搜索路径 
# (推荐使用 target_include_directories，比 include_directories 作用域更清晰安全)
target_include_directories(${PROJECT_NAME} PRIVATE
    inc
    inc/1_Middleware
    inc/2_Device
    
    inc/1_Middleware/soem
    inc/1_Middleware/linkx
    inc/1_Middleware/npcap
    inc/1_Middleware/math

    
    inc/2_Device/ecat_manager
    inc/2_Device/linkx4c_handler
    inc/2_Device/rt_timing
    inc/2_Device/Motor
    inc/2_Device/Remote

    inc/3_Chariot/chassis

    inc/4_Interaction/robot

    inc/5_Task/task

)

# 4. 链接 Linux 系统库
# pcap   : 用于 SOEM 底层的网卡数据收发
# pthread: 用于多线程支持
# rt     : 用于 clock_gettime, clock_nanosleep 等高精度实时时钟函数
target_link_libraries(${PROJECT_NAME} PRIVATE 
    pcap 
    pthread 
    rt
    m
)