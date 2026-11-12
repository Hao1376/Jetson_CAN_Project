cmake_minimum_required (VERSION 3.15)

project (demo)



set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(include)

# Bsp_Can_Linux.c 使用 pthread (epoll 中断接收线程)
find_package(Threads REQUIRED)

add_executable(main
    source/main.c
    source/Bsp_Can_Linux.c
    source/Star_Can_Protocol.c
    source/Camera_Ctrl_Cmd.c
    source/Camera_Task.c
    source/Camera_Shot.c
    source/Log_Udp.c
)

target_link_libraries(main Threads::Threads socketcan)
target_compile_options(main PRIVATE -Wall -Wextra)
