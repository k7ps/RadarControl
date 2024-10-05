cmake_minimum_required(VERSION 3.15)

# set standard C17
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-std=c17")

set(ProjectName "RadarControl")
project(${ProjectName})

add_executable(${ProjectName} 
    main.c
)
