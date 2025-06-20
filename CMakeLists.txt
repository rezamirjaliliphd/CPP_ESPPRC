cmake_minimum_required(VERSION 3.10)
project(CPP_ESPPRC)

set(CMAKE_CXX_STANDARD 17)

add_executable(CPP_ESPPRC
    ESPPRC.cpp
    Edge.cpp
    Graph.cpp
    Label.cpp
    LabelManager.cpp
    Solution.cpp
    Utils.cpp
)
