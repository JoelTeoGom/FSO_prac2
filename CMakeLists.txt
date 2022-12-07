cmake_minimum_required(VERSION 3.21)
project(Prac_2 C)

set(CMAKE_C_STANDARD 11)

include_directories(mur1)


add_executable(Prac_2
        mur0/mur1.c
        mur0/winsuport.c
        mur0/winsuport.h
        mur3/pilota3.c)
