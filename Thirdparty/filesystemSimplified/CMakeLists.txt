cmake_minimum_required(VERSION 3.10)
project(stlplus)

file(GLOB LIBSLTPLUS_HPP *.hpp )
file(GLOB LIBSLTPLUS_CPP *.cpp )

add_library(stlplus ${LIBSLTPLUS_HPP} ${LIBSLTPLUS_CPP})
target_include_directories(stlplus INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS stlplus DESTINATION lib)
install(FILES ${LIBSLTPLUS_HPP} DESTINATION include)
