cmake_minimum_required(VERSION 3.5)

idf_component_register(
                       SRCS "AS5600.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES arduino-esp32
)

project(AS5600)