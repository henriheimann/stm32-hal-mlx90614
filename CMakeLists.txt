add_library(stm32-hal-mlx90614 mlx90614.c mlx90614.h)
target_include_directories(stm32-hal-mlx90614 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
