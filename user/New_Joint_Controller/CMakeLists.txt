include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")

add_executable(new_ctrl main.cpp New_Controller.cpp)
target_link_libraries(new_ctrl robot biomimetics dynacore_param_handler qpOASES)
