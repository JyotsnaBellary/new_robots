link_directories(
  ${CMAKE_BINARY_DIR}/controllers/newepuck_test
  ${CMAKE_BINARY_DIR}/controllers/testbot_test
  ${CMAKE_BINARY_DIR}/controllers/turtlebot4_test
)
set(test_loop_functions_SOURCES test_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(test_loop_functions_SOURCES
    ${test_loop_functions_SOURCES})
endif(ARGOS_QTOPENGL_FOUND)

add_library(test_loop_functions MODULE ${test_loop_functions_SOURCES})
  target_link_libraries(test_loop_functions
  newepuck_shared_test
  testbot_shared_test
  turtlebot4_shared_test
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_newepuck
  argos3plugin_simulator_testbot
  argos3plugin_simulator_turtlebot4
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(test_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
