add_library(testbot_shared_test STATIC testbot_test.h testbot_test.cpp)
add_library(testbot_test MODULE testbot_test.h testbot_test.cpp)
target_link_libraries(testbot_test
  argos3core_simulator
  argos3plugin_simulator_testbot
  argos3plugin_simulator_genericrobot)
