# This is an experimental robot built
add_library(argos3plugin_simulator_myepuck SHARED
  # Add control_interface files
  control_interface/ci_myepuck_light_sensor.cpp
  control_interface/ci_myepuck_light_sensor.h
  control_interface/ci_myepuck_proximity_sensor.cpp
  control_interface/ci_myepuck_proximity_sensor.h
  # Add the simulator files
  simulator/myepuck_light_rotzonly_sensor.cpp
  simulator/myepuck_light_rotzonly_sensor.h
  simulator/dynamics2d_myepuck_model.cpp
  simulator/dynamics2d_myepuck_model.h
  simulator/myepuck_proximity_default_sensor.cpp
  simulator/myepuck_proximity_default_sensor.h
  simulator/myepuck_entity.cpp
  simulator/myepuck_entity.h
  simulator/qtopengl_myepuck.cpp
  simulator/qtopengl_myepuck.h)
target_link_libraries(argos3plugin_simulator_myepuck
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_simulator_myepuck
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_COMPILE_QTOPENGL)

# ToDo: Install the plugin to lib/argos path