add_library(argos3plugin_simulator_newepuck SHARED
  control_interface/ci_newepuck_proximity_sensor.cpp
  control_interface/ci_newepuck_proximity_sensor.h
  simulator/dynamics2d_newepuck_model.cpp
  simulator/dynamics2d_newepuck_model.h
  simulator/newepuck_entity.cpp
  simulator/newepuck_entity.h
  simulator/qtopengl_newepuck.cpp
  simulator/qtopengl_newepuck.h)
target_link_libraries(argos3plugin_simulator_newepuck
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_simulator_newepuck
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_COMPILE_QTOPENGL)
