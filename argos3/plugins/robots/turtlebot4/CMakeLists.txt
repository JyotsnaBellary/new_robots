#Set all if conditions later.

#
# Turtlebot4 headers
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_CONTROLINTERFACE
  control_interface/ci_turtlebot4_base_ground_sensor.h
  control_interface/ci_turtlebot4_lidar_sensor.h
  control_interface/ci_turtlebot4_proximity_sensor.h
)
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_SIMULATOR
    simulator/dynamics2d_turtlebot4_model.h
    simulator/dynamics3d_turtlebot4_model.h
    simulator/turtlebot4_base_ground_rotzonly_sensor.h
    simulator/turtlebot4_lidar_default_sensor.h
    simulator/turtlebot4_proximity_default_sensor.h
    simulator/turtlebot4_entity.h
    )
    
#
# Turtlebot4 sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_TURTLEBOT4
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_CONTROLINTERFACE}
  control_interface/ci_turtlebot4_base_ground_sensor.cpp
  control_interface/ci_turtlebot4_lidar_sensor.cpp
  control_interface/ci_turtlebot4_proximity_sensor.cpp
)

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_TURTLEBOT4
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_TURTLEBOT4}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_SIMULATOR}
  simulator/dynamics2d_turtlebot4_model.cpp
  simulator/dynamics3d_turtlebot4_model.cpp
  simulator/turtlebot4_base_ground_rotzonly_sensor.cpp
  simulator/turtlebot4_lidar_default_sensor.cpp
  simulator/turtlebot4_proximity_default_sensor.cpp
  simulator/turtlebot4_entity.cpp
)

#
# For compiling graphical visualizations
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_SIMULATOR
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_SIMULATOR}
  simulator/qtopengl_turtlebot4.h)

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_TURTLEBOT4
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_TURTLEBOT4}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_SIMULATOR}
  # simulator/qtopengl_turtlebot4.h
  simulator/qtopengl_turtlebot4.cpp)

#
# Create Turtlebot4 plugin
#
add_library(argos3plugin_simulator_turtlebot4 SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_TURTLEBOT4})




# add_library(argos3plugin_simulator_turtlebot4 SHARED
#   control_interface/ci_turtlebot4_proximity_sensor.cpp
#   control_interface/ci_turtlebot4_proximity_sensor.h
#   simulator/dynamics2d_turtlebot4_model.cpp
#   simulator/dynamics2d_turtlebot4_model.h
#   simulator/turtlebot4_entity.cpp
#   simulator/turtlebot4_entity.h
#   simulator/qtopengl_turtlebot4.cpp
#   simulator/qtopengl_turtlebot4.h)

target_link_libraries(argos3plugin_simulator_turtlebot4
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_dynamics3d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_simulator_turtlebot4
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_COMPILE_QTOPENGL)


#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/turtlebot4/control_interface)
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TURTLEBOT4_SIMULATOR} DESTINATION include/argos3/plugins/robots/turtlebot4/simulator)
install(DIRECTORY simulator/models DESTINATION include/argos3/plugins/simulator/visualizations/qt-opengl)
install(FILES simulator/turtlebot4_texture_bottom.png simulator/turtlebot4_texture_side.png simulator/turtlebot4_texture_top.png DESTINATION include/argos3/plugins/simulator/visualizations/qt-opengl/textures)

install(TARGETS argos3plugin_simulator_turtlebot4
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)