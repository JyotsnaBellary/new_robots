#Set all if conditions later.

#
# New E-puck headers
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_CONTROLINTERFACE
  control_interface/ci_newepuck_proximity_sensor.h
  control_interface/ci_newepuck_base_ground_sensor.h
  control_interface/ci_newepuck_lidar_sensor.h
  control_interface/ci_newepuck_light_sensor.h)

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR
    simulator/dynamics2d_newepuck_model.h
    simulator/dynamics3d_newepuck_model.h
    simulator/newepuck_base_ground_rotzonly_sensor.h
    simulator/newepuck_light_rotzonly_sensor.h
    simulator/newepuck_lidar_default_sensor.h
    simulator/newepuck_proximity_default_sensor.h
    simulator/newepuck_entity.h
    )
    
#
# New E-puck sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_CONTROLINTERFACE}
  control_interface/ci_newepuck_proximity_sensor.cpp
  control_interface/ci_newepuck_base_ground_sensor.cpp
  control_interface/ci_newepuck_lidar_sensor.cpp
  control_interface/ci_newepuck_light_sensor.cpp)

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR}
  simulator/dynamics2d_newepuck_model.cpp
  simulator/dynamics3d_newepuck_model.cpp
  simulator/newepuck_base_ground_rotzonly_sensor.cpp
  simulator/newepuck_light_rotzonly_sensor.cpp
  simulator/newepuck_lidar_default_sensor.cpp
  simulator/newepuck_entity.cpp
  simulator/newepuck_proximity_default_sensor.cpp
)

#
# For compiling graphical visualizations
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR}
  simulator/qtopengl_newepuck.h)

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR}
  simulator/qtopengl_newepuck.cpp)

#
# Create new e-puck plugin
#
add_library(argos3plugin_simulator_newepuck SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK})

target_link_libraries(argos3plugin_simulator_newepuck
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_dynamics3d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_simulator_newepuck
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_COMPILE_QTOPENGL)


#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/newepuck/control_interface)
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR} DESTINATION include/argos3/plugins/robots/newepuck/simulator)

install(TARGETS argos3plugin_simulator_newepuck
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)