#Set all if conditions later.

#
#  Testbot headers
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_CONTROLINTERFACE
  # control_interface/ci_testbot_proximity_sensor.h
  )

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_SIMULATOR
    simulator/dynamics2d_testbot_model.h
    simulator/testbot_entity.h
    # simulator/testbot_proximity_default_sensor.h
    )
    
#
# Testbot sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_TESTBOT
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_CONTROLINTERFACE}
  # control_interface/ci_testbot_proximity_sensor.cpp
  )

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_TESTBOT
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_TESTBOT}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_SIMULATOR}
  simulator/dynamics2d_testbot_model.cpp
  simulator/testbot_entity.cpp
  # simulator/testbot_proximity_default_sensor.cpp
)

#
# For compiling graphical visualizations
#
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_SIMULATOR
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_SIMULATOR}
  simulator/qtopengl_testbot.h)

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_TESTBOT
  ${ARGOS3_SOURCES_PLUGINS_ROBOTS_TESTBOT}
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_SIMULATOR}
  simulator/qtopengl_testbot.cpp)

#
# Create  testbot plugin
#
add_library(argos3plugin_simulator_testbot SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_TESTBOT})

target_link_libraries(argos3plugin_simulator_testbot
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    # argos3plugin_simulator_dynamics3d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_simulator_testbot
    argos3plugin_simulator_qtopengl
    ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
endif(ARGOS_COMPILE_QTOPENGL)


#
# Add plugin to ARGOS_PLUGIN_PATH
#
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/testbot/control_interface)
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_TESTBOT_SIMULATOR} DESTINATION include/argos3/plugins/robots/testbot/simulator)

install(TARGETS argos3plugin_simulator_testbot
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)