# ============================================
# New e-puck plugin (macOS/Qt6 friendly)
# ============================================

# Sorgenti/headers comuni (headless + gui)
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_CONTROLINTERFACE
  control_interface/ci_newepuck_proximity_sensor.h
  control_interface/ci_newepuck_lidar_sensor.h
  control_interface/ci_newepuck_light_sensor.h)

set(ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR
  simulator/dynamics2d_newepuck_model.h
  #simulator/dynamics3d_newepuck_model.h
  simulator/newepuck_light_rotzonly_sensor.h
  simulator/newepuck_lidar_sensor.h
  simulator/newepuck_proximity_default_sensor.h
  simulator/newepuck_entity.h
)

set(ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_CONTROLINTERFACE}
  control_interface/ci_newepuck_proximity_sensor.cpp
  control_interface/ci_newepuck_lidar_sensor.cpp
  control_interface/ci_newepuck_light_sensor.cpp

  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR}
  simulator/dynamics2d_newepuck_model.cpp
  #simulator/dynamics3d_newepuck_model.cpp
  simulator/newepuck_light_rotzonly_sensor.cpp
  simulator/newepuck_lidar_sensor.cpp
  simulator/newepuck_entity.cpp
  simulator/newepuck_proximity_default_sensor.cpp
)

# Opzione per abilitare la GUI (di default ON se ARGoS la espone)
# ARGoS di solito definisce ARGOS_QTOPENGL_FOUND nella sua config.
option(NEWEPUCK_WITH_QTOPENGL "Build QtOpenGL renderer for NewEPuck" ${ARGOS_QTOPENGL_FOUND})

if(NEWEPUCK_WITH_QTOPENGL)
  list(APPEND ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR
    simulator/qtopengl_newepuck.h)
  list(APPEND ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK
    simulator/qtopengl_newepuck.cpp)
endif()

add_library(argos3plugin_simulator_newepuck SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_NEWEPUCK})

# --- Link base (senza duplicati) ---
target_link_libraries(argos3plugin_simulator_newepuck
  PRIVATE
    argos3core_simulator
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_entities
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_media
)

# --- GUI: linka plugin QtOpenGL di ARGoS + Qt6 modern targets ---
if(NEWEPUCK_WITH_QTOPENGL)
  # Se usi Qt6 (come nel tuo log)
  find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets Widgets Gui)
  target_link_libraries(argos3plugin_simulator_newepuck
    PRIVATE
      argos3plugin_simulator_qtopengl
      Qt6::OpenGLWidgets
      Qt6::Widgets
      Qt6::Gui
  )
  # (Niente ${QT_LIBRARIES}, ${GLUT_LIBRARY}, ${OPENGL_LIBRARY} – non servono con Qt6 modern)
endif()

# Esporta include per chi installa gli headers
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_CONTROLINTERFACE}
        DESTINATION include/argos3/plugins/robots/newepuck/control_interface)
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_NEWEPUCK_SIMULATOR}
        DESTINATION include/argos3/plugins/robots/newepuck/simulator)

install(TARGETS argos3plugin_simulator_newepuck
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

# Aggiorna ARGOS_PLUGIN_PATH per il runtime
set(ARGOS_PLUGIN_PATH "${ARGOS_PLUGIN_PATH}:${CMAKE_CURRENT_BINARY_DIR}" CACHE INTERNAL "ARGoS plugin path")