# Adding a New ARGoS3 Robot Plugin
Use this document as a checklist when you want to contribute a new robot plugin to this repository or reuse the structure for your own project.

## Prerequisites
- ARGoS3 is installed and working (compile and launch an existing experiment first—try one from [argos3-examples](https://github.com/ilpincy/argos3-examples)).
- You can build this repo (`./build.sh` or `cmake && make`).
- Familiarity with the ARGoS plugin concept (controllers, sensors, actuators, loop functions).

## 1. Pick a Template
Three reference plugins live in `argos3/plugins/robots/`:

- `testbot`: a minimal skeleton focused on defining the entity and Qt-OpenGL visualization (sensors/actuators are stubbed out but the structure is in place).
- `newepuck`: a textured robot with several sensors/actuators.
- `turtlebot4`: a complete, fully textured robot with several sensors/actuators.

Copy the directory that best matches your needs:

```bash
cd argos3/plugins/robots
cp -r testbot myrobot
```

Rename files/types inside the new directory so they describe your robot.

## 2. Register the Plugin with CMake

Edit `argos3/plugins/robots/CMakeLists.txt` and add:

```cmake
add_subdirectory(myrobot)
```

No other top-level change is needed because `argos3/CMakeLists.txt` already adds the `argos3` subtree.

## 3. Describe Headers/Sources in `myrobot/CMakeLists.txt`

Inside `argos3/plugins/robots/myrobot/CMakeLists.txt`:

1. Declare two `set(...)` blocks listing control-interface headers and simulator headers.
2. Append sources (`.cpp`) to `ARGOS3_SOURCES_PLUGINS_ROBOTS_MYROBOT`.
3. Build the shared library:

```cmake
add_library(argos3plugin_simulator_myrobot SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_MYROBOT})
target_link_libraries(argos3plugin_simulator_myrobot
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_dynamics3d
  argos3plugin_simulator_entities)
```

4. Install headers and the library so ARGoS can locate them:

```cmake
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_MYROBOT_CONTROLINTERFACE}
        DESTINATION include/argos3/plugins/robots/myrobot/control_interface)
install(TARGETS argos3plugin_simulator_myrobot
        LIBRARY DESTINATION lib/argos3)
```

Look at `turtlebot4/CMakeLists.txt` for an extensive, working example that also copies textures and Qt OpenGL helpers.

## 4. Organize Sources
- `control_interface/`: derived `CCI_*` classes that expose sensors/actuators to controllers.
- `simulator/`: robot entity, dynamics models, default sensors, and optional Qt visualization code.
- Keep the namespace, class names, and file naming consistent (`myrobot_*`). After copying the template, rename everything inside `control_interface/` and `simulator/` so the symbols match your robot’s name.
- See `docs/COPY_ROBOT_TEMPLATE.md` for a helper script that copies/renames these files.

### Expand on this later 
## 5. Add Controllers and Experiments
1. Clone the structure under `argos3/testing/controllers` to add a new controller.
2. Reference it in an experiment file in `argos3/testing/experiments`.
3. Update the relevant `CMakeLists.txt` files under `argos3/testing` to build your controller/

This ensures contributors can run `argos3 -c argos3/testing/experiments/myrobot.argos` immediately after building.

## 6. Build and Validate

```bash
./build.sh
./build.sh install   

argos3 -c argos3/testing/experiments/myrobot.argos
```

If ARGoS cannot find your plugin, verify that either:

- `sudo make install` copied the shared library into the main ARGoS prefix, or
- `ARGOS_PLUGIN_PATH` points to your build directory.

## Troubleshooting Tips
- Run `make VERBOSE=1` in the build directory to confirm that the new target is compiled.
- `ldd build/argos3/plugins/robots/myrobot/libargos3plugin_simulator_myrobot.so` helps find missing libraries.
- ARGoS will print the list of directories searched for plugins when launched with `--help`. Use this to verify your plugin path is included.
