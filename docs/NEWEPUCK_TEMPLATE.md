# Cloning the e-puck Plugin into `newepuck`
Use this walkthrough if you want a concrete example of porting an existing ARGoS robot (the stock e-puck) into this repository under a new name. The steps are adapted from the [ARGoS forum thread on cloning robots](https://www.argos-sim.info/forum/viewtopic.php?t=108#), with paths updated for this workspace.

> Adjust the paths if your ARGoS source tree or this repository live elsewhere.

## 1. Clone the Original Plugin
```bash
export EXAMPLEDIR=/home/gio-lab/argos3_plugins/new_robots
export ARGOSDIR=/home/gio-lab/argos3

cp -a $ARGOSDIR/src/plugins/robots/e-puck $EXAMPLEDIR/newepuck
rm -f $EXAMPLEDIR/newepuck/simulator/*physx*
```

## 2. Register the Plugin with CMake
Append the directory inside the main `CMakeLists.txt` so it is compiled:
```bash
echo 'add_subdirectory(newepuck)' >> $EXAMPLEDIR/CMakeLists.txt
```

## 3. Rename Files and Symbols
Use `sed`/`mv` to rename classes, namespaces, and files so they reflect the new robot name:
```bash
find $EXAMPLEDIR/newepuck \( -name '*.h' -o -name '*.cpp' \) -print0 \
  | xargs -0 -I{} sed -i.old \
      -e 's/EPuck/NewEPuck/g' \
      -e 's/e-puck/new_e-puck/g' \
      -e 's/epuck/newepuck/g' \
      -e 's/EPUCK/NEWEPUCK/g' \
      -e 's|argos3/plugins/robots/new_e-puck|newepuck|g' {}
find $EXAMPLEDIR/newepuck -name '*.old' -exec rm {} \;

for F in $(find $EXAMPLEDIR/newepuck \( -name '*.h' -o -name '*.cpp' \)); do
  G=$(basename "$F" | sed 's/epuck/newepuck/g')
  mv "$F" "$(dirname "$F")/$G"
done
```

## 4. Create the Plugin `CMakeLists.txt`
Replace the plugin’s `CMakeLists.txt` with:
```bash
cat <<'EOF' > $EXAMPLEDIR/newepuck/CMakeLists.txt
add_library(argos3plugin_simulator_newepuck SHARED
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
endif()

# TODO: add additional sensor/actuator sources to add_library above

install(TARGETS argos3plugin_simulator_newepuck
  LIBRARY DESTINATION lib/argos3)
EOF
```
> If your robot exposes more sensors/actuators, extend the `add_library(...)` list accordingly.

## 5. Build the Plugin
```bash
cd $EXAMPLEDIR
rm -rf build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

## 6. Point ARGoS to the New Plugin
If you do not run `sudo make install`, set the plugin path before invoking ARGoS:
```bash
export ARGOS_PLUGIN_PATH=$EXAMPLEDIR/build/newepuck
argos3 -q entities   # verify new_e-puck appears
```

To test with the stock experiment:
1. Copy `argos3-examples/experiments/epuck_obstacleavoidance.argos`.
2. Replace the entity block with:
   ```xml
   <new_e-puck id="fb">
     <controller config="fdc" />
   </new_e-puck>
   ```
3. Run `argos3 -c epuck_obstacleavoidance.argos`.

## 7. Install into the ARGoS Prefix (optional but recommended)
Add `install(FILES ...)` directives for your headers in `newepuck/CMakeLists.txt`, then run:
```bash
cd $EXAMPLEDIR/build
sudo make install
```
Installing avoids exporting `ARGOS_PLUGIN_PATH` each time because the shared library is copied into `${CMAKE_INSTALL_PREFIX}/lib/argos3`.

## 8. Validate Sensors
Create a small experiment/controller that uses the new sensors, log the readings, and confirm they behave as expected. This repo already contains `argos3/testing` controllers/experiments to use as starting points.
