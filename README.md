# ARGoS3 Turtlebot4 and New Robot Templates
This repository contains ARGoS3 plugins for the Turtlebot4 together with a couple of lightweight robot examples that can be copied when creating new robots.

<p align="center">
  <img src="./turtlebot4.gif" width="500" alt="Turtlebot4 simulation snapshot">
</p>

## Prerequisites
- Install ARGoS3 from the official [repository](https://github.com/ilpincy/argos3) so that headers, libraries, and the `argos3` executable are present on your system.
- Optional (but recommended): skim through the [developer manual](https://www.argos-sim.info/dev_manual.php) to understand how ARGoS3 expects plugins to be structured.

## Build and Install
Use the helper script or run the commands manually:

```bash
# configures and builds in ./build (Debug by default)
./build.sh           

# same as above plus sudo make install
./build.sh install   
```

Manual steps:

```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)

# to install libraries/headers into your ARGoS3 prefix
sudo make install    
```

If you prefer running without `sudo make install`, set `ARGOS_PLUGIN_PATH` to `<repo>/build/argos3/plugins/robots/<plugin>` before launching `argos3` so it can find the shared libraries.
```bash
export EXAMPLEDIR=../argos3_plugins/new_robots
export ARGOS_PLUGIN_PATH=$EXAMPLEDIR/build/newepuck
```

## Repository Layout
- `argos3/plugins/robots/`: houses every robot plugin. See the local README that links to the full “Adding a New Robot” guide.
- `argos3/plugins/robots/turtlebot4`: full Turtlebot4 plugin 
- `argos3/plugins/robots/testbot`: minimal plugin focused on the robot entity and Qt-OpenGL mesh 
- `argos3/plugins/robots/newepuck`: an e-puck robot with additional sensors/ actuators.
- `argos3/testing`: controllers, loop functions, and ARGoS experiment files for exercising the plugins.
- `build.sh`: convenience script for rebuilding and optionally installing.

## Run the Sample Experiment
The provided Turtlebot4 obstacle-avoidance experiment can be executed once the plugin is built (and either installed or discoverable via `ARGOS_PLUGIN_PATH`):

```bash
argos3 -c argos3/testing/experiments/turtlebot4_test.argos
```

## Building Your Own Robot Plugin
See `docs/ADDING_NEW_ROBOT.md` for the detailed walkthrough that covers copying a template robot, wiring CMake, adding controllers/experiments, and validating the plugin inside ARGoS3.

## Possible Future Work
- Port these controllers to a physical Turtlebot4.
- Expand the documentation with additional robot templates as they become available.
 
