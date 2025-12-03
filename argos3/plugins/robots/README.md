# Robot Plugins
Each directory here builds a shared library that ARGoS loads at runtime. To add your own robot:

1. Copy one of the existing directories (for example `testbot`, which focuses on the entity/Qt mesh with sensors commented out) and rename it.
2. Register the directory inside `argos3/plugins/robots/CMakeLists.txt` with `add_subdirectory(<your_robot>)`.
3. Update the new plugin's `CMakeLists.txt` to list your headers/sources, create `argos3plugin_simulator_<your_robot>`, and install the files.
4. Add controllers/experiments under `argos3/testing` so the robot can be exercised quickly.

The full walkthrough with code snippets lives in `docs/ADDING_NEW_ROBOT.md` at the repository root.
