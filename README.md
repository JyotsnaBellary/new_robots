# ARGoS3 Turtlebot-4
 - This repository contains code that allows you to simulate the Turtlebot4 in ARGoS3. 

![Turtlebot4](./turtlebot4.gif)
## Compilation Instruction 
 - To be able to simulate the turtlebot4 plugin on ARGoS3, make sure you have ARGoS3 installed from [ARGoS3](https://github.com/ilpincy/argos3)

 - For further understanding of ARGoS3, go through [ARGoS3 - Developer Manual](https://www.argos-sim.info/dev_manual.php)

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
sudo make install 
```

## Run Experiments
 - An example test setup is provided here. The robot simply avoids obstacles using IR Sensors and logs if its on the white or grey part of the floor.

```bash
argos3 -c argos3/testing/experiments/turtlebot4_test.argos
```

## Possible future implementations
 - Develop ARGoS3 controllers on the real Turtlebot4.
 