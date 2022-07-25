# KinDynFusion

The KinDynFusion project consists of libraries and YARP devices ueful for running whole body kinematics estimation for humans wearing sensorized suit and shoes equipped with distributed inertial and force torque sensing.



## Installation
### Dependencies

Please check the [dependencies](./how-to/dependencies.md) page for the full list of dependencies and latest tested commits for each of the dependencies.

### Prerequisites

Please follow the instructions in the [installation prerequisites](./how-to/install-prerquisites.md) for setting up the environment to launch the `KinDynFusion` software.

### Install KinDynFusion

Clone and build the repository using the following commands.

``` bash
git clone https://github.com/ami-iit/kindynfusion
cd kindynfusion/src/
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DALLOW_IDL_GENERATION=ON \
      -DCMAKE_INSTALL_PREFIX=<where-you-want-to-install> ..
make && make install
```

Please replace `<where-you-want-to-install>` with the desired installation location.

Add the following lines to the `.bashrc` file:

``` bash
export KinDynFusion_DIR=<where-you-installed-kindynfusion>
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${KinDynFusion_DIR}/share/yarp
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${KinDynFusion_DIR}/lib
```



In order to check, if everything was compiled and installed successfully, we can run the following command:

``` bash
yarpdev --list | grep WholeBodyKinematics
```

which should print

``` bash
[INFO] |yarp.dev.Drivers| Device "WholeBodyKinematicsDevice", available on request (found in <where-you-installed-kindynfusion>/lib/yarp/WholeBodyKinematicsDevice.so library).
[INFO] |yarp.dev.Drivers| Device "WholeBodyKinematicsLogger", available on request (found in <where-you-installed-kindynfusion>/lib/yarp/WholeBodyKinematicsLogger.so library).
[INFO] |yarp.dev.Drivers| Device "WholeBodyKinematicsRemapper", available on request (found in <where-you-installed-kindynfusion>/lib/yarp/WholeBodyKinematicsRemapper.so library).
[INFO] |yarp.dev.Drivers| Device "WholeBodyKinematicsVisualizerDevice", available on request (found in <where-you-installed-kindynfusion>/lib/yarp/WholeBodyKinematicsVisualizerDevice.so library).
```
