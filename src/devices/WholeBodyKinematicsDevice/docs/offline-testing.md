- After compilation, upate `.bashrc` as follows,
```
export KinDynFusion_DIR=<where-you-installed-KinDynFusion>
export YARP_DATA_DIRS=${YARP_DATA_DIRS:+${YARP_DATA_DIRS}:}${KinDynFusion_DIR}/share/yarp:${KinDynFusion_DIR}/share/WholeBodyKinematicsDevice/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${KinDynFusion_DIR}/lib
```

- Launch `yarpmanager` and open `KinDynFusion/devices/WholeBodyKinematicsDevice/scripts/applications/WholeBodyKinematicsStartupOffline.xml`.
- Launch `yarpdataplayer` from the listed modules.
- Load a dataset streaming `/FTShoeLeft/WearableData/data:o`, `/FTShoeRight/WearableData/data:o`, and `/XSensSuit/WearableData/data:o`.
- Launch `yarprobotinterface --config WholeBodyKinematicsXSensSuit.xml` from the YARP manager.


