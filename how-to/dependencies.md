### Dependencies

The table below lists the required [dependencies](./../src/cmake/KinDynFusionDependencies.cmake) for running the `KinDynFusion` software.

| Depends                                                      | Minimum Required Version | Most Recent Tested Commit (Tested on 25 July 2022)           |
| ------------------------------------------------------------ | ------------------------ | ------------------------------------------------------------ |
| [Eigen](https://eigen.tuxfamily.org/dox/)                    | **`3.2.92`**             | -                                                            |
| [iDynTree](https://github.com/robotology/idyntree)           | **`3.0.0`**              | [`814c7633c27c1286b9140a00aceb6111422687bc`](https://github.com/robotology/idyntree/commit/814c7633c27c1286b9140a00aceb6111422687bc) |
| [human-gazebo](https://github.com/robotology/human-gazebo)   | **`1.0.1`**              | [`b2328bcf40b13933c74bf76fbe3da8a1171773e3`](https://github.com/robotology/human-gazebo/commit/b2328bcf40b13933c74bf76fbe3da8a1171773e3) |
| [BipedalLocomotionFramework](https://github.com/ami-iit/bipedal-locomotion-framework) | **`0.7.0`**              | [`bb20f7e4a267a0eb5c377f43a3e7c278ac48ce4b`](https://github.com/ami-iit/bipedal-locomotion-framework/commit/bb20f7e4a267a0eb5c377f43a3e7c278ac48ce4b) |
| [manif](https://github.com/artivis/manif)                    | **`0.0.4`**`.1`          | [`496717aa792c1ff4b99049034e53f9fa0ac676a3`](https://github.com/robotology-dependencies/manif/commit/496717aa792c1ff4b99049034e53f9fa0ac676a3) |
| [HumanDynamicsEstimation](https://github.com/robotology/human-dynamics-estimation) | **`2.5.0`**              | [`1907ef1524819da38b60b62e8825dde4216a50bf`](https://github.com/robotology/human-dynamics-estimation/commit/1907ef1524819da38b60b62e8825dde4216a50bf) |
| [OsqpEigen](https://github.com/robotology/osqp-eigen)        | **`0.7.0`**              | [`c7f0e30cb2b60a68225c944376f1f8276e3f6999`](https://github.com/robotology/osqp-eigen/commit/c7f0e30cb2b60a68225c944376f1f8276e3f6999) |
| [YARP](https://github.com/robotology/yarp)                   | **`3.7.0`**              | [`c8450ac9eb3e72f9b9c466ed838f5aead360f160`](https://github.com/robotology/yarp/commit/c8450ac9eb3e72f9b9c466ed838f5aead360f160) |
| [IWear](https://github.com/robotology/wearables)             | **`1.4.0`**              | [`57460c273e9012f07af9ec955758f243a7092c4f`](https://github.com/robotology/wearables/commit/57460c273e9012f07af9ec955758f243a7092c4f) |
| [robometry](https://github.com/robotology/robometry)         | **`1.0.0`**              | [`af906a4135aba418ba120ac68020d2beb474c965`](https://github.com/robotology/robometry/commit/af906a4135aba418ba120ac68020d2beb474c965) |
| [matioCpp](https://github.com/ami-iit/matio-cpp)             | **`0.2.0`**              | [2ae69473953112a670cff623e671540466a4a0c5](https://github.com/ami-iit/matio-cpp/commit/2ae69473953112a670cff623e671540466a4a0c5) |
| [spdlog]()                                                   | **`1.5.0`**              | -                                                            |
| [Catch2]()                                                   | -                        | -                                                            |



It must be noted that for a manual installation these dependencies can be cloned and installed by checking out the most recently tested commit listed below. Optionally, one can pass the absolute path of the [robotology.yaml](./robotology.yaml) containing all the relevant project tags as a value to the CMAKE option **`ROBOTOLOGY_PROJECT_TAGS_CUSTOM_FILE`** tag while installing the `robotology-superbuild` as a prerequisite for installing the `KinDynFusion` software.



Go back to the [main README](./../README.md).
