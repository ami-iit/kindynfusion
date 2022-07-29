## Parameters Configuration



### Overview

- [WholeBodyKinematicsDevice](#WholeBodyKinematicsDevice)
- [WholeBodyKinematicsVisualizerDevice](#WholeBodyKinematicsVisualizerDevice)
- [WholeBodyKinematicsLogger](#WholeBodyKinematicsLogger)



### WholeBodyKinematicsDevice

#### General

The following parameters are general high-level device parameters.

| Group | Parameter             | Type    | Required | Description                                                  |
| ----- | --------------------- | ------- | -------- | ------------------------------------------------------------ |
|       | period                | double  | Yes      | Device period                                                |
|       | rpcPortPrefix         | string  | No       | Not required to start with '/'                               |
|       | constant_floor_height | double  | Yes      | Signed z-direction distance of the floor with respect to the frame which is used for calibration of the world frame, typically `LeftFoot` since we run `TPoseCalibrationAndInitBaseEKF` by passing `target_LeftFoot` as the parameter during the initial `TPose` calibration before run phase of the estimator |
|       | flat_floor            | boolean | Yes      | Flag used to enable flat floor measurement models in the Base EKF. Setting this to true will enable the terrain height updates and contact plane orientation corrections during contact events |

#### Model Data

The following set of parameters populate the struct `ModelData`.

| Group | Parameter | Type              | Required | Description                                                 | Default value                                                |
| ----- | --------- | ----------------- | -------- | ----------------------------------------------------------- | ------------------------------------------------------------ |
|       | urdf      | string            | Yes      | Model URDF                                                  | `model.urdf`                                                 |
|       | jointList | vector of strings | No       | List of joints from the model                               | If an empty list is passed, then model is loaded with all the joints in no assumed order. |
|       | baseFrame | string            | Yes      | Choice of floating base frame or fixed frame from the model | `Pelvis`                                                     |

#### IK Parameters

The following set of parameters populate the struct  `IKParameters`.

| Group | Parameter    | Type    | Required | Description                                             | Default Value |
| ----- | ------------ | ------- | -------- | ------------------------------------------------------- | ------------- |
|       | ikSolver     | string  | Yes      | Solver type: {**`dynamical`**}                          | `dynamical`   |
|       | useFixedBase | boolean | Yes      | Makes the base frame to be fixed to the inertial frame. | `false`       |

**Dynamical IK Parameters**

The following set of parameters populate the struct `DynamicalIKParameters`.

| Group | Parameter                             | Type           | Required | Description                                                  | Default Value    |
| ----- | ------------------------------------- | -------------- | -------- | ------------------------------------------------------------ | ---------------- |
|       | inverseVelocityKinematicsSolver       | string         | No       | solver types: {**`moorePenrose`**,  `QP`, `completeOrthogonalDecomposition`, `leastSquare`, `choleskyDecomposition`, `sparseCholeskyDecomposition`, `robustCholeskyDecomposition`, `sparseRobustCholeskyDecomposition`} | `moorePenrose` |
|       | useDirectBaseMeasurement              | boolean        | Yes      | Flag to enable the use of target measurements of the base link directly as base state | `false` |
|       | costRegularization                    | double         | Yes      |                                                              | `1.0` |
|       | linVelTargetWeight                    | double         | Yes      | Linear velocity target weight | `1.0` |
|       | angVelTargetWeight                    | double         | Yes      | Angular velocity target weight | `1.0` |
|       | dynamicalIKMeasuredVelocityGainLinRot | vector doubles | Yes      | Tuple of two doubles containing measured linear and angular velocity gains. | `(1.0, 1.0)`   |
|       | dynamicalIKCorrectionGainsLinRot      | vector doubles | Yes      | Tuple of two doubles containing linear and angular  velocity correction gains. | `(1.0, 1.0)`     |
|       | dynamicalIKJointVelocityLimit         | double         | No       | Joint velocity limits to be considered while solving IK     | **`1000`** rad/s |
|                    | k_u                                  | double                   | No       |                                                              |`0.0`|
|                    | k_l                                  | double                   | No       |                                                              |`0.0`|



#### Custom Constraints

The following parameter group allows to parse configuration parameters that will add custom constraints to the IK problem when using the QP solver, populating the `CustomIKConstraints` struct.

| Group              | Parameter                            | Type                     | Required | Description                                                  | Default Value               |
| ------------------ | ------------------------------------ | ------------------------ | -------- | ------------------------------------------------------------ | --------------------------- |
| CUSTOM_CONSTRAINTS |                                      |                          | No       | Set of constraint parameters used for formulating QP-based Inverse Velocity Kinematics, if  `ikSolver` is set to  `dynamical` and `inverseVelocityKinematicsSolver` is set to `QP` |                             |
|                    | custom_joints_velocity_limits_names  | vector of strings        | No       | List of selected joints for which custom joint velocity limits are set from the overall joint list |                             |
|                    | custom_joints_velocity_limits_values | vector of doubles        | No       | should match the size of `custom_joints_velocity_limits_names` |                             |
|                    | custom_constraint_variables          | vector of strings        | No       | List of selected joints for which custom joint position limits are set from the overall joint list |                             |
|                    | custom_constraint_matrix             | vector of vector doubles | No       | Constraint matrix when multiplied with the decision variables has inequalities with the lower and upper bounds for the limits |                             |
|                    | custom_constraint_upper_bound        | vector of doubles        | No       | Vector containing the upper bound limits for the joint positions |                             |
|                    | custom_constraint_lower_bound        | vector of doubles        | No       | Vector containing the lower bound limits for the joint positions |                             |
|                    | base_velocity_limit_upper_bound      | vector of doubles        | No       | Custom upper bound limits for the base velocity              | 6D vector of values `1000`  |
|                    | base_velocity_limit_lower_bound      | vector of doubles        | No       | Custom lower bound limits for the base velocity              | 6D vector of values `-1000` |



### Foot Meta Data for Contact Detection

The following parameters group populate the struct `FootMetaData` that is used within the `CoPVertexSchmittTrigger`.

| Group     | Subgroup                   | Parameter                 | Type              | Required | Description                                                  | Default Value |
| --------- | -------------------------- | ------------------------- | ----------------- | -------- | ------------------------------------------------------------ | ------------- |
| FOOT_DATA |                            |                           |                   | Yes      | Group of parameters related to center of pressure based contact detection describing the geometry and detection thresholds for the foot |               |
|           |                            | foot_link_names           | vector of strings | Yes      | List containing two values related to the name of the foot links from the URDF model, currently expecting only `LeftFoot` and `RightFoot` as values. |               |
|           | **LeftFoot**/**RightFoot** |                           |                   |          | Subgroup for `RightFoot` should contain the same parameters  |               |
|           |                            | wearable_name             | string            | Yes      | Name of the wearable FT Shoe associated to the foot, should be same as the wearable name from the IWear interface. |               |
|           |                            | sole_frame                | string            | Yes      | Name of the sole frame on the foot. This frame will be dynamically added to the URDF model during runtime for internal computations |               |
|           |                            | sole_position_in_foot     | vector of double  | Yes      | 3d position of the sole frame with respect to the foot link frame, in meters |               |
|           |                            | sole_rpy_degrees_in_foot  | vector of doubles | Yes      | Orientation of the sole frame with respect to the foot link frame in radians in roll pitch yaw format |               |
|           |                            | foot_length               | double            | Yes      | Length of the FT shoe (in the forward direction) in meters   |               |
|           |                            | foot_width                | double            | Yes      | Width of the FT shoe (in the lateral direction) in meters    |               |
|           |                            | fz_threshold_for_cop      | double            | Yes      | Normal force threshold (fz) from the contact wrench measured by the FT shoe below which CoP has exited the support polygon (in Newtons) |               |
|           |                            | schmitt_make_threshold    | double            | Yes      | Force threshold in Newtons above which the contact normal force at the vertex of the foot infers a contact of the vertex with the environment |               |
|           |                            | schmitt_break_threshold   | double            | Yes      | Force threshold in Newtons below which the contact normal force at the vertex of the foot infers a loss of contact of the vertex with the environment |               |
|           |                            | schmitt_make_switch_time  | double            | Yes      | Timing threshold in seconds above which if the force value is above the make threshold, then a contact is established |               |
|           |                            | schmitt_break_switch_time | double            | Yes      | Timing threshold in seconds above which if the force value is below the break threshold, then a loss of contact is established |               |
|           |                            | top_left_vertex_in_sole   | vector of doubles | Yes      | Position of the top left vertex in the rectangular geometry of the foot with respect to the sole frame in meters |               |

#### Wearable Target Parameters

The following parameters group is used to add the wearable targets for which the target measurements will be used to solve the inverse kinematics.

| Group                   | Parameter           | Type                    | Required | Description                                                  |
| ----------------------- | ------------------- | ----------------------- | -------- | ------------------------------------------------------------ |
| WEARABLE_SENSOR_TARGETS |                     |                         | Yes      | Metadata to convert measurements from WearableData to link based targets of desired types. |
|                         | `target_<LinkName>` | vector of three strings |          | Tuple of link name, wearable sensor name, kinematic target type. Link name should be a link from the URDF, wearable sensor name should be a wearable name from the IWear interface and the kinematic target type denotes the type of target measurement that will be added to the IK problem.<br />The available kinematic target types are {`none`, `pose`, `poseAndVelocity`, `position`, `positionAndVelocity`, `orientation`, `orientationAndVelocity`, `gravity`} depending on the type of target measurement. |

#### Calibration Parameters

The following parameters group is used to set calibration matrices in `CalibrationStorage` struct contained within the `WearableSensorTarget` object for each of the added target.

| Group                             | Parameter    | Type              | Required | Description                                                  |
| --------------------------------- | ------------ | ----------------- | -------- | ------------------------------------------------------------ |
| MEASUREMENT_TO_LINK_TRANSFORMS    |              |                   | No       | Extrinsic transformation required required for sensor-to-segment calibration, required to express sensors measurements from sensor frame in the parent link frame. |
|                                   | `<LinkName>` | vector of doubles |          | tuple of link name and transformation matrix as a vector of 16 elements in row major order |
| WORLD_TO_MEASUREMENT_TRANSFORMS   |              |                   | No       | Extrinsic transformation required to align measurements in a common world frame. |
|                                   | `<LinkName>` | vector of doubles |          | tuple of link name and transformation matrix as a vector of 16 elements in row major order |
| MEASUREMENT_POSITION_SCALE_FACTOR |              |                   | No       | Scaling factors for position targets                         |
|                                   | `<LinkName>` | vector of doubles |          | tuple of link name and position scaling values as a vector of 3 elements denoting x-, y-, and z-directions. |



#### Wearable Shoes Parameters

| Group                 | Parameter         | Type                                                  | Required | Description                                                  |
| --------------------- | ----------------- | ----------------------------------------------------- | -------- | ------------------------------------------------------------ |
| WEARABLE_SENSOR_SHOES |                   |                                                       | Yes      | Metadata relating the Wearable FT Shoe to the URDF Link on which the shoe is worn |
|                       | `<wearable_name>` | vector containing a string and a vector of 16 doubles |          | Tuple of link name from the URDF model associated to the FT shoe and a vector containing 16 doubles in row-major order listing the values of transformation matrix representing the pose of the link frame with respect to the shoe frame. |

#### Base EKF Parameters

It must be noted that the the Bipedal Locomotion Framework Floating Base Estimators were designed with the assumption that an IMU is always rigidly attached to the base link of the robot/human and all the EKF computations are carried out with respect to the IMU frame and then transformed to the base state at the end of every EKF step.

| Group         | Subgroup          | Parameter                                   | Type                | Required | Description                                                  | Default Value |
| ------------- | ----------------- | ------------------------------------------- | ------------------- | -------- | ------------------------------------------------------------ | ------------- |
| BaseEstimator |                   |                                             |                     | Yes      | Group of parameters related to EKF-based floating base estimator |               |
|               |                   | use_model_info                              | boolean             | Yes      | This should be set to `true` since we rely on model-related computations. |               |
|               |                   | sampling_period_in_s                        | double              | Yes      | Discrete time period of the estimator                        |               |
|               |                   | use_velocity_measurements                   | boolean             | No       | Enable the use of measurement models related to the base velocity computations in the EKF. | `false`       |
|               |                   | use_zero_base_velocity                      | boolean             | No       | Force zero base velocity measurements if velocity measurement models are used | `false`       |
|               | **ModelInfo**     |                                             |                     |          | Parameters related to the model computations                 |               |
|               |                   | base_link                                   | string              | Yes      | Name of the base link from the URDF model                    |               |
|               |                   | base_link_imu                               | string              | Yes      | Frame name of the IMU sensor rigidly attached to the base link from the URDF model |               |
|               |                   | left_foot_contact_frame                     | string              | Yes      | Frame name of the sole of the left foot dynamically added to the URDF model using the meta data from the `FOOT_DATA` parameter group defined above. |               |
|               |                   | right_foot_contact_frame                    | string              | Yes      | Frame name of the sole of the right foot dynamically added to the URDF model using the meta data from the `FOOT_DATA` parameter group defined above. |               |
|               | **Options**       |                                             |                     |          |                                                              |               |
|               |                   | enable_imu_bias_estimation                  | boolean             | Yes      | This should be set to `false` since we do not model IMU bias in the EKF state. |               |
|               |                   | enable_ekf_update                           | boolean             | Yes      | This should be set to `true`. Setting this to false, will simply integrate the EKF using the prediction model. |               |
|               |                   | use_kinematics_measure                      | boolean             | Yes      | This should be set to `true` since we rely on model-based kinematic measurement models in the EKF. |               |
|               |                   | use_static_ldmks_pose_measure               | boolean             | Yes      | This should be set to `false` since we do not use any exteroceptive landmark measurements in the EKF. |               |
|               | **SensorsStdDev** |                                             |                     |          | Parameter group containing all the noise parameters related to the prediction model and the measurement models. |               |
|               |                   | contact_foot_linear_velocity_noise_std_dev  | vector of 3 doubles | Yes      | Noise parameter used for constant foot vertex position prediction model when the vertex is in contact. This can be set to very small values to reflect the holonomic constraints imposed by a rigid contact. |               |
|               |                   | contact_foot_angular_velocity_noise_std_dev | vector of 3 doubles | Yes      | Noise parameter used for contact plane orientation measurement model when the foot is inferred to be in rigid contact with the environment. |               |
|               |                   | swing_foot_linear_velocity_noise_std_dev    | vector of 3 doubles | Yes      | Noise parameter used for constant foot vertex position prediction model when the vertex is not in contact. This can be set to very high values to reflect mistrust in the constant prediction model when the vertex is not in contact |               |
|               |                   | swing_foot_angular_velocity_noise_std_dev   | vector of 3 doubles | Yes      | Required but currently unused in the EKF.                    |               |
|               |                   | forward_kinematic_measurement_noise_std_dev | vector of 6 doubles | Yes      | Noise parameter used for the left trivialized base velocity measurement model accounting for the holonomic constraints of the vertex in contact (linear velocity) or foot in rigid contact (angular velocity). |               |
|               |                   | encoders_measurement_noise_std_dev          | vector of doubles   | Yes      | Noise parameters used for encoder measurements to compute the noise acting on the relative kinematics measurement model using the relative Jacobian. <br />The relative kinematics measurement models compute the orientation of the foot with respect to the base and another computes the position of foot vertices relative to base for measurement updates. The list can contain 1 value such as `(0.1)` to use the same value across all the joints or the list must contain values as the same number of joints in the loaded URDF model. |               |
|               |                   | map_height_std_dev                          | vector of 6 doubles | Yes      | Noise parameters associated to the terrain height measurements from the known elevation map. The 3rd value in the list related to the height should be set with very small values while the other values should have high covariance values since we are certain only about the terrain height measurement. |               |
|               |                   | imu_frame_linear_velocity_noise_std_dev     | vector of 3 doubles | Yes      | Noise parameters associated to the null acceleration model used as the prediction model for the base linear velocity. Since the null acceleration is a conservative prediction model, it is safe to use considerably high values. |               |
|               |                   | imu_frame_angular_velocity_noise_std_dev    | vector of 3 doubles | Yes      | Noise parameters associated to the null acceleration model used as the prediction model for the base angule velocity. Since the null acceleration is a conservative prediction model, it is safe to use considerably high values. |               |
|               |                   | imu_frame_gyro_noise_std_dev                | vector of 3 doubles | Yes      | Noise parameters associated to the base angular velocity measurement model corrected using the gyroscope measurement obtained from the base collocated IMU. This parameter should be set depending on the quality of IMU measurements, typically very low values can be used for Xsens measurements. Trusting the base gyroscope usually leads to smoother base state estimates depending on the quality of measurements. |               |
|               | **PriorsStdDev**  |                                             |                     |          | Parameter group describing the standard deviations associated to the initial states set to the EKF. |               |
|               |                   | imu_orientation                             | vector of 3 doubles | Yes      | Standard deviation associated to the initial IMU orientation with respect to the inertial frame (in exponential coordinates). |               |
|               |                   | imu_position                                | vector of 3 doubles | Yes      | Standard deviation associated to the initial IMU position with respect to the inertial frame. |               |
|               |                   | imu_linear_velocity                         | vector of 3 doubles | Yes      | Standard deviation associated to the initial IMU linear velocity with respect to the inertial frame. |               |
|               |                   | imu_angular_velocity                        | vector of 3 doubles | Yes      | Standard deviation associated to the initial IMU angular velocity with respect to the inertial frame. |               |
|               |                   | r_contact_frame_orientation                 | vector of 3 doubles | Yes      | Standard deviation associated to the initial contact frame orientation of the right foot with respect to the inertial frame. |               |
|               |                   | l_contact_frame_orientation                 | vector of 3 doubles | Yes      | Standard deviation associated to the initial contact frame orientation of the left foot with respect to the inertial frame. |               |
|               | **InitialStates** |                                             |                     |          | Parameter group describing the initial states set to the EKF. |               |
|               |                   | imu_orientation_quaternion_wxyz             | vector of 4 doubles | Yes      | Initial orientation of the base link with respect to the inertial frame in quaternion representation |               |
|               |                   | imu_position_xyz                            | vector of 3 doubles | Yes      | Initial position of the base link with respect to the inertial frame in meters |               |
|               |                   | imu_linear_velocity_xyz                     | vector of 3 doubles | Yes      | Initial mixed-trivialized linear velocity of the base link with respect to the inertial frame in meters/second |               |
|               |                   | imu_angular_velocity_xyz                    | vector of 3 doubles | Yes      | Initial local angular velocity of the base link with respect to the inertial frame in radians/second |               |
|               |                   | l_contact_frame_orientation_quaternion_wxyz | vector of 4 doubles | Yes      | Initial contact frame orientation of the left foot with respect to the inertial frame in quaternion representation |               |
|               |                   | r_contact_frame_orientation_quaternion_wxyz | vector of 4 doubles | Yes      | Initial contact frame orientation of the right foot with respect to the inertial frame in quaternion representation |               |



### WholeBodyKinematicsVisualizerDevice

| Group | Parameter                         | Type              | Required | Description                                                  | Default value                                                |
| ----- | --------------------------------- | ----------------- | -------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
|       | period                            | double            | Yes      |                                                              |                                                              |
|       | wholeBodyKinematicsDataPortName   | string            | Yes      | Name of the port over which the whole body kinematics remapper data is published. The port name should begin with / |                                                              |
|       | urdf                              | string            | Yes      | Model URDF                                                   | `model.urdf`                                                 |
|       | jointList                         | vector of strings | Yes      | List of joints from the model                                | If an empty list is passed, then model is loaded with all the joints in no assumed order. |
|       | baseFrame                         | string            | Yes      | Choice of floating base frame or fixed frame from the model  | `Pelvis`                                                     |
|       | visualize_cop_and_contact_normals | boolean           | Yes      | Enable the visualization of the evolution of center of pressure and contact normal forces acting on vertices of the foot |                                                              |

### WholeBodyKinematicsLogger

| Group | Parameter                         | Type    | Required | Description                                                  | Default value |
| ----- | --------------------------------- | ------- | -------- | ------------------------------------------------------------ | ------------- |
|       | period                            | double  | Yes      | Running thread period of the logger device                   |               |
|       | wholeBodyKinematicsDataPortName   | string  | Yes      | Name of the port over which the whole body kinematics client data is published. The port name should begin with / |               |
|       | telemetry_autosave_period         | double  | Yes      | Time in seconds after which the telemetry data is flushed onto a file and a new save file is created. |               |
|       | logIHumanState                    | boolean | Yes      | Flag to enable logging an external device with `IHumanState` interface, for example [`HumanStateProvider`](https://github.com/robotology/human-dynamics-estimation/tree/master/devices/HumanStateProvider). |               |
|       | humanStateProviderDataPortName    | string  |          | Name of the port over which the HumanStateProvider client data is published. The port name should begin with / |               |
|       | logGroundTruth                    | boolean | Yes      | Flag to enable logging of ground truth data published in the form of transforms through the YARP transform server |               |
|       | groundTruthFrameTransformPortName | string  |          | Name of the port over which the transform server is publishing the transforms data |               |
|       | groundTruthReferenceFrame         | string  |          | Name of the reference frame with respect to which the base tracker pose is published |               |
|       | grountTruthBaseTrackerName        | string  |          | Name of the base tracker frame for which the ground truth pose needs to be logged |               |



Go back to the [main README](./../README.md).