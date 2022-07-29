### Launching Whole Body Kinematics Estimation

- [Using  logged data streaming](#Using  logged data streaming)
- [Running online with a sensor suit device](#Running online with a sensor suit device)

#### Using  logged data streaming

(P.S. if using conda environment, activate the environment in every terminal that is opened)

- Launch `yarpserver`

- Launch `yarpdataplayer` to stream logged data. The logged data is expected to be of [`WearableData`](https://github.com/robotology/wearables/blob/master/msgs/thrift/WearableData.thrift) format streamed through a port. Optionally, the ground truth base pose information from, lets say, Vive trackers mounted on the Pelvis link streaming pose information using [`yarp-openvr-trackers`](https://github.com/ami-iit/yarp-openvr-trackers) over YARP's [`transformServer`](https://www.yarp.it/latest/classTransformServer.html) can also be available depending on the experiment conducted for data acquisition.

  ```bash
  yarpdataplayer --withExtraTimeCol 2
  ```

  Note: Some datasets  are available [here](https://istitutoitalianotecnologia.sharepoint.com/sites/ergoCub/Documenti%20condivisi/Forms/AllItems.aspx?csf=1&web=1&e=s9Jyy0&cid=d4372edc%2D353c%2D4957%2D8f52%2D2b8464285bd3&RootFolder=%2Fsites%2FergoCub%2FDocumenti%20condivisi%2FGeneral%2Fexperimental%2Ddatasets%2F2022%5F04%5F22%5FiFeelSuitWholeBodyWithViveTrackers&FolderCTID=0x01200064D341C6B34EB5429E707508F3A075A7) that contains wearable data from full body iFeel suit along with FT shoes and vive trackers. Download and load the dataset `walk2`  in the `yarpdataplayer`. Do not play the data yet. It must be noted that, for all the experiments, the initial phase of the dataset, the human subject is in TPose configuration. Such an experimental setting is enforced such that we run an asynchronous RPC call for performing the calibration  which is necessary before running the estimation.

- Launch the transform server,

  ``` bash
  yarpdev --device transformServer --ROS::enable_ros_publisher 0 --ROS::enable_ros_subscriber 0
  ```

- Launch the `WholeBodyKinematicsDevice`,

  ``` bash
  yarprobotinterface --config WholeBodyKinematics_iFeelSuit_fullBody_1_Subject08_FtShoes41.xml
  ```

  If the device is successfully, launch it enters a loop waiting for the wearable data to be available on the port. This is because we have not started playing the `yarpdataplayer` yet.  If we run play and pause the data player instantly, the availability of wearable data is conveyed to the whole body kinematics device. It is required that the data player is paused instantly so that we remain in the section of data that still reflects the TPose configuration of the human. At this point, we have to run a calibration for the IK and initialization step for the EKF. Before that, we will launch the `WholeBodyKinematicsVisualizerDevice`. The parameter descriptions for the values in the configuration files  passed to the `yarprobotinterface` can be checked in [Parameters Configuration](./parameters-config.md) file.

- Launch the visualizer,
  ``` bash
  yarprobotinterface --config WholeBodyKinematicsVisualizer.xml
  ```

- We can run the calibration step using the [TPoseCalibrationAndInitBaseEKF](./../src/scripts/TPoseCalibrationAndInitBaseEKF.sh) script from the folder in which the script is present `kindynfusion/src/scripts`,
  ``` bash
  bash TPoseCalibrationAndInitBaseEKF.sh
  ```

  It can be observed in the visualizer that the human model has been calibrated and reset to a new position with respect to the world frame. At this point, we can play the data player to stream the data continuously to visualize the estimated kinematics in the visualizer.

- Additionally, if we want to log the data, we can launch the logger that records the estimated outputs and saves them in a mat file as soon as the logger is closed. The logger can be launched using,
  ```
  yarprobotinterface --config WholeBodyKinematicsLogger.xml
  ```

  The logged data can be analyzed in matlab using the plot scripts available in `kindynfusion/src/scripts/matlab`.



**P.S. A `yarpmanager` application has been created in order to launch these steps from a GUI. This can be found in [WholeBodyKinematicsStartupOffline.xml](./../src/devices/WholeBodyKinematicsDevice/scripts/applications/WholeBodyKinematicsStartupOffline.xml).**



### Running online with a sensor suit device

Launching the kinematics estimation with online with data being streamed over the network using a `iFeelSuitDevice` or an [`XSensSuitDevice`](https://github.com/robotology/wearables/tree/master/devices/XsensSuit) follows the same procedure, except the `yarpdataplayer` step is replaced by launching the respective suit. 



Go back to the [main README](./../README.md).





