# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


# Warning: the CONFIGURE_PACKAGE_NAME option should be different from the plugin NAME
add_kindynfusion_yarp_device(
  NAME WholeBodyKinematicsVisualizerDevice
  TYPE KinDynFusion::WholeBodyKinematicsVisualizerDevice
  SOURCES src/WholeBodyKinematicsVisualizerDevice.cpp
          src/WholeBodyKinematicsVisualizer.cpp
  PUBLIC_HEADERS include/KinDynFusion/WholeBodyKinematicsVisualizerDevice.h
                 include/KinDynFusion/WholeBodyKinematicsVisualizer.h
  APP TRUE
  PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES}
                        ${iDynTree_LIBRARIES}
                        KinDynFusion::FloatingBaseEstimators
                        KinDynFusion::InverseKinematicsYarpHelper
  CONFIGURE_PACKAGE_NAME whole_body_kinematics_visualizer_device)

