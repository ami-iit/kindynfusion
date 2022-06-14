# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


# Warning: the CONFIGURE_PACKAGE_NAME option should be different from the plugin NAME
add_kindynfusion_yarp_device(
  NAME WholeBodyKinematicsDevice
  TYPE KinDynFusion::WholeBodyKinematicsDevice
  SOURCES src/WholeBodyKinematicsDevice.cpp
          src/WholeBodyKinematicsYarpHelper.cpp
  PUBLIC_HEADERS include/KinDynFusion/WholeBodyKinematicsDevice.h
                 include/KinDynFusion/WholeBodyKinematicsYarpHelper.h
  APP TRUE
  PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES}
                        ${iDynTree_LIBRARIES}
                        KinDynFusion::FloatingBaseEstimators
                        KinDynFusion::InverseKinematicsYarpHelper
                        BipedalLocomotion::ParametersHandlerYarpImplementation
                        BipedalLocomotion::SystemYarpImplementation
                        IWear::IWear
  CONFIGURE_PACKAGE_NAME whole_body_kinematics_device)

