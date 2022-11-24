# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if (FRAMEWORK_COMPILE_InverseKinematics AND
    FRAMEWORK_COMPILE_YarpImplementation)

  add_kindynfusion_library(
    NAME                   InverseKinematicsYarpHelper
    SOURCES                src/InverseKinematicsYarpHelper.cpp
    PUBLIC_HEADERS         include/KinDynFusion/InverseKinematics/InverseKinematicsYarpHelper.h
    PUBLIC_LINK_LIBRARIES  KinDynFusion::InverseKinematics
                           YARP::YARP_dev
                           YARP::YARP_os
                           YARP::YARP_sig
                           iDynTree::idyntree-model
                           iDynTree::idyntree-core
                           iDynTree::idyntree-modelio
    INSTALLATION_FOLDER    InverseKinematics)
endif()
