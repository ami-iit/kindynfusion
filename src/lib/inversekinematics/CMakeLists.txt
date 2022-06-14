# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if (FRAMEWORK_COMPILE_InverseKinematics)

  set(H_PREFIX include/KinDynFusion/InverseKinematics)

  add_kindynfusion_library(
    NAME                   InverseKinematics
    PUBLIC_HEADERS         ${H_PREFIX}/IKDataTypes.h
                           ${H_PREFIX}/WearableTargets.h
                           ${H_PREFIX}/IWearableTargets.h
                           ${H_PREFIX}/WearableSensorizedShoes.h
                           ${H_PREFIX}/IWearableSensorizedShoes.h
                           ${H_PREFIX}/iWearToiDyn.h
                           ${H_PREFIX}/IKHelper.h
    SOURCES                src/WearableTargets.cpp
                           src/WearableSensorizedShoes.cpp
                           src/iWearToiDyn.cpp
                           src/IKHelper.cpp
    SUBDIRECTORIES         YarpImplementation
    PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-model
                           iDynTree::idyntree-high-level
                           iDynTree::idyntree-core
                           Eigen3::Eigen)

endif()

