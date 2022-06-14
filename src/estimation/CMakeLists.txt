# List of CPP (source) library files.
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_FloatingBaseEstimators AND FRAMEWORK_COMPILE_InverseKinematics)

  set(H_PREFIX include/KinDynFusion/FloatingBaseEstimators)

  # set target name
  add_kindynfusion_library(
    NAME                   FloatingBaseEstimators
    PUBLIC_HEADERS         ${H_PREFIX}/DiligentOdom.h
                           ${H_PREFIX}/CalibrationHelper.h
                           ${H_PREFIX}/CoPVertexSchmittTrigger.h
                           ${H_PREFIX}/IWholeBodyKinematics.h
                           ${H_PREFIX}/WholeBodyKinematics.h
    SOURCES                src/DiligentOdom.cpp
                           src/CalibrationHelper.cpp
                           src/CoPVertexSchmittTrigger.cpp
                           src/WholeBodyKinematics.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen
                           MANIF::manif
                           iDynTree::idyntree-model
                           iDynTree::idyntree-high-level
                           iDynTree::idyntree-core
                           BipedalLocomotion::FloatingBaseEstimators
                           BipedalLocomotion::ContactDetectors
                           BipedalLocomotion::ManifConversions
                           spdlog::spdlog
                           KinDynFusion::Model
                           KinDynFusion::InverseKinematics
                           HumanDynamicsEstimation::algorithms
    )
endif()

