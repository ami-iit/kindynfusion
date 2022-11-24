# List of CPP (source) library files.
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_Model)

  set(H_PREFIX include/KinDynFusion/Model)

  # set target name
  add_kindynfusion_library(
    NAME                   Model
    PUBLIC_HEADERS         ${H_PREFIX}/RectangularFoot.h
    SOURCES                src/RectangularFoot.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen MANIF::manif iDynTree::idyntree-high-level iDynTree::idyntree-modelio iDynTree::idyntree-model iDynTree::idyntree-solid-shapes BipedalLocomotion::TextLogging BipedalLocomotion::ParametersHandler BipedalLocomotion::Math
    )

endif()

