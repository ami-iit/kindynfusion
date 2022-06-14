# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(KinDynFusionFindDependencies)

################################################################################
# Find all packages

find_package(iDynTree 1.1.0 REQUIRED) 
find_package(Eigen3 3.2.92 REQUIRED)
find_package(spdlog REQUIRED)
find_package(BipedalLocomotionFramework REQUIRED)
find_package(IWear REQUIRED)

find_package(YARP COMPONENTS telemetry)

find_package(OsqpEigen 0.4.0 QUIET)
checkandset_dependency(OsqpEigen)

find_package(HumanDynamicsEstimation QUIET)
checkandset_dependency(HumanDynamicsEstimation)

find_package(YARP QUIET)
checkandset_dependency(YARP)

find_package(manif QUIET)
checkandset_dependency(manif)

find_package(matioCpp QUIET)
checkandset_dependency(matioCpp)

find_package(Catch2 QUIET)
checkandset_dependency(Catch2)

framework_dependent_option(FRAMEWORK_COMPILE_tests
  "Compile tests?" ON
  "FRAMEWORK_USE_Catch2;BUILD_TESTING" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_YarpImplementation
  "Compile All the YARP implementations?" ON
  "FRAMEWORK_USE_YARP" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_FloatingBaseEstimators
  "Compile FloatingBaseEstimators libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Model
  "Compile Model libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_InverseKinematics
  "Compile InverseKinematics libraries?" ON
  "FRAMEWORK_USE_HumanDynamicsEstimation;FRAMEWORK_USE_OsqpEigen" OFF)
