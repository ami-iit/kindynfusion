# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(KinDynFusionFindDependencies)

################################################################################
# Find all packages

find_package(iDynTree 3.0.0 REQUIRED)
find_package(Eigen3 3.2.92 REQUIRED)
find_package(spdlog 1.5.0 REQUIRED)
find_package(BipedalLocomotionFramework 0.7.0 REQUIRED)
find_package(IWear 1.4.0 REQUIRED)

find_package(YARP COMPONENTS telemetry)

find_package(OsqpEigen 0.7.0 QUIET)
checkandset_dependency(OsqpEigen)

# make sure commit https://github.com/robotology/human-dynamics-estimation/commit/1907ef1524819da38b60b62e8825dde4216a50bf
# is checked out in the human-dynamics-estimation project
find_package(HumanDynamicsEstimation QUIET)
checkandset_dependency(HumanDynamicsEstimation)

find_package(YARP 3.5.0 QUIET)
checkandset_dependency(YARP)

find_package(manif 0.0.4 QUIET)
checkandset_dependency(manif)

# make sure commit https://github.com/ami-iit/matio-cpp/commit/2ae69473953112a670cff623e671540466a4a0c5
# is checked out in the matio-cpp project
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
