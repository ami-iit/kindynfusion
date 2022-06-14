usage() {
cat << EOF
************************************************************************************************************************************
Script for launching the WholeBodyKinematics calibration assuming the subject is in t-pose.
The scripts will launch both the world yaw calibration and the secondary calibration and init base EKF
OPTION: <PortPrefix for the human-state-provider>
EXAMPLE USAGE: ./TPoseCalibrationAndInitBaseEKF.sh
************************************************************************************************************************************
EOF
}

################################################################################
# "MAIN" FUNCTION:                                                             #
################################################################################
usage

#if [ $# -lt 1 ]; then
    HUMANSTATEPROVIDER_PREFIX=""
#else
#    HUMANSTATEPROVIDER_PREFIX="/$1"
#fi

echo "calibrateAllWorldYaw: resetting sensors yaw offset"
echo "calibrateAllWorldYaw" | yarp rpc ${HUMANSTATEPROVIDER_PREFIX}/WholeBodyKinematicsDevice/rpc:i

sleep 1

echo "calibrateAllWithWorld: resetting sensors to link transform assuming inertia on LeftFoot"
echo "calibrateAllWithWorld target_LeftFoot" | yarp rpc ${HUMANSTATEPROVIDER_PREFIX}/WholeBodyKinematicsDevice/rpc:i

sleep 1

echo "startBaseEstimator: start the Base EKF"
echo "startBaseEstimator" | yarp rpc ${HUMANSTATEPROVIDER_PREFIX}/WholeBodyKinematicsDevice/rpc:i

sleep 0.01

echo "calibrateBaseEstimatorWithWorld: reset inertial frame of Base EKF with respect to LeftFoot"
echo "calibrateBaseEstimatorWithWorld LeftFoot" | yarp rpc ${HUMANSTATEPROVIDER_PREFIX}/WholeBodyKinematicsDevice/rpc:i

