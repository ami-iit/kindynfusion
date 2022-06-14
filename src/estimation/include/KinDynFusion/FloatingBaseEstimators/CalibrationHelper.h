/**
 * @file CalibrationHelper.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_ESTIMATION_CALIBRATION_HELPER_H
#define KINDYNFUSION_ESTIMATION_CALIBRATION_HELPER_H

#include <KinDynFusion/InverseKinematics/WearableTargets.h>
#include <KinDynFusion/InverseKinematics/IKDataTypes.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <mutex>

namespace KinDynFusion
{
namespace Estimators
{

class CalibrationHelper
{
public:
    bool initialize(const KinDynFusion::IK::ModelData& modelData);

    void setConfiguration(const iDynTree::Transform& baseTransformSolution,
                          const iDynTree::VectorDynSize& jointConfigurationSolution);

    // set configuration before calling calibration methods
    bool calibrateAll(WearableSensorTargetMap& targetMap);
    bool calibrateAllWithWorld(const std::string& refTargetName,
                               WearableSensorTargetMap& targetMap);
    bool calibrateAllWorldYaw(WearableSensorTargetMap& targetMap);
    bool setRotationOffset(const std::string& targetName,
                           const double& rollInDegrees,
                           const double& pitchInDegrees,
                           const double& yawInDegrees,
                           WearableSensorTargetMap& targetMap);
    void resetAll(WearableSensorTargetMap& targetMap);
    void resetCalibration(const std::string& targetName,
                          WearableSensorTargetMap& targetMap);


private:
    void computeSecondaryCalibrationRotationsForChain(const std::vector<iDynTree::JointIndex>& jointZeroIndices,
                                                      const iDynTree::Transform &refLinkForCalibrationTransform,
                                                      const std::vector<iDynTree::LinkIndex>& linkToCalibrateIndices,
                                                      const std::string& refTargetForCalibrationName,
                                                      WearableSensorTargetMap& targetMap);
    void calibAllHelper(const std::string& refTargetName,
                        WearableSensorTargetMap& targetMap);
    void fillCalibrationIndices();
    void setZeroJointsAndRobotState(const std::vector<iDynTree::JointIndex>& jointZeroIndices);

    iDynTree::KinDynComputations m_kinDyn;
    iDynTree::VectorDynSize m_jointPos;
    iDynTree::Transform m_basePose;
    iDynTree::VectorDynSize m_jointVel;
    iDynTree::Twist m_baseVel;
    iDynTree::Vector3 m_worldGravity;
    iDynTree::Position m_zeroPosition;

    std::vector<iDynTree::JointIndex> m_jointZeroIndices;
    std::vector<iDynTree::LinkIndex> m_linkToCalibrateIndices;
    mutable std::mutex m_mutex;
};

} // namespace Estimation
} // namespace KinDynFusion

#endif // KINDYNFUSION_ESTIMATION_CALIBRATION_HELPER_H


