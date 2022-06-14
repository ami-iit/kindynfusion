/**
 * @file IKHelper.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_INVERSE_KINEMATICS_IK_HELPER_H
#define KINDYNFUSION_INVERSE_KINEMATICS_IK_HELPER_H

#include <KinDynFusion/InverseKinematics/WearableTargets.h>
#include <KinDynFusion/InverseKinematics/IKDataTypes.h>

#include <hde/algorithms/DynamicalInverseKinematics.hpp>
#include <mutex>

namespace KinDynFusion
{
namespace IK
{

class IKHelper
{
public:
    bool initialize(const ModelData& modelData,
                    const IKParameters& ikParams);
    bool addTarget(const TargetName& targetName,
                   const IKParameters& ikParams,
                   std::weak_ptr<WearableSensorTarget> target);
    bool updateTarget(const TargetName& targetName,
                      std::weak_ptr<WearableSensorTarget> target);
    bool solve(const double& dt);

    const iDynTree::Transform& getBaseTransformSolution() const;
    const iDynTree::Twist& getBaseVelocitySolution() const;
    const iDynTree::VectorDynSize& getJointConfigurationSolution() const;
    const iDynTree::VectorDynSize& getJointVelocitiesSolution() const;
    const double& getTargetsMeanRotationErrorNorm() const;
    const double& getTargetsMeanPositionErrorNorm() const;

private:
    bool initializeDynamicalIK(const ModelData& modelData,
                               const IKParameters& ikParams,
                               hde::algorithms::DynamicalInverseKinematics& dynIK);

    bool checkAddTargetStatus(const bool& ok,
                              const TargetName& targetName,
                              const ModelLinkName& linkName,
                              const std::string& targetType,
                              const std::string& printPrefix);

    mutable std::mutex m_mutex;
    hde::algorithms::DynamicalInverseKinematics m_dynamicalInverseKinematics;

    iDynTree::Transform m_baseTransformSolution;
    iDynTree::VectorDynSize m_jointConfigurationSolution;
    iDynTree::Twist m_baseVelocitySolution;
    iDynTree::VectorDynSize m_jointVelocitiesSolution;
    double m_targetsRotationErrorNorm;
    double m_targetsPositionErrorNorm;
    bool m_useFixedBase{false};
};

} // namespace IK
} // namespace KinDynFusion

#endif // KINDYNFUSION_INVERSE_KINEMATICS_IK_HELPER_H

