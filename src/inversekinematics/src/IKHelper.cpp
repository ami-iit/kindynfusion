/**
 * @file IKHelper.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/InverseKinematics/IKHelper.h>

using namespace KinDynFusion::IK;

bool IKHelper::initialize(const ModelData& modelData,
                          const IKParameters& ikParams)
{
    const std::string printPrefix{"IKHelper::initialize: "};

    if (!initializeDynamicalIK(modelData,
                               ikParams,
                               m_dynamicalInverseKinematics))
    {
        std::cerr << printPrefix
                  << "failed to initialize Dynamical Inverse Kinematics."
                  << std::endl;
        return false;
    }

    m_useFixedBase = ikParams.useFixedBase;
    return true;
}

bool IKHelper::initializeDynamicalIK(const ModelData& modelData,
                                     const IKParameters& ikParams,
                                     hde::algorithms::DynamicalInverseKinematics& dynIK)
{
    const std::string printPrefix{"IKHelper::initializeDynamicalIK: "};

    dynIK.setInverseVelocityKinematicsResolutionMode(ikParams.dynIKParams.inverseVelocityKinematicsSolver);
    dynIK.setInverseVelocityKinematicsRegularization(ikParams.dynIKParams.costRegularization);

    if (!dynIK.setModel(modelData.model))
    {
        std::cerr << printPrefix
                  << "failed to load the model." << std::endl;
        return false;
    }

    if (!dynIK.setFloatingBaseOnFrameNamed(modelData.baseFrame))
    {
        std::cerr << printPrefix
                  << "failed to set base link to "
                  << modelData.baseFrame << "." << std::endl;
        return false;
    }

    if (!dynIK.setAllJointsVelocityLimit(ikParams.dynIKParams.jointVelocityLimit))
    {
        std::cerr << printPrefix
                  << "failed to set joint velocity limits."<< std::endl;
        return false;
    }

    if (!dynIK.setConstraintParametersJointValues(ikParams.dynIKParams.k_u,
                                                  ikParams.dynIKParams.k_l))
    {
        std::cerr << printPrefix
                << "failed to set k_u and k_l."<< std::endl;
        return false;
    }

    if (ikParams.customConstraintsLoaded)
    {
        auto customJointSize{ikParams.customIKConstraints.jointsVelocityLimitsNames.size()};
        if (customJointSize != 0)
        {
            for (std::size_t idx = 0; idx < customJointSize; idx++)
            {
                auto dofIdx = ikParams.customIKConstraints.jointsVelocityLimitsIndexes[idx];
                auto limit = ikParams.customIKConstraints.jointsVelocityLimitsValues[idx];
                if (!dynIK.setJointVelocityLimit(dofIdx, limit))
                {
                    std::cerr << printPrefix
                            << "failed to set joint velocity limit for dof "
                            << dofIdx << "."<< std::endl;
                    return false;
                }
            }
        }

        if (!dynIK.setBaseVelocityLimit(ikParams.customIKConstraints.baseVelocityLowerLimit,
                                        ikParams.customIKConstraints.baseVelocityUpperLimit))
        {
            std::cerr << printPrefix
                        << "failed to set base velocity limit." << std::endl;
            return false;
        }

        if (ikParams.customIKConstraints.constraintVariablesIndex.size() != 0)
        {
            if (!dynIK.setLinearJointConfigurationLimits(ikParams.customIKConstraints.constraintVariablesIndex,
                                                        ikParams.customIKConstraints.constraintUpperBound,
                                                        ikParams.customIKConstraints.constraintLowerBound,
                                                        ikParams.customIKConstraints.constraintMatrix))
            {
                std::cerr << printPrefix
                            << "failed to set custom configuration limit." << std::endl;
                return false;
            }
        }
    }

    m_jointConfigurationSolution.resize(modelData.model.getNrOfDOFs());
    m_jointVelocitiesSolution.resize(modelData.model.getNrOfDOFs());
    m_jointConfigurationSolution.zero();
    m_jointVelocitiesSolution.zero();

    std::cout << printPrefix << " Successfully initialized IKHelper" << std::endl;
    return true;
}

bool IKHelper::addTarget(const TargetName& targetName,
                         const IKParameters& ikParams,
                         std::weak_ptr<WearableSensorTarget> target)
{
    const std::string printPrefix{"IKHelper::addTarget: "};
    std::lock_guard<std::mutex> lock(m_mutex);
    auto targetPtr = target.lock();

    if (targetPtr == nullptr)
    {
        std::cerr << printPrefix
                  << "invalid target pointer to target: "
                  << targetName <<"." << std::endl;
        return false;
    }

    auto targetType = targetPtr->getTargetType();
    auto linkName = targetPtr->getModelLinkName();

    bool ok{true};
    switch (targetType)
    {
        case KinematicTargetType::pose:
        {
            ok = m_dynamicalInverseKinematics.addPoseTarget(linkName,
                                                            targetPtr->getCalibratedPosition(),
                                                            targetPtr->getCalibratedRotation(),
                                                            {true, true, true},
                                                            {true, true, true},
                                                            ikParams.dynIKParams.linearCorrectionGain,
                                                            ikParams.dynIKParams.angularCorrectionGain,
                                                            ikParams.dynIKParams.linVelTargetWeight,
                                                            ikParams.dynIKParams.angVelTargetWeight);
            return checkAddTargetStatus(ok,
                                        targetName,
                                        linkName,
                                        "Pose",
                                        printPrefix);
            break;
        }
        case KinematicTargetType::poseAndVelocity:
        {
            ok = m_dynamicalInverseKinematics.addPoseAndVelocityTarget(linkName,
                                                                       targetPtr->getCalibratedPosition(),
                                                                       targetPtr->getCalibratedRotation(),
                                                                       targetPtr->getCalibratedLinearVelocity(),
                                                                       targetPtr->getCalibratedAngularVelocity(),
                                                                       {true, true, true},
                                                                       {true, true, true},
                                                                       ikParams.dynIKParams.linearCorrectionGain,
                                                                       ikParams.dynIKParams.angularCorrectionGain,
                                                                       ikParams.dynIKParams.measuredLinearVelocityGain,
                                                                       ikParams.dynIKParams.measuredAngularVelocityGain,
                                                                       ikParams.dynIKParams.linVelTargetWeight,
                                                                       ikParams.dynIKParams.angVelTargetWeight);
            return checkAddTargetStatus(ok,
                                        targetName,
                                        linkName,
                                        "Pose and Velocity",
                                        printPrefix);
            break;
        }
        case KinematicTargetType::position:
        {
            ok = m_dynamicalInverseKinematics.addPositionTarget(linkName,
                                                                targetPtr->getCalibratedPosition(),
                                                                {true, true, true},
                                                                ikParams.dynIKParams.linearCorrectionGain,
                                                                ikParams.dynIKParams.linVelTargetWeight);
            return checkAddTargetStatus(ok,
                                        targetName,
                                        linkName,
                                        "Position",
                                        printPrefix);
            break;
        }
        case KinematicTargetType::positionAndVelocity:
        {
            ok = m_dynamicalInverseKinematics.addPositionAndVelocityTarget(linkName,
                                                                           targetPtr->getCalibratedPosition(),
                                                                           targetPtr->getCalibratedLinearVelocity(),
                                                                           {true, true, true},
                                                                           ikParams.dynIKParams.linearCorrectionGain,
                                                                           ikParams.dynIKParams.measuredLinearVelocityGain,
                                                                           ikParams.dynIKParams.linVelTargetWeight);
            return checkAddTargetStatus(ok,
                                        targetName,
                                        linkName,
                                        "Position and Velocity",
                                        printPrefix);
            break;
        }
        case KinematicTargetType::orientation:
        {
            ok = m_dynamicalInverseKinematics.addOrientationTarget(linkName,
                                                                   targetPtr->getCalibratedRotation(),
                                                                   {true, true, true},
                                                                   ikParams.dynIKParams.angularCorrectionGain,
                                                                   ikParams.dynIKParams.angVelTargetWeight);
            return checkAddTargetStatus(ok,
                                        targetName,
                                        linkName,
                                        "Orientation",
                                        printPrefix);
            break;
        }
        case KinematicTargetType::orientationAndVelocity:
        {
            ok = m_dynamicalInverseKinematics.addOrientationAndVelocityTarget(linkName,
                                                                              targetPtr->getCalibratedRotation(),
                                                                              targetPtr->getCalibratedAngularVelocity(),
                                                                              {true, true, true},
                                                                              ikParams.dynIKParams.angularCorrectionGain,
                                                                              ikParams.dynIKParams.measuredAngularVelocityGain,
                                                                              ikParams.dynIKParams.angVelTargetWeight);
            return checkAddTargetStatus(ok,
                                        targetName,
                                        linkName,
                                        "Orientation and Velocity",
                                        printPrefix);
            break;
        }
        case KinematicTargetType::gravity:
        {
            ok = m_dynamicalInverseKinematics.addOrientationTarget(linkName,
                                                                   targetPtr->getCalibratedRotation(),
                                                                   {false, false, true},
                                                                   ikParams.dynIKParams.angularCorrectionGain,
                                                                   ikParams.dynIKParams.angVelTargetWeight);
            return checkAddTargetStatus(ok,
                                        targetName,
                                        linkName,
                                        "Gravity",
                                        printPrefix);
            break;
        }
        default:
        {
            std::cerr << printPrefix
                      << "invalid target type for target: "
                      << targetName <<"." << std::endl;
            return false;
        }
    }

    return true;
}

bool IKHelper::checkAddTargetStatus(const bool& ok,
                                    const TargetName& targetName,
                                    const ModelLinkName& linkName,
                                    const std::string& targetType,
                                    const std::string& printPrefix)
{
    if (!ok)
    {
        std::cerr << printPrefix
                  << "Failed to add " << targetType
                  << " target for "
                  << targetName <<"." << std::endl;
        return false;
    }

    std::cout << printPrefix << targetType
              << " target " << targetName
              << " added for link " << linkName << std::endl;

    return true;
}


bool IKHelper::updateTarget(const TargetName& targetName,
                            std::weak_ptr<WearableSensorTarget> target)
{
    const std::string printPrefix{"IKHelper::updateTarget: "};
    std::lock_guard<std::mutex> lock(m_mutex);
    auto targetPtr = target.lock();

    if (targetPtr == nullptr)
    {
        std::cerr << printPrefix
                  << "invalid target pointer to target: "
                  << targetName <<"." << std::endl;
        return false;
    }

    auto targetType = targetPtr->getTargetType();
    auto linkName = targetPtr->getModelLinkName();

    bool ok{true};
    switch (targetType)
    {
        case KinematicTargetType::pose:
        {
            ok = m_dynamicalInverseKinematics.updateTargetPose(linkName,
                                                               targetPtr->getCalibratedPosition(),
                                                               targetPtr->getCalibratedRotation());
            if (!ok)
            {
                std::cerr << printPrefix << "Failed to update pose"
                          << " target for " << targetName <<"." << std::endl;
                return false;
            }
            break;
        }
        case KinematicTargetType::poseAndVelocity:
        {
            ok = m_dynamicalInverseKinematics.updateTargetPoseAndVelocity(linkName,
                                                                          targetPtr->getCalibratedPosition(),
                                                                          targetPtr->getCalibratedRotation(),
                                                                          targetPtr->getCalibratedLinearVelocity(),
                                                                          targetPtr->getCalibratedAngularVelocity());
            if (!ok)
            {
                std::cerr << printPrefix << "Failed to update pose and velocity"
                          << " target for " << targetName <<"." << std::endl;
                return false;
            }
            break;
        }
        case KinematicTargetType::position:
        {
            ok = m_dynamicalInverseKinematics.updateTargetPosition(linkName,
                                                                    targetPtr->getCalibratedPosition());
            if (!ok)
            {
                std::cerr << printPrefix << "Failed to update position"
                          << " target for " << targetName <<"." << std::endl;
                return false;
            }
            break;
        }
        case KinematicTargetType::positionAndVelocity:
        {
            ok = m_dynamicalInverseKinematics.updateTargetPositionAndVelocity(linkName,
                                                                              targetPtr->getCalibratedPosition(),
                                                                              targetPtr->getCalibratedLinearVelocity());
            if (!ok)
            {
                std::cerr << printPrefix << "Failed to update position and velocity"
                          << " target for " << targetName <<"." << std::endl;
                return false;
            }
            break;
        }
        case KinematicTargetType::orientation:
        {
            ok = m_dynamicalInverseKinematics.updateTargetOrientation(linkName,
                                                                      targetPtr->getCalibratedRotation());
            if (!ok)
            {
                std::cerr << printPrefix << "Failed to update orientation"
                          << " target for " << targetName <<"." << std::endl;
                return false;
            }
            break;
        }
        case KinematicTargetType::orientationAndVelocity:
        {
            ok = m_dynamicalInverseKinematics.updateTargetOrientationAndVelocity(linkName,
                                                                                 targetPtr->getCalibratedRotation(),
                                                                                 targetPtr->getCalibratedAngularVelocity());

            if (!ok)
            {
                std::cerr << printPrefix << "Failed to update orientation and velocity"
                          << " target for " << targetName <<"." << std::endl;
                return false;
            }
            break;
        }
        case KinematicTargetType::gravity:
        {
            ok = m_dynamicalInverseKinematics.updateTargetOrientation(linkName,
                                                                      targetPtr->getCalibratedRotation());
            if (!ok)
            {
                std::cerr << printPrefix << "Failed to update gravity"
                          << " target for " << targetName <<"." << std::endl;
                return false;
            }
            break;
        }
        default:
        {
            std::cerr << printPrefix
                      << "invalid target type for target: "
                      << targetName <<"." << std::endl;
            return false;
        }
    }

    return true;
}

bool IKHelper::solve(const double& dt)
{
    const std::string printPrefix{"IKHelper::solve: "};
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_dynamicalInverseKinematics.solve(dt))
    {
        std::cerr << printPrefix
                  << "Failed to solve Dynamical IK." << std::endl;
        return false;
    }

    m_dynamicalInverseKinematics.getConfigurationSolution(m_baseTransformSolution,
                                                          m_jointConfigurationSolution);
    m_dynamicalInverseKinematics.getVelocitySolution(m_baseVelocitySolution,
                                                     m_jointVelocitiesSolution);
    //m_targetsPositionErrorNorm = m_dynamicalInverseKinematics.getTargetsMeanPositionErrorNorm();
    //m_targetsRotationErrorNorm = m_dynamicalInverseKinematics.getTargetsMeanOrientationErrorNorm();

    if (m_useFixedBase)
    {
        m_baseTransformSolution = iDynTree::Transform::Identity();
        m_baseVelocitySolution.zero();
    }

    return true;
}

const iDynTree::Transform& IKHelper::getBaseTransformSolution() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_baseTransformSolution;
}

const iDynTree::Twist& IKHelper::getBaseVelocitySolution() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_baseVelocitySolution;
}

const iDynTree::VectorDynSize& IKHelper::getJointConfigurationSolution() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_jointConfigurationSolution;
}

const iDynTree::VectorDynSize& IKHelper::getJointVelocitiesSolution() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_jointVelocitiesSolution;
}

const double& IKHelper::getTargetsMeanPositionErrorNorm() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_targetsPositionErrorNorm;
}

const double& IKHelper::getTargetsMeanRotationErrorNorm() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_targetsRotationErrorNorm;
}



