/**
 * @file CalibrationHelper.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/FloatingBaseEstimators/CalibrationHelper.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <cmath>
#include <numeric>

using namespace KinDynFusion::Estimators;
using namespace KinDynFusion::IK;

bool CalibrationHelper::initialize(const ModelData& modelData)
{
    constexpr auto printPrefix = "CalibrationHelper::initialize: ";
    if (!m_kinDyn.loadRobotModel(modelData.model))
    {
        std::cerr << printPrefix
                  << "Failed to load model." << std::endl;
        return false;
    }

    m_basePose = iDynTree::Transform::Identity();
    auto nrDOFs = modelData.model.getNrOfDOFs();
    m_jointPos.resize(nrDOFs);
    m_jointPos.zero();
    m_jointVel.resize(nrDOFs);
    m_jointVel.zero();
    m_baseVel.zero();

    m_zeroPosition = iDynTree::Position(0., 0., 0.);
    m_worldGravity.zero();
    m_worldGravity(2) = -9.80665;

    return true;
}

void CalibrationHelper::setConfiguration(const iDynTree::Transform& baseTransformSolution,
                                         const iDynTree::VectorDynSize& jointConfigurationSolution)
{
    m_basePose = baseTransformSolution;
    m_jointPos = jointConfigurationSolution;
}

bool CalibrationHelper::calibrateAll(WearableSensorTargetMap& targetMap)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    calibAllHelper("", targetMap);
    return true;
}

bool CalibrationHelper::calibrateAllWithWorld(const std::string& refTargetName,
                                              WearableSensorTargetMap& targetMap)
{
    constexpr auto printPrefix = "CalibrationHelper::calibrateAllWithWorld: ";
    std::lock_guard<std::mutex> lock{m_mutex};
    if (targetMap.find(refTargetName) == targetMap.end())
    {
        std::cerr << printPrefix
                  << "Target " << refTargetName
                  << " choosen as base for secondary calibration is not valid."
                  << std::endl;;
        return false;
    }

    calibAllHelper(refTargetName, targetMap);

    return true;
}

bool CalibrationHelper::calibrateAllWorldYaw(WearableSensorTargetMap& targetMap)
{
    constexpr auto printPrefix = "CalibrationHelper::calibrateAllWorldYaw: ";
    std::lock_guard<std::mutex> lock{m_mutex};
    fillCalibrationIndices();
    setZeroJointsAndRobotState(m_jointZeroIndices);

    for (const auto& [targetName, target] : targetMap)
    {
        auto linkName = target->getModelLinkName();
        auto linkIdx = m_kinDyn.model().getLinkIndex(linkName);

        if (std::find(m_linkToCalibrateIndices.begin(),
                      m_linkToCalibrateIndices.end(),
                      linkIdx) != m_linkToCalibrateIndices.end())
        {
            std::cout << printPrefix
                      << "Link Found, target name: " << targetName
                      << " link name: " << linkName
                      << " link index: " << linkIdx
                      << std::endl;

            target->clearWorldCalibrationMatrix();

            auto CWnew_R_L = m_kinDyn.getWorldTransform(linkName).getRotation();
            auto L_R_CW_old = target->getCalibratedRotation().inverse();
            auto rpyOffset = (CWnew_R_L*L_R_CW_old).asRPY();
            target->getCalibrationStorage().calibrationWorld_H_sensorWorld.setRotation(iDynTree::Rotation::RotZ(rpyOffset.getVal(2)));

            std::cout << printPrefix
                      << "Calibration matrices set for "
                      << targetName  << std::endl;
        }
    }

    return true;
}

bool CalibrationHelper::setRotationOffset(const std::string& targetName,
                                          const double& rollInDegrees,
                                          const double& pitchInDegrees,
                                          const double& yawInDegrees,
                                          WearableSensorTargetMap& targetMap)
{
    constexpr auto printPrefix = "CalibrationHelper::calibrateRelativeLink: ";
    std::lock_guard<std::mutex> lock{m_mutex};
    if (targetMap.find(targetName) == targetMap.end())
    {
        std::cerr << printPrefix
                  << "Target " << targetName
                  << " choosen for secondary calibration is not valid."
                  << std::endl;;
        return false;
    }

    targetMap.at(targetName)->resetCalibrationMatrices();
    auto S_R_L = iDynTree::Rotation::RPY(M_PI*rollInDegrees/180.0,
                                         M_PI*pitchInDegrees/180.0,
                                         M_PI*yawInDegrees/180.0);
    targetMap.at(targetName)->getCalibrationStorage().sensor_H_segment.setRotation(S_R_L);
    std::cout << printPrefix
              << "sensor_R_segment calibration matrix set for "
              << targetName << std::endl;
    return true;
}

void CalibrationHelper::resetCalibration(const std::string& targetName,
                                         WearableSensorTargetMap& targetMap)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    if (targetMap.find(targetName) != targetMap.end())
    {
        targetMap.at(targetName)->resetCalibrationMatrices();
    }
}

void CalibrationHelper::resetAll(WearableSensorTargetMap& targetMap)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    for (const auto& [targetName, target] : targetMap)
    {
        target->resetCalibrationMatrices();
    }
}

void CalibrationHelper::calibAllHelper(const std::string& refTargetName,
                                       WearableSensorTargetMap& targetMap)
{
    fillCalibrationIndices();

    // Compute secondary calibration for the selected links setting to zero the given joints
    computeSecondaryCalibrationRotationsForChain(m_jointZeroIndices,
                                                 iDynTree::Transform::Identity(),
                                                 m_linkToCalibrateIndices,
                                                 refTargetName,
                                                 targetMap);
}

void CalibrationHelper::fillCalibrationIndices()
{
    // Select all the links and the joints
    // add all the links of the model to [linkToCalibrateIndices]
    m_linkToCalibrateIndices.resize(m_kinDyn.getNrOfLinks());
    std::iota(m_linkToCalibrateIndices.begin(), m_linkToCalibrateIndices.end(), 0);

    // add all the joints of the model to [jointZeroIndices]
    m_jointZeroIndices.resize(m_kinDyn.getNrOfDegreesOfFreedom());
    std::iota(m_jointZeroIndices.begin(), m_jointZeroIndices.end(), 0);
}

void CalibrationHelper::setZeroJointsAndRobotState(const std::vector<iDynTree::JointIndex>& jointZeroIndices)
{
    // setting to zero all the selected joints
    for (const auto& jointZeroIdx : jointZeroIndices)
    {
        m_jointPos.setVal(jointZeroIdx, 0);
    }

    // TODO check which value to give to the base (before we were using the base target measurement)
    // should we set m_basePose instead?
    m_kinDyn.setRobotState(iDynTree::Transform::Identity(),
                           m_jointPos,
                           m_baseVel,
                           m_jointVel,
                           m_worldGravity);
}

void
CalibrationHelper::computeSecondaryCalibrationRotationsForChain(const std::vector<iDynTree::JointIndex>& jointZeroIndices,
                                                                const iDynTree::Transform& refLinkForCalibrationTransform,
                                                                const std::vector<iDynTree::LinkIndex>& linkToCalibrateIndices,
                                                                const std::string& refTargetForCalibrationName,
                                                                WearableSensorTargetMap& targetMap)
{
    constexpr auto printPrefix = "CalibrationHelper::computeSecondaryCalibrationRotationsForChain: ";
    setZeroJointsAndRobotState(jointZeroIndices);

    // If needed compute world calibration matrix
    // in this case the same world calibration transform i used for all the targets
    auto cw_H_sw = iDynTree::Transform::Identity();
    if (refTargetForCalibrationName != "")
    {
        auto linkName = targetMap.at(refTargetForCalibrationName)->getModelLinkName();
        auto CW_H_L = m_kinDyn.getWorldTransform(linkName);
        cw_H_sw = refLinkForCalibrationTransform * CW_H_L.inverse();
    }

    for (const auto& [targetName, target] : targetMap)
    {
        auto linkName = target->getModelLinkName();
        auto linkIdx = m_kinDyn.model().getLinkIndex(linkName);

        if (std::find(linkToCalibrateIndices.begin(),
                      linkToCalibrateIndices.end(),
                      linkIdx) != linkToCalibrateIndices.end())
        {
            std::cout << printPrefix
                      << "Link Found, target name: " << targetName
                      << " link name: " << linkName
                      << " link index: " << linkIdx
                      << std::endl;

            target->clearSegmentSensorCalibrationMatrix();
            auto cw_H_l = m_kinDyn.getWorldTransform(linkName);
            auto cw_H_sw_old = target->getCalibrationStorage().calibrationWorld_H_sensorWorld;

            auto measTargetPosition = iDynTree::Position(target->getRawPosition()(0),
                                                         target->getRawPosition()(1),
                                                         target->getRawPosition()(2));

            auto s_R_cw = target->getCalibratedRotation().inverse();
            auto cw_R_l = cw_H_l.getRotation();
            auto s_R_l = s_R_cw*cw_R_l;
            target->getCalibrationStorage().sensor_H_segment.setRotation(s_R_l);

            auto cw_p_l = cw_H_l.getPosition();
            auto cw_p_sw_old = cw_H_sw_old.getPosition();
            auto sw_old_R_cw = cw_H_sw_old.getRotation().inverse();
            auto s_R_sw = target->getRawRotation().inverse();
            auto s_p_l = ( (cw_p_l - cw_p_sw_old).changeCoordinateFrame(sw_old_R_cw) - measTargetPosition ).changeCoordinateFrame(s_R_sw);
            target->getCalibrationStorage().sensor_H_segment.setPosition(s_p_l);

            target->getCalibrationStorage().calibrationWorld_H_sensorWorld = cw_H_sw*cw_H_sw_old;

            std::cout << printPrefix
                      << "Calibration matrices set for "
                      << targetName  << std::endl;
        }
    }
}


