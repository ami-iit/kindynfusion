/**
 * @file WearableTargets.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/InverseKinematics/WearableTargets.h>
#include <iDynTree/Core/EigenHelpers.h>

using namespace KinDynFusion::IK;

WearableSensorTarget::WearableSensorTarget(WearableName wearableName_,
                                           ModelLinkName modelLinkName_,
                                           KinematicTargetType targetType_)
                                         : m_wearableName(wearableName_)
                                         , m_modelLinkName(modelLinkName_)
                                         , m_targetType(targetType_)
{
    m_position.zero();
    m_rotation.Identity();
    m_linearVelocity.zero();
    m_angularVelocity.zero();

    m_positionInWorld.zero();
    m_positionScaled.zero();
    m_linearVelocityInWorld.zero();
    m_linearVelocityScaled.zero();

    resetCalibrationMatrices();
};

WearableName WearableSensorTarget::getWearableName() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_wearableName;
}

ModelLinkName WearableSensorTarget::getModelLinkName() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_modelLinkName;
}

KinematicTargetType WearableSensorTarget::getTargetType() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_targetType;
}

void WearableSensorTarget::setPose(const iDynTree::Transform& H)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_position = H.getPosition();
    m_rotation = H.getRotation();
}

void WearableSensorTarget::setPosition(const iDynTree::Vector3& p)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_position = p;
}

void WearableSensorTarget::setRotation(const iDynTree::Rotation& R)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rotation = R;
}

void WearableSensorTarget::setTwist(const iDynTree::Twist& twist)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_linearVelocity = twist.getLinearVec3();
    m_angularVelocity = twist.getAngularVec3();
}

void WearableSensorTarget::setLinearVelocity(const iDynTree::Vector3& v)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_linearVelocity = v;
}

void WearableSensorTarget::setAngularVelocity(const iDynTree::Vector3& omega)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_angularVelocity = omega;
}

void WearableSensorTarget::resetCalibrationMatrices()
{
    clearSegmentSensorCalibrationMatrix();
    clearWorldCalibrationMatrix();
}

void WearableSensorTarget::resetCalibrationStorage()
{
    resetCalibrationMatrices();
    std::lock_guard<std::mutex> lock(m_mutex);
    m_calib.positionScalingFactor(0) = 1.0;
    m_calib.positionScalingFactor(1) = 1.0;
    m_calib.positionScalingFactor(2) = 1.0;
}

void WearableSensorTarget::clearSegmentSensorCalibrationMatrix()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_calib.sensor_H_segment = iDynTree::Transform::Identity();
}

void WearableSensorTarget::clearWorldCalibrationMatrix()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_calib.calibrationWorld_H_sensorWorld = iDynTree::Transform::Identity();
}


void WearableSensorTarget::setCalibrationStorage(const CalibrationStorage& calib)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_calib = calib;
}

CalibrationStorage& WearableSensorTarget::getCalibrationStorage()
{
    return m_calib;
}

iDynTree::Vector3  WearableSensorTarget::getRawPosition() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_position;
}

iDynTree::Rotation WearableSensorTarget::getRawRotation() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rotation;
}

iDynTree::Vector3  WearableSensorTarget::getRawLinearVelocity() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_linearVelocity;
}

iDynTree::Vector3  WearableSensorTarget::getRawAngularVelocity() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_angularVelocity;
}

iDynTree::Vector3  WearableSensorTarget::getCalibratedPosition()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    using iDynTree::toEigen;
    auto cw_R_sw = toEigen(m_calib.calibrationWorld_H_sensorWorld.getRotation());
    auto cw_p_sw = toEigen(m_calib.calibrationWorld_H_sensorWorld.getPosition());
    auto sw_p_s = toEigen(m_position);
    auto sw_R_s = toEigen(m_rotation);
    auto s_p_l = toEigen(m_calib.sensor_H_segment.getPosition());
    auto cw_p_l = toEigen(m_positionInWorld);

    // linear part of cw_H_l = cw_H_sw*sw_H_s*s_H_l
    cw_p_l = cw_R_sw*(sw_R_s*s_p_l + sw_p_s) + cw_p_sw;

    // scale position
    for (auto idx = 0; idx < 3; idx++)
    {
        m_positionScaled.setVal(idx,
                                m_positionInWorld(idx)*m_calib.positionScalingFactor(idx));
    };

    return m_positionScaled;
}

iDynTree::Rotation WearableSensorTarget::getCalibratedRotation()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_calib.calibrationWorld_H_sensorWorld.getRotation() *
           m_rotation *
           m_calib.sensor_H_segment.getRotation();
}

iDynTree::Vector3  WearableSensorTarget::getCalibratedLinearVelocity()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    using iDynTree::toEigen;
    auto cw_R_sw = toEigen(m_calib.calibrationWorld_H_sensorWorld.getRotation());
    auto l_H_s = m_calib.sensor_H_segment.inverse();
    auto l_p_s = toEigen(l_H_s.getPosition());
    auto v = toEigen(m_linearVelocity);
    auto omega = toEigen(m_angularVelocity);

    auto cw_v = toEigen(m_linearVelocityInWorld);

    cw_v = cw_R_sw*(v + iDynTree::skew(l_p_s)*omega);
    // scale linear velocity
    for (auto idx = 0; idx < 3; idx++)
    {
        m_linearVelocityScaled.setVal(idx,
                                      m_linearVelocityInWorld(idx)*
                                      m_calib.positionScalingFactor(idx));
    }

    return m_linearVelocityScaled;
}

iDynTree::Vector3  WearableSensorTarget::getCalibratedAngularVelocity()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    // cw_R_sw*sw_omega_s
    return iDynTree::AngularMotionVector3(m_angularVelocity).changeCoordFrame(
        m_calib.calibrationWorld_H_sensorWorld.getRotation());
}
