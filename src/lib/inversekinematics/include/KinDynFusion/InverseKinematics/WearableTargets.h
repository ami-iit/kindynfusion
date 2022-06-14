/**
 * @file WearableTargets.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_INVERSE_KINEMATICS_WEARABLE_TARGETS_H
#define KINDYNFUSION_INVERSE_KINEMATICS_WEARABLE_TARGETS_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Span.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>
#include <memory>
namespace KinDynFusion
{
namespace IK
{

enum class KinematicTargetType
{
    none,
    pose,
    poseAndVelocity,
    position,
    positionAndVelocity,
    orientation,
    orientationAndVelocity,
    gravity
};

struct CalibrationStorage
{
    CalibrationStorage()
    {
        calibrationWorld_H_sensorWorld = iDynTree::Transform::Identity();
        sensor_H_segment = iDynTree::Transform::Identity();
        positionScalingFactor(0) = 1.0;
        positionScalingFactor(1) = 1.0;
        positionScalingFactor(2) = 1.0;
    }

    iDynTree::Transform sensor_H_segment;
    iDynTree::Transform calibrationWorld_H_sensorWorld;
    iDynTree::Vector3 positionScalingFactor;
};

using TargetName = std::string;
using ModelLinkName = std::string;
using WearableName = std::string;

class WearableSensorTarget
{
public:
    WearableSensorTarget(WearableName wearableName_,
                         ModelLinkName modelLinkName_,
                         KinematicTargetType targetType_);
    WearableName getWearableName() const;
    ModelLinkName getModelLinkName() const;
    KinematicTargetType getTargetType() const;

    void setPose(const iDynTree::Transform& H);
    void setPosition(const iDynTree::Vector3& p);
    void setRotation(const iDynTree::Rotation& R);
    void setTwist(const iDynTree::Twist& twist);
    void setLinearVelocity(const iDynTree::Vector3& v);
    void setAngularVelocity(const iDynTree::Vector3& omega);

    // CalibrationHelper class may use these methods
    // to set the internal calibration storage
    void resetCalibrationStorage();
    void resetCalibrationMatrices();
    void clearWorldCalibrationMatrix();
    void clearSegmentSensorCalibrationMatrix();

    void setCalibrationStorage(const CalibrationStorage& calib);
    CalibrationStorage& getCalibrationStorage();

    iDynTree::Vector3  getRawPosition() const;
    iDynTree::Rotation getRawRotation() const;
    iDynTree::Vector3  getRawLinearVelocity() const;
    iDynTree::Vector3  getRawAngularVelocity() const;

    iDynTree::Vector3  getCalibratedPosition();
    iDynTree::Rotation getCalibratedRotation();
    iDynTree::Vector3  getCalibratedLinearVelocity();
    iDynTree::Vector3  getCalibratedAngularVelocity();

private:
    WearableName m_wearableName;
    ModelLinkName m_modelLinkName;
    KinematicTargetType m_targetType;
    mutable std::mutex m_mutex;

    CalibrationStorage m_calib;

    iDynTree::Vector3 m_position;
    iDynTree::Rotation m_rotation;
    iDynTree::Vector3 m_linearVelocity;
    iDynTree::Vector3 m_angularVelocity;

    // buffer variables
    iDynTree::Vector3 m_positionInWorld;
    iDynTree::Vector3 m_positionScaled;
    iDynTree::Vector3 m_linearVelocityInWorld;
    iDynTree::Vector3 m_linearVelocityScaled;
};


} // namespace IK
} // namespace KinDynFusion

using WearableSensorTargetPtr = std::shared_ptr<KinDynFusion::IK::WearableSensorTarget>;
using WearableSensorTargetMap = std::unordered_map<std::string, WearableSensorTargetPtr>;

#endif // KINDYNFUSION_INVERSE_KINEMATICS_WEARABLE_TARGETS_H
