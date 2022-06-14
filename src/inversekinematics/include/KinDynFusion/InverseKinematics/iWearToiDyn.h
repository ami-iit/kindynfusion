/**
 * @file iWearToiDyn.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_INVERSE_KINEMATICS_IWEAR_TO_IDYN_H
#define KINDYNFUSION_INVERSE_KINEMATICS_IWEAR_TO_IDYN_H

#include <KinDynFusion/InverseKinematics/WearableTargets.h>
#include <KinDynFusion/InverseKinematics/WearableSensorizedShoes.h>
#include <Wearable/IWear/IWear.h>

#include <memory>
#include <mutex>
#include <vector>
#include <string>

namespace KinDynFusion
{
namespace IK
{

class iWearToiDyn
{
public:
    bool updateWearableTargets(const wearable::IWear* iWear,
                               const TargetName& targetName,
                               std::weak_ptr<WearableSensorTarget> target);
    bool checkWearableTargetiniWearInterface(const wearable::IWear* iWear,
                                             std::weak_ptr<const WearableSensorTarget> target);
    bool checkWearableSensorizedShoesiniWearInterface(const wearable::IWear* iWear,
                                             std::weak_ptr<const WearableSensorizedShoe> shoe);
    bool updateWearableSensorizedShoes(const wearable::IWear* iWear,
                                       std::weak_ptr<WearableSensorizedShoe> shoe);
private:
    template <typename iDynVec3orPos>
    void iWearToiDynVec3(const wearable::Vector3& vec3,
                         iDynVec3orPos& vec3Dyn)
    {
        for (auto idx = 0; idx < 3; idx++)
        {
            vec3Dyn.setVal(idx, vec3.at(idx));
        }
    }

    void iWearToiDynWrench(const wearable::Vector6& vec6,
                           iDynTree::Wrench& wrenchDyn)
    {
        for (auto idx = 0; idx < 6; idx++)
        {
            wrenchDyn.setVal(idx, vec6.at(idx));
        }
    }

    mutable std::mutex m_mutex;

    // place holder buffers
    wearable::sensor::SensorStatus m_status;
    wearable::Vector3 m_position;
    wearable::Quaternion m_orientation;
    wearable::Vector3 m_linearVelocity;
    wearable::Vector3 m_angularVelocity;
    wearable::Vector6 m_wrench;

    iDynTree::Position m_dynPosition;
    iDynTree::Rotation m_dynOrientation;
    iDynTree::Vector3 m_dynLinearVelocity;
    iDynTree::Vector3 m_dynAngularVelocity;
    iDynTree::Wrench m_dynWrench;
};

} // IK
} // KinDynFusion

#endif // KINDYNFUSION_INVERSE_KINEMATICS_IWEAR_TO_IDYN_H
