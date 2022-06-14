/**
 * @file WearableSensorizedShoes.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_INVERSE_KINEMATICS_WEARABLE_SENSORIZED_SHOES_H
#define KINDYNFUSION_INVERSE_KINEMATICS_WEARABLE_SENSORIZED_SHOES_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Wrench.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

namespace KinDynFusion
{
namespace IK
{

class WearableSensorizedShoe
{
public:
    WearableSensorizedShoe(const std::string& wearableName_,
                           const std::string& footLinkName_);
    std::string getWearableName() const;
    std::string getModelLinkName() const;

    void setWrench(const iDynTree::Wrench& wrench);
    void setForce(const iDynTree::Vector3& force);
    void setTorque(const iDynTree::Vector3& torque);

    void resetFootHShoeTransform();
    void setFootHShoe(const iDynTree::Transform& foot_H_shoe);
    iDynTree::Transform getFootHShoe() const;

    iDynTree::Wrench  getRawWrench() const;
    iDynTree::Vector3  getRawForce() const;
    iDynTree::Vector3  getRawTorque() const;

    iDynTree::Wrench  getWrenchInFootFrame();
    iDynTree::Vector3 getForceInFootFrame();
    iDynTree::Vector3  getTorqueInFootFrame();

private:
    std::string m_wearableName;
    std::string m_footLinkName;
    mutable std::mutex m_mutex;

    iDynTree::Transform m_footHshoe;
    iDynTree::Wrench m_wrench;

    // buffer variables
    iDynTree::Wrench m_wrenchInFoot;
};


} // namespace IK
} // namespace KinDynFusion

using WearableSensorizedShoePtr = std::shared_ptr<KinDynFusion::IK::WearableSensorizedShoe>;
using WearableSensorizedShoeMap = std::unordered_map<std::string, WearableSensorizedShoePtr>;

#endif // KINDYNFUSION_INVERSE_KINEMATICS_WEARABLE_SENSORIZED_SHOES_H

