/**
 * @file IWearableSensorizedShoes.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_INVERSE_KINEMATICS_IWEARABLE_SENSORIZED_SHOES_H
#define KINDYNFUSION_INVERSE_KINEMATICS_IWEARABLE_SENSORIZED_SHOES_H

#include <KinDynFusion/InverseKinematics/WearableSensorizedShoes.h>

#include <memory>
#include <vector>
#include <string>

namespace KinDynFusion
{
namespace IK
{

class IWearableSensorizedShoes
{
public:
    virtual ~IWearableSensorizedShoes() = default;
    virtual std::vector<std::string> getAllWearableShoeNames() = 0;
    virtual std::shared_ptr<WearableSensorizedShoe> getWearableSensorizedShoe(const std::string& name) = 0;
    virtual WearableSensorizedShoeMap getAllWearableSensorizedShoes() = 0;
};

} // IK
} // KinDynFusion

#endif // KINDYNFUSION_INVERSE_KINEMATICS_IWEARABLE_SENSORIZED_SHOES_H

