/**
 * @file IWearableTargets.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_INVERSE_KINEMATICS_IWEARABLE_TARGETS_H
#define KINDYNFUSION_INVERSE_KINEMATICS_IWEARABLE_TARGETS_H

#include <KinDynFusion/InverseKinematics/WearableTargets.h>

#include <memory>
#include <vector>
#include <string>

namespace KinDynFusion
{
namespace IK
{

class IWearableTargets
{
public:
    virtual ~IWearableTargets() = default;
    virtual std::vector<TargetName> getAllWearableTargetNames() = 0;
    virtual std::shared_ptr<WearableSensorTarget> getWearableTarget(const TargetName& name) = 0;
    virtual WearableSensorTargetMap getAllWearableTargets()  = 0;
};

} // IK
} // KinDynFusion

#endif // KINDYNFUSION_INVERSE_KINEMATICS_IWEARABLE_TARGETS_H
