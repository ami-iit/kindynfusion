/**
 * @file IWearableTargets.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_ESTIMATION_IWHOLE_BODY_KINEMATICS_H
#define KINDYNFUSION_ESTIMATION_IWHOLE_BODY_KINEMATICS_H

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <hde/interfaces/IHumanState.h>

#include <array>
#include <vector>

namespace KinDynFusion
{
namespace Estimators
{

struct VertexContactData
{
    std::string name;
    std::array<double, 3> position;
    double magnitude;
    bool isActive;
    std::string linkName; // name of the contact link
};

struct EstimatedContactPosition
{
    std::string name;
    std::array<double, 3> position;
};

class IWholeBodyKinematics : public hde::interfaces::IHumanState
{
public:
    virtual ~IWholeBodyKinematics() = default;

    // estimated contact point positions
    virtual std::vector<EstimatedContactPosition> getContactPointsPosition() = 0;
    virtual std::array<double, 4> getLeftFootOrientation() const = 0;
    virtual std::array<double, 4> getRightFootOrientation() const = 0;

    // COP position in foot frame
    virtual std::array<double, 3> getLeftFootCoPPosition() = 0;
    virtual std::array<double, 3> getRightFootCoPPosition() = 0;

    virtual std::array<double, 3> getGlobalCoPPosition() = 0;

    // vertex position data expressed in foot frame
    virtual std::vector<VertexContactData> getAllVertexContactData() = 0;
    virtual std::vector<std::string> getAllVertexContactNames() = 0;

    virtual std::string getLeftFootLinkName() const = 0;
    virtual std::string getRightFootLinkName() const = 0;
};

} // IK
} // KinDynFusion

#endif // KINDYNFUSION_ESTIMATION_IWHOLE_BODY_KINEMATICS_H

