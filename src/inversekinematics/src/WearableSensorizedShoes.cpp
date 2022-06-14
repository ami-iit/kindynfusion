/**
 * @file WearableSensorizedShoes.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/InverseKinematics/WearableSensorizedShoes.h>
#include <iDynTree/Core/EigenHelpers.h>

using namespace KinDynFusion::IK;

WearableSensorizedShoe::WearableSensorizedShoe(const std::string& wearableName_,
                                               const std::string& footLinkName_)
                                                : m_wearableName(wearableName_),
                                                  m_footLinkName(footLinkName_)
{
    m_wrench.zero();
    m_wrenchInFoot.zero();
    m_footHshoe = iDynTree::Transform::Identity();
};

std::string WearableSensorizedShoe::getWearableName() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_wearableName;
}

std::string WearableSensorizedShoe::getModelLinkName() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_footLinkName;
}

void WearableSensorizedShoe::setWrench(const iDynTree::Wrench& wrench)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_wrench = wrench;
}

void WearableSensorizedShoe::setForce(const iDynTree::Vector3& force)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_wrench.setLinearVec3(force);
}

void WearableSensorizedShoe::setTorque(const iDynTree::Vector3& torque)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_wrench.setAngularVec3(torque);
}

void WearableSensorizedShoe::resetFootHShoeTransform()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_footHshoe.Identity();
}

void WearableSensorizedShoe::setFootHShoe(const iDynTree::Transform& footHshoe)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_footHshoe = footHshoe;
}

iDynTree::Transform WearableSensorizedShoe::getFootHShoe() const
{
    return m_footHshoe;
}

iDynTree::Vector3 WearableSensorizedShoe::getRawForce() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_wrench.getLinearVec3();
}


iDynTree::Vector3 WearableSensorizedShoe::getRawTorque() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_wrench.getAngularVec3();
}

iDynTree::Wrench WearableSensorizedShoe::getRawWrench() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_wrench;
}

iDynTree::Wrench WearableSensorizedShoe::getWrenchInFootFrame()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_wrenchInFoot = m_footHshoe*m_wrench;

    return m_wrenchInFoot;
}

iDynTree::Vector3  WearableSensorizedShoe::getForceInFootFrame()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    using iDynTree::toEigen;
    m_wrenchInFoot = m_footHshoe*m_wrench;

    return m_wrenchInFoot.getLinearVec3();
}

iDynTree::Vector3  WearableSensorizedShoe::getTorqueInFootFrame()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    using iDynTree::toEigen;
    m_wrenchInFoot = m_footHshoe*m_wrench;

    return m_wrenchInFoot.getAngularVec3();
}
