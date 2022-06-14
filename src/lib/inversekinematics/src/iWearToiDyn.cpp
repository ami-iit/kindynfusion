/**
 * @file iWearToiDyn.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/InverseKinematics/iWearToiDyn.h>

using namespace KinDynFusion::IK;

bool iWearToiDyn::checkWearableTargetiniWearInterface(const wearable::IWear* iWearPtr,
                                                      std::weak_ptr<const WearableSensorTarget> target)
{
    constexpr auto printPrefix = "iWearToiDyn::checkWearableTargetiniWearInterface: ";

    if (iWearPtr == nullptr)
    {
        std::cerr << printPrefix
                  << "Invalid iWear ptr." << std::endl;
        return false;
    }

    auto targetPtr = target.lock();
    if (targetPtr == nullptr)
    {
        std::cerr << printPrefix
                  << "Invalid WearableSensorTarget ptr." << std::endl;
        return false;
    }

    auto wearableName = targetPtr->getWearableName();
    auto sensor = iWearPtr->getSensor(wearableName);
    if (!sensor)
    {
        std::cerr << printPrefix
                  << "Failed to find sensor " << wearableName
                  << " in the iWear interface." << std::endl;
        return false;
    }

    return true;
}

bool iWearToiDyn::checkWearableSensorizedShoesiniWearInterface(const wearable::IWear* iWearPtr,
                                                               std::weak_ptr<const WearableSensorizedShoe> shoe)
{
    constexpr auto printPrefix = "iWearToiDyn::checkWearableTargetiniWearInterface: ";

    if (iWearPtr == nullptr)
    {
        std::cerr << printPrefix
                  << "Invalid iWear ptr." << std::endl;
        return false;
    }

    auto shoePtr = shoe.lock();
    if (shoePtr == nullptr)
    {
        std::cerr << printPrefix
                  << "Invalid WearableSensorizedShoe ptr." << std::endl;
        return false;
    }

    auto wearableName = shoePtr->getWearableName();
    auto sensor = iWearPtr->getSensor(wearableName);
    if (!sensor)
    {
        std::cerr << printPrefix
                  << "Failed to find sensor " << wearableName
                  << " in the iWear interface." << std::endl;
        return false;
    }

    return true;
}


bool iWearToiDyn::updateWearableTargets(const wearable::IWear* iWearPtr,
                                        const TargetName& targetName,
                                        std::weak_ptr<WearableSensorTarget> target)
{
    constexpr auto printPrefix = "iWearToiDyn::updateWearableTargets: ";
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!checkWearableTargetiniWearInterface(iWearPtr, target))
    {
         std::cerr << printPrefix
                   << " Cannot find sensor associated to target ["
                   << targetName << "] in iWear interface." << std::endl;
        return false;
    }

    auto targetPtr = target.lock();
    auto wearableName = targetPtr->getWearableName();

    switch (iWearPtr->getSensor(wearableName)->getSensorType())
    {
        case wearable::sensor::SensorType::VirtualLinkKinSensor:
        {
            auto sensor = iWearPtr->getVirtualLinkKinSensor(wearableName);
            if (!sensor)
            {
                std::cerr << printPrefix << "Sensor" << wearableName
                          << " has been added but is not properly configured." << std::endl;
                return false;
            }

            m_status = sensor->getSensorStatus();
            if (m_status != wearable::sensor::SensorStatus::Ok)
            {
                std::cerr << printPrefix << "Sensor status of " << wearableName
                          << " is not ok (" << static_cast<double>(m_status) << ")." << std::endl;
                return false;
            }

            if (sensor->getLinkPosition(m_position))
            {
                iWearToiDynVec3(m_position, m_dynPosition);
                targetPtr->setPosition(m_dynPosition);
            }
            else
            {
                std::cerr << printPrefix
                          << "Failed to read position from VirtualLinkKinSensor: "
                          << wearableName << std::endl;
            }

            if (sensor->getLinkOrientation(m_orientation))
            {
                m_dynOrientation.fromQuaternion({m_orientation.data(), 4});
                targetPtr->setRotation(m_dynOrientation);
            }
            else
            {
                std::cerr << printPrefix
                          << "Failed to read orientation from VirtualLinkKinSensor: "
                          << wearableName << std::endl;
            }

            if (sensor->getLinkLinearVelocity(m_linearVelocity))
            {
                iWearToiDynVec3(m_linearVelocity, m_dynLinearVelocity);
                targetPtr->setLinearVelocity(m_dynLinearVelocity);
            }
            else
            {
                std::cerr << printPrefix
                          << "Failed to read linear velocity from VirtualLinkKinSensor: "
                          << wearableName << std::endl;
            }

            if (sensor->getLinkAngularVelocity(m_angularVelocity))
            {
                iWearToiDynVec3(m_angularVelocity, m_dynAngularVelocity);
                targetPtr->setAngularVelocity(m_dynAngularVelocity);
            }
            else
            {
                std::cerr << printPrefix
                          << "Failed to read angular velocity from VirtualLinkKinSensor: "
                          << wearableName << std::endl;
            }

            break;
        }
        case wearable::sensor::SensorType::PoseSensor:
        {
            auto sensor = iWearPtr->getPoseSensor(wearableName);
            if (!sensor)
            {
                std::cerr << printPrefix << "Sensor" << wearableName
                          << " has been added but is not properly configured." << std::endl;
                return false;
            }

            m_status = sensor->getSensorStatus();
            if (m_status != wearable::sensor::SensorStatus::Ok)
            {
                std::cerr << printPrefix << "Sensor status of " << wearableName
                          << " is not ok (" << static_cast<double>(m_status) << ")." << std::endl;
                return false;
            }

            if (sensor->getPosePosition(m_position))
            {
                iWearToiDynVec3(m_position, m_dynPosition);
                targetPtr->setPosition(m_dynPosition);
            }
            else
            {
                std::cerr << printPrefix
                          << "Failed to read position from PoseSensor: "
                          << wearableName << std::endl;
            }

            if (sensor->getPoseOrientationAsQuaternion(m_orientation))
            {
                m_dynOrientation.fromQuaternion({m_orientation.data(), 4});
                targetPtr->setRotation(m_dynOrientation);
            }
            else
            {
                std::cerr << printPrefix
                          << "Failed to read orientation from PoseSensor: "
                          << wearableName << std::endl;
            }

            break;
        }
        default:
        {
            std::cerr << printPrefix
                      << "Sensor type for target" << targetName
                      << " cannot be used as a target." << std::endl;
            return false;
        }
    }

    return true;
}


bool iWearToiDyn::updateWearableSensorizedShoes(const wearable::IWear* iWearPtr,
                                                std::weak_ptr<WearableSensorizedShoe> shoe)
{
    constexpr auto printPrefix = "iWearToiDyn::updateWearableSensorizedShoes: ";
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!checkWearableSensorizedShoesiniWearInterface(iWearPtr, shoe))
    {
         std::cerr << printPrefix
                   << " Cannot find sensor in iWear interface." << std::endl;
        return false;
    }

    auto shoePtr = shoe.lock();
    auto wearableName = shoePtr->getWearableName();

    switch (iWearPtr->getSensor(wearableName)->getSensorType())
    {
        case wearable::sensor::SensorType::ForceTorque6DSensor:
        {
            auto sensor = iWearPtr->getForceTorque6DSensor(wearableName);
            if (!sensor)
            {
                std::cerr << printPrefix << "Sensor" << wearableName
                          << " has been added but is not properly configured." << std::endl;
                return false;
            }

            m_status = sensor->getSensorStatus();
            if (m_status != wearable::sensor::SensorStatus::Ok)
            {
                std::cerr << printPrefix << "Sensor status of " << wearableName
                          << " is not ok (" << static_cast<double>(m_status) << ")." << std::endl;
                return false;
            }

            if (sensor->getForceTorque6D(m_wrench))
            {
                iWearToiDynWrench(m_wrench, m_dynWrench);
                shoePtr->setWrench(m_dynWrench);;
            }
            else
            {
                std::cerr << printPrefix
                          << "Failed to read position from ForceTorque6DSensor: "
                          << wearableName << std::endl;
            }
            break;
        }
        default:
        {
            std::cerr << printPrefix
                      << "Sensor type for shoe " << wearableName
                      << " cannot be used for sensorized shoes." << std::endl;
            return false;
        }
    }
    return true;
}
