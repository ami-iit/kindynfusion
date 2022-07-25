/**
 * @file WholeBodyKinematicsDevice.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/WholeBodyKinematicsDevice.h>
#include <KinDynFusion/WholeBodyKinematicsYarpHelper.h>
#include <KinDynFusion/InverseKinematics/InverseKinematicsYarpHelper.h>

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

#include <yarp/os/LogStream.h>

using namespace KinDynFusion;
using namespace KinDynFusion::Estimators;
using namespace KinDynFusion::IK;
using namespace BipedalLocomotion::Contacts;

WholeBodyKinematicsDevice::WholeBodyKinematicsDevice(double period,
                                                     yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());
}

WholeBodyKinematicsDevice::WholeBodyKinematicsDevice()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());
}

WholeBodyKinematicsDevice::~WholeBodyKinematicsDevice()
{
}

bool WholeBodyKinematicsDevice::open(yarp::os::Searchable& config)
{
    auto params = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(config);
    if (params->getParameter("period", m_devicePeriod))
    {
        this->setPeriod(m_devicePeriod);
    }

    ModelData modelData;
    IKParameters ikParams;
    if (!KinDynFusion::IK::YarpHelper::configure(config, modelData, ikParams))
    {
        yError() << m_printPrefix
                 << " [open] Unable to load IK parameters from configuration file.";
        return false;
    }

    // need to initialize contact detector before initializing ik helper
    FootMetaData lFootData, rFootData;
    if (!WholeBodyKinematicsYarpHelper::setFeetDataFromConfig(config,
                                                              lFootData,
                                                              rFootData))
    {
        yError() << m_printPrefix
                 << " [open] Unable to load foot parameters from configuration file.";
        return false;
    }

    if (!m_wbk.initializeContactDetectors(lFootData,
                                          rFootData,
                                          modelData.model))
    {
        yError() << m_printPrefix
                 << " [open] Unable to initialize contact detection block.";
        return false;
    }

    if (!m_wbk.initializeIKHelper(modelData, ikParams))
    {
        yError() << m_printPrefix
                 << " [open] Unable to initialize IK block in WholeBodyKinematics.";
        return false;
    }

    if (!WholeBodyKinematicsYarpHelper::addTargetsFromConfig(config, m_wbk, m_printPrefix))
    {
        yError() << m_printPrefix
                 << " [open] Failed to add one or more target(s).";
        return false;
    }

    m_targetNames = m_wbk.getAllWearableTargetNames();

    // needs to be called after adding targets
    if (!WholeBodyKinematicsYarpHelper::setWearableCalibMatricesFromConfig(config, m_wbk, m_printPrefix))
    {
        yError() << m_printPrefix
                 << " [open] Failed to set calibration data for one or more target(s).";
        return false;
    }


    if (!WholeBodyKinematicsYarpHelper::addFTShoesFromConfig(config, m_wbk, m_printPrefix))
    {
        yError() << m_printPrefix
                 << " [open] Failed to add one or more FT Shoe(s).";
        return false;
    }

    m_shoeNames = m_wbk.getAllWearableShoeNames();

    if (!m_calibHelper.initialize(modelData))
    {
        yError() << m_printPrefix
                 << " [open] Failed to initialize calibration helper.";
        return false;
    }

    if (!m_wbk.initializeBaseEKF(params, modelData))
    {
        yError() << m_printPrefix
                 << " [open] Failed to initialize Base EKF.";
        return false;
    }

    if (!initializeRPCPort(config))
    {
        yError() << m_printPrefix
                 << " [open] Failed to initialize RPC Server.";
        return false;
    }

    m_solution.jPos.resize(m_wbk.getModelData().jointList.size());
    m_solution.jVel.resize(m_wbk.getModelData().jointList.size());

    return true;
}

bool WholeBodyKinematicsDevice::initializeRPCPort(yarp::os::Searchable& config)
{
    std::string rpcPortName;
    if (!(config.check("rpcPortPrefix") &&
          config.find("rpcPortPrefix").isString()))
    {
        rpcPortName = "/WholeBodyKinematicsDevice/rpc:i";
    }
    else
    {
        rpcPortName = "/" + config.find("rpcPortPrefix").asString() + "/WholeBodyKinematicsDevice/rpc:i";
    }

    if (!m_rpcPort.open(rpcPortName))
    {
        yError() << m_printPrefix
                 << " [initializeRPCPort] Unable to open rpc port " << rpcPortName;
        return false;
    }

    // Set rpc port reader
    m_rpcPort.setReader(m_commandPro);
    return true;
}

bool WholeBodyKinematicsDevice::attachAll(const yarp::dev::PolyDriverList & poly)
{
    bool broken{false};
    for (std::size_t i = 0; i < poly.size(); i++)
    {
        const yarp::dev::PolyDriverDescriptor* driver = poly[i];

        if (!driver)
        {
            yError() << m_printPrefix
                     << " [attachAll] Passed PolyDriverDescriptor is nullptr.";
            return false;
        }

        auto devName = driver->poly->getValue("device").asString();
        if (devName != "iwear_remapper")
        {
            continue;
        }

        if (!driver->poly->view(m_iWear) || !m_iWear)
        {
            yError() << m_printPrefix
                     << " [attachAll] Failed to view the IWear interface from the PolyDriver.";
            return false;
        }

        double waitTimeInSec{5};
        while (m_iWear->getStatus() == wearable::WearStatus::WaitingForFirstRead)
        {
            yInfo() << m_printPrefix
                    << " [attachAll] IWear interface waiting for first data. Waiting...";
            yarp::os::Time::delay(waitTimeInSec);
        }

        if (m_iWear->getStatus() != wearable::WearStatus::Ok)
        {
            yError() << m_printPrefix
                     << " [attachAll] The status of the attached IWear interface is not ok ("
                     << static_cast<int>(m_iWear->getStatus()) << ").";
            return false;
        }

        broken = true;
        break;
    }

    if (!broken)
    {
        yError() << m_printPrefix
                 << " [attachAll] Could not attach iwear_remapper."
                 << "Expecting to attach an iwear_remapper to publish all wearables data.";
        return false;
    }

    if (!checkAttachIMUNodes())
    {
        yError() << m_printPrefix
                 << " [attachAll] Failed to attach IMU nodes.";
        return false;
    }

    if (!checkAttachFTShoes())
    {
        yError() << m_printPrefix
                 << " [attachAll] Failed to attach FT shoes.";
        return false;
    }

    if (!this->start())
    {
        yError() << m_printPrefix
                 << " [attachAll] Failed to start the periodic thread.";
        return false;
    }
    return true;
}

bool WholeBodyKinematicsDevice::checkAttachIMUNodes()
{
    for (const auto& targetName : m_targetNames)
    {
        auto target = m_wbk.getWearableTarget(targetName);
        if (!m_iwear2idyn.checkWearableTargetiniWearInterface(m_iWear, target))
        {
            yError() << m_printPrefix
                     << " [checkAttachIMUNodes] One or more targets unavailable in iWear interface.";
            return false;
        }
    }

    yInfo() << m_printPrefix << "[checkAttachIMUNodes] Attach IMU nodes succesful.";
    return true;
}

bool WholeBodyKinematicsDevice::checkAttachFTShoes()
{
    for (const auto& shoeName : m_shoeNames)
    {
        auto shoe = m_wbk.getWearableSensorizedShoe(shoeName);
        if (!m_iwear2idyn.checkWearableSensorizedShoesiniWearInterface(m_iWear, shoe))
        {
            yError() << m_printPrefix
                     << " [checkAttachFTShoes] One or more shoes unavailable in iWear interface.";
            return false;
        }
    }

    yInfo() << m_printPrefix << "[checkAttachFTShoes] Attach FT shoes succesful.";
    return true;
}

void WholeBodyKinematicsDevice::run()
{
    // compute timestep
    double dt;
    if (m_lastTime < 0.0)
    {
        dt = m_devicePeriod;
    }
    else
    {
        dt = yarp::os::Time::now() - m_lastTime;
    }
    m_lastTime = yarp::os::Time::now();

    if (!updateTargetsData())
    {
        return;
    }

    if (!updateFTShoesData(m_lastTime))
    {
        return;
    }

    if (!updateBaseEKFInputs(m_lastTime))
    {
        return;
    }

    if (!m_wbk.advance(dt))
    {
        return;
    }

    updateWholeBodyKinematicsSolution();
    checkAndApplyRpcCommand();
}


bool WholeBodyKinematicsDevice::updateTargetsData()
{
    std::lock_guard<std::mutex> lock{m_deviceMutex};
    for (const auto& targetName: m_targetNames)
    {
        auto target = m_wbk.getWearableTarget(targetName);
        if (!m_iwear2idyn.updateWearableTargets(m_iWear, targetName, target))
        {
            yError() << m_printPrefix
                     << " [updateTargetsData] One or more targets could not be updated.";
            return false;
        }
    }

    m_wbk.updateIKTargets();

    return true;
}

bool WholeBodyKinematicsDevice::updateFTShoesData(const double& timeNow)
{
    std::lock_guard<std::mutex> lock{m_deviceMutex};
    for (const auto& targetName: m_shoeNames)
    {
        auto shoe = m_wbk.getWearableSensorizedShoe(targetName);
        if (!m_iwear2idyn.updateWearableSensorizedShoes(m_iWear, shoe))
        {
            yError() << m_printPrefix
                     << " [updateTargetsData] One or more shoes data could not be updated.";
            return false;
        }
    }

    if (!m_wbk.updateContactDetectors(timeNow))
    {
        return false;
    }

    return true;
}

bool WholeBodyKinematicsDevice::updateBaseEKFInputs(const double& timeNow)
{
    if (!m_wbk.updateBaseEKF(timeNow))
    {
        yError() << m_printPrefix
                 << " [updateBaseEKFInputs] Base EKF inputs could not be updated.";
        return false;
    }

    return true;
}


void WholeBodyKinematicsDevice::updateWholeBodyKinematicsSolution()
{
    std::lock_guard<std::mutex> lock{m_deviceMutex};
    auto basePos = m_wbk.getBasePose().getPosition();
    for (auto idx = 0; idx < 3; idx++)
    {
        m_solution.basePos[idx] = basePos.getVal(idx);
    }

    auto baseQuat = m_wbk.getBasePose().getRotation().asQuaternion();
    auto lfQuat = m_wbk.getLeftFootOrientation();
    auto rfQuat = m_wbk.getRightFootOrientation();
    for (auto idx = 0; idx < 4; idx++)
    {
        m_solution.baseOrient[idx] = baseQuat.getVal(idx);
        m_solution.lfOrient[idx] = lfQuat.getVal(idx);
        m_solution.rfOrient[idx] = rfQuat.getVal(idx);
    }

    auto jPos = m_wbk.getJointConfiguration();
    auto jVel = m_wbk.getJointVelocities();
    for (auto idx = 0; idx < jPos.size(); idx++)
    {
        m_solution.jPos[idx] = jPos.getVal(idx);
        m_solution.jVel[idx] = jVel.getVal(idx);
    }

    auto baseVel = m_wbk.getBaseVelocity();
    for (auto idx = 0; idx < 6; idx++)
    {
        m_solution.baseVel[idx] = baseVel.getVal(idx);
    }

    auto contactPositions = m_wbk.getAllContactFramePositions();
    m_solution.estContactPos.clear();
    for (const auto& [name, pos] : contactPositions)
    {
        EstimatedContactPosition cpos;
        cpos.name = name;
        for (auto idx = 0; idx < 3; idx++)
        {
            cpos.position[idx] = pos.getVal(idx);
        }
        m_solution.estContactPos.emplace_back(cpos);
    }

    iDynTree::Position lCoPInLeftFoot, rCoPInLeftFoot;
    m_wbk.getLeftFootCenterOfPressure(lCoPInLeftFoot);
    m_wbk.getRightFootCenterOfPressure(rCoPInLeftFoot);

    iDynTree::Position globalCoP;
    m_wbk.getGlobalCenterOfPressure(globalCoP);

    for (auto idx = 0; idx < 3; idx++)
    {
        m_solution.lCoPPos[idx] = lCoPInLeftFoot.getVal(idx);
        m_solution.rCoPPos[idx] = rCoPInLeftFoot.getVal(idx);
        m_solution.globalCoPPos[idx] = globalCoP.getVal(idx);
    }

    auto lfFrames = m_wbk.getLeftFootCandidateContactFrames();
    auto rfFrames = m_wbk.getRightFootCandidateContactFrames();
    m_solution.contacts.clear();
    for (const auto& frame : lfFrames)
    {
        iDynTree::Position vertexPos;
        VertexContactData vertex;
        m_wbk.getLeftFootVertexData(frame,
                                    vertex.linkName,
                                    vertex.magnitude,
                                    vertex.isActive,
                                    vertexPos);
        vertex.name = frame;
        for (auto idx = 0; idx < 3; idx++)
        {
            vertex.position[idx] = vertexPos.getVal(idx);
        }
        m_solution.contacts.emplace_back(vertex);
    }

    for (const auto& frame : rfFrames)
    {
        iDynTree::Position vertexPos;
        VertexContactData vertex;
        m_wbk.getRightFootVertexData(frame,
                                    vertex.linkName,
                                    vertex.magnitude,
                                    vertex.isActive,
                                    vertexPos);
        vertex.name = frame;
        for (auto idx = 0; idx < 3; idx++)
        {
            vertex.position[idx] = vertexPos.getVal(idx);
        }
        m_solution.contacts.emplace_back(vertex);
    }
}


bool WholeBodyKinematicsDevice::detachAll()
{
    while (isRunning())
    {
        stop();
    }

    m_iWear = nullptr;
    return true;
}

bool WholeBodyKinematicsDevice::close()
{
    yInfo() << m_printPrefix
            << "[close] Closing WholeBodyKinematicsDevice thread.";
    return true;
}

void WholeBodyKinematicsDevice::CmdParser::resetInternalVariables()
{
    parentLinkName = "";
    childLinkName = "";
    cmdStatus = rpcCommand::empty;
}

bool WholeBodyKinematicsDevice::CmdParser::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle command, response;
    if (command.read(connection))
    {
        if (command.get(0).asString() == "help")
        {
            response.addVocab32(yarp::os::Vocab32::encode("many"));
            response.addString("The following commands can be used to apply a secondary calibration assuming the subject is in the zero configuration of the model for the calibrated links. \n");
            response.addString("Enter <calibrateAll> to apply a secondary calibration for all the targets using the measured base pose \n");
            response.addString("Enter <calibrateAllWithWorld <refTarget>> to apply a secondary calibration for all the targets assuming the <refTarget> to be in the world origin \n");
            response.addString("Enter <calibrateAllWorldYaw> to remove the yaw offset for all the data \n");
            response.addString("Enter <setRotationOffset <targetName> <r p y [deg]>> to apply a secondary calibration for the given target using the given rotation offset (defined using rpy)\n");
            response.addString("Enter <reset <targetName>> to remove secondary calibration for the given target \n");
            response.addString("Enter <resetAll> to remove all the secondary calibrations");
            response.addString("Enter <startBaseEstimator> to start the base estimator");
            response.addString("Enter <calibrateBaseEstimatorWithWorld <refLink>> to set inertial frame transform for the base estimator");
        }
        else if (command.get(0).asString() == "calibrateAllWorldYaw")
        {
            this->parentLinkName = "";
            response.addString("Entered command <calibrateAllWorldYaw> is correct, trying to set yaw calibration for all the targets");
            this->cmdStatus = rpcCommand::calibrateAllWorldYaw;
        }
        else if (command.get(0).asString() == "calibrateAll")
        {
            this->parentLinkName = "";
            response.addString("Entered command <calibrateAll> is correct, trying to set offset calibration for all the targets");
            this->cmdStatus = rpcCommand::calibrateAll;
        }
        else if (command.get(0).asString() == "calibrateAllWithWorld")
        {
            this->parentLinkName = "";
            this->refLinkName = command.get(1).asString();
            response.addString("Entered command <calibrateAllWithWorld> is correct, trying to set offset calibration for all the targets, and setting target " + this->refLinkName + " to the origin");
            this->cmdStatus = rpcCommand::calibrateAllWithWorld;
        }
        else if (command.get(0).asString() == "setRotationOffset" &&
                !command.get(1).isNull() &&
                 command.get(2).isFloat64() &&
                 command.get(3).isFloat64() &&
                 command.get(4).isFloat64())
        {
            this->parentLinkName = command.get(1).asString();
            this->roll = command.get(2).asFloat64();
            this->pitch = command.get(3).asFloat64();
            this->yaw = command.get(4).asFloat64();
            response.addString("Entered command <calibrate> is correct, trying to set rotation offset for the target " + this->parentLinkName);
            this->cmdStatus = rpcCommand::setRotationOffset;
        }
        else if (command.get(0).asString() == "resetAll")
        {
            response.addString("Entered command <resetAll> is correct,  trying to remove calibration transforms (right and left) for all the targets");
            this->cmdStatus = rpcCommand::resetAll;
        }
        else if (command.get(0).asString() == "reset" &&
                !command.get(1).isNull())
        {
            this->parentLinkName = command.get(1).asString();
            response.addString("Entered command <reset> is correct, trying to remove calibration transforms (right and left) for the target " + this->parentLinkName);
            this->cmdStatus = rpcCommand::resetCalibration;
        }
        else if (command.get(0).asString() == "startBaseEstimator")
        {
            this->parentLinkName = "";
            response.addString("Entered command <startBaseEstimator> is correct, trying to start the Base EKF.");
            this->cmdStatus = rpcCommand::startBaseEstimator;
        }
        else if (command.get(0).asString() == "calibrateBaseEstimatorWithWorld")
        {
            this->parentLinkName = "";
            this->refLinkName = command.get(1).asString();
            response.addString("Entered command <calibrateBaseEstimatorWithWorld> is correct, trying to reset Base EKF in reference to " + this->refLinkName);
            this->cmdStatus = rpcCommand::calibrateBaseEstimatorWithWorld;
        }
        else
        {
            response.addString(
                "Entered command is incorrect. Enter help to know available commands");
        }
    }
    else
    {
        yInfo() << "read rpc resetInternalVariables";
        resetInternalVariables();
        return false;
    }

    yarp::os::ConnectionWriter* reply = connection.getWriter();
    if (reply != NULL)
    {
        response.write(*reply);
    }
    else
    {
        return false;
    }

    return true;
}

void WholeBodyKinematicsDevice::checkAndApplyRpcCommand()
{
    // Check for rpc command status and apply command
    if (m_commandPro.cmdStatus != rpcCommand::empty)
    {
        if (!applyRpcCommand())
        {
            yWarning() << m_printPrefix
                       << "Failed to execute the rpc command";
        }

        // reset the rpc internal status
        {
            std::lock_guard<std::mutex> lock(m_deviceMutex);
            m_commandPro.resetInternalVariables();
        }
    }
}

bool WholeBodyKinematicsDevice::applyRpcCommand()
{

    auto wearableTargets = m_wbk.getAllWearableTargets();
    {
        m_calibHelper.setConfiguration(m_wbk.getBasePose(),
                                       m_wbk.getJointConfiguration());
    }

    switch(m_commandPro.cmdStatus)
    {
    case rpcCommand::resetAll:
    {
        m_calibHelper.resetAll(wearableTargets);
        break;
    }
    case rpcCommand::resetCalibration:
    {
        auto& targetName = m_commandPro.parentLinkName;
        m_calibHelper.resetCalibration(targetName, wearableTargets);
        break;
    }
    case rpcCommand::calibrateAll:
    {
        return m_calibHelper.calibrateAll(wearableTargets);
        break;
    }
    case rpcCommand::calibrateAllWithWorld:
    {
        auto& targetName = m_commandPro.refLinkName;
        return m_calibHelper.calibrateAllWithWorld(targetName, wearableTargets);
        break;
    }
    case rpcCommand::calibrateAllWorldYaw:
    {
        return m_calibHelper.calibrateAllWorldYaw(wearableTargets);
        break;
    }
    case rpcCommand::setRotationOffset:
    {
        auto& targetName = m_commandPro.parentLinkName;
        return m_calibHelper.setRotationOffset(targetName,
                                               m_commandPro.roll,
                                               m_commandPro.pitch,
                                               m_commandPro.yaw,
                                               wearableTargets);
        break;
    }
    case rpcCommand::startBaseEstimator:
    {
        m_wbk.startBaseEKF();
        return true;
        break;
    }
    case rpcCommand::calibrateBaseEstimatorWithWorld:
    {
        auto& targetName = m_commandPro.refLinkName;
        return m_wbk.resetBaseEKFWorldFrameTo(targetName);
        break;
    }
    default:
    {
        yError() << m_printPrefix
                 << " [applyRpcCommand] invalid command";
    }
    }
    return true;
}

WearableSensorizedShoeMap WholeBodyKinematicsDevice::getAllWearableSensorizedShoes()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getAllWearableSensorizedShoes();
}

std::vector<std::string> WholeBodyKinematicsDevice::getAllWearableShoeNames()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getAllWearableShoeNames();
}


std::vector<KinDynFusion::IK::TargetName> WholeBodyKinematicsDevice::getAllWearableTargetNames()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getAllWearableTargetNames();
}


WearableSensorTargetMap WholeBodyKinematicsDevice::getAllWearableTargets()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getAllWearableTargets();
}

WearableSensorizedShoePtr WholeBodyKinematicsDevice::getWearableSensorizedShoe(const std::string& name)
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getWearableSensorizedShoe(name);
}

WearableSensorTargetPtr WholeBodyKinematicsDevice::getWearableTarget(const KinDynFusion::IK::TargetName& name)
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getWearableTarget(name);
}

std::string WholeBodyKinematicsDevice::getBaseName() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getModelData().baseFrame;
}


std::array<double, 3> WholeBodyKinematicsDevice::getBasePosition() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.basePos;
}

std::array<double, 4> WholeBodyKinematicsDevice::getBaseOrientation() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.baseOrient;
}

std::array<double, 6> WholeBodyKinematicsDevice::getBaseVelocity() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.baseVel;
}

std::vector<KinDynFusion::Estimators::EstimatedContactPosition> WholeBodyKinematicsDevice::getContactPointsPosition()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.estContactPos;
}

std::array<double, 4> WholeBodyKinematicsDevice::getLeftFootOrientation() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.lfOrient;
}

std::array<double, 4> WholeBodyKinematicsDevice::getRightFootOrientation() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.rfOrient;
}

std::vector<KinDynFusion::Estimators::VertexContactData> WholeBodyKinematicsDevice::getAllVertexContactData()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.contacts;
}

std::array<double, 3> WholeBodyKinematicsDevice::getLeftFootCoPPosition()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.lCoPPos;
}

std::array<double, 3> WholeBodyKinematicsDevice::getRightFootCoPPosition()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.rCoPPos;
}

std::array<double, 3> WholeBodyKinematicsDevice::getGlobalCoPPosition()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.globalCoPPos;
}

std::size_t WholeBodyKinematicsDevice::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getModelData().model.getNrOfDOFs();
}

std::vector<std::string> WholeBodyKinematicsDevice::getJointNames() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getModelData().jointList;
}

std::vector<double> WholeBodyKinematicsDevice::getJointPositions() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.jPos;
}

std::vector<double> WholeBodyKinematicsDevice::getJointVelocities() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_solution.jVel;
}


std::vector<std::string> WholeBodyKinematicsDevice::getAllVertexContactNames()
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getCandidateContactFrames();
}

std::string WholeBodyKinematicsDevice::getLeftFootLinkName() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getLeftFootLinkName();
}

std::string WholeBodyKinematicsDevice::getRightFootLinkName() const
{
    std::lock_guard<std::mutex> guard{m_deviceMutex};
    return m_wbk.getRightFootLinkName();
}

std::array<double, 3> WholeBodyKinematicsDevice::getCoMPosition() const
{
    std::array<double, 3> ret;
    return ret;
}

std::array<double, 3> WholeBodyKinematicsDevice::getCoMVelocity() const
{
    std::array<double, 3> ret;
    return ret;
}
