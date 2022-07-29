/**
 * @file WBKHumanStateWrapper.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/WBKHumanStateWrapper.h>
//#include <KinDynFusion/msgs/WholeBodyKinematicsOutput.h>
#include <KinDynFusion/FloatingBaseEstimators/IWholeBodyKinematics.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <hde/msgs/HumanState.h>

using namespace KinDynFusion;
using namespace KinDynFusion::Estimators;
//using namespace KinDynFusion::msgs;

const std::string DeviceName = "WBKHumanStateWrapper";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

class WBKHumanStateWrapper::Impl
{
public:
    IWholeBodyKinematics* iWBK{nullptr};
    yarp::os::BufferedPort<hde::msgs::HumanState> outputPort;

    // buffer variables
    std::array<double, 3> lCoPPositionInterface;
    std::array<double, 3> rCoPPositionInterface;
    std::array<double, 3> globalCoPPositionInterface;
    std::array<double, 3> basePositionInterface;
    std::array<double, 4> baseOrientationInterface;
    std::array<double, 6> baseVelocityInterface;
    std::vector<double> jointPositionsInterface;
    std::vector<double> jointVelocitiesInterface;
    std::vector<std::string> jointNamesInterface;
    std::string baseNameInterface;
    std::string lFootNameInterface;
    std::string rFootNameInterface;
    std::vector<VertexContactData> contactInterface;
    std::vector<std::string> contactNamesInterface;

    std::array<double, 4> lfOrientationInterface;
    std::array<double, 4> rfOrientationInterface;
    std::vector<KinDynFusion::Estimators::EstimatedContactPosition> contactPosInterface;
};

WBKHumanStateWrapper::WBKHumanStateWrapper(double period,
                                                     yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock),
        pImpl{std::make_unique<WBKHumanStateWrapper::Impl>()}
{
}

WBKHumanStateWrapper::WBKHumanStateWrapper()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No),
        pImpl{std::make_unique<WBKHumanStateWrapper::Impl>()}
{

}

WBKHumanStateWrapper::~WBKHumanStateWrapper()
{
}

bool WBKHumanStateWrapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("outputPort") && config.find("outputPort").isString())) {
        yError() << LogPrefix << "outputPort option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    std::string outputPortName = config.find("outputPort").asString();

    // =============
    // OPEN THE PORT
    // =============

    if (!pImpl->outputPort.open(outputPortName)) {
        yError() << LogPrefix << "Failed to open port" << outputPortName;
        return false;
    }

    // ================
    // SETUP THE THREAD
    // ================

    setPeriod(period);

    return true;
}


void WBKHumanStateWrapper::run()
{
    // Get data from the interface
    pImpl->baseNameInterface = pImpl->iWBK->getBaseName();
    pImpl->jointNamesInterface = pImpl->iWBK->getJointNames();
    pImpl->jointPositionsInterface = pImpl->iWBK->getJointPositions();
    pImpl->jointVelocitiesInterface = pImpl->iWBK->getJointVelocities();

    pImpl->basePositionInterface = pImpl->iWBK->getBasePosition();
    pImpl->baseOrientationInterface = pImpl->iWBK->getBaseOrientation();
    pImpl->baseVelocityInterface = pImpl->iWBK->getBaseVelocity();

    pImpl->lCoPPositionInterface = pImpl->iWBK->getLeftFootCoPPosition();
    pImpl->rCoPPositionInterface = pImpl->iWBK->getRightFootCoPPosition();
    pImpl->globalCoPPositionInterface = pImpl->iWBK->getGlobalCoPPosition();

    pImpl->contactInterface = pImpl->iWBK->getAllVertexContactData();
    pImpl->contactNamesInterface = pImpl->iWBK->getAllVertexContactNames();

    pImpl->lFootNameInterface = pImpl->iWBK->getLeftFootLinkName();
    pImpl->rFootNameInterface = pImpl->iWBK->getRightFootLinkName();

    pImpl->contactPosInterface = pImpl->iWBK->getContactPointsPosition();
    pImpl->lfOrientationInterface = pImpl->iWBK->getLeftFootOrientation();
    pImpl->rfOrientationInterface = pImpl->iWBK->getRightFootOrientation();

    // prepare the message
    hde::msgs::HumanState& data = pImpl->outputPort.prepare();
    data.jointNames.resize(pImpl->jointNamesInterface.size());
    for (std::size_t idx = 0; idx < pImpl->jointNamesInterface.size(); idx++)
    {
        data.jointNames[idx] = pImpl->jointNamesInterface[idx];
    }

    data.positions.resize(pImpl->jointPositionsInterface.size());
    for (std::size_t idx = 0; idx < pImpl->jointPositionsInterface.size(); idx++)
    {
        data.positions[idx] = pImpl->jointPositionsInterface[idx];
    }

    data.velocities.resize(pImpl->jointVelocitiesInterface.size());
    for (std::size_t idx = 0; idx < pImpl->jointVelocitiesInterface.size(); idx++)
    {
        data.velocities[idx] = pImpl->jointVelocitiesInterface[idx];
    }

    data.baseName = pImpl->baseNameInterface;
    data.baseOriginWRTGlobal = {pImpl->basePositionInterface[0],
                                pImpl->basePositionInterface[1],
                                pImpl->basePositionInterface[2]};

    data.baseOrientationWRTGlobal = { pImpl->baseOrientationInterface[0],
                                     {pImpl->baseOrientationInterface[1],
                                      pImpl->baseOrientationInterface[2],
                                      pImpl->baseOrientationInterface[3]} };
    data.baseVelocityWRTGlobal.resize(6);
    for (std::size_t idx = 0; idx < 6; idx++)
    {
        data.baseVelocityWRTGlobal[idx] = pImpl->baseVelocityInterface[idx];
    }

    // Send the data
    pImpl->outputPort.write(/*forceStrict=*/true);
}

bool WBKHumanStateWrapper::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    yarp::dev::PolyDriver* poly = driver->poly;

    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iWBK || !poly->view(pImpl->iWBK) || !pImpl->iWBK) {
        yError() << LogPrefix << "Failed to view the IWholeBodyKinematics interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    yInfo() << pImpl->iWBK->getNumberOfJoints() << " "
            << pImpl->iWBK->getJointNames().size();

    for (int i = 0; i < pImpl->iWBK->getJointNames().size(); i++) {
        yInfo() << "Joint name (" << i << "): " << pImpl->iWBK->getJointNames()[i];
    }

    if (pImpl->iWBK->getNumberOfJoints() == 0
        || pImpl->iWBK->getNumberOfJoints() != pImpl->iWBK->getJointNames().size()) {
        yError() << "The IWholeBodyKinematics interface might not be ready";
        return false;
    }

    yDebug() << LogPrefix << "Read" << pImpl->iWBK->getNumberOfJoints() << "joints";

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop";
        return false;
    }

    return true;
}

bool WBKHumanStateWrapper::detachAll()
{
    while (isRunning())
    {
        stop();
    }

    while (!pImpl->outputPort.isClosed()) {
        pImpl->outputPort.close();
    }
    pImpl->iWBK = nullptr;

    return true;
}

bool WBKHumanStateWrapper::close()
{
    return true;
}

