/**
 * @file WholeBodyKinematicsWrapper.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/WholeBodyKinematicsRemapper.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>

using namespace KinDynFusion;
using namespace KinDynFusion::Estimators;
using namespace KinDynFusion::msgs;

const std::string RemapperName = "WholeBodyKinematicsRemapper";
const std::string LogPrefix = RemapperName + " :";
constexpr double DefaultPeriod = 0.01;
constexpr double NetworkTimeout = 5.0;

class WholeBodyKinematicsRemapper::Impl
{
public:
    yarp::os::Network network;
    yarp::os::BufferedPort<WholeBodyKinematicsOutput> inputPort;
    bool terminationCall = false;

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
    std::vector<KinDynFusion::Estimators::EstimatedContactPosition > contactPosInterface;
};

WholeBodyKinematicsRemapper::WholeBodyKinematicsRemapper(double period,
                                                     yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock),
        pImpl{std::make_unique<WholeBodyKinematicsRemapper::Impl>()}
{
}

WholeBodyKinematicsRemapper::WholeBodyKinematicsRemapper()
        : yarp::os::PeriodicThread(DefaultPeriod, yarp::os::ShouldUseSystemClock::No),
        pImpl{std::make_unique<WholeBodyKinematicsRemapper::Impl>()}
{

}

WholeBodyKinematicsRemapper::~WholeBodyKinematicsRemapper()
{
}

bool WholeBodyKinematicsRemapper::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    if (!(config.check("wholeBodyKinematicsDataPort") &&
          config.find("wholeBodyKinematicsDataPort").isString())) {
        yError() << LogPrefix
                 << "wholeBodyKinematicsDataPort option does not exist or it is not a list";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    std::string wbkDataPortName = config.find("wholeBodyKinematicsDataPort").asString();

    // Initialize the network
    // TODO: is this required in every DeviceDriver?
    pImpl->network = yarp::os::Network();
    if (!yarp::os::Network::initialized() ||
        !yarp::os::Network::checkNetwork(NetworkTimeout)) {
        yError() << LogPrefix << "YARP server wasn't found active.";
        return false;
    }

    // ==========================
    // CONFIGURE INPUT DATA PORTS
    // ==========================
    yDebug() << LogPrefix << "Configuring input data ports";

    pImpl->inputPort.useCallback(*this);
    if (!pImpl->inputPort.open("...")) {
        yError() << LogPrefix << "Failed to open port" << wbkDataPortName;
        return false;
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << LogPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(wbkDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << LogPrefix << "Failed to connect " << wbkDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << LogPrefix << "Opened correctly";
    return true;
}


void WholeBodyKinematicsRemapper::run()
{
    return;
}


bool WholeBodyKinematicsRemapper::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void WholeBodyKinematicsRemapper::onRead(WholeBodyKinematicsOutput& out)
{
    if(!pImpl->terminationCall)
    {
        pImpl->baseNameInterface = out.baseName;
        pImpl->jointNamesInterface = out.jointNames;
        pImpl->jointPositionsInterface = out.positions;
        pImpl->jointVelocitiesInterface = out.velocities;

        pImpl->basePositionInterface = {out.baseOriginWRTGlobal.x,
                                        out.baseOriginWRTGlobal.y,
                                        out.baseOriginWRTGlobal.z};
        pImpl->baseOrientationInterface = {out.baseOrientationWRTGlobal.w,
                                           out.baseOrientationWRTGlobal.imaginary.x,
                                           out.baseOrientationWRTGlobal.imaginary.y,
                                           out.baseOrientationWRTGlobal.imaginary.z};
        pImpl->baseVelocityInterface = {out.baseVelocityWRTGlobal[0],
                                        out.baseVelocityWRTGlobal[1],
                                        out.baseVelocityWRTGlobal[2],
                                        out.baseVelocityWRTGlobal[3],
                                        out.baseVelocityWRTGlobal[4],
                                        out.baseVelocityWRTGlobal[5]};

        pImpl->lCoPPositionInterface = {out.lFootCoPPositionWRTLink.x,
                                        out.lFootCoPPositionWRTLink.y,
                                        out.lFootCoPPositionWRTLink.z};
        pImpl->rCoPPositionInterface = {out.rFootCoPPositionWRTLink.x,
                                        out.rFootCoPPositionWRTLink.y,
                                        out.rFootCoPPositionWRTLink.z};
        pImpl->globalCoPPositionInterface = {out.CoPPositionWRTGlobal.x,
                                            out.CoPPositionWRTGlobal.y,
                                            out.CoPPositionWRTGlobal.z};

        pImpl->contactInterface.resize(out.vertexContacts.size());
        for (std::size_t idx = 0; idx < out.vertexContacts.size(); idx++)
        {
            pImpl->contactInterface[idx].isActive = out.vertexContacts[idx].isActive;
            pImpl->contactInterface[idx].magnitude = out.vertexContacts[idx].magnitude;
            pImpl->contactInterface[idx].name = out.vertexContacts[idx].name;
            pImpl->contactInterface[idx].position = {out.vertexContacts[idx].positionWRTLink.x,
                                                     out.vertexContacts[idx].positionWRTLink.y,
                                                     out.vertexContacts[idx].positionWRTLink.z};
            pImpl->contactInterface[idx].linkName = out.vertexContacts[idx].linkName;
        }

        pImpl->contactNamesInterface = out.vertexContactNames;
        pImpl->lFootNameInterface = out.lFootLinkName;
        pImpl->rFootNameInterface = out.rFootLinkName;


        pImpl->lfOrientationInterface = {out.lfOrientationWRTGlobal.w,
                                         out.lfOrientationWRTGlobal.imaginary.x,
                                         out.lfOrientationWRTGlobal.imaginary.y,
                                         out.lfOrientationWRTGlobal.imaginary.z};

        pImpl->rfOrientationInterface = {out.rfOrientationWRTGlobal.w,
                                         out.rfOrientationWRTGlobal.imaginary.x,
                                         out.rfOrientationWRTGlobal.imaginary.y,
                                         out.rfOrientationWRTGlobal.imaginary.z};

        pImpl->contactPosInterface.resize(out.contactPositionWRTGlobal.size());
        for (std::size_t idx = 0; idx < out.contactPositionWRTGlobal.size(); idx++)
        {
            pImpl->contactPosInterface[idx].name = out.contactPositionWRTGlobal[idx].name;
            pImpl->contactPosInterface[idx].position = {out.contactPositionWRTGlobal[idx].positionWRTGlobal.x,
                                                        out.contactPositionWRTGlobal[idx].positionWRTGlobal.y,
                                                        out.contactPositionWRTGlobal[idx].positionWRTGlobal.z};
        }
    }
}

std::vector<std::string> WholeBodyKinematicsRemapper::getJointNames() const
{
    return pImpl->jointNamesInterface;
}
std::string WholeBodyKinematicsRemapper::getBaseName() const
{
    return pImpl->baseNameInterface;
}
std::size_t WholeBodyKinematicsRemapper::getNumberOfJoints() const
{
    return pImpl->jointNamesInterface.size();
}

std::vector<double> WholeBodyKinematicsRemapper::getJointPositions() const
{
    return pImpl->jointPositionsInterface;
}
std::vector<double> WholeBodyKinematicsRemapper::getJointVelocities() const
{
    return pImpl->jointVelocitiesInterface;
}

std::array<double, 3> WholeBodyKinematicsRemapper::getBasePosition() const
{
    return pImpl->basePositionInterface;
}

std::array<double, 4> WholeBodyKinematicsRemapper::getBaseOrientation() const
{
    return pImpl->baseOrientationInterface;
}

std::array<double, 6> WholeBodyKinematicsRemapper::getBaseVelocity() const
{
    return pImpl->baseVelocityInterface;
}

std::array<double, 3> WholeBodyKinematicsRemapper::getLeftFootCoPPosition()
{
    return pImpl->lCoPPositionInterface;
}

std::array<double, 3> WholeBodyKinematicsRemapper::getRightFootCoPPosition()
{
    return pImpl->rCoPPositionInterface;
}

std::vector<VertexContactData> WholeBodyKinematicsRemapper::getAllVertexContactData()
{
    return pImpl->contactInterface;
}

std::vector<std::string> WholeBodyKinematicsRemapper::getAllVertexContactNames()
{
    return pImpl->contactNamesInterface;
}

std::string WholeBodyKinematicsRemapper::getLeftFootLinkName() const
{
    return pImpl->lFootNameInterface;
}

std::string WholeBodyKinematicsRemapper::getRightFootLinkName() const
{
    return pImpl->rFootNameInterface;
}

std::vector<KinDynFusion::Estimators::EstimatedContactPosition> WholeBodyKinematicsRemapper::getContactPointsPosition()
{
    return pImpl->contactPosInterface;
}

std::array<double, 4> WholeBodyKinematicsRemapper::getLeftFootOrientation() const
{
    return pImpl->lfOrientationInterface;
}

std::array<double, 4> WholeBodyKinematicsRemapper::getRightFootOrientation() const
{
    return pImpl->rfOrientationInterface;
}

std::array<double, 3> WholeBodyKinematicsRemapper::getGlobalCoPPosition()
{
    return pImpl->globalCoPPositionInterface;
}

std::array<double, 3> WholeBodyKinematicsRemapper::getCoMPosition() const
{
    std::array<double, 3> ret;
    return ret;
}

std::array<double, 3> WholeBodyKinematicsRemapper::getCoMVelocity() const
{
    std::array<double, 3> ret;
    return ret;
}
