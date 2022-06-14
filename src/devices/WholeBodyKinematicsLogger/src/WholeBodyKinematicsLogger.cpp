/**
 * @file WholeBodyKinematicsLogger.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/WholeBodyKinematicsLogger.h>

#include <KinDynFusion/InverseKinematics/InverseKinematicsYarpHelper.h>
#include <KinDynFusion/FloatingBaseEstimators/IWholeBodyKinematics.h>
#include <hde/interfaces/IHumanState.h>

#include <yarp/os/LogStream.h>
#include <yarp/dev/IFrameTransform.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <yarp/telemetry/experimental/BufferManager.h>

using namespace KinDynFusion;
using namespace KinDynFusion::Estimators;
using namespace KinDynFusion::IK;

const std::string DeviceName = "WholeBodyKinematicsLogger";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

class WholeBodyKinematicsLogger::Impl
{
public:
    bool setupLogger();
    bool initializeLogger();

    IWholeBodyKinematics* iWBK{nullptr};
    std::string wbkDataPortName;
    yarp::dev::PolyDriver remapperDevice;

    bool logIHumanState{false};
    hde::interfaces::IHumanState* iHumanState{nullptr};
    std::string hspDataPortName;
    yarp::dev::PolyDriver humanStateClient;

    double devicePeriod;
    double telemetryAutosavePeriod;
    double timeNow;

    bool logGroundTruth{false};
    yarp::dev::IFrameTransform* iTF{nullptr};
    std::string tfPortName;
    std::string groundTruthRefFrame;
    std::string baseTrackerName;
    yarp::dev::PolyDriver transformClient;

    yarp::telemetry::experimental::BufferManager<double> bufferManager;

    // buffer variables
    std::array<double, 3> basePositionInterface;
    std::array<double, 4> baseOrientationInterface;
    std::array<double, 6> baseVelocityInterface;
    std::array<double, 3> lCoPPositionInterface;
    std::array<double, 3> rCoPPositionInterface;
    std::array<double, 3> globalCoPPositionInterface;
    std::vector<double> jointPositionsInterface;
    std::vector<double> jointVelocitiesInterface;
    std::vector<std::string> jointNamesInterface;
    std::vector<VertexContactData> contactDataInterface;
    std::array<double, 4> lfOrientationInterface;
    std::array<double, 4> rfOrientationInterface;
    std::vector<KinDynFusion::Estimators::EstimatedContactPosition > contactPosInterface;

    std::string baseNameInterface;
    iDynTree::Vector4 baseOrientationQuaternion, lfOrientationQuaternion, rfOrientationQuaternion;
    iDynTree::Rotation wRb, wRlf, wRrf;
    iDynTree::Vector3 wRbRPY, wRlfRPY, wRrfRPY;
    std::vector<std::string> vertexContactNamesInterface;


    std::array<double, 3> hspBasePositionInterface;
    std::array<double, 4> hspBaseOrientationInterface;
    std::array<double, 6> hspBaseVelocityInterface;
    std::vector<double> hspJointPositionsInterface;
    std::vector<double> hspJointVelocitiesInterface;
    std::vector<std::string> hspJointNamesInterface;
    iDynTree::Vector4 hspBaseOrientationQuaternion;
    iDynTree::Rotation hspWRb;
    iDynTree::Vector3 hspWRbRPY;

    yarp::sig::Matrix groundTruthBasePoseYarp;
    iDynTree::Transform groundTruthBasePoseDyn;
    iDynTree::Vector3 groundTruthwRbRPY, groundTruthwPb;
};

WholeBodyKinematicsLogger::WholeBodyKinematicsLogger(double period,
                                                     yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock),
        pImpl{std::make_unique<WholeBodyKinematicsLogger::Impl>()}
{
}

WholeBodyKinematicsLogger::WholeBodyKinematicsLogger()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No),
        pImpl{std::make_unique<WholeBodyKinematicsLogger::Impl>()}
{

}

WholeBodyKinematicsLogger::~WholeBodyKinematicsLogger()
{
}

bool WholeBodyKinematicsLogger::open(yarp::os::Searchable& config)
{
    KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config, "period", pImpl->devicePeriod, 0.02, false);
    this->setPeriod(pImpl->devicePeriod);

    KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                     "telemetry_autosave_period",
                                                     pImpl->telemetryAutosavePeriod,
                                                     /*defaultValue=*/600.0,
                                                     /*required=*/false);

    if (!KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                          std::string{"wholeBodyKinematicsDataPortName"},
                                                          pImpl->wbkDataPortName,
                                                          /*defaultValue=*/std::string{"/WholeBodyKinematicsOutput/data:o"},
                                                          /*required=*/true))
    {
        yError() << LogPrefix
                 << " [open] Unable to load parameter \"wholeBodyKinematicsDataPortName\" from configuration file.";
        return false;
    }


    KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                     "logIHumanState",
                                                     pImpl->logIHumanState,
                                                     /*defaultValue=*/false,
                                                     /*required=*/false);
    if (pImpl->logIHumanState)
    {
        if (!KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                            std::string{"humanStateProviderDataPortName"},
                                                            pImpl->hspDataPortName,
                                                            /*defaultValue=*/std::string{"/HumanStateProvider/data:o"},
                                                            /*required=*/true))
        {
            yError() << LogPrefix
                    << " [open] Unable to load parameter \"humanStateProviderDataPortName\" from configuration file.";
            return false;
        }
    }

    KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                     "logGroundTruth",
                                                     pImpl->logGroundTruth,
                                                     /*defaultValue=*/false,
                                                     /*required=*/false);
    if (pImpl->logGroundTruth)
    {
        if (!KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                            std::string{"groundTruthFrameTransformPortName"},
                                                            pImpl->tfPortName,
                                                            /*defaultValue=*/std::string{"/transformServer/transforms:o"},
                                                            /*required=*/true))
        {
            yError() << LogPrefix
                    << " [open] Unable to load parameter \"groundTruthFrameTransformPortName\" from configuration file.";
            return false;
        }

        if (!KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                            std::string{"groundTruthReferenceFrame"},
                                                            pImpl->groundTruthRefFrame,
                                                            /*defaultValue=*/std::string{"world"},
                                                            /*required=*/true))
        {
            yError() << LogPrefix
                    << " [open] Unable to load parameter \"groundTruthReferenceFrame\" from configuration file.";
            return false;
        }

        if (!KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                            std::string{"grountTruthBaseTrackerName"},
                                                            pImpl->baseTrackerName,
                                                            /*defaultValue=*/std::string{"base_link"},
                                                            /*required=*/true))
        {
            yError() << LogPrefix
                    << " [open] Unable to load parameter \"grountTruthBaseTrackerName\" from configuration file.";
            return false;
        }
    }

    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<< LogPrefix <<"[main] Unable to find YARP network";
        return false;
    }

    // open WholeBodyKinematicsRemapper
    yarp::os::Property remapperOptions;
    remapperOptions.put("device", "WholeBodyKinematicsRemapper");
    remapperOptions.put("wholeBodyKinematicsDataPort", pImpl->wbkDataPortName);

    if(!pImpl->remapperDevice.open(remapperOptions))
    {
        yError() << LogPrefix << "Failed to connect remapper device";
        return false;
    }

    pImpl->iWBK = nullptr;
    pImpl->remapperDevice.view(pImpl->iWBK);
    if (!pImpl->iWBK) {
        yError() << LogPrefix << "Failed to view the IWholeBodyKinematics interface from the remapper";
        return false;
    }

    double waitSecondsBeforePortHasData{1.0};
    yarp::os::Time::delay(waitSecondsBeforePortHasData);
    // ===================
    // CHECK THE INTERFACE
    // ===================
    // wait for the IWholeBodyKinematics to be initialized
    while (pImpl->iWBK->getBaseName().empty())
    {
        yarp::os::Time::delay(waitSecondsBeforePortHasData);
        yInfo() << LogPrefix << "Waiting for data from WholeBodyKinematicsRemapper";
    }

    // compare the IWholeBodyKinematics base and joint names with the visualization model
    yInfo() << LogPrefix << "Whole Body Kinematics Interface providing data for the base link [ " << pImpl->iWBK->getBaseName() << " ]";

    yInfo() << pImpl->iWBK->getNumberOfJoints() << " "
            << pImpl->iWBK->getJointNames().size();

    pImpl->jointNamesInterface = pImpl->iWBK->getJointNames();
    for (int i = 0; i < pImpl->jointNamesInterface.size(); i++) {
        yInfo() << "Joint name (" << i << "): " << pImpl->jointNamesInterface[i];
    }

    if (pImpl->iWBK->getNumberOfJoints() == 0
        || pImpl->iWBK->getNumberOfJoints() != pImpl->jointNamesInterface.size()) {
        yError() << "The IWholeBodyKinematics interface might not be ready";
        return false;
    }
    pImpl->vertexContactNamesInterface = pImpl->iWBK->getAllVertexContactNames();

    pImpl->bufferManager.setDescriptionList(pImpl->jointNamesInterface);

    if (pImpl->logIHumanState)
    {
        // open human state wrapper
        yarp::os::Property humanClientOptions;
        humanClientOptions.put("device", "human_state_remapper");
        humanClientOptions.put("humanStateDataPort", pImpl->hspDataPortName);

        if(!pImpl->humanStateClient.open(humanClientOptions))
        {
            yError() << LogPrefix << "Failed to connect human state client device";
            return false;
        }

        pImpl->iHumanState = nullptr;
        pImpl->humanStateClient.view(pImpl->iHumanState);
        if (!pImpl->iHumanState) {
            yError() << LogPrefix << "Failed to view the iHumanState interface from the human state wrapper";
            return false;
        }

        double waitSecondsBeforePortHasData{1.0};
        yarp::os::Time::delay(waitSecondsBeforePortHasData);
        // ===================
        // CHECK THE INTERFACE
        // ===================
        // wait for the IWholeBodyKinematics to be initialized
        while (pImpl->iHumanState->getBaseName().empty())
        {
            yarp::os::Time::delay(waitSecondsBeforePortHasData);
            yInfo() << LogPrefix << "Waiting for data from human state wrapper";
        }

        yInfo() << LogPrefix << "Human State Interface providing data for the base link [ " << pImpl->iHumanState->getBaseName() << " ]";

        pImpl->hspJointNamesInterface = pImpl->iHumanState->getJointNames();
        yInfo() << pImpl->iHumanState->getNumberOfJoints() << " "
                << pImpl->hspJointNamesInterface.size();

        for (int i = 0; i < pImpl->hspJointNamesInterface.size(); i++) {
            yInfo() << "Joint name (" << i << "): " << pImpl->hspJointNamesInterface[i];
        }

        if (pImpl->iHumanState->getNumberOfJoints() == 0
            || pImpl->iHumanState->getNumberOfJoints() != pImpl->hspJointNamesInterface.size()) {
            yError() << "The iHumanState interface might not be ready";
            return false;
        }
    }

    // Open TF client
    yarp::os::Property tfOptions;
    tfOptions.put("device", "transformClient");
    tfOptions.put("remote", pImpl->tfPortName);
    tfOptions.put("local", "/WBKlogger/transformClient");

    if(!pImpl->transformClient.open(tfOptions))
    {
        yError() << LogPrefix << "Failed to connect transform client";
        return false;
    }

    pImpl->iTF = nullptr;
    pImpl->transformClient.view(pImpl->iTF);
    if (!pImpl->iTF) {
        yError() << LogPrefix << "Failed to view the IFrameTransform interface from the tfClient";
        return false;
    }


    if (!pImpl->setupLogger())
    {
        yError() << LogPrefix << "Failed to configure telemetry logger.";
        return false;
    }

    if (!pImpl->initializeLogger())
    {
        yError() << LogPrefix << "Failed to initialize telemetry logger.";
        return false;
    }

    // start the logging thread
    this->start();

    return true;
}


void WholeBodyKinematicsLogger::run()
{
    // Get data from the interface
    pImpl->timeNow = yarp::os::Time::now();
    pImpl->basePositionInterface = pImpl->iWBK->getBasePosition();
    pImpl->baseOrientationInterface = pImpl->iWBK->getBaseOrientation();
    pImpl->baseVelocityInterface = pImpl->iWBK->getBaseVelocity();
    pImpl->jointPositionsInterface = pImpl->iWBK->getJointPositions();
    pImpl->jointVelocitiesInterface = pImpl->iWBK->getJointVelocities();
    pImpl->lCoPPositionInterface = pImpl->iWBK->getLeftFootCoPPosition();
    pImpl->rCoPPositionInterface = pImpl->iWBK->getRightFootCoPPosition();
    pImpl->globalCoPPositionInterface = pImpl->iWBK->getGlobalCoPPosition();

    pImpl->contactPosInterface = pImpl->iWBK->getContactPointsPosition();
    pImpl->lfOrientationInterface = pImpl->iWBK->getLeftFootOrientation();
    pImpl->rfOrientationInterface = pImpl->iWBK->getRightFootOrientation();

    for (std::size_t idx = 0; idx < 4; idx++)
    {
        pImpl->baseOrientationQuaternion.setVal(idx, pImpl->baseOrientationInterface[idx]);
    }

    pImpl->wRb.fromQuaternion(pImpl->baseOrientationQuaternion);
    pImpl->wRbRPY = pImpl->wRb.asRPY();

    for (std::size_t idx = 0; idx < 4; idx++)
    {
        pImpl->lfOrientationQuaternion.setVal(idx, pImpl->lfOrientationInterface[idx]);
    }

    pImpl->wRlf.fromQuaternion(pImpl->lfOrientationQuaternion);
    pImpl->wRlfRPY = pImpl->wRlf.asRPY();

    for (std::size_t idx = 0; idx < 4; idx++)
    {
        pImpl->rfOrientationQuaternion.setVal(idx, pImpl->rfOrientationInterface[idx]);
    }

    pImpl->wRrf.fromQuaternion(pImpl->rfOrientationQuaternion);
    pImpl->wRrfRPY = pImpl->wRrf.asRPY();

    pImpl->contactDataInterface = pImpl->iWBK->getAllVertexContactData();
    for (const auto& vertexData : pImpl->contactDataInterface)
    {
        if (std::find(pImpl->vertexContactNamesInterface.begin(),
                      pImpl->vertexContactNamesInterface.end(),
                      vertexData.name) != pImpl->vertexContactNamesInterface.end())
        {
            std::string pos{"foot_contact::"+vertexData.name+"::position"};
            std::string force{"foot_contact::"+vertexData.name+"::normal_force"};
            std::string state{"foot_contact::"+vertexData.name+"::contact_state"};

            auto forceV = std::vector<double>{vertexData.magnitude};
            auto contactV = std::vector<double>{static_cast<double>(vertexData.isActive)};
            pImpl->bufferManager.push_back(vertexData.position,
                                           pImpl->timeNow,
                                           pos);
            pImpl->bufferManager.push_back(forceV,
                                           pImpl->timeNow,
                                           force);
            pImpl->bufferManager.push_back(contactV,
                                           pImpl->timeNow,
                                           state);
        }
    }

    for (const auto& vertexPos : pImpl->contactPosInterface)
    {
        if (std::find(pImpl->vertexContactNamesInterface.begin(),
                      pImpl->vertexContactNamesInterface.end(),
                      vertexPos.name) != pImpl->vertexContactNamesInterface.end())
        {
            std::string estPos{"foot_contact::"+vertexPos.name+"::est_global_position"};
            pImpl->bufferManager.push_back(vertexPos.position,
                                           pImpl->timeNow,
                                           estPos);
        }
    }

    pImpl->bufferManager.push_back(pImpl->basePositionInterface,
                                   pImpl->timeNow,
                                   "base_state::position");
    pImpl->bufferManager.push_back(pImpl->wRbRPY,
                                   pImpl->timeNow,
                                   "base_state::orientation");
    pImpl->bufferManager.push_back(pImpl->baseVelocityInterface,
                                   pImpl->timeNow,
                                   "base_state::velocity");

    pImpl->bufferManager.push_back(pImpl->jointPositionsInterface,
                                   pImpl->timeNow,
                                   "joints_state::positions");
    pImpl->bufferManager.push_back(pImpl->jointVelocitiesInterface,
                                   pImpl->timeNow,
                                   "joints_state::velocities");
    pImpl->bufferManager.push_back(pImpl->lCoPPositionInterface,
                                   pImpl->timeNow,
                                   "left_foot::cop");
    pImpl->bufferManager.push_back(pImpl->rCoPPositionInterface,
                                   pImpl->timeNow,
                                   "right_foot::cop");
    pImpl->bufferManager.push_back(pImpl->wRlfRPY,
                                   pImpl->timeNow,
                                   "left_foot::orientation");
    pImpl->bufferManager.push_back(pImpl->wRrfRPY,
                                   pImpl->timeNow,
                                   "right_foot::orientation");
    pImpl->bufferManager.push_back(pImpl->globalCoPPositionInterface,
                                   pImpl->timeNow,
                                   "global_cop");

    if (pImpl->logIHumanState)
    {
        pImpl->hspBasePositionInterface = pImpl->iHumanState->getBasePosition();
        pImpl->hspBaseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
        pImpl->hspBaseVelocityInterface = pImpl->iHumanState->getBaseVelocity();
        pImpl->hspJointPositionsInterface = pImpl->iHumanState->getJointPositions();
        pImpl->hspJointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();

        for (std::size_t idx = 0; idx < 4; idx++)
        {
            pImpl->hspBaseOrientationQuaternion.setVal(idx, pImpl->hspBaseOrientationInterface[idx]);
        }

        pImpl->hspWRb.fromQuaternion(pImpl->hspBaseOrientationQuaternion);
        pImpl->hspWRbRPY = pImpl->hspWRb.asRPY();

        pImpl->bufferManager.push_back(pImpl->hspBasePositionInterface,
                                       pImpl->timeNow,
                                       "hsp::base_state::position");
        pImpl->bufferManager.push_back(pImpl->hspWRbRPY,
                                       pImpl->timeNow,
                                       "hsp::base_state::orientation");
        pImpl->bufferManager.push_back(pImpl->hspBaseVelocityInterface,
                                       pImpl->timeNow,
                                       "hsp::base_state::velocity");
        pImpl->bufferManager.push_back(pImpl->hspJointPositionsInterface,
                                       pImpl->timeNow,
                                       "hsp::joints_state::positions");
        pImpl->bufferManager.push_back(pImpl->hspJointVelocitiesInterface,
                                       pImpl->timeNow,
                                       "hsp::joints_state::velocities");
    }

    if (pImpl->logGroundTruth)
    {
        if (pImpl->iTF->frameExists(pImpl->baseTrackerName))
        {
            std::string parentFrame;
            pImpl->iTF->getParent(pImpl->baseTrackerName, parentFrame);
            if (parentFrame == pImpl->groundTruthRefFrame)
            {
                bool ok = pImpl->iTF->getTransform(pImpl->baseTrackerName,
                                                   parentFrame,
                                                   pImpl->groundTruthBasePoseYarp);
                if (ok)
                {
                    toiDynTree(pImpl->groundTruthBasePoseYarp, pImpl->groundTruthBasePoseDyn);
                    for (std::size_t idx = 0; idx < 3; idx++)
                    {
                        pImpl->groundTruthwPb(idx) = pImpl->groundTruthBasePoseDyn.getPosition()(idx);
                    }
                    pImpl->groundTruthwRbRPY = pImpl->groundTruthBasePoseDyn.getRotation().asRPY();
                }
            }
        }
        pImpl->bufferManager.push_back(pImpl->groundTruthwPb,
                                       pImpl->timeNow,
                                       "ground_truth::base_state::position");
        pImpl->bufferManager.push_back(pImpl->groundTruthwRbRPY,
                                       pImpl->timeNow,
                                       "ground_truth::base_state::orientation");
    }
}

bool WholeBodyKinematicsLogger::close()
{
    while (isRunning())
    {
        stop();
    }

    pImpl->iWBK = nullptr;
    return true;
}

bool WholeBodyKinematicsLogger::Impl::initializeLogger()
{
    bool ok{true};
    ok = ok && bufferManager.addChannel({"base_state::position", {3, 1}});
    ok = ok && bufferManager.addChannel({"base_state::orientation", {3, 1}});
    ok = ok && bufferManager.addChannel({"base_state::velocity", {6, 1}});

    ok = ok && bufferManager.addChannel({"joints_state::positions", {jointNamesInterface.size(), 1}});
    ok = ok && bufferManager.addChannel({"joints_state::velocities", {jointNamesInterface.size(), 1}});
    ok = ok && bufferManager.addChannel({"left_foot::cop", {3, 1}});
    ok = ok && bufferManager.addChannel({"right_foot::cop", {3, 1}});
    ok = ok && bufferManager.addChannel({"left_foot::orientation", {3, 1}});
    ok = ok && bufferManager.addChannel({"right_foot::orientation", {3, 1}});
    ok = ok && bufferManager.addChannel({"global_cop", {3, 1}});

    for (const auto& vertexName : vertexContactNamesInterface)
    {
        std::string pos{"foot_contact::"+vertexName+"::position"};
        std::string force{"foot_contact::"+vertexName+"::normal_force"};
        std::string state{"foot_contact::"+vertexName+"::contact_state"};
        std::string estPos{"foot_contact::"+vertexName+"::est_global_position"};
        ok = ok && bufferManager.addChannel({pos, {3, 1}});
        ok = ok && bufferManager.addChannel({force, {1, 1}});
        ok = ok && bufferManager.addChannel({state, {1, 1}});
        ok = ok && bufferManager.addChannel({estPos, {3, 1}});
    }

    if (logIHumanState)
    {
        ok = ok && bufferManager.addChannel({"hsp::base_state::position", {3, 1}});
        ok = ok && bufferManager.addChannel({"hsp::base_state::orientation", {3, 1}});
        ok = ok && bufferManager.addChannel({"hsp::base_state::velocity", {6, 1}});

        ok = ok && bufferManager.addChannel({"hsp::joints_state::positions", {hspJointNamesInterface.size(), 1}});
        ok = ok && bufferManager.addChannel({"hsp::joints_state::velocities", {hspJointNamesInterface.size(), 1}});
    }

    if (logGroundTruth)
    {
        ok = ok && bufferManager.addChannel({"ground_truth::base_state::position", {3, 1}});
        ok = ok && bufferManager.addChannel({"ground_truth::base_state::orientation", {3, 1}});
    }

    return true;
}

bool WholeBodyKinematicsLogger::Impl::setupLogger()
{
    yarp::telemetry::experimental::BufferConfig config;
    config.filename = "whole_body_kinematics_logger";
    config.auto_save = true;
    config.save_periodically = true;
    config.file_indexing = "%Y_%m_%d_%H_%M_%S";
    config.mat_file_version = matioCpp::FileVersion::MAT7_3;
    config.save_period = telemetryAutosavePeriod;

    // the telemetry will flush the content of its storage every config.save_period
    // and this device runs every devicePeriod
    // so the size of the telemetry buffer must be at least config.save_period / devicePeriod
    // to be sure we are not going to lose data the buffer will be 10% longer
    constexpr double percentage = 0.1;
    config.n_samples = static_cast<int>(std::ceil((1 + percentage) //
                                                  * (config.save_period / devicePeriod)));

    return bufferManager.configure(config);
}
