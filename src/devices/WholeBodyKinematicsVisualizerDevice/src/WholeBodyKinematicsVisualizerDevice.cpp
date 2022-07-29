/**
 * @file WholeBodyKinematicsVisualizerDevice.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/WholeBodyKinematicsVisualizerDevice.h>

#include <KinDynFusion/InverseKinematics/InverseKinematicsYarpHelper.h>
#include <KinDynFusion/FloatingBaseEstimators/IWholeBodyKinematics.h>
#include <KinDynFusion/WholeBodyKinematicsVisualizer.h>

#include <yarp/os/LogStream.h>

using namespace KinDynFusion;
using namespace KinDynFusion::Estimators;
using namespace KinDynFusion::IK;

const std::string DeviceName = "WholeBodyKinematicsVisualizerDevice";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

class WholeBodyKinematicsVisualizerDevice::Impl
{
public:
    bool initializeVisualizer();
    IWholeBodyKinematics* iWBK{nullptr};
    std::string wbkDataPortName;
    ModelData modelData;
    yarp::dev::PolyDriver remapperDevice;

    std::string modelName{"Human1"};
    std::string lCoPName{"lCoP"}, rCoPName{"rCoP"};
    bool visualizeCoPAndContactNormals{false};

    double devicePeriod;
    WholeBodyKinematicsVisualizer viz;

    // buffer variables
    std::array<double, 3> lCoPPositionInterface;
    std::array<double, 3> rCoPPositionInterface;
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

    iDynTree::Vector4 baseOrientationQuaternion;
    iDynTree::Position basePosition;
    iDynTree::Position bufferPosition;
    iDynTree::Transform wHl, wHr;
    iDynTree::Transform wHb;
    iDynTree::Rotation wRb;
    iDynTree::VectorDynSize joints;
};

WholeBodyKinematicsVisualizerDevice::WholeBodyKinematicsVisualizerDevice(double period,
                                                     yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock),
        pImpl{std::make_unique<WholeBodyKinematicsVisualizerDevice::Impl>()}
{
}

WholeBodyKinematicsVisualizerDevice::WholeBodyKinematicsVisualizerDevice()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No),
        pImpl{std::make_unique<WholeBodyKinematicsVisualizerDevice::Impl>()}
{

}

WholeBodyKinematicsVisualizerDevice::~WholeBodyKinematicsVisualizerDevice()
{
}

bool WholeBodyKinematicsVisualizerDevice::open(yarp::os::Searchable& config)
{
    KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config, "period", pImpl->devicePeriod, 0.02, false);
    this->setPeriod(pImpl->devicePeriod);

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

    if (!KinDynFusion::IK::YarpHelper::checkAndLoadScalar(config,
                                                          std::string{"visualize_cop_and_contact_normals"},
                                                          pImpl->visualizeCoPAndContactNormals,
                                                          /*defaultValue=*/false,
                                                          /*required=*/true))
    {
        yError() << LogPrefix
                 << " [open] Unable to load parameter \"visualize_cop_and_contact_normals\" from configuration file.";
        return false;
    }

    if (!KinDynFusion::IK::YarpHelper::loadModelData(config, pImpl->modelData))
    {
        yError() << LogPrefix
                 << " [open] Unable to load IK parameters from configuration file.";
        return false;
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
    if ( pImpl->iWBK->getBaseName() != pImpl->modelData.model.getLinkName(pImpl->modelData.model.getDefaultBaseLink()))
    {
        pImpl->modelData.model.setDefaultBaseLink(pImpl->modelData.model.getLinkIndex(pImpl->iWBK->getBaseName()));
        yInfo() << LogPrefix << "Default base link of the visualized model is changed to " << pImpl->iWBK->getBaseName();
    }
    yInfo() << LogPrefix << "Whole Body Kinematics Interface providing data from [ " << pImpl->iWBK->getJointNames().size() << " ] joints";

    for (auto jointName : pImpl->iWBK->getJointNames())
    {
        if (pImpl->modelData.model.getJointIndex(jointName) == iDynTree::JOINT_INVALID_INDEX)
        {
            yWarning() << LogPrefix << "joint [ " << jointName << " ] not found in the visualized model, "
                       << "the joint will be ignored.";
        }
    }

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

    // start the visualization thread
    this->start();

    return true;
}

bool WholeBodyKinematicsVisualizerDevice::threadInit()
{
    // ====
    // MISC
    // ====
    // initialize visualizer if IWholeBodyKinematics interface is available
    if (!pImpl->initializeVisualizer())
    {
        yError() << LogPrefix
                 << " [open] Failed to initialize visualizer.";
        return false;
    }
    return true;
}

void WholeBodyKinematicsVisualizerDevice::threadRelease()
{
    pImpl->viz.close();
}


void WholeBodyKinematicsVisualizerDevice::run()
{
    // Get data from the interface
    pImpl->jointPositionsInterface = pImpl->iWBK->getJointPositions();

    pImpl->basePositionInterface = pImpl->iWBK->getBasePosition();
    pImpl->baseOrientationInterface = pImpl->iWBK->getBaseOrientation();

    for (std::size_t idx = 0; idx < 3; idx++)
    {
        pImpl->basePosition.setVal(idx, pImpl->basePositionInterface[idx]);
    }

    for (std::size_t idx = 0; idx < 4; idx++)
    {
        pImpl->baseOrientationQuaternion.setVal(idx, pImpl->baseOrientationInterface[idx]);
    }

    pImpl->wHb.setPosition(pImpl->basePosition);
    pImpl->wRb.fromQuaternion(pImpl->baseOrientationQuaternion);
    pImpl->wHb.setRotation(pImpl->wRb);

    pImpl->joints.resize(pImpl->jointPositionsInterface.size());
    for (std::size_t idx = 0; idx < pImpl->jointPositionsInterface.size(); idx++)
    {
        pImpl->joints.setVal(idx, pImpl->jointPositionsInterface[idx]);
    }

    pImpl->viz.setConfiguration(pImpl->modelName, pImpl->wHb, pImpl->joints);
    pImpl->viz.advance();

    if (pImpl->visualizeCoPAndContactNormals)
    {
        pImpl->lFootNameInterface = pImpl->iWBK->getLeftFootLinkName();
        pImpl->rFootNameInterface = pImpl->iWBK->getRightFootLinkName();
        pImpl->lCoPPositionInterface = pImpl->iWBK->getLeftFootCoPPosition();
        pImpl->rCoPPositionInterface = pImpl->iWBK->getRightFootCoPPosition();

        pImpl->contactInterface = pImpl->iWBK->getAllVertexContactData();

        for (std::size_t idx = 0; idx < 3; idx++)
        {
            pImpl->bufferPosition.setVal(idx, pImpl->lCoPPositionInterface[idx]);
        }

        if (pImpl->viz.getWorldLinkTransform(pImpl->modelName,
                                             pImpl->lFootNameInterface,
                                             pImpl->wHl))
        {
            pImpl->viz.updateCenterOfPressureViz(pImpl->modelName,
                                                 pImpl->lCoPName,
                                                 pImpl->wHl*pImpl->bufferPosition);
        }

        for (std::size_t idx = 0; idx < 3; idx++)
        {
            pImpl->bufferPosition.setVal(idx, pImpl->rCoPPositionInterface[idx]);
        }
        if (pImpl->viz.getWorldLinkTransform(pImpl->modelName,
                                             pImpl->rFootNameInterface,
                                             pImpl->wHr))
        {
            pImpl->viz.updateCenterOfPressureViz(pImpl->modelName,
                                                 pImpl->rCoPName,
                                                 pImpl->wHr*pImpl->bufferPosition);
        }

        for (std::size_t idx = 0; idx < pImpl->contactInterface.size(); idx++)
        {
            auto contactNormal = pImpl->contactInterface[idx];
            for (std::size_t idx = 0; idx < 3; idx++)
            {
                pImpl->bufferPosition.setVal(idx, contactNormal.position[idx]);
            }
            if (contactNormal.linkName == pImpl->lFootNameInterface)
            {
                pImpl->viz.updateContactNormalViz(pImpl->modelName,
                                                  contactNormal.name,
                                                  pImpl->wHl*pImpl->bufferPosition,
                                                  contactNormal.magnitude,
                                                  contactNormal.isActive);
            }
            if (contactNormal.linkName == pImpl->rFootNameInterface)
            {
                pImpl->viz.updateContactNormalViz(pImpl->modelName,
                                                  contactNormal.name,
                                                  pImpl->wHr*pImpl->bufferPosition,
                                                  contactNormal.magnitude,
                                                  contactNormal.isActive);
            }
        }
    }
}

bool WholeBodyKinematicsVisualizerDevice::close()
{
    while (isRunning())
    {
        stop();
    }

    pImpl->iWBK = nullptr;
    return true;
}

bool WholeBodyKinematicsVisualizerDevice::Impl::initializeVisualizer()
{
    bool ok{true};
    ok = ok && viz.initialize();

//     ok = ok && viz.addModel(modelName,
//                             modelData.model,
//                             iDynTree::ColorViz(0.83, 0.5, 0.2412, 0.10));
//     ok = ok && viz.addModel(modelName,
//                             modelData.model,
//                             iDynTree::ColorViz(0.73, 0.4, 0.1412, 0.10));
    ok = ok && viz.addModel(modelName,
                            modelData.model,
                            iDynTree::ColorViz(0.63, 0.3, 0.0412, 0.10));
    if (visualizeCoPAndContactNormals)
    {
        auto contactNames = iWBK->getAllVertexContactNames();
        for (auto& name : contactNames)
        {
            ok = ok && viz.addContactNormalViz(modelName,
                                               name);
        }

        ok = ok && viz.addCenterOfPressureViz(modelName,
                                              lCoPName,
                                              rCoPName);
        if (!ok)
        {
            yError() << LogPrefix
                     << " [initializeVisualizer] Failed to initialize visualizer.";
        }
    }

    return true;
}
