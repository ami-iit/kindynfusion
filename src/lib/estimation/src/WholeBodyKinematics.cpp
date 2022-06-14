/**
 * @file WholeBodyKinematics.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/FloatingBaseEstimators/WholeBodyKinematics.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iterator>

using namespace KinDynFusion::IK;
using namespace KinDynFusion::Estimators;
using namespace BipedalLocomotion::Contacts;

WholeBodyKinematics::WholeBodyKinematics()
{
    m_wearableTargets.clear();
    m_wearableTargetNames.clear();
    m_wearableShoes.clear();
    m_wearableShoeNames.clear();
    m_I3 = Eigen::Matrix3d::Identity();
    m_v3 = Eigen::Vector3d::Zero();
}

bool WholeBodyKinematics::initializeIKHelper(const ModelData& modelData,
                                             const IKParameters& ikParams)
{
    const std::string printPrefix{"WholeBodyKinematics::initializeInverseKinematics "};
    if (!m_ikHelper.initialize(modelData, ikParams))
    {
        std::cerr << printPrefix
                  << "failed to initialize IKHelper."
                  << std::endl;
        return false;
    }

    // store for book-keeping
    m_modelData = modelData;
    m_ikParams = ikParams;
    m_jointConfigurationSolution.resize(modelData.model.getNrOfDOFs());
    m_jointVelocitiesSolution.resize(modelData.model.getNrOfDOFs());
    return true;
}

bool WholeBodyKinematics::initializeContactDetectors(const FootMetaData& lfootData,
                                                     const FootMetaData& rfootData,
                                                     iDynTree::Model& model)
{
    const std::string printPrefix{"WholeBodyKinematics::initializeContactDetectors "};

    if (!initializeContactDetector(lfootData, model, m_leftFootContactDetector))
    {
        std::cerr << printPrefix
                  << "failed to initialize left foot contact detector."
                  << std::endl;
        return false;
    }

    if (!initializeContactDetector(rfootData, model, m_rightFootContactDetector))
    {
        std::cerr << printPrefix
                  << "failed to initialize left foot contact detector."
                  << std::endl;
        return false;
    }

    m_candidateContactFrames.clear();
    auto lfContacts = m_leftFootContactDetector.getCandidateContactFrames();
    auto rfContacts = m_rightFootContactDetector.getCandidateContactFrames();
    m_candidateContactFrames.insert(m_candidateContactFrames.end(),
                                    std::move_iterator(lfContacts.begin()),
                                    std::move_iterator(lfContacts.end()));
    m_candidateContactFrames.insert(m_candidateContactFrames.end(),
                                    std::move_iterator(rfContacts.begin()),
                                    std::move_iterator(rfContacts.end()));

    return true;
}

bool WholeBodyKinematics::initializeContactDetector(const FootMetaData& data,
                                                    iDynTree::Model& model,
                                                    CoPVertexSchmittTrigger& detector)
{
    const std::string printPrefix{"WholeBodyKinematics::initializeContactDetector "};
    bool ok{true};
    ok = ok && detector.setSoleInFootLink(data.footLinkName,
                                          data.soleFrame,
                                          data.wearableName,
                                          data.soleRPYInDegInFoot,
                                          data.solePositionInFoot);
    ok = ok && detector.setRectangularFoot(data.footLength,
                                           data.footWidth,
                                           data.topLeftPositionInSole,
                                           data.contactForceThresholdForCOPComputation);
    ok = ok && detector.setSchmittParams(data.schmittParams);
    ok = ok && detector.initializeAndAddCandidateContactFramesToModel(model);

    if (!ok)
    {
        std::cerr << printPrefix
                  << "failed to initialize contact detector."
                  << std::endl;
        return false;
    }

    detector.printMetaData();
    return true;
}

bool WholeBodyKinematics::initializeBaseEKF(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                            const KinDynFusion::IK::ModelData& modelData)
{
    const std::string printPrefix{"WholeBodyKinematics::initializeBaseEKF "};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << printPrefix
                  << "Invalid parameters handler for configuration."
                  << std::endl;
        return false;
    }

    if (!handle->getParameter("constant_floor_height", m_constantFloorHeight))
    {
        std::cerr << printPrefix
                  << "Failed to find parameter \"constant_floor_height\" in configuration."
                  << std::endl;
        return false;
    }

    if (!handle->getParameter("force_flat_floor", m_forceFlatFloor))
    {
        std::cerr << printPrefix
                  << "Failed to find parameter \"force_flat_floor\" in configuration."
                  << std::endl;
        return false;
    }

    auto estHandler = handle->getGroup("BaseEstimator");
    auto estHandle = estHandler.lock();
    if (estHandle == nullptr)
    {
        std::cerr << printPrefix
                  << "failed to find group \"BaseEstimator\" in configuration."
                  << std::endl;
        return false;
    }

    m_baseEKFKinDyn = std::make_shared<iDynTree::KinDynComputations>();
    if (!m_baseEKFKinDyn->loadRobotModel(modelData.model))
    {
        std::cerr << printPrefix
                  << "failed to initialize base EKF KinDynComputations."
                  << std::endl;
        return false;
    }

    if (!m_baseEKF.initialize(estHandler, m_baseEKFKinDyn))
    {
        std::cerr << printPrefix
                  << "failed to initialize base EKF."
                  << std::endl;
        return false;
    }

    m_baseEKF.setSameFootVertices(m_leftFootContactDetector.getSoleFrame(),
                                  m_leftFootContactDetector.getCandidateContactFrames());

    m_baseEKF.setSameFootVertices(m_rightFootContactDetector.getSoleFrame(),
                                  m_rightFootContactDetector.getCandidateContactFrames());
    m_baseEKF.useFlatFloorMeasurements(m_forceFlatFloor);
    return true;
}


bool WholeBodyKinematics::addWearableSensorTarget(const std::string& wearableName,
                                                  const std::string& modelLinkName,
                                                  const std::string& targetType,
                                                  const std::string& targetName)
{
    const std::string printPrefix{"WholeBodyKinematics::addWearableSensorTarget "};
    std::lock_guard<std::mutex> lock{m_mutex};
    if (m_wearableTargets.find(targetName) != m_wearableTargets.end())
    {
        std::cerr << printPrefix << "Duplicated target name found ["
                  << targetName << "]." << std::endl;
        return false;
    }

    if (stringToKinematicTargetType.find(targetType) == stringToKinematicTargetType.end())
    {
        std::cerr << printPrefix << "Invalid target type for ["
                  << targetName << "]." << std::endl;
        return false;
    }

    if (m_modelData.model.getLinkIndex(modelLinkName) == iDynTree::LINK_INVALID_INDEX)
    {
        std::cerr << printPrefix << "Failed to find link "
                  << modelLinkName << " in the model for ["
                  << targetName << "]." << std::endl;
        return false;
    }

    m_wearableTargets[targetName] = std::make_shared<WearableSensorTarget>(wearableName,
                                                                           modelLinkName,
                                                                           stringToKinematicTargetType.at(targetType));
    if (!m_ikHelper.addTarget(targetName, m_ikParams, m_wearableTargets[targetName]))
    {
        std::cerr << printPrefix << "Failed to add target ["
                  << targetName << "] to IKHelper block." << std::endl;
        return false;
    }

    if (modelLinkName == m_modelData.baseFrame)
    {
        m_baseTargetName = targetName;
    }

    if (modelLinkName == m_leftFootContactDetector.getFootLinkName())
    {
        m_leftFootTargetName = targetName;
    }

    if (modelLinkName == m_rightFootContactDetector.getFootLinkName())
    {
        m_rightFootTargetName = targetName;
    }

    std::cout << printPrefix << "Added target "
              << targetName << " of type " << targetType
              << " for link " << modelLinkName
              << " with sensor " << wearableName << std::endl;
    m_wearableTargetNames.emplace_back(targetName);
    return true;
}

bool WholeBodyKinematics::addWearableSensorizedShoes(const std::string& wearableName,
                                                     const std::string& modelLinkName,
                                                     const iDynTree::Transform& footHshoe)
{
    const std::string printPrefix{"WholeBodyKinematics::addWearableSensorizedShoes "};
    std::lock_guard<std::mutex> lock{m_mutex};
    if (m_wearableShoes.find(wearableName) != m_wearableShoes.end())
    {
        std::cerr << printPrefix << "Duplicated shoes name found ["
                  << wearableName << "]." << std::endl;
        return false;
    }

    m_wearableShoes[wearableName] = std::make_shared<WearableSensorizedShoe>(wearableName, modelLinkName);
    m_wearableShoes.at(wearableName)->setFootHShoe(footHshoe);

    std::cout << printPrefix << "Added shoe "
              << wearableName
              << " for foot link: " << modelLinkName
              <<  " with transform footHshoe: \n"
              << m_wearableShoes.at(wearableName)->getFootHShoe().toString() <<std::endl;
    m_wearableShoeNames.emplace_back(wearableName);
    return true;
}


bool WholeBodyKinematics::advance(const double& dt)
{
    const std::string printPrefix{"WholeBodyKinematics::advance "};
    if (!advanceContactDetectorBlock())
    {
        std::cerr << printPrefix
                  << "Failed to infer contact states." << std::endl;
        return false;
    }

    if (!advanceIKBlock(dt))
    {
        std::cerr << printPrefix
                  << "Failed to compute IK." << std::endl;
        return false;
    }

    if (m_isBaseEKFStarted)
    {
        if (!advanceBaseEKF())
        {
             std::cerr << printPrefix
                       << "Failed to estimate base pose." << std::endl;
             return false;
        }

        computeGlobalCoP();
    }

    return true;
}

bool WholeBodyKinematics::advanceIKBlock(const double& dt)
{
    const std::string printPrefix{"WholeBodyKinematics::advanceIKBlock "};

    if (!m_ikHelper.solve(dt))
    {
         std::cerr << printPrefix
                   << "Failed to solve IK." << std::endl;
    }

    if (!m_isBaseEKFStarted)
    {
        m_basePoseSolution = m_ikHelper.getBaseTransformSolution();
    }
    m_jointConfigurationSolution = m_ikHelper.getJointConfigurationSolution();
    m_jointVelocitiesSolution = m_ikHelper.getJointVelocitiesSolution();

    return true;
}

bool WholeBodyKinematics::advanceContactDetectorBlock()
{
    const std::string printPrefix{"WholeBodyKinematics::advanceContactDetectorBlock "};

    if (!m_leftFootContactDetector.advance())
    {
         std::cerr << printPrefix
                   << "Failed to advance right foot contact detector." << std::endl;
    }

    if (!m_rightFootContactDetector.advance())
    {
         std::cerr << printPrefix
                   << "Failed to advance right foot contact detector." << std::endl;
    }

    m_lfContacts = m_leftFootContactDetector.getContactStates();
    m_rfContacts = m_rightFootContactDetector.getContactStates();

    return true;
}

bool WholeBodyKinematics::advanceBaseEKF()
{
    const std::string printPrefix{"WholeBodyKinematics::advanceBaseEKF "};
    if (!m_baseEKF.advance())
    {
        std::cerr << printPrefix
                   << "Failed to advance base EKF." << std::endl;
    }

    m_baseEKFOutput = m_baseEKF.getOutput();
    iDynTree::Rotation baseRot;
    iDynTree::Position basePos;
    iDynTree::toEigen(baseRot) = m_baseEKFOutput.basePose.quat().toRotationMatrix();
    iDynTree::toEigen(basePos) = m_baseEKFOutput.basePose.translation();
    m_basePoseSolution.setPosition(basePos);
    m_basePoseSolution.setRotation(baseRot);

    iDynTree::Vector3 vb;
    iDynTree::Vector3 omegab;
    iDynTree::toEigen(vb) = m_baseEKFOutput.baseTwist.head<3>();
    iDynTree::toEigen(omegab) = m_baseEKFOutput.baseTwist.tail<3>();
    m_baseVelocitySolution.setLinearVec3(vb);
    m_baseVelocitySolution.setAngularVec3(omegab);

    m_leftFootOrientSolution.setVal(0, m_baseEKFOutput.state.lContactFrameOrientation.w());
    m_leftFootOrientSolution.setVal(1, m_baseEKFOutput.state.lContactFrameOrientation.x());
    m_leftFootOrientSolution.setVal(2, m_baseEKFOutput.state.lContactFrameOrientation.y());
    m_leftFootOrientSolution.setVal(3, m_baseEKFOutput.state.lContactFrameOrientation.z());

    m_rightFootOrientSolution.setVal(0, m_baseEKFOutput.state.rContactFrameOrientation.w());
    m_rightFootOrientSolution.setVal(1, m_baseEKFOutput.state.rContactFrameOrientation.x());
    m_rightFootOrientSolution.setVal(2, m_baseEKFOutput.state.rContactFrameOrientation.y());
    m_rightFootOrientSolution.setVal(3, m_baseEKFOutput.state.rContactFrameOrientation.z());

    for (const auto& [id, contact] : m_baseEKFOutput.state.supportFrameData)
    {
        auto name = m_baseEKF.modelComputations().kinDyn()->getFrameName(id);
        iDynTree::toEigen(m_contactFramePositionSolution[name]) = contact.pose.translation();
    }

    return true;
}

bool WholeBodyKinematics::computeGlobalCoP()
{
    bool lfValidCoP = m_leftFootContactDetector.getFootRectangle().isCoPInsideSupportPolygon();
    bool rfValidCoP = m_rightFootContactDetector.getFootRectangle().isCoPInsideSupportPolygon();
    iDynTree::Position lfcopInertial, rfcopInertial;
    if (lfValidCoP)
    {
        auto lfName{m_leftFootContactDetector.getFootLinkName()};
        auto lfcop = m_leftFootContactDetector.getCenterOfPressureInLinkFrame();
        auto w_H_lf = m_baseEKF.modelComputations().kinDyn()->getWorldTransform(lfName);
        lfcopInertial =  w_H_lf*lfcop;
    }

    if (rfValidCoP)
    {
        auto rfName{m_rightFootContactDetector.getFootLinkName()};
        auto rfcop = m_rightFootContactDetector.getCenterOfPressureInLinkFrame();
        auto w_H_rf = m_baseEKF.modelComputations().kinDyn()->getWorldTransform(rfName);
        rfcopInertial =  w_H_rf*rfcop;
    }

    if (lfValidCoP && rfValidCoP)
    {
        iDynTree::toEigen(m_globalCoP) = iDynTree::toEigen(lfcopInertial + rfcopInertial)/2;
    }

    if (lfValidCoP && !rfValidCoP)
    {
        m_globalCoP = lfcopInertial;
    }

    if (!lfValidCoP && rfValidCoP)
    {
        m_globalCoP = rfcopInertial;
    }

    return true;
}

bool WholeBodyKinematics::updateIKTargets()
{
    const std::string printPrefix{"WholeBodyKinematics::updateWearableSensorTargets "};
    std::lock_guard<std::mutex> lock{m_mutex};
    for (auto [name, target] : m_wearableTargets)
    {
        if (!m_ikHelper.updateTarget(name, target))
        {
            std::cerr << printPrefix
                      << "Unable to update target ["
                      << name << "]." << std::endl;
        }
    }
    return true;
}

bool WholeBodyKinematics::updateContactDetectors(const double& timeNow)
{
    const std::string printPrefix{"WholeBodyKinematics::updateContactDetectors "};
    if (!updateContactDetector(timeNow, m_leftFootContactDetector))
    {
        std::cerr << printPrefix
                  << "Unable to update left foot contact detector." << std::endl;
        return false;
    }

    if (!updateContactDetector(timeNow, m_rightFootContactDetector))
    {
        std::cerr << printPrefix
                  << "Unable to update right foot contact detector." << std::endl;
        return false;
    }
    return true;
}

bool WholeBodyKinematics::updateContactDetector(const double& timeNow,
                                                CoPVertexSchmittTrigger& detector)
{
    const std::string printPrefix{"WholeBodyKinematics::updateContactDetectors "};
    auto wearableName = detector.getAssociatedWearableName();
    std::lock_guard<std::mutex> lock{m_mutex};
    if (m_wearableShoes.find(wearableName) == m_wearableShoes.end())
    {
        std::cerr << printPrefix
                  << "There seems to be a mismatch between WearableSensorizedShoe"
                  << " and contact detector with associated wearable name ["
                  << wearableName << "]." << std::endl;
        return false;
    }

    auto& shoe = m_wearableShoes.at(wearableName);
    auto wrenchInSole = detector.getFootHSole()*shoe->getWrenchInFootFrame();
    if (!detector.setNetWrenchInSole(wrenchInSole, timeNow))
    {
        std::cerr << printPrefix
                  << "Could not set wrench input for contact detector "
                  << " with with associated wearable name ["
                  << wearableName << "]." << std::endl;
        return false;
    }

    return true;
}

bool WholeBodyKinematics::updateBaseEKF(const double& timeNow)
{
    const std::string printPrefix{"[WholeBodyKinematics::updateBaseEKF]"};
    std::lock_guard<std::mutex> lock{m_mutex};
    m_baseEKF.setKinematics(iDynTree::toEigen(m_jointConfigurationSolution),
                            iDynTree::toEigen(m_jointVelocitiesSolution));
    for (const auto& [name, contact] : m_lfContacts)
    {
        m_baseEKF.setContactStatus(name,
                                   contact.isActive,
                                   contact.switchTime,
                                   timeNow);
        if (contact.isActive)
        {
            m_baseEKF.setContactHeight(name, m_constantFloorHeight);
        }
    }

    for (const auto& [name, contact] : m_rfContacts)
    {
        m_baseEKF.setContactStatus(name,
                                   contact.isActive,
                                   contact.switchTime,
                                   timeNow);
        if (contact.isActive)
        {
            m_baseEKF.setContactHeight(name, m_constantFloorHeight);
        }
    }

    if (m_wearableTargets.find(m_baseTargetName) != m_wearableTargets.end())
    {
        auto baseTargetPtr = m_wearableTargets.at(m_baseTargetName);
        // L_R_A * A_omega_L
        Eigen::Matrix3Xd L_R_A = iDynTree::toEigen(baseTargetPtr->getCalibratedRotation().inverse());
        Eigen::Vector3d A_omega = iDynTree::toEigen(baseTargetPtr->getCalibratedAngularVelocity());
        Eigen::Vector3d L_omega = L_R_A*A_omega;
        m_baseEKF.setLeftTrivializedBaseVelocity(L_omega);
    }

    // assuming sole has the same orientation as foot
    if (m_wearableTargets.find(m_leftFootTargetName) != m_wearableTargets.end())
    {
        auto leftFootTargetPtr = m_wearableTargets.at(m_leftFootTargetName);

        iDynTree::Vector4 contactPlaneRot;
        if (m_forceFlatFloor)
        {
            auto contactPlaneRPY = leftFootTargetPtr->getCalibratedRotation().asRPY();
            contactPlaneRPY(0) = 0.0;
            contactPlaneRPY(1) = 0.0;
            contactPlaneRot = iDynTree::Rotation::RPY(contactPlaneRPY(0),
                                                      contactPlaneRPY(1),
                                                      contactPlaneRPY(2)).asQuaternion();
        }
        else
        {
            contactPlaneRot = leftFootTargetPtr->getCalibratedRotation().asQuaternion();
        }

        Eigen::Quaterniond lfQuat(contactPlaneRot(0),
                                  contactPlaneRot(1),
                                  contactPlaneRot(2),
                                  contactPlaneRot(3));
        auto soleName = m_baseEKF.modelComputations().leftFootContactFrame();
        m_baseEKF.setContactPlaneOrientation(soleName,
                                             lfQuat);
    }

    if (m_wearableTargets.find(m_rightFootTargetName) != m_wearableTargets.end())
    {
        auto rightFootTargetPtr = m_wearableTargets.at(m_rightFootTargetName);
        iDynTree::Vector4 contactPlaneRot;
        if (m_forceFlatFloor)
        {
            auto contactPlaneRPY = rightFootTargetPtr->getCalibratedRotation().asRPY();
            contactPlaneRPY(0) = 0.0;
            contactPlaneRPY(1) = 0.0;
            contactPlaneRot = iDynTree::Rotation::RPY(contactPlaneRPY(0),
                                                      contactPlaneRPY(1),
                                                      contactPlaneRPY(2)).asQuaternion();
        }
        else
        {
            contactPlaneRot = rightFootTargetPtr->getCalibratedRotation().asQuaternion();
        }

        Eigen::Quaterniond rfQuat(contactPlaneRot(0),
                                  contactPlaneRot(1),
                                  contactPlaneRot(2),
                                  contactPlaneRot(3));
        auto soleName = m_baseEKF.modelComputations().rightFootContactFrame();
        m_baseEKF.setContactPlaneOrientation(soleName,
                                             rfQuat);
    }

    return true;
}

bool WholeBodyKinematics::resetBaseEKFWorldFrameTo(const std::string& refLinkName)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    const std::string printPrefix{"[WholeBodyKinematics::updateBaseEKF]"};

    if (!m_baseEKF.setWorldFrameInRefTo(refLinkName, m_I3, m_v3))
    {
        std::cerr << printPrefix
                  << "Could not reset BaseEKF world frame to "
                  << refLinkName << std::endl;
        return false;
    }
    return true;
}

void WholeBodyKinematics::startBaseEKF()
{
    std::lock_guard<std::mutex> lock{m_mutex};
    m_isBaseEKFStarted = true;
}

const WearableSensorTargetMap& WholeBodyKinematics::getAllWearableTargets() const
{
    return m_wearableTargets;
}

WearableSensorTargetPtr WholeBodyKinematics::getWearableTarget(const KinDynFusion::IK::TargetName& name) const
{
    const std::string printPrefix{"WholeBodyKinematics::getWearableTarget "};
    std::lock_guard<std::mutex> lock{m_mutex};
    if (m_wearableTargets.find(name) == m_wearableTargets.end())
    {
        std::cerr << printPrefix
                  << "Invalid target name [ " << name << "]."
                  << std::endl;
        return nullptr;
    }

    return m_wearableTargets.at(name);
}

std::vector<KinDynFusion::IK::TargetName> WholeBodyKinematics::getAllWearableTargetNames() const
{
    return m_wearableTargetNames;
}

const WearableSensorizedShoeMap& WholeBodyKinematics::getAllWearableSensorizedShoes() const
{
    return m_wearableShoes;
}

WearableSensorizedShoePtr WholeBodyKinematics::getWearableSensorizedShoe(const std::string& name) const
{
    const std::string printPrefix{"WholeBodyKinematics::getWearableSensorizedShoe "};
    std::lock_guard<std::mutex> lock{m_mutex};
    if (m_wearableShoes.find(name) == m_wearableShoes.end())
    {
        std::cerr << printPrefix
                  << "Invalid shoe name [ " << name << "]."
                  << std::endl;
        return nullptr;
    }

    return m_wearableShoes.at(name);
}

std::vector<KinDynFusion::IK::TargetName> WholeBodyKinematics::getAllWearableShoeNames() const
{
    return m_wearableShoeNames;
}

const std::vector<std::string> WholeBodyKinematics::getCandidateContactFrames() const
{
    return m_candidateContactFrames;
}

const iDynTree::Transform& WholeBodyKinematics::getBasePose() const
{
    return m_basePoseSolution;
}

const iDynTree::VectorDynSize& WholeBodyKinematics::getJointConfiguration() const
{
    return m_jointConfigurationSolution;
}

const KinDynFusion::IK::ModelData& WholeBodyKinematics::getModelData() const
{
    return m_modelData;
}

const iDynTree::Twist& WholeBodyKinematics::getBaseVelocity() const
{
    return m_baseVelocitySolution;
}

const iDynTree::VectorDynSize& WholeBodyKinematics::getJointVelocities() const
{
    return m_jointVelocitiesSolution;
}

const iDynTree::Vector4& WholeBodyKinematics::getLeftFootOrientation() const
{
    return m_leftFootOrientSolution;
}

const iDynTree::Vector4& WholeBodyKinematics::getRightFootOrientation() const
{
    return m_rightFootOrientSolution;
}

const std::unordered_map<std::string, iDynTree::Position>& WholeBodyKinematics::getAllContactFramePositions() const
{
    return m_contactFramePositionSolution;
}

const std::string& WholeBodyKinematics::getLeftFootLinkName() const
{
    return m_leftFootContactDetector.getFootLinkName();
}

const std::string& WholeBodyKinematics::getRightFootLinkName() const
{
    return m_rightFootContactDetector.getFootLinkName();
}

bool WholeBodyKinematics::getLeftFootVertexData(const std::string& vertexName,
                                                std::string& linkName,
                                                double& magnitude,
                                                bool& isActive,
                                                iDynTree::Position& position)
{
    if (! m_leftFootContactDetector.getContactFramePositionInFootLink(vertexName, position))
    {
        return false;
    }

    magnitude = m_leftFootContactDetector.getVertexContactNormalForce(vertexName);
    linkName = m_leftFootContactDetector.getFootLinkName();
    isActive = m_leftFootContactDetector.getContactStates().at(vertexName).isActive;
    return true;
}

bool WholeBodyKinematics::getRightFootVertexData(const std::string& vertexName,
                                                 std::string& linkName,
                                                 double& magnitude,
                                                 bool& isActive,
                                                 iDynTree::Position& position)
{
    if (! m_rightFootContactDetector.getContactFramePositionInFootLink(vertexName, position))
    {
        return false;
    }

    magnitude = m_rightFootContactDetector.getVertexContactNormalForce(vertexName);
    linkName = m_rightFootContactDetector.getFootLinkName();
    isActive = m_rightFootContactDetector.getContactStates().at(vertexName).isActive;
    return true;
}

void WholeBodyKinematics::getLeftFootCenterOfPressure(iDynTree::Position& cop)
{
    cop = m_leftFootContactDetector.getCenterOfPressureInLinkFrame();
}

void WholeBodyKinematics::getRightFootCenterOfPressure(iDynTree::Position& cop)
{
    cop = m_rightFootContactDetector.getCenterOfPressureInLinkFrame();
}

void WholeBodyKinematics::getGlobalCenterOfPressure(iDynTree::Position& cop)
{
    cop = m_globalCoP;
}

std::vector<std::string> WholeBodyKinematics::getLeftFootCandidateContactFrames()
{
    return m_leftFootContactDetector.getCandidateContactFrames();
}

std::vector<std::string> WholeBodyKinematics::getRightFootCandidateContactFrames()
{
    return m_rightFootContactDetector.getCandidateContactFrames();
}
