/**
 * @file WholeBodyKinematics.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_ESTIMATION_WHOLE_BODY_KINEMATICS_H
#define KINDYNFUSION_ESTIMATION_WHOLE_BODY_KINEMATICS_H

#include <KinDynFusion/InverseKinematics/IKDataTypes.h>

#include <KinDynFusion/InverseKinematics/WearableTargets.h>
#include <KinDynFusion/InverseKinematics/WearableSensorizedShoes.h>

#include <KinDynFusion/InverseKinematics/IKHelper.h>
#include <KinDynFusion/FloatingBaseEstimators/CoPVertexSchmittTrigger.h>
#include <KinDynFusion/FloatingBaseEstimators/CalibrationHelper.h>
#include <KinDynFusion/FloatingBaseEstimators/DiligentOdom.h>

#include <mutex>

namespace KinDynFusion
{
namespace Estimators
{


class WholeBodyKinematics
{
public:
    WholeBodyKinematics();
    bool initializeIKHelper(const KinDynFusion::IK::ModelData& modelData,
                            const KinDynFusion::IK::IKParameters& ikParams);
    bool initializeContactDetectors(const FootMetaData& lfootData,
                                    const FootMetaData& rfootData,
                                    iDynTree::Model& model);
    bool initializeBaseEKF(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                           const KinDynFusion::IK::ModelData& modelData);
    bool addWearableSensorTarget(const std::string& wearableName,
                                 const std::string& modelLinkName,
                                 const std::string& targetType,
                                 const std::string& targetName);
    bool addWearableSensorizedShoes(const std::string& wearableName,
                                    const std::string& modelLinkName,
                                    const iDynTree::Transform& footHshoe);
    // updates IK targets from the wearable sensor targets
    // which are supposed to be updated externally
    // using iWear2iDyn class which gets the
    // shared pointer of the wearable targets and
    // shoes to updates their internal measurements
    // once wearable targets and shoes are updated
    // one may call updateIKTargets and updateContactDetectors
    // to pass the measurements to internal objects
    bool updateIKTargets();
    bool updateContactDetectors(const double& timeNow);
    bool updateBaseEKF(const double& timeNow);
    bool advance(const double& dt);

    bool resetBaseEKFWorldFrameTo(const std::string& refLinkName);
    void startBaseEKF();

    const std::vector<std::string> getCandidateContactFrames() const;
    const KinDynFusion::IK::ModelData& getModelData() const;

    std::vector<KinDynFusion::IK::TargetName> getAllWearableTargetNames() const;
    WearableSensorTargetPtr getWearableTarget(const KinDynFusion::IK::TargetName& name) const;
    const WearableSensorTargetMap& getAllWearableTargets() const;



    std::vector<std::string> getAllWearableShoeNames() const;
    WearableSensorizedShoePtr getWearableSensorizedShoe(const std::string& name) const;
    const WearableSensorizedShoeMap& getAllWearableSensorizedShoes() const;

    const iDynTree::Transform& getBasePose() const;
    const iDynTree::VectorDynSize& getJointConfiguration() const;
    const iDynTree::VectorDynSize& getJointVelocities() const;
    const iDynTree::Twist& getBaseVelocity() const;
    const iDynTree::Vector4& getLeftFootOrientation() const;
    const iDynTree::Vector4& getRightFootOrientation() const;
    const std::unordered_map<std::string, iDynTree::Position>& getAllContactFramePositions() const;

    const std::string& getLeftFootLinkName() const;
    const std::string& getRightFootLinkName() const;
    bool getLeftFootVertexData(const std::string& vertexName,
                               std::string& linkName,
                               double& magnitude,
                               bool& isActive,
                               iDynTree::Position& position);
    bool getRightFootVertexData(const std::string& vertexName,
                                std::string& linkName,
                                double& magnitude,
                                bool& isActive,
                                iDynTree::Position& position);
    void getLeftFootCenterOfPressure(iDynTree::Position& cop);
    void getRightFootCenterOfPressure(iDynTree::Position& cop);
    void getGlobalCenterOfPressure(iDynTree::Position& cop);
    std::vector<std::string> getLeftFootCandidateContactFrames();
    std::vector<std::string> getRightFootCandidateContactFrames();
private:
    bool initializeContactDetector(const FootMetaData& data,
                                   iDynTree::Model& model,
                                   CoPVertexSchmittTrigger& detector);
    bool updateContactDetector(const double& timeNow,
                               CoPVertexSchmittTrigger& detector);
    bool advanceIKBlock(const double& dt);
    bool advanceContactDetectorBlock();
    bool advanceBaseEKF();
    bool computeGlobalCoP();

    iDynTree::Transform m_basePoseSolution;
    iDynTree::Twist m_baseVelocitySolution;
    iDynTree::VectorDynSize m_jointConfigurationSolution;
    iDynTree::VectorDynSize m_jointVelocitiesSolution;
    std::unordered_map<std::string, iDynTree::Position> m_contactFramePositionSolution;
    iDynTree::Vector4 m_leftFootOrientSolution, m_rightFootOrientSolution;
    iDynTree::Position m_globalCoP;

    KinDynFusion::IK::IKHelper m_ikHelper;

    WearableSensorTargetMap m_wearableTargets;
    std::vector<KinDynFusion::IK::TargetName> m_wearableTargetNames;
    std::string m_baseTargetName;
    std::string m_leftFootTargetName, m_rightFootTargetName;

    WearableSensorizedShoeMap m_wearableShoes;
    std::vector<std::string> m_wearableShoeNames;
    CoPVertexSchmittTrigger m_leftFootContactDetector;
    CoPVertexSchmittTrigger m_rightFootContactDetector;
    std::vector<std::string> m_candidateContactFrames;
    BipedalLocomotion::Contacts::EstimatedContactList m_lfContacts;
    BipedalLocomotion::Contacts::EstimatedContactList m_rfContacts;

    KinDynFusion::IK::ModelData m_modelData;
    KinDynFusion::IK::IKParameters m_ikParams;

    DiligentOdom m_baseEKF;
    std::shared_ptr<iDynTree::KinDynComputations> m_baseEKFKinDyn;
    BipedalLocomotion::Estimators::FloatingBaseEstimators::Output m_baseEKFOutput;
    bool m_isBaseEKFStarted{false};
    double m_constantFloorHeight{0.0};
    bool m_forceFlatFloor{false};

    Eigen::Matrix3d m_I3;
    Eigen::Vector3d m_v3;

    mutable std::mutex m_mutex;

    std::unordered_map<std::string,
                       KinDynFusion::IK::KinematicTargetType> stringToKinematicTargetType{{"pose",                   KinDynFusion::IK::KinematicTargetType::pose},
                                                                                          {"poseAndVelocity",        KinDynFusion::IK::KinematicTargetType::poseAndVelocity},
                                                                                          {"position",               KinDynFusion::IK::KinematicTargetType::position},
                                                                                          {"positionAndVelocity",    KinDynFusion::IK::KinematicTargetType::positionAndVelocity},
                                                                                          {"orientation",            KinDynFusion::IK::KinematicTargetType::orientation},
                                                                                          {"orientationAndVelocity", KinDynFusion::IK::KinematicTargetType::orientationAndVelocity},
                                                                                          {"gravity",                KinDynFusion::IK::KinematicTargetType::gravity}};
};

} // namespace Estimators
} // namespace KinDynFusion

#endif // KINDYNFUSION_ESTIMATION_WHOLE_BODY_KINEMATICS_H
