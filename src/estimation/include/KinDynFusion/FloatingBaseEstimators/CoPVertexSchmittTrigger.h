/**
 * @file CoPVertexSchmittTrigger.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_ESTIMATION_COP_VERTEX_SCHMITT_TRIGGER_H
#define KINDYNFUSION_ESTIMATION_COP_VERTEX_SCHMITT_TRIGGER_H

#include <BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <KinDynFusion/Model/RectangularFoot.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Wrench.h>
#include <Eigen/Dense>

#include <unordered_map>
#include <array>

namespace KinDynFusion
{
namespace Estimators
{

struct FootMetaData
{
    std::string footLinkName;
    std::string soleFrame;
    iDynTree::Vector3 soleRPYInDegInFoot;
    iDynTree::Position solePositionInFoot;
    double footLength;
    double footWidth;
    iDynTree::Position topLeftPositionInSole;
    double contactForceThresholdForCOPComputation;
    BipedalLocomotion::Contacts::SchmittTriggerParams schmittParams;

    // used externally - associated FT sensor
    std::string wearableName;
};

class CoPVertexSchmittTrigger
{
public:
    bool setSoleInFootLink(const std::string& footLinkName,
                           const std::string& soleFrame,
                           const std::string& wearableName,
                           const iDynTree::Vector3& footRsoleRPYInDeg,
                           const iDynTree::Position& footpsole);
    bool setRectangularFoot(const double& xLength,
                            const double& yLength,
                            const iDynTree::Position& topLeftPositionInSole,
                            const double& fzThreshold);
    bool setSchmittParams(const BipedalLocomotion::Contacts::SchmittTriggerParams& schmittParams);

    // call this to modify the model
    // by adding the candidate contact point
    // frames to the model
    // which will be used by the base estimator
    bool initializeAndAddCandidateContactFramesToModel(iDynTree::Model& model);

    bool setNetWrenchInSole(const iDynTree::Wrench& wrench,
                            const double& timeNow);
    bool advance();

    const BipedalLocomotion::Contacts::EstimatedContactList& getContactStates() const;
    const BipedalLocomotion::Contacts::SchmittTriggerParams& getSchmittTriggerParameters() const;

    const std::string& getSoleFrame() const;
    const std::string& getFootLinkName() const;
    const std::string& getAssociatedWearableName() const;
    const iDynTree::Transform& getFootHSole() const;
    bool getContactFramePositionInFootLink(const std::string& frameName,
                                           iDynTree::Position& pos);
    std::vector<std::string> getCandidateContactFrames();
    const KinDynFusion::Model::RectangularFoot& getFootRectangle() const;
    double getVertexContactNormalForce(const std::string& vertexName);
    iDynTree::Position getCenterOfPressureInLinkFrame();
    void printMetaData();
private:
    std::string m_footLinkName; // link from the URDF model
    std::string m_soleFrame; // associated sole
    std::string m_wearableName; // associated sensor
    iDynTree::Transform m_footHsole;

    std::array<std::string, 4> m_contactFrameNames;
    std::unordered_map<std::string, iDynTree::Transform> m_contactFramePoseInFoot;

    // Rectangular foot related data
    KinDynFusion::Model::RectangularFoot m_soleRectangle;
    BipedalLocomotion::Contacts::SchmittTriggerParams m_schmittParams;
    BipedalLocomotion::Contacts::SchmittTriggerDetector m_schmittTrigger;

    // inputs
    iDynTree::Wrench m_inWrench;
    double m_timeNow;

    bool m_initialized{true};
};

}
}
#endif // KINDYNFUSION_ESTIMATION_COP_VERTEX_SCHMITT_TRIGGER_H
