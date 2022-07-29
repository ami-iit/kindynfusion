/**
 * @file CoPVertexSchmittTrigger.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/FloatingBaseEstimators/CoPVertexSchmittTrigger.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <cmath>

using namespace KinDynFusion::Model;
using namespace KinDynFusion::Estimators;
using namespace BipedalLocomotion::Contacts;
using namespace BipedalLocomotion::ParametersHandler;


bool CoPVertexSchmittTrigger::setSoleInFootLink(const std::string& footLinkName,
                                                const std::string& soleFrame,
                                                const std::string& wearableName,
                                                const iDynTree::Vector3& footRsoleRPYInDeg,
                                                const iDynTree::Position& footpsole)
{
    m_footLinkName = footLinkName;
    m_soleFrame = soleFrame;
    m_wearableName = wearableName;
    m_footHsole = iDynTree::Transform(iDynTree::Rotation::RPY(M_PI*footRsoleRPYInDeg(0)/180.0,
                                                              M_PI*footRsoleRPYInDeg(1)/180.0,
                                                              M_PI*footRsoleRPYInDeg(2)/180.0),
                                      footpsole);
    return true;
}

const iDynTree::Transform& CoPVertexSchmittTrigger::getFootHSole() const
{
    return m_footHsole;
}

const std::string& CoPVertexSchmittTrigger::getFootLinkName() const
{
    return m_footLinkName;
}

const std::string& CoPVertexSchmittTrigger::getSoleFrame() const
{
    return m_soleFrame;
}

const std::string& CoPVertexSchmittTrigger::getAssociatedWearableName() const
{
    return m_wearableName;
}

bool CoPVertexSchmittTrigger::setRectangularFoot(const double& xLength,
                                                 const double& yLength,
                                                 const iDynTree::Position& topLeftPositionInSole,
                                                 const double& fzThreshold)
{
    const std::string printPrefix{"CoPVertexSchmittTrigger::setRectangularFoot "};
    if (!m_soleRectangle.setFoot(xLength,
                                 yLength,
                                 iDynTree::toEigen(topLeftPositionInSole)))
    {
        std::cerr << printPrefix
                  << "Could not setup rectangular foot." << std::endl;
        return false;
    }

    m_soleRectangle.setNormalForceThresholdForContact(fzThreshold);
    return true;
}

iDynTree::Position CoPVertexSchmittTrigger::getCenterOfPressureInLinkFrame()
{
    iDynTree::Position copInSole;
    iDynTree::toEigen(copInSole) = m_soleRectangle.getCoPPositionInSoleFrame();
    return  m_footHsole*copInSole;
}

const RectangularFoot& CoPVertexSchmittTrigger::getFootRectangle() const
{
    return m_soleRectangle;
}

bool CoPVertexSchmittTrigger::setSchmittParams(const SchmittTriggerParams& schmittParams)
{
    m_schmittParams = schmittParams;
    return true;
}

const SchmittTriggerParams& CoPVertexSchmittTrigger::getSchmittTriggerParameters() const
{
    return m_schmittParams;
}

const EstimatedContactList& CoPVertexSchmittTrigger::getContactStates() const
{
    return m_schmittTrigger.getOutput();
}

bool CoPVertexSchmittTrigger::initializeAndAddCandidateContactFramesToModel(iDynTree::Model& model)
{
    const std::string printPrefix{"CoPVertexSchmittTrigger::initializeAndAddCandidateContactFramesToModel "};
    if (!model.isLinkNameUsed(m_footLinkName))
    {
        std::cerr << printPrefix
                  << m_footLinkName << " not available in the model." << std::endl;
        return false;
    }

    // if sole is not there, then add the sole frame
    if (!model.isFrameNameUsed(m_soleFrame))
    {
        if (!model.addAdditionalFrameToLink(m_footLinkName, m_soleFrame, m_footHsole))
        {
            std::cerr << printPrefix
                      << "Could not add frame " << m_soleFrame << "to link "
                      << m_footLinkName << " in the model." << std::endl;
            return false;
        }
        std::cout << printPrefix
                  << "Adding " << m_soleFrame << " to " << m_footLinkName
                  << " link to the URDF model with Transform: \n "
                  << m_footHsole.toString() << std::endl;
    }

    for (std::size_t idx = 0; idx < 4; idx++)
    {
        m_contactFrameNames[idx] =  m_soleFrame + "_vertex" + std::to_string(idx);
        m_contactFramePoseInFoot[m_contactFrameNames[idx]] = iDynTree::Transform::Identity();
        auto& vertexName = m_contactFrameNames[idx];
        auto& footHvertex = m_contactFramePoseInFoot.at(m_contactFrameNames[idx]);
        iDynTree::Position solepvertex;
        iDynTree::toEigen(solepvertex) = m_soleRectangle.getVertexPositionsInSoleFrame().col(idx);
        footHvertex.setPosition(m_footHsole*solepvertex);
        if (!model.addAdditionalFrameToLink(m_footLinkName,
                                            vertexName,
                                            footHvertex))
        {
            std::cerr << printPrefix
                      << "Could not add frame " << vertexName << "to link "
                      << m_footLinkName << " in the model." << std::endl;
            return false;
        }
        std::cout << printPrefix
                  << "Adding " << vertexName << " to " << m_footLinkName
                  << " link to the URDF model with Transform: \n "
                  << footHvertex.toString() << std::endl;
    }

    auto originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    parameterHandler->setParameter("contacts", std::vector<std::string>{m_contactFrameNames[0],
                                                                        m_contactFrameNames[1],
                                                                        m_contactFrameNames[2],
                                                                        m_contactFrameNames[3]});
    parameterHandler->setParameter("contact_make_thresholds", std::vector<double>{m_schmittParams.onThreshold,
                                                                                  m_schmittParams.onThreshold,
                                                                                  m_schmittParams.onThreshold,
                                                                                  m_schmittParams.onThreshold});
    parameterHandler->setParameter("contact_break_thresholds", std::vector<double>{m_schmittParams.offThreshold,
                                                                                   m_schmittParams.offThreshold,
                                                                                   m_schmittParams.offThreshold,
                                                                                   m_schmittParams.offThreshold});
    parameterHandler->setParameter("contact_make_switch_times", std::vector<double>{m_schmittParams.switchOnAfter,
                                                                                    m_schmittParams.switchOnAfter,
                                                                                    m_schmittParams.switchOnAfter,
                                                                                    m_schmittParams.switchOnAfter});
    parameterHandler->setParameter("contact_break_switch_times", std::vector<double>{m_schmittParams.switchOffAfter,
                                                                                     m_schmittParams.switchOffAfter,
                                                                                     m_schmittParams.switchOffAfter,
                                                                                     m_schmittParams.switchOffAfter});

    if (!m_schmittTrigger.initialize(parameterHandler))
    {
        std::cerr << printPrefix
                  << "Failed to initialize Schmitt Trigger."
                  << std::endl;
        return false;
    }

    m_initialized = true;
    return true;
}

bool CoPVertexSchmittTrigger::setNetWrenchInSole(const iDynTree::Wrench& wrench,
                                                 const double& timeNow)
{
    const std::string printPrefix{"CoPVertexSchmittTrigger::setNetWrenchInSole "};
    if (!m_initialized)
    {
        std::cerr << printPrefix
                  << "Initialize before setting inputs."
                  << std::endl;
        return false;
    }
    m_inWrench = wrench;
    m_timeNow = timeNow;
    return true;
}

bool CoPVertexSchmittTrigger::advance()
{
    const std::string printPrefix{"CoPVertexSchmittTrigger::advance "};
    if (!m_initialized)
    {
        std::cerr << printPrefix
                  << "Initialize before calling advance."
                  << std::endl;
        return false;
    }

    if (!m_soleRectangle.computeNormalForcesAtVertices(iDynTree::toEigen(m_inWrench)))
    {
        std::cerr << printPrefix
                  << "Could not compute contact normal forces at vertices."
                  << std::endl;
        return false;
    }

    for (std::size_t idx = 0; idx < 4; idx++)
    {
        m_schmittTrigger.setTimedTriggerInput(m_contactFrameNames[idx],
                                              m_timeNow,
                                              m_soleRectangle.getVertexNormalForces()(idx));
    }

    if (!m_schmittTrigger.advance())
    {
        std::cerr << printPrefix
                  << "Could not advance Schmitt Trigger."
                  << std::endl;
        return false;
    }

    return true;
}


std::vector<std::string> CoPVertexSchmittTrigger::getCandidateContactFrames()
{
    return std::vector<std::string>{m_contactFrameNames.begin(), m_contactFrameNames.end()};
}

bool CoPVertexSchmittTrigger::getContactFramePositionInFootLink(const std::string& frameName,
                                                                iDynTree::Position& pos)
{
    const std::string printPrefix{"CoPVertexSchmittTrigger::getContactFramePositionInFootLink "};
    if (m_contactFramePoseInFoot.find(frameName) == m_contactFramePoseInFoot.end())
    {
        std::cerr << printPrefix
                  << "Could not find frame " << frameName
                  << "." << std::endl;
        return false;
    }

    pos = m_contactFramePoseInFoot.at(frameName).getPosition();
    return true;
}


void CoPVertexSchmittTrigger::printMetaData()
{
    const std::string printPrefix{"CoPVertexSchmittTrigger::printMetaData "};
    std::cout << "********************"  << printPrefix << std::endl;
    std::cout << "Foot link name              : "   << m_footLinkName << std::endl;
    std::cout << "Sole frame name             : "   << m_soleFrame << std::endl;
    std::cout << "Sole length                 : "   << m_soleRectangle.getLength() << std::endl;
    std::cout << "Sole width                  : "   << m_soleRectangle.getWidth() << std::endl;
    std::cout << "CoP contact force threshold : "   << m_soleRectangle.getNormalForceThresholdForContact() << std::endl;
    std::cout << "Schmitt Param on threshold  : "   << m_schmittParams.onThreshold << std::endl;
    std::cout << "Schmitt Param off threshold : "   << m_schmittParams.offThreshold << std::endl;
    std::cout << "Schmitt Param on after (s)  : "   << m_schmittParams.switchOnAfter << std::endl;
    std::cout << "Schmitt Param off after (s) : "   << m_schmittParams.switchOffAfter << std::endl;
    std::cout << "Vertex positions in sole    : \n" << m_soleRectangle.getVertexPositionsInSoleFrame() << std::endl;
    std::cout << "foot_H_sole                 : \n" << m_footHsole.toString() << std::endl;
    std::cout << "********************" << std::endl;
}

double CoPVertexSchmittTrigger::getVertexContactNormalForce(const std::string& vertexName)
{
    std::size_t idx{0};
    for (auto& frame : m_contactFrameNames)
    {
        if (frame == vertexName)
        {
            return m_soleRectangle.getVertexNormalForces()(idx);
        }
        idx++;
    }

    return 0.0;
}


