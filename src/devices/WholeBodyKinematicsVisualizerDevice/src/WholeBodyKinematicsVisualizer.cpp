/**
 * @file WholeBodyKinematicsVisualizer.cpp
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */


#include <KinDynFusion/WholeBodyKinematicsVisualizer.h>
#include <iDynTree/Core/EigenHelpers.h>

using namespace KinDynFusion;

WholeBodyKinematicsVisualizer::WholeBodyKinematicsVisualizer()
{
    red  = iDynTree::ColorViz(1.0, 0.0, 0.0, 1.0);
    green= iDynTree::ColorViz(0.0, 1.0, 0.0, 1.0);
    blue = iDynTree::ColorViz(0.0, 0.0, 1.0, 1.0);
    black= iDynTree::ColorViz(0.0, 0.0, 0.0, 1.0);
    grey = iDynTree::ColorViz(0.5, 0.5, 0.5, 1.0);
}

bool WholeBodyKinematicsVisualizer::initialize()
{
    m_viz.init();
    m_viz.camera().animator()->enableMouseControl();

    iDynTree::Position cameraDeltaPosition(1.0, 0.0, 0.5);
    iDynTree::Position fixedCameraTarget(0, 0, 0);

    m_viz.camera().setPosition(cameraDeltaPosition);
    m_viz.camera().setTarget(fixedCameraTarget);

    m_viz.setColorPalette("meshcat");

    m_viz.environment().setBackgroundColor(grey);
    m_viz.environment().setFloorGridColor(black);
    return true;
}

bool WholeBodyKinematicsVisualizer::advance()
{
    auto& vectors = m_viz.vectors();

    for (auto& [modelName, modelData] : m_vizModelMap)
    {
        m_viz.modelViz(modelName).setPositions(modelData.w_H_b,
                                               modelData.jPos);
        updateVertexViz(modelData);
        updateCoPViz(modelData);
    }

    vectors.setVectorsAspect(m_zeroModolusRadius,
                             m_modulusMultiplier,
                             m_heightScale);

    m_viz.draw();
    m_viz.run();
    return true;
}


void WholeBodyKinematicsVisualizer::close()
{
    m_viz.close();
}

void WholeBodyKinematicsVisualizer::updateCoPViz(VizModelData& modelData)
{
    if (modelData.copVectors.size() < 1)
    {
        return;
    }

    auto& vectors = m_viz.vectors();
    for (auto& [copName, cop] : modelData.copVectors)
    {
        if (!cop.addedToVisualizer)
        {
            cop.idx = vectors.addVector(cop.pos,
                                        cop.dir,
                                        cop.magnitude);
            vectors.setVectorColor(cop.idx, black);
            cop.addedToVisualizer = true;
        }
        else
        {
            vectors.updateVector(cop.idx,
                                    cop.pos,
                                    cop.dir,
                                    cop.magnitude);
        }
    }
}

void WholeBodyKinematicsVisualizer::updateVertexViz(VizModelData& modelData)
{
    if (modelData.vertexVectors.size() < 1)
    {
        return;
    }

    auto& vectors = m_viz.vectors();
    for (auto& [vertexName, vertex] : modelData.vertexVectors)
    {
        if (!vertex.addedToVisualizer)
        {
            vertex.idx = vectors.addVector(vertex.pos,
                                           vertex.dir,
                                           vertex.magnitude);
            vectors.setVectorColor(vertex.idx, red);
            vertex.addedToVisualizer = true;
        }
        else
        {
            vectors.updateVector(vertex.idx,
                                 vertex.pos,
                                 vertex.dir,
                                 vertex.magnitude);
            vertex.isActive ? vectors.setVectorColor(vertex.idx, blue) :
                            vectors.setVectorColor(vertex.idx, red);
        }
    }
}

bool WholeBodyKinematicsVisualizer::addModel(const std::string& modelName,
                                             const iDynTree::Model& model)
{
    const std::string printPrefix{"WholeBodyKinematicsVisualizer::addModel "};
    if (m_vizModelMap.find(modelName) != m_vizModelMap.end())
    {
        std::cerr << printPrefix
                  << "Model name already exists."
                  << " Please choose a different model name."
                  << std::endl;
        return false;
    }

    m_viz.addModel(model, modelName);

    m_vizModelMap[modelName] = VizModelData();
    m_vizModelMap[modelName].w_H_b = iDynTree::Transform::Identity();
    m_vizModelMap[modelName].jPos.resize(model.getNrOfDOFs());
    m_vizModelMap[modelName].jPos.zero();
    m_vizModelMap[modelName].copVectors.clear();
    m_vizModelMap[modelName].vertexVectors.clear();

    return true;
}

bool WholeBodyKinematicsVisualizer::addModel(const std::string& modelName,
                                             const iDynTree::Model& model,
                                             const iDynTree::ColorViz& color)
{
    auto ok = addModel(modelName, model);
    if (ok)
    {
        m_viz.modelViz(modelName).setModelColor(color);
    }

    return ok;
}

bool WholeBodyKinematicsVisualizer::setConfiguration(const std::string& modelName,
                                                     const iDynTree::Transform& w_H_b,
                                                     const iDynTree::VectorDynSize& jPos)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    const std::string printPrefix{"WholeBodyKinematicsVisualizer::setConfiguration "};
    if (!checkModelExists(modelName, printPrefix))
    {
        return false;
    }

    m_vizModelMap.at(modelName).w_H_b = w_H_b;
    iDynTree::toEigen(m_vizModelMap.at(modelName).jPos) = iDynTree::toEigen(jPos);

    return true;
}

bool WholeBodyKinematicsVisualizer::addCenterOfPressureViz(const std::string& modelName,
                                                           const std::string& lcopName,
                                                           const std::string& rcopName)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    const std::string printPrefix{"WholeBodyKinematicsVisualizer::addCenterOfPressureViz "};
    if (!checkModelExists(modelName, printPrefix))
    {
        return false;
    }

    VertexViz vertex;
    initializeVertex(lcopName, vertex);
    m_vizModelMap.at(modelName).copVectors[lcopName] = vertex;

    vertex.name = rcopName;
    m_vizModelMap.at(modelName).copVectors[rcopName] = vertex;

    return true;
}

bool WholeBodyKinematicsVisualizer::addContactNormalViz(const std::string& modelName,
                                                        const std::string& vertexName)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    const std::string printPrefix{"WholeBodyKinematicsVisualizer::addContactNormalViz "};
    if (!checkModelExists(modelName, printPrefix))
    {
        return false;
    }

    VertexViz vertex;
    initializeVertex(vertexName, vertex);
    m_vizModelMap.at(modelName).vertexVectors[vertexName] = vertex;

    return true;
}

void WholeBodyKinematicsVisualizer::initializeVertex(const std::string& vertexName,
                                                     VertexViz& vertex)
{
    vertex.name = vertexName;
    vertex.dir = iDynTree::Direction(0., 0., 1.);
    vertex.magnitude = 0.0;
    vertex.isActive = false;
    vertex.pos = iDynTree::Position(0., 0., 0.);
}

bool WholeBodyKinematicsVisualizer::updateCenterOfPressureViz(const std::string& modelName,
                                                               const std::string& copName,
                                                               const iDynTree::Position& positionInInertial)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    const std::string printPrefix{"WholeBodyKinematicsVisualizer::updateCenterOfPressurelViz "};
    if (!checkCoPExists(modelName, copName))
    {
        std::cerr << printPrefix
                  << "Invalid inputs" << std::endl;
        return false;
    }

    auto& vertex = m_vizModelMap.at(modelName).copVectors.at(copName);
    vertex.pos = positionInInertial;

    return true;
}

bool WholeBodyKinematicsVisualizer::updateContactNormalViz(const std::string& modelName,
                                                           const std::string& vertexName,
                                                           const iDynTree::Position& positionInInertial,
                                                           const double& magnitude,
                                                           const bool& isActive)
{
    std::lock_guard<std::mutex> lock{m_mutex};
    const std::string printPrefix{"WholeBodyKinematicsVisualizer::updateContactNormalViz "};
    if (!checkContactNormalExists(modelName, vertexName))
    {
        std::cerr << printPrefix
                  << "Invalid inputs" << std::endl;
        return false;
    }

    auto& vertex = m_vizModelMap.at(modelName).vertexVectors.at(vertexName);
    vertex.pos = positionInInertial;
    vertex.isActive = isActive;
    vertex.magnitude = magnitude;

    return true;
}

bool WholeBodyKinematicsVisualizer::checkModelExists(const std::string& modelName,
                                                     const std::string& callerMethodPrefix)
{
    if (m_vizModelMap.find(modelName) == m_vizModelMap.end())
    {
        std::cerr << callerMethodPrefix
                  << "Model does not exist."
                  << std::endl;
        return false;
    }
    return true;
}

bool WholeBodyKinematicsVisualizer::checkContactNormalExists(const std::string& modelName,
                                                             const std::string& vertexName)
{
    if (m_vizModelMap.find(modelName) == m_vizModelMap.end())
    {
        return false;
    }

    if (m_vizModelMap.at(modelName).vertexVectors.find(vertexName) ==
        m_vizModelMap.at(modelName).vertexVectors.end())
    {
        return false;
    }

    return true;
}

bool WholeBodyKinematicsVisualizer::checkCoPExists(const std::string& modelName,
                                                   const std::string& vertexName)
{
    if (m_vizModelMap.find(modelName) == m_vizModelMap.end())
    {
        return false;
    }

    if (m_vizModelMap.at(modelName).copVectors.find(vertexName) ==
        m_vizModelMap.at(modelName).copVectors.end())
    {
        return false;
    }

    return true;
}

bool WholeBodyKinematicsVisualizer::getWorldLinkTransform(const std::string& modelName,
                                                          const std::string& linkName,
                                                          iDynTree::Transform& w_H_l)
{
    const std::string printPrefix{"WholeBodyKinematicsVisualizer::getWorldLinkTransform "};
    if (m_vizModelMap.find(modelName) == m_vizModelMap.end())
    {
        return false;
    }

    if (m_viz.modelViz(modelName).model().getLinkIndex(linkName) == iDynTree::LINK_INVALID_INDEX)
    {
        std::cerr << printPrefix
                  << "Invalid link name." << std::endl;
        return false;
    }

    w_H_l = m_viz.modelViz(modelName).getWorldLinkTransform(linkName);
    return true;
}

