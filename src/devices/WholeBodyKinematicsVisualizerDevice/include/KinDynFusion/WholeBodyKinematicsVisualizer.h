/**
 * @file WholeBodyKinematicsVisualizer.h
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_WHOLE_BODY_KINEMATICS_VISUALIZER_H
#define KINDYNFUSION_WHOLE_BODY_KINEMATICS_VISUALIZER_H

#include <iDynTree/Visualizer.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Position.h>

#include <unordered_map>
#include <string>
#include <mutex>

namespace KinDynFusion
{

struct VertexViz
{
    std::size_t idx; // visualization Vectors ID
    std::string name;
    iDynTree::Position pos;
    iDynTree::Direction dir;
    double magnitude;
    bool isActive;
    bool addedToVisualizer{false};
};

struct VizModelData
{
    iDynTree::Transform w_H_b;
    iDynTree::JointPosDoubleArray jPos;
    std::unordered_map<std::string, VertexViz> vertexVectors;
    std::unordered_map<std::string, VertexViz> copVectors; // center of pressure
};

using VizModelMap = std::unordered_map<std::string, VizModelData>;

class WholeBodyKinematicsVisualizer
{
public:
    WholeBodyKinematicsVisualizer();
    bool initialize();
    bool addModel(const std::string& modelName,
                  const iDynTree::Model& model);
    bool addModel(const std::string& modelName,
                  const iDynTree::Model& model,
                  const iDynTree::ColorViz& color);
    bool addContactNormalViz(const std::string& modelName,
                             const std::string& vertexName);
    bool addCenterOfPressureViz(const std::string& modelName,
                                const std::string& lcopName,
                                const std::string& rcopName);

    bool setConfiguration(const std::string& modelName,
                          const iDynTree::Transform& w_H_b,
                          const iDynTree::VectorDynSize& jPos);
    bool updateContactNormalViz(const std::string& modelName,
                                const std::string& vertexName,
                                const iDynTree::Position& positionInInertial,
                                const double& magnitude,
                                const bool& isActive);
    bool updateCenterOfPressureViz(const std::string& modelName,
                                   const std::string& copName,
                                   const iDynTree::Position& positionInInertial);
    bool getWorldLinkTransform(const std::string& modelName,
                               const std::string& linkName,
                               iDynTree::Transform& w_H_l);
    bool advance();
    void close();

private:
    bool checkModelExists(const std::string& modelName,
                          const std::string& callerMethodPrefix);
    bool checkContactNormalExists(const std::string& modelName,
                                  const std::string& vertexName);
    bool checkCoPExists(const std::string& modelName,
                        const std::string& copName);
    void initializeVertex(const std::string& vertexName,
                          VertexViz& vertex);

    void updateVertexViz(KinDynFusion::VizModelData& modelData);
    void updateCoPViz(KinDynFusion::VizModelData& modelData);

    std::mutex m_mutex;
    iDynTree::Visualizer m_viz;
    VizModelMap m_vizModelMap;
    iDynTree::ColorViz red;
    iDynTree::ColorViz blue;
    iDynTree::ColorViz green;
    iDynTree::ColorViz black;
    iDynTree::ColorViz grey;

    double m_zeroModolusRadius{0.01};
    double m_modulusMultiplier{0.001};
    double m_heightScale{0.005};
};

}

#endif // KINDYNFUSION_WHOLE_BODY_KINEMATICS_VISUALIZER_H
