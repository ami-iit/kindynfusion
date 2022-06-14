/**
 * @file RectangularFoot.h
 * @authors Stefano Dafarra
 * modified by Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYN_FUSION_MODEL_RECTANGULAR_FOOT_H
#define KINDYN_FUSION_MODEL_RECTANGULAR_FOOT_H

#include <Eigen/Dense>
#include <memory>
namespace KinDynFusion
{
namespace Model
{
    class RectangularFoot;
}
}

class KinDynFusion::Model::RectangularFoot
{
public:
    RectangularFoot();
    ~RectangularFoot();

    // Assumption for the sole frame
    // The z-direction is supposed to be perpendicular to the foot surface. X is considered forward
    bool setFoot(double xLength, double yLength,
                 Eigen::Ref<const Eigen::Vector3d> topLeftPositionInSole);

    // in case there are offsets while passing
    // estimated/noisy FT sensor measurements
    // default value at 1e-7
    void setNormalForceThresholdForContact(const double& forceZOffset);
    const double& getNormalForceThresholdForContact();

    const double& getLength() const;
    const double& getWidth() const;
    const Eigen::Vector3d& getTopLeftPositionInSole() const;

    const Eigen::Matrix<double, 3, 4>& getVertexPositionsInSoleFrame() const;

    // In a frame with rotation the same as foot center position
    // position of the forces at the vertices
    bool computeNormalForcesAtVertices(Eigen::Ref<const Eigen::Matrix<double, 6, 1> > wrenchInSoleFrame);

    // Will be valid only after computeNormalForcesAtVertices() is called
    const Eigen::Matrix<double, 4, 1>& getVertexNormalForces() const;
    const Eigen::Vector3d& getCoPPositionInSoleFrame() const;
    const bool& isCoPInsideSupportPolygon() const;


private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

#endif // KINDYN_FUSION_MODEL_RECTANGULAR_FOOT_H
