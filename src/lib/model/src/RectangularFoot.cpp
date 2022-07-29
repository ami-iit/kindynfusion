/**
 * @file RectangularFoot.cpp
 * @authors Stefano Dafarra
 * modified by Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/Model/RectangularFoot.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <iostream>
using namespace BipedalLocomotion;
using namespace KinDynFusion::Model;
using namespace BipedalLocomotion::Math;

class RectangularFoot::Impl
{
public:
    void getNormalRatiosFromCoP(const double& copX,
                                const double& copY,
                                Eigen::Ref<Eigen::Matrix<double, 4, 1> > alphas);

    Eigen::Matrix<double, 4, 1> vertexNormalForces;
    Eigen::Matrix<double, 4, 1> alphas;
    Eigen::Vector3d footCenterInSole;
    Eigen::Vector3d topLeftPositionInSole;
    Eigen::Matrix<double, 3, 4> vertexPositionsInCenterFrame;
    Eigen::Matrix<double, 3, 4> vertexInSole;
    double halfL, halfD;
    double l, d;
    bool footSet;
    manif::SE3d center_H_sole;
    double fzThreshold{1e-7};

    double copXinCenterFrame{0.}, copYinCenterFrame{0.};
    Eigen::Vector3d copInCenter, copInSole;
    bool isCoPInside{false};
};

RectangularFoot::RectangularFoot() : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->footSet = false;
}

RectangularFoot::~RectangularFoot() = default;

bool RectangularFoot::setFoot(double xLength,
                              double yLength,
                              Eigen::Ref<const Eigen::Vector3d> topLeftPositionInSole)
{
    constexpr auto printPrefix{"[RectangularFoot::setFoot]"};

    if (xLength <= 0 || yLength <= 0)
    {
        log()->error("{} The dimensions of foot must be positive.", printPrefix);
        return false;
    }

    m_pimpl->l = xLength;
    m_pimpl->d = yLength;
    m_pimpl->halfL = xLength/2.0;
    m_pimpl->halfD = yLength/2.0;

    Eigen::Vector3d centerInTopLeftCoordinates;
    centerInTopLeftCoordinates << -m_pimpl->halfL, -m_pimpl->halfD, 0.0;
    m_pimpl->topLeftPositionInSole = topLeftPositionInSole;
    m_pimpl->footCenterInSole = topLeftPositionInSole + centerInTopLeftCoordinates;

    // this is different from the one used in the dynamical planner
    // the positions 1 and 2 are interchanged
    // top left + +
    m_pimpl->vertexPositionsInCenterFrame.col(0) << m_pimpl->halfL, m_pimpl->halfD, 0.0;

    // bottom left - +
    m_pimpl->vertexPositionsInCenterFrame.col(1) << -m_pimpl->halfL, m_pimpl->halfD, 0.0;

    // top right + -
    m_pimpl->vertexPositionsInCenterFrame.col(2) << m_pimpl->halfL, -m_pimpl->halfD, 0.0;

    // bottom right - -
    m_pimpl->vertexPositionsInCenterFrame.col(3) << -m_pimpl->halfL, -m_pimpl->halfD, 0.0;

    m_pimpl->footSet = true;

    // center to sole frame transform
    m_pimpl->center_H_sole.rotation().setIdentity();
    m_pimpl->center_H_sole.translation(-m_pimpl->footCenterInSole);

    // vertex in sole frame
    auto sole_H_center = m_pimpl->center_H_sole.inverse();
    m_pimpl->vertexInSole.col(0) = sole_H_center.act(m_pimpl->vertexPositionsInCenterFrame.col(0));
    m_pimpl->vertexInSole.col(1) = sole_H_center.act(m_pimpl->vertexPositionsInCenterFrame.col(1));
    m_pimpl->vertexInSole.col(2) = sole_H_center.act(m_pimpl->vertexPositionsInCenterFrame.col(2));
    m_pimpl->vertexInSole.col(3) = sole_H_center.act(m_pimpl->vertexPositionsInCenterFrame.col(3));
    return true;
}

void RectangularFoot::setNormalForceThresholdForContact(const double& forceZOffset)
{
    m_pimpl->fzThreshold = forceZOffset;
}

const Eigen::Vector3d& RectangularFoot::getCoPPositionInSoleFrame() const
{
    return m_pimpl->copInSole;
}

const bool& RectangularFoot::isCoPInsideSupportPolygon() const
{
    return m_pimpl->isCoPInside;
}

const double& RectangularFoot::getLength() const
{
    return m_pimpl->l;
}

const double& RectangularFoot::getWidth() const
{
    return m_pimpl->d;
}

const Eigen::Matrix<double, 3, 4>& RectangularFoot::getVertexPositionsInSoleFrame() const
{
    return m_pimpl->vertexInSole;
}

const double& RectangularFoot::getNormalForceThresholdForContact()
{
    return m_pimpl->fzThreshold;
}

const Eigen::Vector3d& RectangularFoot::getTopLeftPositionInSole() const
{
    return m_pimpl->topLeftPositionInSole;
}

const Eigen::Matrix<double, 4, 1>& RectangularFoot::getVertexNormalForces() const
{
    return m_pimpl->vertexNormalForces;
}

bool RectangularFoot::computeNormalForcesAtVertices(Eigen::Ref<const Eigen::Matrix<double, 6, 1> > wrenchInSoleFrame)
{
    constexpr auto printPrefix{"[RectangularFoot::computeNormalForcesAtVertices]"};

    if (!m_pimpl->footSet)
    {
        log()->error("{} The foot dimensions seem not to be set."
        "Please call setFoot() first.", printPrefix);
        return false;
    }

    Wrenchd wrench = wrenchInSoleFrame;

    Wrenchd wrenchInCenter = m_pimpl->center_H_sole*wrench;

    double fz{wrenchInCenter.force()(2)};
    double tx{wrenchInCenter.torque()(0)};
    double ty{wrenchInCenter.torque()(1)};

    m_pimpl->copXinCenterFrame = -ty/fz;
    m_pimpl->copYinCenterFrame = tx/fz;

    if ( (std::fabs(m_pimpl->copXinCenterFrame) > m_pimpl->halfL) ||
         (std::fabs(m_pimpl->copYinCenterFrame) > m_pimpl->halfD) )
    {
        m_pimpl->vertexNormalForces.setZero();
        log()->warn("{} CoP outside foot polygon, returning zero forces.", printPrefix);
        m_pimpl->isCoPInside = false;
        return true;
    }

    if (fz < m_pimpl->fzThreshold)
    {
        m_pimpl->vertexNormalForces.setZero();
        log()->warn("{} Contact normal force at sole is below threshold, returning zero forces.", printPrefix);
        m_pimpl->isCoPInside = false;
        return true;
    }

    m_pimpl->isCoPInside = true;
    m_pimpl->getNormalRatiosFromCoP(m_pimpl->copXinCenterFrame,
                                    m_pimpl->copYinCenterFrame,
                                    m_pimpl->alphas);

    for (size_t i = 0; i < 4; ++i)
    {
        m_pimpl->vertexNormalForces(i) = m_pimpl->alphas(i)*fz;
    }

    // transform CoP in sole frame

    m_pimpl->copInCenter << m_pimpl->copXinCenterFrame,
                            m_pimpl->copYinCenterFrame,
                            0.0;

    m_pimpl->copInSole = m_pimpl->center_H_sole.inverse().act(m_pimpl->copInCenter);

    return true;
}

void RectangularFoot::Impl::getNormalRatiosFromCoP(const double& copX,
                                                   const double& copY,
                                                   Eigen::Ref<Eigen::Matrix<double, 4, 1> > alphas)
{
    // this is different from the one used in the dynamical planner
    // the alpha 1 and 2 are interchanged to follow the
    // interchanged vertex ordering
    alphas(3) = (std::max(0.0, - copX/l - copY/d)  + std::min(0.5 - copY/d, 0.5 - copX/l))/2;
    alphas(0) = alphas(3) + copX/l + copY/d;
    alphas(2) = -alphas(3) + 0.5 - copY/d;
    alphas(1) = 1.0 - alphas(0) - alphas(2) - alphas(3);
}
