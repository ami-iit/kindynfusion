/**
 * @file DiligentOdom.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYN_FUSION_ESTIMATION_DILIGENT_ODOM_H
#define KINDYN_FUSION_ESTIMATION_DILIGENT_ODOM_H


#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>

namespace KinDynFusion
{
namespace Estimators
{


class DiligentOdom : public BipedalLocomotion::Estimators::FloatingBaseEstimator
{
public:
    /**
     * Constructor
     */
    DiligentOdom();

    /**
     * Destructor (necessary for PIMPL idiom)
     */
    ~DiligentOdom();

    /**
     * To prevent function hiding due to overloading of virtual methods
     */
    using BipedalLocomotion::Estimators::FloatingBaseEstimator::resetEstimator;

    /**
     * Reset the base pose estimate and consequently the internal state of the estimator
     * @param[in] newState internal state of the estimator
     * @param[in] newPriorDev internal state priors
     * @return True in case of success, false otherwise.
     *
     * @note reset and advance estimator to get updated estimator output
     */
    bool resetEstimator(const BipedalLocomotion::Estimators::FloatingBaseEstimators::InternalState& newState,
                        const BipedalLocomotion::Estimators::FloatingBaseEstimators::StateStdDev& newPriorDev);

    /**
     * Reset the base pose estimate and consequently the internal state of the estimator
     * @param[in] newState internal state of the estimator
     * @param[in] newPriorDev internal state priors
     * @param[in] newSensorsDev sensor measurement noise
     * @return True in case of success, false otherwise.
     *
     * @note reset and advance estimator to get updated estimator output
     */
    bool resetEstimator(const BipedalLocomotion::Estimators::FloatingBaseEstimators::InternalState& newState,
                        const BipedalLocomotion::Estimators::FloatingBaseEstimators::StateStdDev& newPriorDev,
                        const BipedalLocomotion::Estimators::FloatingBaseEstimators::SensorsStdDev& newSensorsDev);

    bool useFlatFloorMeasurements(const bool& flag);

    bool setContactHeight(const std::string& name,
                          const double& contactHeight);
    bool setLeftTrivializedBaseVelocity(Eigen::Ref<Eigen::Vector3d> omega);
    bool setWorldFrameInRefTo(const std::string& refModelLinkName,
                              Eigen::Ref<Eigen::Matrix3d> w_R_l,
                              Eigen::Ref<Eigen::Vector3d> w_p_l);
    bool setSameFootVertices(const std::string& footSoleFrame,
                             const std::vector<std::string>& vertexNames);
    // Assumption: Foot makes planar contact and plane orientation is same as foot orientation
    bool setContactPlaneOrientation(const std::string& footSoleName,
                                    Eigen::Quaterniond orientation);

protected:
    /**
    * These custom parameter specifications should be specified by the derived class.
    * @param[in] handler configure the custom parameters for the estimator
    * @return bool
    */
    virtual bool customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;


    bool setupCustomSensorDevs(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
    * Propagate the states through the prediction model, if there exists any (eg. a strap-down IMU model)
    * @param[in] meas measurements passed as exogeneous inputs to the prediction model
    * @param[in] dt sampling period in seconds
    * @return True in case of success, false otherwise.
    */
    virtual bool predictState(const BipedalLocomotion::Estimators::FloatingBaseEstimators::Measurements& meas,
                              const double& dt) override;

    /**
    * Update the predicted state estimates using kinematics and contact measurements
    * @param[in] meas measurements to update the predicted states
    * @param[in] dt sampling period in seconds
    * @return True in case of success, false otherwise.
    */
    virtual bool updateKinematics(BipedalLocomotion::Estimators::FloatingBaseEstimators::Measurements& meas,
                                  const double& dt) override;

private:
    /**
    * Private implementation of the class
    */
    class Impl;
    std::unique_ptr<Impl> m_pimpl; /**< Pointer to implementation */
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // KINDYN_FUSION_ESTIMATION_DILIGENT_ODOM_H
