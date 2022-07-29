/**
 * @file DiligentOdom.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/FloatingBaseEstimators/DiligentOdom.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <iDynTree/Model/Model.h>
#include <manif/manif.h>
#include <unordered_map>
#include <cmath>
#include <Eigen/Dense>
using namespace KinDynFusion::Estimators;
using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::Conversions;
using namespace BipedalLocomotion;
class DiligentOdom::Impl
{
public:
    void propagateStates(const FloatingBaseEstimators::Measurements& meas,
                         const double& dt,
                         FloatingBaseEstimators::InternalState& X);

    bool constructStateVar(const FloatingBaseEstimators::InternalState& state,
                           const FloatingBaseEstimators::StateStdDev& stateStdDev,
                           Eigen::MatrixXd& P);

    void calcFc(const FloatingBaseEstimators::InternalState& X,
                const std::size_t& dim,
                Eigen::MatrixXd& Fc);

    void calcLc(const FloatingBaseEstimators::InternalState& X,
                const std::size_t& dim,
                Eigen::MatrixXd& Lc);

    void calcQc(const FloatingBaseEstimators::InternalState& X,
                const FloatingBaseEstimators::SensorsStdDev& sensDev,
                const FloatingBaseEstimators::Measurements& meas,
                const std::size_t& dim,
                Eigen::MatrixXd& Qc);

    bool extractStateVar(const Eigen::MatrixXd& P,
                         const std::size_t& dim,
                         const  FloatingBaseEstimators::InternalState& state,
                         FloatingBaseEstimators::StateStdDev& stateStdDev);

    bool updateWithRelativeVertexPositions(FloatingBaseEstimators::Measurements& meas,
                                           FloatingBaseEstimator::ModelComputations& modelComp,
                                           const std::size_t& Xsize,
                                           const double& dt,
                                           FloatingBaseEstimators::InternalState& state);

    bool updateWithRelativeFootOrientations(FloatingBaseEstimators::Measurements& meas,
                                            FloatingBaseEstimator::ModelComputations& modelComp,
                                            const double& dt,
                                            FloatingBaseEstimators::InternalState& state);

    bool updateWithTerrainHeight(FloatingBaseEstimators::Measurements& meas,
                                 FloatingBaseEstimator::ModelComputations& modelComp,
                                 const double& dt,
                                 const int& nrContacts,
                                 const FloatingBaseEstimators::StateStdDev& stateDev,
                                 const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                 FloatingBaseEstimators::InternalState& state);

    bool updateWithZUPTLinearVelocity(FloatingBaseEstimators::Measurements& meas,
                                      FloatingBaseEstimator::ModelComputations& modelComp,
                                      const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                      const std::size_t& Xsize,
                                      const double& dt,
                                      const int& nrContacts,
                                      FloatingBaseEstimators::InternalState& state);

    bool updateWithZUPTAngularVelocity(FloatingBaseEstimators::Measurements& meas,
                                       FloatingBaseEstimator::ModelComputations& modelComp,
                                       const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                       const std::size_t& Xsize,
                                       const double& dt,
                                       const int& nrContacts,
                                       FloatingBaseEstimators::InternalState& state);
    bool updateWithBaseAngularVelocity(const std::size_t& Xsize,
                                       const double& dt,
                                       FloatingBaseEstimators::InternalState& state);
    bool updateWithContactPlaneOrientation(FloatingBaseEstimators::Measurements& meas,
                                           FloatingBaseEstimator::ModelComputations& modelComp,
                                           const double& dt,
                                           const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                           FloatingBaseEstimators::InternalState& state);
    void checkSoleRigidContact(const std::string soleName,
                               FloatingBaseEstimators::Measurements& meas,
                               FloatingBaseEstimator::ModelComputations& modelComp,
                               std::size_t& nrContacts,
                               bool& isInRigidContact);
    bool isContactPointInSole(const std::string soleName,
                              FloatingBaseEstimators::Measurements& meas,
                              FloatingBaseEstimator::ModelComputations& modelComp,
                              const int& id);

    bool updateLeftInvariantObservations(Eigen::Ref<const Eigen::VectorXd> Y,
                                         Eigen::Ref<const Eigen::VectorXd> b,
                                         Eigen::Ref<const Eigen::MatrixXd> H,
                                         Eigen::Ref<const Eigen::MatrixXd> N,
                                         Eigen::Ref<const Eigen::MatrixXd> Pi,
                                         FloatingBaseEstimators::InternalState& state,
                                         Eigen::MatrixXd& P);

    bool updateRightInvariantObservations(Eigen::Ref<const Eigen::VectorXd> Y,
                                          Eigen::Ref<const Eigen::VectorXd> b,
                                          Eigen::Ref<const Eigen::MatrixXd> H,
                                          Eigen::Ref<const Eigen::MatrixXd> N,
                                          Eigen::Ref<const Eigen::MatrixXd> Pi,
                                          FloatingBaseEstimators::InternalState& state,
                                          Eigen::MatrixXd& P);

    bool updateNonInvariantObservations(Eigen::Ref<const Eigen::VectorXd> deltaY,
                                        Eigen::Ref<const Eigen::MatrixXd> H,
                                        Eigen::Ref<const Eigen::MatrixXd> N,
                                        FloatingBaseEstimators::InternalState& state,
                                        Eigen::MatrixXd& P);

    bool addContact(const int& idx,
                    const double& time,
                    const bool& isActive,
                    const manif::SE3d& poseEstimate,
                    const FloatingBaseEstimators::SensorsStdDev& sensStdDev,
                    const double& dt,
                    FloatingBaseEstimators::InternalState& state,
                    FloatingBaseEstimators::StateStdDev& stateDev,
                    Eigen::MatrixXd& P);

    bool removeContact(const int& idx,
                       FloatingBaseEstimators::InternalState& state,
                       FloatingBaseEstimators::StateStdDev& stateDev,
                       Eigen::MatrixXd& P);

    void constructMatrixLieGroup(const FloatingBaseEstimators::InternalState& state,
                                 Eigen::MatrixXd& X);
    bool extractFromMatrixLieGroup(const Eigen::MatrixXd& X,
                                   FloatingBaseEstimators::InternalState& state);

    bool calcExpHatX(const Eigen::VectorXd& v, Eigen::MatrixXd& X);
    bool composeX(const Eigen::MatrixXd& X1,
                  const Eigen::MatrixXd& X2,
                  Eigen::MatrixXd& Xout);
    bool calcljac(const Eigen::VectorXd& v, Eigen::MatrixXd& J);
    bool calcAdjX(const FloatingBaseEstimators::InternalState& state,
                  Eigen::MatrixXd& AdjX);
    void calcXinv(const FloatingBaseEstimators::InternalState& state,
                  FloatingBaseEstimators::InternalState& Xinv);

    bool compareKeys(const std::map<int, BipedalLocomotion::Contacts::EstimatedContact>& lhs,
                     const FloatingBaseEstimators::PoseCovariance& rhs)
    {
        return (lhs.size() == rhs.size()) &&
               (std::equal(lhs.begin(), lhs.end(), rhs.begin(),
                           [] (auto a, auto b) { return a.first == b.first; }));
    }

    std::size_t stateDimensions(const FloatingBaseEstimators::InternalState& state)
    {
        // base pose and linear vel,
        // feet vertices,
        // base angular vel,
        // lf rotation and rf rotation
        return extMotionDim +
               (dim3*state.supportFrameData.size()) +
               (dim3) +
               (dim3*nrFeet);
    }

    std::size_t imuAngVelOffset(const FloatingBaseEstimators::InternalState& state)
    {
        return extMotionDim + (dim3*state.supportFrameData.size());
    }

    std::size_t lfRotOffset(const FloatingBaseEstimators::InternalState& state)
    {
        return extMotionDim + (dim3*state.supportFrameData.size()) + dim3;
    }

    const std::size_t nrFeet{2};
    const std::size_t dim3{3};
    const std::size_t extMotionDim{9}; // R p v
    const std::size_t motionDim{6};
    const std::size_t imuPositionOffset{0};
    const std::size_t imuOrientationOffset{3};
    const std::size_t imuLinearVelOffset{6};
    Eigen::MatrixXd m_P, m_Fc, m_Qc, m_Lc;
    Eigen::MatrixXd m_Fk, m_Qk;
    Eigen::MatrixXd m_Hk, m_Nk, m_Pik;
    Eigen::VectorXd m_Yk, m_bk;
    Eigen::Matrix3d m_I3;
    Eigen::MatrixXd m_In;
    Eigen::MatrixXd F_J_IMUF, F_S_FIMU, m_Nenc, IMU_J_IMUF;
    Eigen::VectorXd m_encodersVar;
    Eigen::MatrixXd m_PHT, m_S, m_K, m_IminusKH;
    Eigen::MatrixXd m_AdjX, m_AdjXinv, m_Pl, m_Xinv;
    FloatingBaseEstimators::InternalState m_stateInv;

    Eigen::MatrixXd m_X, m_dX, m_Xout, m_dXJr;
    Eigen::VectorXd m_deltaX, m_deltaY;
    Eigen::Matrix<double, 6, 6> m_Jb;
    Eigen::Matrix<double, 6, 1> m_vb;

    Eigen::Matrix3d IMU_R_B;
    Eigen::Vector3d m_omegab; // base angular velocity measurement
    Eigen::Vector3d m_omegabNoise;

    Eigen::MatrixXd Fnew, Qnew;
    std::vector<int> existingContact;
    std::unordered_map<std::string, std::vector<std::string>> footVertexMap;
    std::unordered_map<std::string, Eigen::Quaterniond> footPlaneOrientationMeasurementMap;

    bool m_forceFlatFloor{false};
    bool m_useVelocityMeasurements{false};
    bool m_useZeroBaseVelocities{false};
};


DiligentOdom::DiligentOdom() : m_pimpl(std::make_unique<Impl>())
{
    m_useIMUForAngVelEstimate = false; // we do not use IMU measurements for computing angular velocity
    m_useIMUVelForBaseVelComputation = true; // we do all computations wrt IMU frame then transform to base link

    m_state.imuOrientation.setIdentity();
    m_state.imuPosition.setZero();
    m_state.imuLinearVelocity.setZero();
    m_state.lContactFrameOrientation.setIdentity();
    m_state.rContactFrameOrientation.setIdentity();

    m_statePrev = m_state;
    m_estimatorOut.state = m_state;

    m_measPrev = m_meas;

    m_stateStdDev.imuOrientation.setZero();
    m_stateStdDev.imuPosition.setZero();
    m_stateStdDev.imuLinearVelocity.setZero();

    m_priors = m_stateStdDev;
    m_estimatorOut.stateStdDev = m_stateStdDev;

    m_sensorsDev.contactFootLinvelNoise.setZero();
    m_sensorsDev.contactFootAngvelNoise.setZero();
    m_sensorsDev.swingFootLinvelNoise.setZero();
    m_sensorsDev.swingFootAngvelNoise.setZero();

    m_pimpl->m_I3 = Eigen::Matrix3d::Identity();
}

DiligentOdom::~DiligentOdom() = default;

bool DiligentOdom::setupCustomSensorDevs(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        return false;
    }

    constexpr auto printPrefix{"[DiligentOdom::setupCustomSensorDevs]"};

    if (m_useModelInfo)
    {
        std::vector<double> contactFootLinvelNoise(3);
        if (!handle->getParameter("contact_foot_linear_velocity_noise_std_dev", contactFootLinvelNoise))
        {
            log()->error("{} The parameter handler could not find"
            "required parameter \"contact_foot_linear_velocity_noise_std_dev\"", printPrefix);
            return false;
        }

        std::vector<double> contactFootAngvelNoise(3);
        if (!handle->getParameter("contact_foot_angular_velocity_noise_std_dev", contactFootAngvelNoise))
        {
            log()->error("{} The parameter handler could not find"
            "required parameter \"contact_foot_angular_velocity_noise_std_dev\"", printPrefix);
            return false;
        }

        std::vector<double> swingFootLinvelNoise(3);
        if (!handle->getParameter("swing_foot_linear_velocity_noise_std_dev", swingFootLinvelNoise))
        {
            log()->error("{} The parameter handler could not find"
            "required parameter \"swing_foot_linear_velocity_noise_std_dev\"", printPrefix);
            return false;
        }

        std::vector<double> swingFootAngvelNoise(3);
        if (!handle->getParameter("swing_foot_angular_velocity_noise_std_dev", swingFootAngvelNoise))
        {
            log()->error("{} The parameter handler could not find"
            "required parameter \"swing_foot_angular_velocity_noise_std_dev\"", printPrefix);
            return false;
        }

        std::vector<double> encodersNoise;
        if (!handle->getParameter("encoders_measurement_noise_std_dev", encodersNoise))
        {
            log()->error("{} The parameter handler could not find"
            "required parameter \"encoders_measurement_noise_std_dev\"", printPrefix);
            return false;
        }

        if (encodersNoise.size() == 1)
        {
            encodersNoise.resize(m_modelComp.nrJoints(), encodersNoise[0]);
        }

        std::vector<double> fkNoise(6);
        if (!handle->getParameter("forward_kinematic_measurement_noise_std_dev", fkNoise))
        {
            log()->error("{} The parameter handler could not find"
            "required parameter \"forward_kinematic_measurement_noise_std_dev\"", printPrefix);
            return false;
        }

        std::vector<double> landmarkNoise(6);
        if (!handle->getParameter("map_height_std_dev", landmarkNoise))
        {
            log()->error("{} The parameter handler could not find"
            "required parameter \"map_height_std_dev\"", printPrefix);
            return false;
        }

        m_sensorsDev.contactFootLinvelNoise << contactFootLinvelNoise[0], contactFootLinvelNoise[1], contactFootLinvelNoise[2];
        m_sensorsDev.contactFootAngvelNoise << contactFootAngvelNoise[0], contactFootAngvelNoise[1], contactFootAngvelNoise[2];

        m_sensorsDev.swingFootLinvelNoise << swingFootLinvelNoise[0], swingFootLinvelNoise[1], swingFootLinvelNoise[2];
        m_sensorsDev.swingFootAngvelNoise << swingFootAngvelNoise[0], swingFootAngvelNoise[1], swingFootAngvelNoise[2];

        m_sensorsDev.encodersNoise.resize(encodersNoise.size());
        m_sensorsDev.encodersNoise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(encodersNoise.data(), encodersNoise.size());

        m_sensorsDev.forwardKinematicsNoise << fkNoise[0], fkNoise[1], fkNoise[2],
                                               fkNoise[3], fkNoise[4], fkNoise[5];
        m_sensorsDev.landmarkMeasurementNoise << landmarkNoise[0], landmarkNoise[1], landmarkNoise[2],
                                                 landmarkNoise[3], landmarkNoise[4], landmarkNoise[5];
    }

    std::vector<double> imuLinVelNoise(3);
    if (!handle->getParameter("imu_frame_linear_velocity_noise_std_dev", imuLinVelNoise))
    {
        log()->error("{} The parameter handler could not find"
        "required parameter \"imu_frame_linear_velocity_noise_std_dev\"", printPrefix);
        return false;
    }

    std::vector<double> imuAngVelNoise(3);
    if (!handle->getParameter("imu_frame_angular_velocity_noise_std_dev", imuAngVelNoise))
    {
        log()->error("{} The parameter handler could not find"
        "required parameter \"imu_frame_angular_velocity_noise_std_dev\"", printPrefix);
        return false;
    }

    m_sensorsDev.imuFrameLinearVelocityNoise << imuLinVelNoise[0], imuLinVelNoise[1], imuLinVelNoise[2];
    m_sensorsDev.imuFrameAngularVelocityNoise << imuAngVelNoise[0], imuAngVelNoise[1], imuAngVelNoise[2];

    std::vector<double> baseGyroNoise(3);
    if (!handle->getParameter("imu_frame_gyro_noise_std_dev", baseGyroNoise))
    {
        log()->error("{} The parameter handler could not find"
        "required parameter \"imu_frame_gyro_noise_std_dev\"", printPrefix);
        return false;
    }

    m_pimpl->m_omegabNoise << baseGyroNoise[0], baseGyroNoise[1], baseGyroNoise[2];

    return true;
}

bool DiligentOdom::customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    std::string printPrefix{"[DiligentOdom::customInitialization] "};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("{} The parameter handler has expired. Please check its scope.", printPrefix);
        return false;
    }

    handle->getParameter("use_velocity_measurements", m_pimpl->m_useVelocityMeasurements);
    if (m_pimpl->m_useVelocityMeasurements)
    {
        handle->getParameter("use_zero_base_velocity", m_pimpl->m_useZeroBaseVelocities);
    }

    // setup options related entities
    auto optionsHandle = handle->getGroup("Options");
    if (!setupOptions(optionsHandle))
    {
        log()->error("{} Could not load options related parameters.", printPrefix);
        return false;
    }

    // setup sensor standard deviations
    auto sensorDevHandle = handle->getGroup("SensorsStdDev");
    if (!setupCustomSensorDevs(sensorDevHandle))
    {
        log()->error("{} Could not load sensor stddev related parameters.", printPrefix);
        return false;
    }

    // setup initial states
    auto initStateHandle = handle->getGroup("InitialStates");
    if (!setupInitialStates(initStateHandle))
    {
        log()->error("{} Could not load initial states related parameters.", printPrefix);
        return false;
    }

    std::vector<double> lContactFrameOrientation(4), rContactFrameOrientation(4);
    if (!initStateHandle.lock()->getParameter("l_contact_frame_orientation_quaternion_wxyz", lContactFrameOrientation))
    {
        log()->error("{} Could not load prior parameter {}.", printPrefix, "l_contact_frame_orientation_quaternion_wxyz");
        return false;
    }

    if (!initStateHandle.lock()->getParameter("r_contact_frame_orientation_quaternion_wxyz", rContactFrameOrientation))
    {
        log()->error("{} Could not load prior parameter {}.", printPrefix, "r_contact_frame_orientation_quaternion_wxyz");
        return false;
    }

    std::vector<double> imuAngularVelocity(3);
    if (!initStateHandle.lock()->getParameter("imu_angular_velocity_xyz", imuAngularVelocity))
    {
        log()->error("{} Could not load prior parameter {}.", printPrefix, "imu_angular_velocity_xyz");
        return false;
    }
    m_statePrev.lContactFrameOrientation = Eigen::Quaterniond(lContactFrameOrientation[0], lContactFrameOrientation[1], lContactFrameOrientation[2], lContactFrameOrientation[3]);
    m_statePrev.lContactFrameOrientation.normalize();
    m_statePrev.rContactFrameOrientation = Eigen::Quaterniond(rContactFrameOrientation[0], rContactFrameOrientation[1], rContactFrameOrientation[2], rContactFrameOrientation[3]);
    m_statePrev.rContactFrameOrientation.normalize();
    m_statePrev.imuAngularVelocity << imuAngularVelocity[0], imuAngularVelocity[1], imuAngularVelocity[2];
    m_state = m_statePrev;

    // setup initial state standard deviations
    auto priorDevHandle = handle->getGroup("PriorsStdDev");
    if (!setupPriorDevs(priorDevHandle))
    {
        log()->error("{} Could not load prior stddev related parameters.", printPrefix);
        return false;
    }

    std::vector<double> imuAngularVelocityStdDev(3);
    if (!priorDevHandle.lock()->getParameter("imu_angular_velocity", imuAngularVelocityStdDev))
    {
        log()->error("{} Could not load prior parameter {}.", printPrefix, "imu_angular_velocity");
        return false;
    }

    std::vector<double> lContactFrameOrientationDev(3), rContactFrameOrientationDev(3);
    if (!priorDevHandle.lock()->getParameter("l_contact_frame_orientation", lContactFrameOrientationDev))
    {
        log()->error("{} Could not load prior parameter {}.", printPrefix, "l_contact_frame_orientation");
        return false;
    }

    if (!priorDevHandle.lock()->getParameter("r_contact_frame_orientation", rContactFrameOrientationDev))
    {
        log()->error("{} Could not load prior parameter {}.", printPrefix, "r_contact_frame_orientation");
        return false;
    }

    m_priors.imuAngularVelocity << imuAngularVelocityStdDev[0], imuAngularVelocityStdDev[1], imuAngularVelocityStdDev[2];
    m_priors.lContactFrameOrientation << lContactFrameOrientationDev[0], lContactFrameOrientationDev[1], lContactFrameOrientationDev[2];
    m_priors.rContactFrameOrientation << rContactFrameOrientationDev[0], rContactFrameOrientationDev[1], rContactFrameOrientationDev[2];

    // construct priors
    if (!m_pimpl->constructStateVar(m_state, m_priors, m_pimpl->m_P))
    {
        log()->error("{} Could not construct initial state covariance matrix.", printPrefix);
        return false;
    }

    // allocate buffers for Jacobians
    m_pimpl->F_J_IMUF.resize(m_pimpl->motionDim, m_modelComp.kinDyn()->getNrOfDegreesOfFreedom());
    m_pimpl->F_J_IMUF.setZero();

    m_pimpl->F_S_FIMU.resize(m_pimpl->motionDim, m_modelComp.kinDyn()->getNrOfDegreesOfFreedom());
    m_pimpl->F_S_FIMU.setZero();

    m_pimpl->IMU_J_IMUF.resize(m_pimpl->motionDim, m_modelComp.kinDyn()->getNrOfDegreesOfFreedom());
    m_pimpl->IMU_J_IMUF.setZero();

    // initialize encoder measurement noise
    m_pimpl->m_encodersVar = m_sensorsDev.encodersNoise.array().square();
    m_pimpl->m_Nenc = m_pimpl->m_encodersVar.asDiagonal();

    m_pimpl->IMU_R_B = m_modelComp.base_H_IMU().inverse().quat().toRotationMatrix();
    return true;
}

bool DiligentOdom::useFlatFloorMeasurements(const bool& flag)
{
    m_pimpl->m_forceFlatFloor = flag;
    return true;
}

bool DiligentOdom::setContactHeight(const std::string& name,
                                    const double& contactHeight)
{
    auto idx = m_modelComp.kinDyn()->getFrameIndex(name);
    if (idx == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("[DiligentOdom::setContactHeight] Contact frame index {} not found specified in model .",
                     idx);
        return false;
    }

    auto& contacts = m_meas.stampedContactsStatus;

    if (contacts.find(idx) != contacts.end())
    {
        Eigen::Vector3d position = contacts.at(idx).pose.translation();
        position(2) = contactHeight;

        contacts.at(idx).pose.translation(position);
    }

    return true;
}

bool DiligentOdom::setContactPlaneOrientation(const std::string& footSoleName,
                                              Eigen::Quaterniond orientation)
{
    if (m_modelComp.leftFootContactFrame() == footSoleName ||
        m_modelComp.rightFootContactFrame() == footSoleName)
    {
        m_pimpl->footPlaneOrientationMeasurementMap[footSoleName] = orientation;
        return true;
    }

    log()->error("[DiligentOdom::setContactPlaneOrientation] Contact frame {} not found specified in model .",
                footSoleName);

    return false;
}

bool DiligentOdom::setLeftTrivializedBaseVelocity(Eigen::Ref<Eigen::Vector3d> omega)
{
    // all computations wrt base IMU
    m_pimpl->m_omegab = m_pimpl->IMU_R_B*omega;
    return true;
}

bool DiligentOdom::setWorldFrameInRefTo(const std::string& refModelLinkName,
                                        Eigen::Ref<Eigen::Matrix3d> w_R_l,
                                        Eigen::Ref<Eigen::Vector3d> w_p_l)
{
    auto refLinkIdx = m_modelComp.kinDyn()->getFrameIndex(refModelLinkName);
    if (refLinkIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("[DiligentOdom::setWorldFrameInRefTo] {} not found specified in model .",
                     refModelLinkName);
        return false;
    }

    auto l_H_b = toManifPose(m_modelComp.kinDyn()->getRelativeTransform(refModelLinkName,
                                                                        m_modelComp.baseLink()));
    auto w_H_l = toManifPose(w_R_l, w_p_l);

    auto w_H_b = w_H_l*l_H_b;

    if (!resetEstimator(w_H_b.quat(), w_H_b.translation()))
    {
        return false;
    }

    for (auto& [frame, contact] : m_state.supportFrameData)
    {
        auto l_H_f = toManifPose(m_modelComp.kinDyn()->getRelativeTransform(refLinkIdx,
                                                                            frame));
        contact.pose = w_H_l*l_H_f;
        double initPoseCov{1e-3};
        m_priors.supportFramePose[frame] = initPoseCov*Eigen::Matrix<double, 6, 1>::Ones();
    }


    m_statePrev = m_state;
    m_stateStdDev = m_priors;
    // construct priors
    if (!m_pimpl->constructStateVar(m_state, m_priors, m_pimpl->m_P))
    {
        log()->error("[DiligentOdom::setWorldFrameInRefTo] Could not construct initial state covariance matrix.");
        return false;
    }

    return true;
}

bool DiligentOdom::setSameFootVertices(const std::string& footSoleFrame,
                                       const std::vector<std::string>& vertexNames)
{
    auto refLinkIdx = m_modelComp.kinDyn()->getFrameIndex(footSoleFrame);
    if (refLinkIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("[DiligentOdom::setSameFootVertices] {} not found specified in model .",
                     footSoleFrame);
        return false;
    }

    for (auto& name : vertexNames)
    {
        auto refLinkIdx = m_modelComp.kinDyn()->getFrameIndex(name);
        if (refLinkIdx == iDynTree::FRAME_INVALID_INDEX)
        {
            log()->error("[DiligentOdom::setSameFootVertices] {} not found specified in model for sole {} .",
                         name, footSoleFrame);
            return false;
        }
    }

    m_pimpl->footVertexMap[footSoleFrame] = vertexNames;
    return true;
}

bool DiligentOdom::resetEstimator(const FloatingBaseEstimators::InternalState& newState,
                                  const FloatingBaseEstimators::StateStdDev& newPriorDev)
{
    m_state = newState;
    m_stateStdDev = newPriorDev;
    m_priors = newPriorDev;

    if (!m_pimpl->constructStateVar(m_state, m_priors, m_pimpl->m_P))
    {
        log()->error("[DiligentOdom::resetEstimator] Could not construct state covariance matrix.");
        return false;
    }

    return true;
}

bool DiligentOdom::resetEstimator(const FloatingBaseEstimators::InternalState& newState,
                                  const FloatingBaseEstimators::StateStdDev& newPriorDev,
                                  const FloatingBaseEstimators::SensorsStdDev& newSensorsDev)
{
    m_sensorsDev = newSensorsDev;
    resetEstimator(newState, newPriorDev);
    return true;
}

bool DiligentOdom::predictState(const FloatingBaseEstimators::Measurements& meas,
                            const double& dt)
{
    // m_state is now predicted state after this function call
    m_pimpl->propagateStates(meas, dt, m_state);

    auto dim = m_pimpl->stateDimensions(m_state);
    m_pimpl->m_In = Eigen::MatrixXd::Identity(dim, dim);

    m_pimpl->calcFc(m_statePrev, dim, m_pimpl->m_Fc); // compute Fc at priori state
    m_pimpl->calcQc(m_statePrev, m_sensorsDev, meas, dim, m_pimpl->m_Qc); // compute Qc at priori state and previous measure
    m_pimpl->calcLc(m_statePrev, dim, m_pimpl->m_Lc); // compute Lc at priori state

    // discretize linearized dynamics and propagate covariance
    m_pimpl->m_Fk = m_pimpl->m_In + (m_pimpl->m_Fc*dt);  // read as Fk = I + (Fc*dt)
    m_pimpl->m_Qk = (m_pimpl->m_Fk*m_pimpl->m_Lc*m_pimpl->m_Qc*(m_pimpl->m_Lc.transpose())*(m_pimpl->m_Fk.transpose()))*dt; // read as Qk = Fk Lc Qc Lc.T Fk.T
    m_pimpl->m_P = m_pimpl->m_Fk*m_pimpl->m_P*(m_pimpl->m_Fk.transpose()) + m_pimpl->m_Qk; // read as P = Fk P Fk.T + Qk

    if (!m_pimpl->extractStateVar(m_pimpl->m_P, dim, m_state, m_stateStdDev))
    {
        // unwrap state covariance matrix diagonal
        log()->error("[DiligentOdom::predictState] Could not update state uncertanties.");
        return false;
    }

    return true;
}

bool DiligentOdom::updateKinematics(FloatingBaseEstimators::Measurements& meas,
                                    const double& dt)
{
    // extract current predicted state
    Eigen::Matrix3d R = m_state.imuOrientation.toRotationMatrix();
    const Eigen::Vector3d& p = m_state.imuPosition;
    manif::SE3d imuPosePred = Conversions::toManifPose(R, p);

    // for each measurement
    // check for existing contact
    // if not available add new contact
    m_pimpl->existingContact.clear();
    for (auto& iter : m_state.supportFrameData)
    {
        m_pimpl->existingContact.push_back(iter.first);
    }

    std::unordered_map<int, manif::SE3d> measPoses;
    size_t nrContacts{0};
    // first iterate through measurement and add new contacts
    for (auto& iter : meas.stampedContactsStatus)
    {
        auto contactid = iter.first;
        auto relContactPose = Conversions::toManifPose(m_modelComp.kinDyn()->getRelativeTransform(m_modelComp.baseIMUIdx(), contactid));
        measPoses[contactid] = relContactPose;
        auto contactData = iter.second;
        bool isActive{contactData.isActive};
        auto timestamp = contactData.lastUpdateTime;

        if (std::find(m_pimpl->existingContact.begin(), m_pimpl->existingContact.end(),
            iter.first) == m_pimpl->existingContact.end())
        {
            if (!m_pimpl->addContact(contactid, timestamp, isActive,
                                               imuPosePred*relContactPose,
                                               m_sensorsDev, m_dt,
                                               m_state, m_stateStdDev, m_pimpl->m_P))
            {
                continue;
            }
        }
        else
        {
            m_state.supportFrameData.at(contactid).isActive = isActive;
            m_state.supportFrameData.at(contactid).lastUpdateTime = timestamp;
        }

        if (isActive)
        {
            nrContacts++;
        }
    }

    // update the underlying matrix Lie group from the state
    m_pimpl->constructMatrixLieGroup(m_state, m_pimpl->m_X);


    if (!m_pimpl->updateWithRelativeVertexPositions(meas, m_modelComp, m_pimpl->m_X.rows(), dt, m_state))
    {
        log()->error("[DiligentOdom::updateKinematics] Could not update states with relative contact positions.");
        return false;
    }

    if (m_pimpl->m_useVelocityMeasurements)
    {
        if (!m_pimpl->updateWithZUPTLinearVelocity(meas, m_modelComp, m_sensorsDev, m_pimpl->m_X.rows(), dt, nrContacts,  m_state))
        {
            log()->error("[DiligentOdom::updateKinematics] Could not update states with ZUPT linear velocity.");
            return false;
        }

        if (!m_pimpl->updateWithZUPTAngularVelocity(meas, m_modelComp, m_sensorsDev, m_pimpl->m_X.rows(), dt, nrContacts,  m_state))
        {
            log()->error("[DiligentOdom::updateKinematics] Could not update states with ZUPT angular velocity.");
            return false;
        }

        if (!m_pimpl->updateWithBaseAngularVelocity(m_pimpl->m_X.rows(), dt, m_state))
        {
            log()->error("[DiligentOdom::updateKinematics] Could not update states with base angular velocity measurement.");
            return false;
        }
    }

    if (!m_pimpl->updateWithRelativeFootOrientations(meas, m_modelComp, dt, m_state))
    {
        log()->error("[DiligentOdom::updateKinematics] Could not update states with relative foot orientations.");
        return false;
    }

    if (m_pimpl->m_forceFlatFloor)
    {
        if (!m_pimpl->updateWithTerrainHeight(meas, m_modelComp, dt, nrContacts, m_stateStdDev, m_sensorsDev,  m_state))
        {
            log()->error("[DiligentOdom::updateKinematics] Could not update states with terrain height.");
            return false;
        }

        if (!m_pimpl->updateWithContactPlaneOrientation(meas, m_modelComp, dt, m_sensorsDev,  m_state))
        {
            log()->error("[DiligentOdom::updateKinematics] Could not update states with contact plane orientation.");
            return false;
        }
    }

    return true;
}

void DiligentOdom::Impl::propagateStates(const FloatingBaseEstimators::Measurements& meas,
                                         const double& dt,
                                         FloatingBaseEstimators::InternalState& X)
{
    const Eigen::Matrix3d R = X.imuOrientation.toRotationMatrix();
    const Eigen::Vector3d& v = X.imuLinearVelocity;
    const Eigen::Vector3d& p = X.imuPosition;
    const Eigen::Vector3d& omega = X.imuAngularVelocity;

    manif::SO3Tangentd omega_skew_dt(omega*dt);
    Eigen::Matrix3d R_pred = R*(omega_skew_dt.exp().rotation());

    X.imuOrientation = Eigen::Quaterniond(R_pred);
    X.imuPosition = p + (v*dt);

    // base velocity constant dynamics - nothing to change
    // feet vertex position constant dynamics - nothing to change
    // feet link rotation constant dynamics - nothing to change
}


bool DiligentOdom::Impl::extractStateVar(const Eigen::MatrixXd& P,
                                         const std::size_t& dim,
                                         const FloatingBaseEstimators::InternalState& state,
                                         FloatingBaseEstimators::StateStdDev& stateStdDev)
{
    if (dim != P.rows() || dim != P.cols())
    {
        log()->error("[DiligentOdom::predictState] state covariance size mismatch.");
        return false;
    }

    stateStdDev.imuPosition =  P.block<3, 3>(imuPositionOffset, imuPositionOffset).diagonal().array().sqrt();
    stateStdDev.imuOrientation =  P.block<3, 3>(imuOrientationOffset, imuOrientationOffset).diagonal().array().sqrt();
    stateStdDev.imuLinearVelocity =  P.block<3, 3>(imuLinearVelOffset, imuLinearVelOffset).diagonal().array().sqrt();

    int idx = 0;
    for (auto& [frameIdx, supportFrame] : state.supportFrameData)
    {
        int q = extMotionDim + (dim3*idx);
        stateStdDev.supportFramePose[frameIdx].head<3>() = P.block<3, 3>(q, q).diagonal().array().sqrt();
        idx++;
    }

    auto ZlfOffset = lfRotOffset(state);
    stateStdDev.lContactFrameOrientation = P.block<3, 3>(ZlfOffset, ZlfOffset).diagonal().array().sqrt();
    stateStdDev.rContactFrameOrientation = P.bottomRightCorner<3, 3>().diagonal().array().sqrt();
    return true;
}

bool DiligentOdom::Impl::constructStateVar(const FloatingBaseEstimators::InternalState& state,
                                           const FloatingBaseEstimators::StateStdDev& stateStdDev,
                                           Eigen::MatrixXd& P)
{
    if (!compareKeys(state.supportFrameData, stateStdDev.supportFramePose))
    {
        log()->error("[DiligentOdom::constuctStateVar] support frame data mismatch.");
        return false;
    }

    auto dim = stateDimensions(state);
    P.resize(dim, dim);
    P.setZero();

    Eigen::Vector3d temp;
    temp = stateStdDev.imuPosition.array().square();
    P.block<3, 3>(imuPositionOffset, imuPositionOffset) = temp.asDiagonal();
    temp = stateStdDev.imuOrientation.array().square();
    P.block<3, 3>(imuOrientationOffset, imuOrientationOffset) = temp.asDiagonal();
    temp = stateStdDev.imuLinearVelocity.array().square();
    P.block<3, 3>(imuLinearVelOffset, imuLinearVelOffset) = temp.asDiagonal();

    int idx = 0;
    Eigen::Vector3d frameVar;
    for (auto& [frameIdx, stddev] : stateStdDev.supportFramePose)
    {
        frameVar = stddev.head<3>().array().square();
        int q = extMotionDim + (dim3*idx);
        P.block<3, 3>(q, q) = frameVar.asDiagonal();
        idx++;
    }

    Eigen::Matrix<double, 9, 1> diagVec;
    diagVec << stateStdDev.imuAngularVelocity.array().square(),
               stateStdDev.lContactFrameOrientation.array().square(),
               stateStdDev.rContactFrameOrientation.array().square();

    P.bottomRightCorner<9, 9>() = diagVec.asDiagonal();
    return true;
}

void DiligentOdom::Impl::calcFc(const FloatingBaseEstimators::InternalState& X,
                                const std::size_t& dim,
                                Eigen::MatrixXd& Fc)
{
    //            e_p    e_R   e_v  e_d   e_w  e_Z
    // Fc = e_p [   0     0    I_3   0  S(p)R   0]
    //      e_R [   0     0     0    0      R   0]
    //      e_v [   0     0     0    0  S(v)R   0]
    //      e_d [   0     0     0    0  S(d)R   0]
    //      e_w [   0     0     0    0      0   0]
    //      e_Z [   0     0     0    0      0   0]
    Fc.resize(dim, dim);
    Fc.setZero();

    const Eigen::Matrix3d R = X.imuOrientation.toRotationMatrix();
    const Eigen::Vector3d& v = X.imuLinearVelocity;
    const Eigen::Vector3d& p = X.imuPosition;

    std::size_t angVelOffset = imuAngVelOffset(X);

    Fc.block<3, 3>(imuPositionOffset, imuLinearVelOffset) = m_I3; // J_p_v
    Fc.block<3, 3>(imuPositionOffset, angVelOffset) = manif::skew(p)*R; // J_p_w

    Fc.block<3, 3>(imuOrientationOffset, angVelOffset) = R; // J_R_w

    Fc.block<3, 3>(imuLinearVelOffset, angVelOffset) = manif::skew(v)*R; // J_v_w

    int idx = 0;
    for (auto& [frameIdx, supportFrame] : X.supportFrameData)
    {
        const Eigen::Vector3d& d = supportFrame.pose.translation();
        int q = extMotionDim + (dim3*idx);
        Fc.block<3, 3>(q, angVelOffset) = manif::skew(d)*R; // J_d_w
        idx++;
    }
}

void DiligentOdom::Impl::calcLc(const FloatingBaseEstimators::InternalState& X,
                                const std::size_t& dim,
                                Eigen::MatrixXd& Lc)
{
    calcAdjX(X, Lc);
}

void DiligentOdom::Impl::calcQc(const FloatingBaseEstimators::InternalState& X,
                                const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                const FloatingBaseEstimators::Measurements& meas,
                                const std::size_t& dim,
                                Eigen::MatrixXd& Qc)
{
    Qc.resize(dim, dim);
    Qc.setZero();

    Eigen::Vector3d Qv, Qomega;
    Qv = sensDev.imuFrameLinearVelocityNoise.array().square();
    Qomega = sensDev.imuFrameAngularVelocityNoise.array().square();

    Qc.block<3, 3>(imuLinearVelOffset, imuLinearVelOffset) = Qv.asDiagonal();

    Eigen::Matrix<double, 3, 1> Qf;
    Eigen::Matrix<double, 3, 1> supportFrameNoise;

    // foot vertex positions
    int idx = 0;
    for (auto& [frameIdx, supportFrame] : X.supportFrameData)
    {
        // prepare the noise covariance submatrix
        if (supportFrame.isActive)
        {
            supportFrameNoise << sensDev.contactFootLinvelNoise;
        }
        else
        {
            supportFrameNoise << sensDev.swingFootLinvelNoise;
        }

        Qf = supportFrameNoise.array().square();
        int q = extMotionDim + (dim3*idx);
        Qc.block<3, 3>(q, q) = Qf.asDiagonal();
        idx++;
    }

    // angular velocity
    auto q = imuAngVelOffset(X);
    Qc.block<3, 3>(q, q) = Qomega.asDiagonal();

    // foot rotations
    Eigen::Vector3d Qlf, Qrf;
    if (meas.lfInContact)
    {
        Qlf = sensDev.contactFootLinvelNoise.array().square();
    }
    else
    {
        Qlf = sensDev.swingFootLinvelNoise.array().square();
    }

    if (meas.rfInContact)
    {
        Qrf = sensDev.contactFootLinvelNoise.array().square();
    }
    else
    {
        Qrf = sensDev.swingFootLinvelNoise.array().square();
    }

    q = lfRotOffset(X);
    Qc.block<3, 3>(q, q) = Qlf.asDiagonal();
    Qc.bottomRightCorner<3, 3>() = Qrf.asDiagonal();
}

bool DiligentOdom::Impl::updateWithRelativeVertexPositions(FloatingBaseEstimators::Measurements& meas,
                                                           FloatingBaseEstimator::ModelComputations& modelComp,
                                                           const std::size_t& Xsize,
                                                           const double& dt,
                                                           FloatingBaseEstimators::InternalState& state)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateWithRelativeVertexPositions]"};
    std::size_t nrVertices{state.supportFrameData.size()};
    std::size_t observationDim{dim3*nrVertices};
    std::size_t stateDim{stateDimensions(state)};

    const Eigen::Matrix3d R = state.imuOrientation.toRotationMatrix();

    m_Hk.resize(observationDim, stateDim);
    m_Hk.setZero();

    m_Yk.resize(Xsize*nrVertices);
    m_Yk.setZero();
    m_bk = m_Yk;

    m_Pik.resize(observationDim, m_Yk.size());
    m_Pik.setZero();
    m_Nk.resize(observationDim, observationDim);
    m_Nk.setZero();

    int idx = 1;
    const int feetOffsetInX{4};
    for (auto& [frameIdx, supportFrame] : state.supportFrameData)
    {
        auto yStart{Xsize*(idx-1)};
        auto obsStart{dim3*(idx-1)};

        // if we consider only one contact, Y and b are of the form
        // Y = [d; 1; 0; -1; zeros(3,1); 0; zeros(3,1)];
        // b = [zeros(3,1); 1; 0; -1; zeros(3,1); 0; zeros(3,1)];
        auto b_pos_f = modelComp.kinDyn()->getRelativeTransform(modelComp.baseIMUIdx(), frameIdx).getPosition();
        m_Yk.segment<4>(yStart) << iDynTree::toEigen(b_pos_f), 1;

        m_Yk(yStart + feetOffsetInX + idx) = -1;
        m_bk(yStart + 3) = 1;
        m_bk(yStart + feetOffsetInX + idx) = -1;
        m_Pik.block<3, 3>(obsStart, yStart) = m_I3;

        int q = extMotionDim + (dim3*(idx-1));
        m_Hk.block<3, 3>(obsStart, imuPositionOffset) = -m_I3; // J_y_p
        m_Hk.block<3, 3>(obsStart, q) = m_I3; // J_y_d

//         modelComp.kinDyn()->getRelativeJacobianExplicit(modelComp.baseIMUIdx(),
//                                                         frameIdx,
//                                                         modelComp.baseIMUIdx(), modelComp.baseIMUIdx(), IMU_J_IMUF);
        modelComp.kinDyn()->getRelativeJacobian(modelComp.baseIMUIdx(), frameIdx, IMU_J_IMUF);
//         m_Nk.block<3, 3>(obsStart, obsStart) = R*IMU_J_IMUF.topRows<3>()*m_Nenc*(IMU_J_IMUF.topRows<3>().transpose())*R.transpose();
        m_Nk.block<3, 3>(obsStart, obsStart) = IMU_J_IMUF.topRows<3>()*m_Nenc*(IMU_J_IMUF.topRows<3>().transpose());
        idx++;
    }

    m_Nk /= dt;

    if (!updateRightInvariantObservations(m_Yk, m_bk, m_Hk, m_Nk, m_Pik, state, m_P))
    {
        log()->error("{} Could not update states with invariant relative position observations", printPrefix);
        return false;
    }

    return true;
}

bool DiligentOdom::Impl::updateWithRelativeFootOrientations(FloatingBaseEstimators::Measurements& meas,
                                                            FloatingBaseEstimator::ModelComputations& modelComp,
                                                            const double& dt,
                                                            FloatingBaseEstimators::InternalState& state)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateWithRelativeFootOrientations]"};

    auto lFrameIdx = modelComp.kinDyn()->getFrameIndex(modelComp.leftFootContactFrame());
    auto rFrameIdx = modelComp.kinDyn()->getFrameIndex(modelComp.rightFootContactFrame());
    auto imuIdx = modelComp.baseIMUIdx();

    std::size_t observationDim{dim3*nrFeet};
    std::size_t stateDim{stateDimensions(state)};

    m_deltaY.resize(observationDim);
    m_deltaY.setZero();

    m_Hk.resize(observationDim, stateDim);
    m_Hk.setZero();

    m_Nk.resize(observationDim, observationDim);
    m_Nk.setZero();

    Eigen::Matrix3d R = state.imuOrientation.toRotationMatrix();
    Eigen::Matrix3d ZlT = state.lContactFrameOrientation.toRotationMatrix().transpose();
    Eigen::Matrix3d ZrT = state.rContactFrameOrientation.toRotationMatrix().transpose();

    auto angVelOffset = imuAngVelOffset(state);

    // handle left foot measurements
    Eigen::Matrix3d hOfXLFinv = ZlT*R;
    Eigen::Matrix3d yLF = iDynTree::toEigen(modelComp.kinDyn()->getRelativeTransform(imuIdx, lFrameIdx).getRotation());
    Eigen::Matrix3d lfRotError = hOfXLFinv*yLF;
//     modelComp.kinDyn()->getRelativeJacobianExplicit(imuIdx, lFrameIdx,
//                                                         lFrameIdx, lFrameIdx, F_J_IMUF);
    modelComp.kinDyn()->getRelativeJacobian(imuIdx, lFrameIdx, F_J_IMUF);
    m_deltaY.head<3>() = toManifRot(lfRotError).log().coeffs();
    m_Hk.block<3, 3>(0, imuOrientationOffset) = -ZlT;
    m_Hk.block<3, 3>(0, angVelOffset+3) = ZlT;
    m_Nk.topLeftCorner<3, 3>() = F_J_IMUF.bottomRows<3>()*m_Nenc*(F_J_IMUF.bottomRows<3>().transpose());

    Eigen::Matrix3d hOfXRFinv = ZrT*R;
    Eigen::Matrix3d yRF = iDynTree::toEigen(modelComp.kinDyn()->getRelativeTransform(imuIdx, rFrameIdx).getRotation());
    Eigen::Matrix3d rfRotError = hOfXRFinv*yRF;
//     modelComp.kinDyn()->getRelativeJacobianExplicit(imuIdx, rFrameIdx,
//                                                     rFrameIdx, rFrameIdx, F_J_IMUF);
    modelComp.kinDyn()->getRelativeJacobian(imuIdx, rFrameIdx, F_J_IMUF);
    m_deltaY.tail<3>() = toManifRot(rfRotError).log().coeffs();
    m_Hk.block<3, 3>(3, imuOrientationOffset) = -ZrT;
    m_Hk.bottomRightCorner<3, 3>() = ZrT;
    m_Nk.bottomRightCorner<3, 3>() = F_J_IMUF.bottomRows<3>()*m_Nenc*(F_J_IMUF.bottomRows<3>().transpose());

    m_Nk /= dt;

    if (!updateNonInvariantObservations(m_deltaY, m_Hk, m_Nk, state, m_P))
    {
        log()->error("{} Could not update states with non-invariant relative orientation observations", printPrefix);
        return false;
    }

    return true;
}

bool DiligentOdom::Impl::updateWithZUPTLinearVelocity(FloatingBaseEstimators::Measurements& meas,
                                                      FloatingBaseEstimator::ModelComputations& modelComp,
                                                      const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                                      const std::size_t& Xsize,
                                                      const double& dt,
                                                      const int& nrContacts,
                                                      FloatingBaseEstimators::InternalState& state)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateWithRelativeVertexPositions]"};

    std::size_t observationDim{dim3*nrContacts};
    std::size_t stateDim{stateDimensions(state)};
    Eigen::Matrix3d R = state.imuOrientation.toRotationMatrix();

    m_Hk.resize(observationDim, stateDim);
    m_Hk.setZero();

    m_Yk.resize(Xsize*nrContacts);
    m_Yk.setZero();
    m_bk = m_Yk;

    m_Pik.resize(observationDim, m_Yk.size());
    m_Pik.setZero();
    m_Nk.resize(observationDim, observationDim);
    m_Nk.setZero();

    Eigen::Vector3d Rv;
    Rv = sensDev.forwardKinematicsNoise.head<3>().array().square();

    int k = 0; // measurement space index jump
    for (auto& obs : meas.stampedContactsStatus)
    {
        for (auto& iter : state.supportFrameData)
        {
            auto& contactId = obs.first;
            auto& contactState = obs.second.isActive;
            if (iter.first == contactId && contactState) // update if contact is active
            {
                // left trivialized velocity Jacobian
                modelComp.kinDyn()->getRelativeJacobianExplicit(contactId, modelComp.baseIMUIdx(),
                                                        contactId, contactId, F_S_FIMU);

                auto F_H_B = modelComp.kinDyn()->getRelativeTransform(contactId, modelComp.baseIMUIdx());
                m_Jb = iDynTree::toEigen(F_H_B.asAdjointTransform());
                m_vb = -(m_Jb.inverse())*F_S_FIMU*meas.encodersSpeed;

                auto yStart = Xsize*k;
                auto obsStart = dim3*k;

                // if we consider only one contact, Y and b are of the form
                // Y = [vlin; 0; -1; 0; zeros(3, 1); 0, zeros(3,1)];
                //  b = [zeros(3, 1); 0; -1; 0; zeros(3,1); 0; zeros(3,1)];

                if (!m_useZeroBaseVelocities)
                {
                    m_Yk.segment<5>(yStart) << m_vb.head<3>(), 0, -1;
                }
                else
                {
                    m_Yk.segment<5>(yStart) << 0, 0, 0, 0, -1;
                }
                m_bk(yStart+4) = -1;
                m_Pik.block<3, 3>(obsStart, yStart) = m_I3;
                m_Hk.block<3, 3>(obsStart, imuLinearVelOffset) = m_I3; // J_y_v
//                 m_Nk.block<3, 3>(obsStart, obsStart) = R*Rv.asDiagonal()*(R.transpose());
                m_Nk.block<3, 3>(obsStart, obsStart) = Rv.asDiagonal();

                // only if active contact
                // increase k count
                k++;
                break;
            }
        }
    }

     m_Nk /= dt;

    if (nrContacts > 1)
    {
        if (!updateRightInvariantObservations(m_Yk, m_bk, m_Hk, m_Nk, m_Pik, state, m_P))
        {
            log()->error("{} Could not update states with invariant ZUPT linear velocity observations", printPrefix);
            return false;
        }
    }

    return true;
}

bool DiligentOdom::Impl::updateWithZUPTAngularVelocity(FloatingBaseEstimators::Measurements& meas,
                                                       FloatingBaseEstimator::ModelComputations& modelComp,
                                                       const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                                       const std::size_t& Xsize,
                                                       const double& dt,
                                                       const int& nrContacts,
                                                       FloatingBaseEstimators::InternalState& state)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateWithZUPTAngularVelocity]"};

    std::size_t observationDim{dim3*nrContacts};
    std::size_t stateDim{stateDimensions(state)};

    m_Hk.resize(observationDim, stateDim);
    m_Hk.setZero();

    m_Yk.resize(Xsize*nrContacts);
    m_Yk.setZero();
    m_bk = m_Yk;

    m_Pik.resize(observationDim, m_Yk.size());
    m_Pik.setZero();
    m_Nk.resize(observationDim, observationDim);
    m_Nk.setZero();

    Eigen::Vector3d Rw;
    Rw = sensDev.forwardKinematicsNoise.tail<3>().array().square();

    std::vector <iDynTree::LinkIndex> visitedLinks;
    int k = 0; // measurement space index jump
    for (auto& obs : meas.stampedContactsStatus)
    {
        for (auto& iter : state.supportFrameData)
        {
            auto& contactId = obs.first;
            auto& contactState = obs.second.isActive;
            if (iter.first == contactId && contactState) // update if contact is active
            {
                auto linkIdx = modelComp.kinDyn()->model().getFrameLink(contactId);
                if (std::find(visitedLinks.begin(),
                              visitedLinks.end(),
                              linkIdx) != visitedLinks.end())
                {
                    // update for the link already done
                    continue;
                }
                visitedLinks.emplace_back(contactId);
                // left trivialized velocity Jacobian
                modelComp.kinDyn()->getRelativeJacobianExplicit(contactId, modelComp.baseIMUIdx(),
                                                        contactId, contactId, F_S_FIMU);

                auto F_H_B = modelComp.kinDyn()->getRelativeTransform(contactId, modelComp.baseIMUIdx());
                m_Jb = iDynTree::toEigen(F_H_B.asAdjointTransform());
                m_vb = -(m_Jb.inverse())*F_S_FIMU*meas.encodersSpeed;

                auto yStart = Xsize*k + 5 + state.supportFrameData.size();
                auto obsStart = dim3*k;
                auto angVelOffset = imuAngVelOffset(state);
                // if we consider only one contact, Y and b are of the form
                // Y = [zeros(3, 1); 0; 0; 0; omega; -1, zeros(3,1)];
                // b = [zeros(3, 1); 0; 0; 0; zeros(3, 1); -1, zeros(3,1)];
                if (!m_useZeroBaseVelocities)
                {
                    m_Yk.segment<4>(yStart) << m_vb.tail<3>(), -1;
                }
                else
                {
                    m_Yk.segment<4>(yStart) << 0, 0, 0, -1;
                }
                m_bk(yStart+3) = -1;
                m_Pik.block<3, 3>(obsStart, yStart) = m_I3;
                m_Hk.block<3, 3>(obsStart, angVelOffset) = m_I3; // J_y_v
                m_Nk.block<3, 3>(obsStart, obsStart) = Rw.asDiagonal();

                // only if active contact
                // increase k count
                k++;
                break;
            }
        }
    }
    m_Nk /= dt;

    if (nrContacts > 1)
    {
        if (!updateRightInvariantObservations(m_Yk, m_bk, m_Hk, m_Nk, m_Pik, state, m_P))
        {
            log()->error("{} Could not update states with invariant ZUPT angular velocity observations", printPrefix);
            return false;
        }
    }

    return true;
}

bool DiligentOdom::Impl::updateWithBaseAngularVelocity(const std::size_t& Xsize,
                                                       const double& dt,
                                                       FloatingBaseEstimators::InternalState& state)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateWithBaseAngularVelocity]"};

    std::size_t observationDim{3};
    std::size_t stateDim{stateDimensions(state)};

    m_Hk.resize(observationDim, stateDim);
    m_Hk.setZero();

    m_Yk.resize(Xsize);
    m_Yk.setZero();
    m_bk = m_Yk;

    m_Pik.resize(observationDim, m_Yk.size());
    m_Pik.setZero();
    m_Nk.resize(observationDim, observationDim);
    m_Nk.setZero();

    Eigen::Vector3d Rw;
    Rw = m_omegabNoise.array().square();

    auto yStart = 5 + state.supportFrameData.size();
    auto obsStart = 0;
    auto angVelOffset = imuAngVelOffset(state);
    // if we consider only one contact, Y and b are of the form
    // Y = [zeros(3, 1); 0; 0; 0; omega; 1, zeros(3,1)];
    // b = [zeros(3, 1); 0; 0; 0; zeros(3, 1); 1, zeros(3,1)];
    if (!m_useZeroBaseVelocities)
    {
        m_Yk.segment<4>(yStart) << m_omegab, 1;
    }
    else
    {
        m_Yk.segment<4>(yStart) << 0, 0, 0, 1;
    }
    m_bk(yStart+3) = 1;
    m_Pik.block<3, 3>(obsStart, yStart) = m_I3;
    m_Hk.block<3, 3>(obsStart, angVelOffset) = m_I3;
    m_Nk.block<3, 3>(obsStart, obsStart) = Rw.asDiagonal();
    m_Nk /= dt;

    if (!updateLeftInvariantObservations(m_Yk, m_bk, m_Hk, m_Nk, m_Pik, state, m_P))
    {
        log()->error("{} Could not update states with base-collocated angular velocity observations", printPrefix);
        return false;
    }

    return true;
}


bool DiligentOdom::Impl::updateWithTerrainHeight(FloatingBaseEstimators::Measurements& meas,
                                                 FloatingBaseEstimator::ModelComputations& modelComp,
                                                 const double& dt,
                                                 const int& nrContacts,
                                                 const FloatingBaseEstimators::StateStdDev& stateDev,
                                                 const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                                 FloatingBaseEstimators::InternalState& state)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateWithTerrainHeight]"};
    std::size_t observationDim{dim3*nrContacts};
    std::size_t stateDim{stateDimensions(state)};

    auto lSole = modelComp.leftFootContactFrame();
    auto rSole = modelComp.rightFootContactFrame();
    std::size_t nrLFContacts{0}, nrRFContacts{0};
    bool lfInRigidContact{false}, rfInRigidContact{false};
    checkSoleRigidContact(lSole, meas, modelComp, nrLFContacts, lfInRigidContact);
    checkSoleRigidContact(rSole, meas, modelComp, nrRFContacts, rfInRigidContact);

    m_deltaY.resize(observationDim);
    m_deltaY.setZero();

    m_Hk.resize(observationDim, stateDim);
    m_Hk.setZero();

    m_Nk.resize(observationDim, observationDim);
    m_Nk.setZero();

    Eigen::Vector3d Rw;
    Rw << 1.0, 1.0, 1.0;

    int k = 0; // measurement space index jump
    for (auto& obs : meas.stampedContactsStatus)
    {
        int j = 0; // state space index jump
        for (auto& iter : state.supportFrameData)
        {
            auto& contactId = obs.first;
            auto& contactState = obs.second.isActive;
            if (iter.first == contactId && contactState) // update if contact is active
            {
                auto obsStart = dim3*k;
                const Eigen::Vector3d yd = obs.second.pose.translation();
                const Eigen::Vector3d d = iter.second.pose.translation();
                m_deltaY.segment<3>(obsStart) << 0, 0, yd(2) - d(2);

                // use state var for x,y and measurement var for z
                //Rw << stateDev.supportFramePose.at(contactId).head<2>(), sensDev.landmarkMeasurementNoise(2);
                bool rigidPointLF = isContactPointInSole(lSole, meas, modelComp, contactId) && lfInRigidContact;
                bool rigidPointRF = isContactPointInSole(rSole, meas, modelComp, contactId) && rfInRigidContact;
                if (rigidPointLF || rigidPointRF)
                {
                    Rw(0) = sensDev.landmarkMeasurementNoise(0);
                    Rw(1) = sensDev.landmarkMeasurementNoise(1);
                }

                Rw(2) =  sensDev.landmarkMeasurementNoise(2);
                Rw = Rw.array().square();

                int q = extMotionDim + (dim3*j);
                m_Hk.block<3, 3>(obsStart, imuOrientationOffset) = -manif::skew(d);
                m_Hk.block<3, 3>(obsStart, q) = m_I3; // J_y_d
                m_Nk.block<3, 3>(obsStart, obsStart) = Rw.asDiagonal();

                // only if active contact
                // increase k count
                k++;
                break;
            }
            j++;
        }
    }

    m_Nk /= dt;

    if (!updateNonInvariantObservations(m_deltaY, m_Hk, m_Nk, state, m_P))
    {
        log()->error("{} Could not update states with non-invariant terrain height observations.", printPrefix);
        return false;
    }

    return true;
}

void DiligentOdom::Impl::checkSoleRigidContact(const std::string soleName,
                                               FloatingBaseEstimators::Measurements& meas,
                                               FloatingBaseEstimator::ModelComputations& modelComp,
                                               std::size_t& nrContacts,
                                               bool& isInRigidContact)
{
    isInRigidContact = false;
    if (footPlaneOrientationMeasurementMap.find(soleName) !=
        footPlaneOrientationMeasurementMap.end())
    {
        if (footVertexMap.find(soleName) != footVertexMap.end())
        {
            for (auto& frame : footVertexMap.at(soleName))
            {
                auto frameIdx = modelComp.kinDyn()->getFrameIndex(frame);
                for (auto& [id, contact] : meas.stampedContactsStatus)
                {
                    if (frameIdx == id)
                    {
                        if (contact.isActive)
                        {
                            nrContacts++;
                        }
                        break;
                    }
                }
            }

            if (nrContacts == footVertexMap.at(soleName).size())
            {
                isInRigidContact = true;
            }
        }
    }

}

bool DiligentOdom::Impl::isContactPointInSole(const std::string soleName,
                                              FloatingBaseEstimators::Measurements& meas,
                                              FloatingBaseEstimator::ModelComputations& modelComp,
                                              const int& id)
{
    if (footVertexMap.find(soleName) != footVertexMap.end())
    {
        for (auto& frame : footVertexMap.at(soleName))
        {
            auto frameIdx = modelComp.kinDyn()->getFrameIndex(frame);
            if (frameIdx == id)
            {
                return true;
            }
        }
    }
    return false;
}

bool DiligentOdom::Impl::updateWithContactPlaneOrientation(FloatingBaseEstimators::Measurements& meas,
                                                           FloatingBaseEstimator::ModelComputations& modelComp,
                                                           const double& dt,
                                                           const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                                           FloatingBaseEstimators::InternalState& state)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateWithContactPlaneOrientation]"};
    std::size_t observationDim{0}; // left foot contact frame and right foot contact frame
    auto lSole = modelComp.leftFootContactFrame();
    auto rSole = modelComp.rightFootContactFrame();
    std::size_t nrLFContacts{0}, nrRFContacts{0};
    bool lfInRigidContact{false}, rfInRigidContact{false};
    checkSoleRigidContact(lSole, meas, modelComp, nrLFContacts, lfInRigidContact);
    checkSoleRigidContact(rSole, meas, modelComp, nrRFContacts, rfInRigidContact);


//     lfInRigidContact = true;
//     rfInRigidContact = true;

    if (lfInRigidContact && !rfInRigidContact)
    {
        observationDim = 3;
    }
    if (!lfInRigidContact && rfInRigidContact)
    {
        observationDim = 3;
    }
    if (lfInRigidContact && rfInRigidContact)
    {
        observationDim = 6;
    }

    // if no foot in rigid contact, do nothing and return early
    if (observationDim == 0)
    {
        return true;
    }

    std::size_t stateDim{stateDimensions(state)};
    m_deltaY.resize(observationDim);
    m_deltaY.setZero();

    m_Hk.resize(observationDim, stateDim);
    m_Hk.setZero();

    m_Nk.resize(observationDim, observationDim);
    m_Nk.setZero();

    Eigen::Vector3d Rw = sensDev.contactFootAngvelNoise.array().square();
    Eigen::Matrix3d ZlT = state.lContactFrameOrientation.toRotationMatrix().transpose();
    Eigen::Matrix3d ZrT = state.rContactFrameOrientation.toRotationMatrix().transpose();
    auto lfOffset = lfRotOffset(state);
    auto rfOffset = lfOffset+3;
    if (lfInRigidContact)
    {
        Eigen::Matrix3d yLF = footPlaneOrientationMeasurementMap.at(lSole).toRotationMatrix();
        m_deltaY.head<3>() = toManifRot(ZlT*yLF).log().coeffs();
        m_Hk.block<3, 3>(0, lfOffset) = ZlT;
        m_Nk.block<3, 3>(0, 0) = Rw.asDiagonal();
        // double supprot
        if (rfInRigidContact && observationDim ==6)
        {
            Eigen::Matrix3d yRF = footPlaneOrientationMeasurementMap.at(rSole).toRotationMatrix();
            m_deltaY.tail<3>() = toManifRot(ZrT*yRF).log().coeffs();
            m_Hk.block<3, 3>(3, rfOffset) = ZrT;
            m_Nk.block<3, 3>(3, 3) = Rw.asDiagonal();
        }
    }
    else
    {
        // only right foot in contact
        if (rfInRigidContact && observationDim ==3)
        {
            Eigen::Matrix3d yRF = footPlaneOrientationMeasurementMap.at(rSole).toRotationMatrix();
            m_deltaY.head<3>() = toManifRot(ZrT*yRF).log().coeffs();
            m_Hk.block<3, 3>(0, rfOffset) = ZrT;
            m_Nk.block<3, 3>(0, 0) = Rw.asDiagonal();
        }
    }

    m_Nk /= dt;

    if (!updateNonInvariantObservations(m_deltaY, m_Hk, m_Nk, state, m_P))
    {
        log()->error("{} Could not update states with non-invariant contact plane observations.", printPrefix);
        return false;
    }

    // after all computations, clear measurements map
    footPlaneOrientationMeasurementMap.clear();
    return true;
}

bool DiligentOdom::Impl::updateRightInvariantObservations(Eigen::Ref<const Eigen::VectorXd> Y,
                                                          Eigen::Ref<const Eigen::VectorXd> b,
                                                          Eigen::Ref<const Eigen::MatrixXd> H,
                                                          Eigen::Ref<const Eigen::MatrixXd> N,
                                                          Eigen::Ref<const Eigen::MatrixXd> Pi,
                                                          FloatingBaseEstimators::InternalState& state,
                                                          Eigen::MatrixXd& P)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateRightInvariantObservations]"};

    if (H.cols() != P.rows())
    {
        log()->error("{} Measurement model Jacobian size mismatch.", printPrefix);
        return false;
    }

    if (H.rows() != N.rows())
    {
        log()->error("{} Measurement noise covariance matrix size mismatch.", printPrefix);
        return false;
    }

    m_PHT = P*H.transpose();
    m_S = H*m_PHT + N;
    m_K = m_PHT*(m_S.inverse());

    if (Eigen::isnan(m_K.array()).sum() == true)
    {
        log()->error("{} NaN values in Kalman gain.", printPrefix);
        return false;
    }

    constructMatrixLieGroup(state, m_X);

    int Xsize = m_X.cols();
    int copyTimes = Y.rows()/Xsize;
    m_deltaY.resize(Y.rows());

    // compute innovation delta
    for (int iter = 0; iter < copyTimes; iter++)
    {
        m_deltaY.segment(iter*Xsize, Xsize) = (m_X*Y.segment(iter*Xsize, Xsize)) - b.segment(iter*Xsize, Xsize);
    }

    m_deltaX = m_K*Pi*m_deltaY;

    // update state
    if (!calcExpHatX(m_deltaX, m_dX))
    {
        log()->error("{} Could not compute state exphat", printPrefix);
        return false;
    }

    // update using right multiplication
    // since deltaX is a right trivialized tangent parametrization
    if (!composeX(m_dX, m_X, m_Xout))
    {
        log()->error("{} Could not compose state Lie groups", printPrefix);
        return false;
    }
    m_X = m_Xout;
    if (!extractFromMatrixLieGroup(m_Xout, state)) // right invariant update
    {
        log()->error("{} Could not compute state exphat", printPrefix);
        return false;
    }

    // update covariance
    m_IminusKH = m_In - m_K*H;

    P = m_IminusKH*P*(m_IminusKH.transpose()) + m_K*N*(m_K.transpose());

    return true;
}

bool DiligentOdom::Impl::updateLeftInvariantObservations(Eigen::Ref<const Eigen::VectorXd> Y,
                                                         Eigen::Ref<const Eigen::VectorXd> b,
                                                         Eigen::Ref<const Eigen::MatrixXd> H,
                                                         Eigen::Ref<const Eigen::MatrixXd> N,
                                                         Eigen::Ref<const Eigen::MatrixXd> Pi,
                                                         FloatingBaseEstimators::InternalState& state,
                                                         Eigen::MatrixXd& P)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateLeftInvariantObservations]"};

    if (H.cols() != P.rows())
    {
        log()->error("{} Measurement model Jacobian size mismatch.", printPrefix);
        return false;
    }

    if (H.rows() != N.rows())
    {
        log()->error("{} Measurement noise covariance matrix size mismatch.", printPrefix);
        return false;
    }

    calcXinv(state, m_stateInv);
    calcAdjX(m_stateInv, m_AdjXinv);
    calcAdjX(state, m_AdjX);
    m_Pl = m_AdjXinv*P*(m_AdjXinv.transpose());
    m_PHT = m_Pl*H.transpose();
    m_S = H*m_PHT + N;
    m_K = m_PHT*(m_S.inverse());

    if (Eigen::isnan(m_K.array()).sum() == true)
    {
        log()->error("{} NaN values in Kalman gain.", printPrefix);
        return false;
    }

    constructMatrixLieGroup(m_stateInv, m_Xinv);

    int Xsize = m_Xinv.cols();
    int copyTimes = Y.rows()/Xsize;
    m_deltaY.resize(Y.rows());

    // compute innovation delta = Xinv Y - b
    for (int iter = 0; iter < copyTimes; iter++)
    {
        m_deltaY.segment(iter*Xsize, Xsize) = (m_Xinv*Y.segment(iter*Xsize, Xsize)) - b.segment(iter*Xsize, Xsize);
    }

    m_deltaX = m_K*Pi*m_deltaY;

    // update state
    if (!calcExpHatX(m_deltaX, m_dX))
    {
        log()->error("{} Could not compute state exphat", printPrefix);
        return false;
    }

    // update using left multiplication
    // since deltaX is a left trivialized tangent parametrization
    if (!composeX(m_X, m_dX, m_Xout))
    {
        log()->error("{} Could not compose state Lie groups", printPrefix);
        return false;
    }
    m_X = m_Xout;
    if (!extractFromMatrixLieGroup(m_Xout, state)) // right invariant update
    {
        log()->error("{} Could not compute state exphat", printPrefix);
        return false;
    }

    // update covariance
    m_IminusKH = m_In - m_K*H;
    m_Pl = m_IminusKH*m_Pl;
    P = m_AdjX*m_Pl*(m_AdjX.transpose());
    return true;
}

bool DiligentOdom::Impl::updateNonInvariantObservations(Eigen::Ref<const Eigen::VectorXd> deltaY,
                                                        Eigen::Ref<const Eigen::MatrixXd> H,
                                                        Eigen::Ref<const Eigen::MatrixXd> N,
                                                        FloatingBaseEstimators::InternalState& state,
                                                        Eigen::MatrixXd& P)
{
    const std::string printPrefix{"[DiligentOdom::Impl::updateNonInvariantObservations]"};
    if (H.cols() != P.rows())
    {
        log()->error("{} Measurement model Jacobian size mismatch.", printPrefix);
        return false;
    }

    if (H.rows() != N.rows())
    {
        log()->error("{} Measurement noise covariance matrix size mismatch.", printPrefix);
        return false;
    }

    m_PHT = P*H.transpose();
    m_S = H*m_PHT + N;
    m_K = m_PHT*(m_S.inverse());

    if (Eigen::isnan(m_K.array()).sum() == true)
    {
        log()->error("{} NaN values in Kalman gain.", printPrefix);
        return false;
    }

    m_deltaX = m_K*deltaY;

    constructMatrixLieGroup(state, m_X);

    // update state
    if (!calcExpHatX(m_deltaX, m_dX))
    {
        log()->error("{} Could not compute state exphat", printPrefix);
        return false;
    }

    if (!calcljac(m_deltaX, m_dXJr))
    {
        log()->error("{} Could not compute left Jacobian of X", printPrefix);
        return false;
    }

    // update using right multiplication
    // since deltaX is a right trivialized tangent parametrization
    if (!composeX(m_dX, m_X, m_Xout))
    {
        log()->error("{} Could not compose state Lie groups", printPrefix);
        return false;
    }
    m_X = m_Xout;
    if (!extractFromMatrixLieGroup(m_Xout, state)) // right invariant update
    {
        log()->error("{} Could not compute state exphat", printPrefix);
        return false;
    }

    // update covariance
    // pay attention to the Jacobian acting on the covariance
    m_IminusKH = m_In - m_K*H;
    P = m_dXJr*m_IminusKH*P*(m_dXJr.transpose());

    return true;
}

void DiligentOdom::Impl::constructMatrixLieGroup(const FloatingBaseEstimators::InternalState& state,
                                                 Eigen::MatrixXd& X)
{
    // base + vertices, angular vel, foot rotations
    std::size_t n{5 + state.supportFrameData.size() + 4 + 6};
    X.resize(n, n);
    X.setIdentity();

    X.block<3, 3>(0, 0) = state.imuOrientation.toRotationMatrix();
    X.block<3, 1>(0, 3) = state.imuPosition;
    X.block<3, 1>(0, 4) = state.imuLinearVelocity;

    std::size_t idx{0};
    std::size_t feetOffsetInX{5};
    for (auto& [frameIdx, supportFrame] : state.supportFrameData)
    {
       X.block<3, 1>(0, feetOffsetInX + idx) =  supportFrame.pose.translation();
       idx++;
    }

    std::size_t angVelOffset{5+state.supportFrameData.size()};
    X.block<3, 1>(angVelOffset, angVelOffset+3) = state.imuAngularVelocity;
    X.block<3, 3>(angVelOffset+4, angVelOffset+4) = state.lContactFrameOrientation.toRotationMatrix();
    X.bottomRightCorner<3, 3>() = state.rContactFrameOrientation.toRotationMatrix();
}

void DiligentOdom::Impl::calcXinv(const FloatingBaseEstimators::InternalState& state,
                                  FloatingBaseEstimators::InternalState& stateInv)
{

    Eigen::Matrix3d RT = state.imuOrientation.toRotationMatrix().transpose();
    stateInv.imuOrientation = state.imuOrientation.inverse();
    stateInv.imuPosition = -RT*state.imuPosition;
    stateInv.imuLinearVelocity = -RT*state.imuLinearVelocity;

    for (auto& [frameIdx, supportFrame] : state.supportFrameData)
    {
       stateInv.supportFrameData[frameIdx].pose.translation(-RT*supportFrame.pose.translation());
    }

    stateInv.imuAngularVelocity = -state.imuAngularVelocity;
    stateInv.lContactFrameOrientation = state.lContactFrameOrientation.inverse();
    stateInv.rContactFrameOrientation = state.rContactFrameOrientation.inverse();
}

bool DiligentOdom::Impl::extractFromMatrixLieGroup(const Eigen::MatrixXd& X,
                                                   FloatingBaseEstimators::InternalState& state)
{
    const std::string printPrefix{"[DiligentOdom::Impl::extractFromMatrixLieGroup]"};
    // base (R p v), angular velocity, feet rotations
    long int nrFeetVertices{X.cols() - 5 - 4 - 6};

    if (nrFeetVertices != state.supportFrameData.size())
    {
        log()->error("{} Feet vertices size mismatch.", printPrefix);
        return false;
    }

    state.imuOrientation =  toManifRot(X.block<3, 3>(0, 0)).quat();
    state.imuPosition = X.block<3, 1>(0, 3);
    state.imuLinearVelocity = X.block<3, 1>(0, 4);

    std::size_t idx{0};
    std::size_t feetOffsetInX{5};
    for (auto& [frameIdx, supportFrame] : state.supportFrameData)
    {
       supportFrame.pose.translation( X.block<3, 1>(0, feetOffsetInX + idx));
       idx++;
    }

    std::size_t angVelOffset{5+state.supportFrameData.size()};
    state.imuAngularVelocity = X.block<3, 1>(angVelOffset, angVelOffset+3);
    state.lContactFrameOrientation = toManifRot(X.block<3, 3>(angVelOffset+4, angVelOffset+4)).quat();
    state.rContactFrameOrientation = toManifRot(X.bottomRightCorner<3, 3>()).quat();

    return true;
}

bool DiligentOdom::Impl::calcExpHatX(const Eigen::VectorXd& v, Eigen::MatrixXd& X)
{
    const std::string printPrefix{"[DiligentOdom::Impl::exphatG]"};
    // base (R p v), angular velocity, feet rotations
    // in tangent space
    auto vertexDim{v.rows() - 9 - 3 - 6};
    if (vertexDim%3 != 0)
    {
        log()->error("{} Feet vertices size mismatch.", printPrefix);
        return false;
    }
    auto nrVertices{vertexDim/3};

    auto n{5 + nrVertices + 4 + 6};
    X.resize(n, n);
    X.setIdentity();
    manif::SO3Tangentd so3basetan(v.segment<3>(imuOrientationOffset));
    Eigen::Matrix3d JlSO3b = so3basetan.ljac();

    X.block<3, 3>(0, 0) = so3basetan.exp().rotation();
    X.block<3, 1>(0, 3) = JlSO3b*v.head<3>();
    X.block<3, 1>(0, 4) =  JlSO3b*v.segment<3>(imuLinearVelOffset);

    auto feetVertexOffsetInX{5};
    auto feetVertexOffsetInv{9};
    for (std::size_t idx = 0; idx < nrVertices; idx++)
    {
        const Eigen::Vector3d& v_d = v.segment<3>(feetVertexOffsetInv + (dim3*idx));
       X.block<3, 1>(0, feetVertexOffsetInX + idx) =  JlSO3b*v_d;
    }

    auto angVelOffsetInX{5+nrVertices};
    auto angVelOffsetInv{9+vertexDim};
    X.block<3, 1>(angVelOffsetInX, angVelOffsetInX+3) = v.segment<3>(angVelOffsetInv);

    manif::SO3Tangentd so3lrottan(v.segment<3>(angVelOffsetInv+3));
    manif::SO3Tangentd so3rrottan(v.tail<3>());
    X.block<3, 3>(angVelOffsetInX+4, angVelOffsetInX+4) = so3lrottan.exp().rotation();
    X.bottomRightCorner<3, 3>() = so3rrottan.exp().rotation();
    return true;
}

bool DiligentOdom::Impl::calcAdjX(const FloatingBaseEstimators::InternalState& state,
                                  Eigen::MatrixXd& AdjX)
{
    const std::string printPrefix{"[DiligentOdom::Impl::calcAdjX]"};
    auto nrVertices{state.supportFrameData.size()};
    auto n{9+(nrVertices*3)+3+6};
    AdjX.resize(n, n);
    AdjX.setZero();

    Eigen::Matrix3d R = state.imuOrientation.toRotationMatrix();
    AdjX.block<3, 3>(imuPositionOffset, imuPositionOffset) = R;
    AdjX.block<3, 3>(imuPositionOffset, imuOrientationOffset) = manif::skew(state.imuPosition)*R;

    AdjX.block<3, 3>(imuOrientationOffset, imuOrientationOffset) = R;

    AdjX.block<3, 3>(imuLinearVelOffset, imuOrientationOffset) = manif::skew(state.imuLinearVelocity)*R;
    AdjX.block<3, 3>(imuLinearVelOffset, imuLinearVelOffset) = R;

    int idx = 0;
    for (auto& [frameIdx, supportFrame] : state.supportFrameData)
    {
        const Eigen::Vector3d& d = supportFrame.pose.translation();
        int q = extMotionDim + (dim3*idx);
        AdjX.block<3, 3>(q, imuOrientationOffset) = manif::skew(d)*R;
        AdjX.block<3, 3>(q, q) = R;
        idx++;
    }

    auto angOffset = imuAngVelOffset(state);
    AdjX.block<3, 3>(angOffset, angOffset) = m_I3;

    auto q = lfRotOffset(state);
    AdjX.block<3, 3>(q, q) = state.lContactFrameOrientation.toRotationMatrix();
    AdjX.bottomRightCorner<3, 3>() = state.rContactFrameOrientation.toRotationMatrix();
    return true;
}

bool DiligentOdom::Impl::composeX(const Eigen::MatrixXd& X1,
                                  const Eigen::MatrixXd& X2,
                                  Eigen::MatrixXd& X)
{
    const std::string printPrefix{"[DiligentOdom::Impl::composeX]"};

    if (X1.rows() != X2.rows() || X1.cols() != X2.cols())
    {
        log()->error("{} matrix size mismatch.", printPrefix);
        return false;
    }

    auto nrVertices{X1.rows() - 5 - 4 -6};

    X.resize(X1.rows(), X1.cols());
    X.setIdentity();

    const Eigen::Matrix3d& R1 = X1.block<3, 3>(0, 0);
    const Eigen::Vector3d& p1 = X1.block<3, 1>(0, 3);
    const Eigen::Vector3d& v1 = X1.block<3, 1>(0, 4);

    const Eigen::Matrix3d& R2 = X2.block<3, 3>(0, 0);
    const Eigen::Vector3d& p2 = X2.block<3, 1>(0, 3);
    const Eigen::Vector3d& v2 = X2.block<3, 1>(0, 4);

    X.block<3, 3>(0, 0) = R1*R2;
    X.block<3, 1>(0, 3) = R1*p2 + p1;
    X.block<3, 1>(0, 4) = R1*v2 + v1;

    auto feetOffsetInX{5};
    for (std::size_t idx = 0; idx < nrVertices; idx++)
    {
        const Eigen::Vector3d& d1 = X1.block<3, 1>(0, feetOffsetInX + idx);
        const Eigen::Vector3d& d2 = X2.block<3, 1>(0, feetOffsetInX + idx);
        X.block<3, 1>(0, feetOffsetInX + idx) =  R1*d2 + d1;
    }

    auto angVelOffsetInX{5+nrVertices};
    const Eigen::Vector3d& w1 = X1.block<3, 1>(angVelOffsetInX, angVelOffsetInX+3);
    const Eigen::Vector3d& w2 = X2.block<3, 1>(angVelOffsetInX, angVelOffsetInX+3);
    X.block<3, 1>(angVelOffsetInX, angVelOffsetInX+3) = w1 + w2;

    const Eigen::Matrix3d& Zl1 = X1.block<3, 3>(angVelOffsetInX+4, angVelOffsetInX+4);
    const Eigen::Matrix3d& Zl2 = X2.block<3, 3>(angVelOffsetInX+4, angVelOffsetInX+4);

    X.block<3, 3>(angVelOffsetInX+4, angVelOffsetInX+4) = Zl1*Zl2;

    const Eigen::Matrix3d& Zr1 = X1.bottomRightCorner<3, 3>();
    const Eigen::Matrix3d& Zr2 = X2.bottomRightCorner<3, 3>();
    X.bottomRightCorner<3, 3>() = Zr1*Zr2;

    return true;
}

bool DiligentOdom::Impl::calcljac(const Eigen::VectorXd& v, Eigen::MatrixXd& J)
{
    const std::string printPrefix{"[DiligentOdom::Impl::calcljac]"};
    // base (R p v), angular velocity, feet rotations
    // in tangent space
    auto feetDim{v.rows() - 9 - 3 - 6};
    if (feetDim%3 != 0)
    {
        log()->error("{} Feet size mismatch.", printPrefix);
        return false;
    }
    auto nrVertices{feetDim/3};
    J.resize(v.rows(), v.rows());
    J.setZero();

    const Eigen::Vector3d& wb = v.segment<3>(imuOrientationOffset);
    manif::SO3Tangentd so3basetan(wb);
    Eigen::Matrix3d JlSO3b = so3basetan.ljac();
    J.topLeftCorner<3, 3>() = J.block<3, 3>(imuOrientationOffset, imuOrientationOffset) =
                J.block<3, 3>(imuLinearVelOffset, imuLinearVelOffset) = JlSO3b;

    Eigen::Ref<Eigen::Matrix3d> Qv = J.template block<3, 3>(imuPositionOffset, imuOrientationOffset);
    manif::SE3Tangentd::fillQ(Qv, v.head<6>());

    Eigen::Matrix<double, 6, 1> tempv;
    tempv << v.segment<3>(imuLinearVelOffset), wb;
    Eigen::Ref<Eigen::Matrix3d> Qa = J.template block<3, 3>(imuLinearVelOffset, imuOrientationOffset);
    manif::SE3Tangentd::fillQ(Qa, tempv);

    auto feetOffsetInv{9};
    for (std::size_t idx = 0; idx < nrVertices; idx++)
    {
        auto currentFootOffset = feetOffsetInv + (dim3*idx);
        J.block<3, 3>(currentFootOffset, currentFootOffset) = JlSO3b;

        tempv << v.segment<3>(currentFootOffset), wb;
        Eigen::Ref<Eigen::Matrix3d> Qd = J.template block<3, 3>(currentFootOffset, imuOrientationOffset);
        manif::SE3Tangentd::fillQ(Qd, tempv);
    }

    auto angVelOffsetInv{9+feetDim};
    J.block<3, 3>(angVelOffsetInv, angVelOffsetInv).setIdentity();

    auto lfRotOffsetInv{angVelOffsetInv+3};
    manif::SO3Tangentd so3lrottan(v.segment<3>(lfRotOffsetInv));
    manif::SO3Tangentd so3rrottan(v.tail<3>());
    J.block<3, 3>(lfRotOffsetInv, lfRotOffsetInv) = so3lrottan.ljac();
    J.bottomRightCorner<3, 3>() = so3rrottan.ljac();

    return true;
}


bool DiligentOdom::Impl::addContact(const int& idx,
                                    const double& time,
                                    const bool& isActive,
                                    const manif::SE3d& poseEstimate,
                                    const FloatingBaseEstimators::SensorsStdDev& sensStdDev,
                                    const double& dt,
                                    FloatingBaseEstimators::InternalState& state,
                                    FloatingBaseEstimators::StateStdDev& stateDev,
                                    Eigen::MatrixXd& P)
{
    // Adding contact is like,
    // [v1]    [I 0][v1]   [0 ]
    // [v2] =  [0 0][v3] + [n2]
    // [v3]    [0 I]       [0 ]

    const std::string printPrefix{"[DiligentOdom::Impl::addContact]"};
    int oldCols = P.cols();
    int newRows = P.rows() + dim3;
    int newCols = P.cols() + dim3;
    Fnew.resize(newRows, oldCols);
    Fnew.setZero();
    Fnew.topLeftCorner(extMotionDim, extMotionDim).setIdentity();
    Qnew.resize(newRows, newCols);
    Qnew.setZero();

    // add contact
    if (state.supportFrameData.find(idx) != state.supportFrameData.end())
    {
        log()->error("{} contact already exists.", printPrefix);
        return false;
    }

    BipedalLocomotion::Contacts::EstimatedContact newContact;
    newContact.index = idx;
    newContact.name = "Contact#" + std::to_string(idx);
    newContact.lastUpdateTime = time;
    newContact.pose = poseEstimate;
    newContact.isActive = isActive;
    state.supportFrameData[idx] = newContact;

    Eigen::Vector3d supportFrameNoise, Qf;
    if (isActive)
    {
        supportFrameNoise << sensStdDev.contactFootLinvelNoise;
    }
    else
    {
        supportFrameNoise << sensStdDev.swingFootLinvelNoise;
    }
    stateDev.supportFramePose[idx].head<3>() << supportFrameNoise;

    int j = 0;
    for (auto& iter : state.supportFrameData)
    {
        int q = extMotionDim + (j*dim3);
        if (iter.first < idx)
        {
            Fnew.block<3, 3>(q, q).setIdentity();
        }
        else if (iter.first == idx)
        {
            Qf = supportFrameNoise.array().square();
            Qnew.block<3, 3>(q, q) = dt*static_cast<Eigen::Matrix<double, 3, 3> >(Qf.asDiagonal());
        }
        else if (iter.first > idx)
        {
            Fnew.block<3, 3>(q+dim3, q).setIdentity();
        }
        j++;
    }

    // preserve structure related to ang vel, foot rotations
    Fnew.bottomRightCorner<9, 9>().setIdentity();

    // update covariance
    P = Fnew * P * Fnew.transpose() + Qnew;
    m_In = Eigen::MatrixXd::Identity(P.rows(), P.cols());

    return true;
}

bool DiligentOdom::Impl::removeContact(const int& idx,
                                       FloatingBaseEstimators::InternalState& state,
                                       FloatingBaseEstimators::StateStdDev& stateDev,
                                       Eigen::MatrixXd& P)
{
    // Removing contact is like,
    // [v1]    [I 0 0][v1]
    // [v2] =  [0 I 0][v2]
    //                [v3]
    const std::string printPrefix{"[DiligentOdom::Impl::removeContact]"};
    int oldCols = P.cols();
    int newRows = P.rows() - dim3;

    Fnew.resize(newRows, oldCols);
    Fnew.setZero();
    Fnew.topLeftCorner(extMotionDim, extMotionDim).setIdentity();

    // remove contact
    if (state.supportFrameData.find(idx) == state.supportFrameData.end())
    {
        log()->error("{} contact already exists.", printPrefix);

        return false;
    }

    int j = 0;
    for (auto& iter : state.supportFrameData)
    {
        int q = extMotionDim + (j*dim3);
        if (iter.first < idx)
        {
            Fnew.block<3, 3>(q, q).setIdentity();
        }
        else if (iter.first > idx)
        {
            Fnew.block<3, 3>(q-dim3, q).setIdentity();
        }
        j++;
    }

    // preserve structure related to ang vel, foot rotations
    Fnew.bottomRightCorner<9, 9>().setIdentity();

    state.supportFrameData.erase(idx);
    stateDev.supportFramePose.erase(idx);

    // update covariance
    P = Fnew * P * Fnew.transpose();
    m_In = Eigen::MatrixXd::Identity(P.rows(), P.cols());
    return true;
}
