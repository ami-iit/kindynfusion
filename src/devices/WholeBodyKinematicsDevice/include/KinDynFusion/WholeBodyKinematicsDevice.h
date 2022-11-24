/**
 * @file WholeBodyKinematicsDevice.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_WHOLE_BODY_KINEMATICS_DEVICE_H
#define KINDYNFUSION_WHOLE_BODY_KINEMATICS_DEVICE_H

#include <KinDynFusion/FloatingBaseEstimators/WholeBodyKinematics.h>
#include <KinDynFusion/FloatingBaseEstimators/CalibrationHelper.h>
#include <KinDynFusion/InverseKinematics/iWearToiDyn.h>

#include <KinDynFusion/InverseKinematics/IWearableTargets.h>
#include <KinDynFusion/InverseKinematics/IWearableSensorizedShoes.h>
#include <KinDynFusion/FloatingBaseEstimators/IWholeBodyKinematics.h>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Vocab.h>

#include <Wearable/IWear/IWear.h>

#include <mutex>
#include <atomic>
#include <memory>

namespace KinDynFusion
{

enum rpcCommand
{
    empty,
    calibrateAll,
    calibrateAllWorldYaw,
    calibrateAllWithWorld,
    setRotationOffset,
    resetCalibration,
    resetAll,
    startBaseEstimator,
    calibrateBaseEstimatorWithWorld
};

struct WholeBodyKinematicsOutput
{
    std::vector<double> jPos;
    std::vector<double> jVel;

    std::array<double, 3> basePos;
    std::array<double, 4> baseOrient;
    std::array<double, 6> baseVel;

    std::array<double, 3> lCoPPos;
    std::array<double, 3> rCoPPos;
    std::array<double, 3> globalCoPPos;

    std::array<double, 4> lfOrient;
    std::array<double, 4> rfOrient;
    std::vector<KinDynFusion::Estimators::EstimatedContactPosition> estContactPos;

    std::vector<KinDynFusion::Estimators::VertexContactData> contacts;
};

class WholeBodyKinematicsDevice : public yarp::dev::DeviceDriver,
                                  public yarp::dev::IMultipleWrapper,
                                  public yarp::os::PeriodicThread,
                                  public KinDynFusion::IK::IWearableTargets,
                                  public KinDynFusion::IK::IWearableSensorizedShoes,
                                  public KinDynFusion::Estimators::IWholeBodyKinematics
{
public:
    WholeBodyKinematicsDevice(double period,
                              yarp::os::ShouldUseSystemClock useSystemClock
                              = yarp::os::ShouldUseSystemClock::No);
    WholeBodyKinematicsDevice();
    ~WholeBodyKinematicsDevice();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual bool attachAll(const yarp::dev::PolyDriverList & poly) final;
    virtual bool detachAll() final;
    virtual void run() final;

    // IWearableTargets
    std::vector<KinDynFusion::IK::TargetName> getAllWearableTargetNames() override;
    WearableSensorTargetPtr getWearableTarget(const KinDynFusion::IK::TargetName& name) override;
    WearableSensorTargetMap getAllWearableTargets() override;

    // IWearableSensorizedShoes
    std::vector<std::string> getAllWearableShoeNames() override;
    WearableSensorizedShoePtr getWearableSensorizedShoe(const std::string& name) override;
    WearableSensorizedShoeMap getAllWearableSensorizedShoes() override;

    // IWholeBodyKinematics
    std::vector<std::string> getJointNames() const override;
    std::string getBaseName() const override;
    std::size_t getNumberOfJoints() const override;

    std::vector<double> getJointPositions() const override;
    std::vector<double> getJointVelocities() const override;

    std::array<double, 3> getBasePosition() const override;
    std::array<double, 4> getBaseOrientation() const override;

    std::array<double, 6> getBaseVelocity() const override;

    // TODO missing implementation
    virtual std::array<double, 3> getCoMPosition() const override;
    virtual std::array<double, 3> getCoMVelocity() const override;

    std::vector<KinDynFusion::Estimators::EstimatedContactPosition> getContactPointsPosition() override;
    std::array<double, 4> getLeftFootOrientation() const override;
    std::array<double, 4> getRightFootOrientation() const override;

    std::array<double, 3> getLeftFootCoPPosition() override;
    std::array<double, 3> getRightFootCoPPosition() override;
    std::array<double, 3> getGlobalCoPPosition() override;

    std::vector<KinDynFusion::Estimators::VertexContactData> getAllVertexContactData() override;
    std::vector<std::string> getAllVertexContactNames() override;

    std::string getLeftFootLinkName() const override;
    std::string getRightFootLinkName() const override;

private:
    bool initializeRPCPort(yarp::os::Searchable& config);

    bool checkAttachIMUNodes();
    bool checkAttachFTShoes();
    bool updateTargetsData();
    bool updateFTShoesData(const double& timeNow);
    bool updateBaseEKFInputs(const double& timeNow);
    void updateWholeBodyKinematicsSolution();
    void checkAndApplyRpcCommand();
    bool applyRpcCommand();

    std::string m_printPrefix{"KinDynFusion::WholeBodyKinematics"};
    mutable std::mutex m_deviceMutex;
    double m_lastTime{-1.0}, m_devicePeriod{0.02};

    std::vector<KinDynFusion::IK::TargetName> m_targetNames;
    std::vector<std::string> m_shoeNames;

    KinDynFusion::Estimators::WholeBodyKinematics m_wbk;
    KinDynFusion::IK::iWearToiDyn m_iwear2idyn;
    wearable::IWear* m_iWear{nullptr};
    KinDynFusion::Estimators::CalibrationHelper m_calibHelper;

    WholeBodyKinematicsOutput m_solution;

    class CmdParser : public yarp::os::PortReader
    {
    public:
        void resetInternalVariables();
        bool read(yarp::os::ConnectionReader& connection) override;
        std::atomic<rpcCommand> cmdStatus{rpcCommand::empty};
        std::string parentLinkName;
        std::string childLinkName;
        std::string refLinkName;
        // variables for manual calibration
        std::atomic<double> roll;  // [deg]
        std::atomic<double> pitch; // [deg]
        std::atomic<double> yaw;   // [deg]
    };

    CmdParser m_commandPro;
    yarp::os::RpcServer m_rpcPort;
};

} // namespace KinDynFusion

#endif //KINDYNFUSION_WHOLE_BODY_KINEMATICS_DEVICE_H
