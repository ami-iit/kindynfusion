/**
 * @file WholeBodyKinematicsRemapper.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_WHOLE_BODY_KINEMATICS_REMAPPER_H
#define KINDYNFUSION_WHOLE_BODY_KINEMATICS_REMAPPER_H

#include <KinDynFusion/FloatingBaseEstimators/IWholeBodyKinematics.h>
#include <KinDynFusion/msgs/WholeBodyKinematicsOutput.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace KinDynFusion
{

class WholeBodyKinematicsRemapper final : public yarp::dev::DeviceDriver,
                                          public KinDynFusion::Estimators::IWholeBodyKinematics,
                                          public yarp::os::TypedReaderCallback<KinDynFusion::msgs::WholeBodyKinematicsOutput>,
                                          public yarp::os::PeriodicThread
{
public:
    WholeBodyKinematicsRemapper(double period,
                              yarp::os::ShouldUseSystemClock useSystemClock
                              = yarp::os::ShouldUseSystemClock::No);
    WholeBodyKinematicsRemapper();
    ~WholeBodyKinematicsRemapper();

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    void onRead(KinDynFusion::msgs::WholeBodyKinematicsOutput& out) override;

    void run() override;

    // IWholeBodyKinematics
    virtual std::vector<std::string> getJointNames() const override;
    virtual std::string getBaseName() const override;
    virtual std::size_t getNumberOfJoints() const override;

    virtual std::vector<double> getJointPositions() const override;
    virtual std::vector<double> getJointVelocities() const override;

    virtual std::array<double, 3> getBasePosition() const override;
    virtual std::array<double, 4> getBaseOrientation() const override;

    virtual std::array<double, 6> getBaseVelocity() const override;

    // TODO missing implementation
    virtual std::array<double, 3> getCoMPosition() const override;
    virtual std::array<double, 3> getCoMVelocity() const override;

    virtual std::vector<KinDynFusion::Estimators::EstimatedContactPosition> getContactPointsPosition() override;
    virtual std::array<double, 4> getLeftFootOrientation() const override;
    virtual std::array<double, 4> getRightFootOrientation() const override;

    virtual std::array<double, 3> getLeftFootCoPPosition() override;
    virtual std::array<double, 3> getRightFootCoPPosition() override;
    virtual std::array<double, 3> getGlobalCoPPosition() override;

    virtual std::vector<KinDynFusion::Estimators::VertexContactData> getAllVertexContactData() override;
    virtual std::vector<std::string> getAllVertexContactNames() override;

    virtual std::string getLeftFootLinkName() const override;
    virtual std::string getRightFootLinkName() const override;
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace KinDynFusion

#endif //KINDYNFUSION_WHOLE_BODY_KINEMATICS_REMAPPER_H
