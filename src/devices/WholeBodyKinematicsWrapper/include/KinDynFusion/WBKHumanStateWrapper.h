/**
 * @file WBKHumanStateWrapper.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_WHOLE_BODY_KINEMATICS_WRAPPER_H
#define KINDYNFUSION_WHOLE_BODY_KINEMATICS_WRAPPER_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>

#include <memory>

namespace KinDynFusion
{

class WBKHumanStateWrapper : public yarp::dev::DeviceDriver,
                                   public yarp::dev::IMultipleWrapper,
                                   public yarp::os::PeriodicThread
{
public:
    WBKHumanStateWrapper(double period,
                              yarp::os::ShouldUseSystemClock useSystemClock
                              = yarp::os::ShouldUseSystemClock::No);
    WBKHumanStateWrapper();
    ~WBKHumanStateWrapper();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual bool attachAll(const yarp::dev::PolyDriverList & poly) final;
    virtual bool detachAll() final;
    virtual void run() final;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace KinDynFusion

#endif //KINDYNFUSION_WHOLE_BODY_KINEMATICS_WRAPPER_H
