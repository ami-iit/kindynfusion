/**
 * @file WholeBodyKinematicsVisualizerDevice.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_WHOLE_BODY_KINEMATICS_VISUALIZER_DEVICE_H
#define KINDYNFUSION_WHOLE_BODY_KINEMATICS_VISUALIZER_DEVICE_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>

#include <memory>

namespace KinDynFusion
{

class WholeBodyKinematicsVisualizerDevice : public yarp::dev::DeviceDriver,
                                            public yarp::os::PeriodicThread
{
public:
    WholeBodyKinematicsVisualizerDevice(double period,
                              yarp::os::ShouldUseSystemClock useSystemClock
                              = yarp::os::ShouldUseSystemClock::No);
    WholeBodyKinematicsVisualizerDevice();
    ~WholeBodyKinematicsVisualizerDevice();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;

    virtual bool threadInit() final;
    virtual void run() final;
    virtual void threadRelease() final;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace KinDynFusion

#endif //KINDYNFUSION_WHOLE_BODY_KINEMATICS_VISUALIZER_DEVICE_H
