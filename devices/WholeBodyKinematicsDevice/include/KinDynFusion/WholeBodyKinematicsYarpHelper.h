/**
 * @file WholeBodyKinematicsYarpHelper.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_WHOLE_BODY_KINEMATICS_YARP_HELPER_H
#define KINDYNFUSION_WHOLE_BODY_KINEMATICS_YARP_HELPER_H

#include <KinDynFusion/FloatingBaseEstimators/WholeBodyKinematics.h>
#include <yarp/os/ResourceFinder.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Position.h>

namespace KinDynFusion
{

class WholeBodyKinematicsYarpHelper
{
public:
    static bool addFTShoesFromConfig(yarp::os::Searchable& config,
                                     KinDynFusion::Estimators::WholeBodyKinematics& wbk,
                                     const std::string& printPrefix);
    static bool addTargetsFromConfig(yarp::os::Searchable& config,
                                     KinDynFusion::Estimators::WholeBodyKinematics& wbk,
                                     const std::string& printPrefix);
    static bool setWearableCalibMatricesFromConfig(yarp::os::Searchable& config,
                                                   KinDynFusion::Estimators::WholeBodyKinematics& wbk,
                                                   const std::string& printPrefix);

    static bool readTransformFromConfig(const yarp::os::Bottle* list,
                                        iDynTree::Rotation& rot,
                                        iDynTree::Position& pos);
    static bool setFeetDataFromConfig(yarp::os::Searchable& config,
                                      KinDynFusion::Estimators::FootMetaData& lFoot,
                                      KinDynFusion::Estimators::FootMetaData& rFoot);
    static bool setFootDataFromConfig(yarp::os::Searchable& config,
                                      const std::string& linkName,
                                      KinDynFusion::Estimators::FootMetaData& foot);
};

}

#endif // KINDYNFUSION_WHOLE_BODY_KINEMATICS_YARP_HELPER_H
