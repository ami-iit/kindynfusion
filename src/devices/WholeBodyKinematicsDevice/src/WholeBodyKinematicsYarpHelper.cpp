/**
 * @file WholeBodyKinematicsYarpHelper.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/WholeBodyKinematicsYarpHelper.h>
#include <yarp/os/LogStream.h>

using namespace KinDynFusion;
using namespace KinDynFusion::Estimators;
using namespace BipedalLocomotion::Contacts;

bool WholeBodyKinematicsYarpHelper::addFTShoesFromConfig(yarp::os::Searchable& config,
                                                         WholeBodyKinematics& wbk,
                                                         const std::string& printPrefix)
{
    yarp::os::Bottle& shoesGroup = config.findGroup("WEARABLE_SENSOR_SHOES");
    if (shoesGroup.isNull())
    {
        yError() << printPrefix
                 << " [addFTShoesFromConfig] Failed to find group WEARABLE_SENSOR_SHOES.";
        return false;
    }

    for (size_t i = 1; i < shoesGroup.size(); ++i)
    {
        if (!(shoesGroup.get(i).isList() &&
              shoesGroup.get(i).asList()->size() == 2))
        {
            yError() << printPrefix
                     << " [addFTShoesFromConfig] Children of WEARABLE_SENSOR_SHOES must be lists of two elements.";
            return false;
        }

        yarp::os::Bottle* list = shoesGroup.get(i).asList();
        yarp::os::Bottle* sublist = list->get(1).asList();
        std::string shoeWearableName = list->get(0).asString();
        std::string modelLinkName = sublist->get(0).asString();
        iDynTree::Rotation footRshoe;
        iDynTree::Position footpshoe;
        bool ok = readTransformFromConfig(sublist,
                                          footRshoe,
                                          footpshoe);

        if (!ok)
        {
            yError() << printPrefix << " [addFTShoesFromConfig] "
                     << " WEARABLE_SENSOR_SHOES "
                     << shoeWearableName << " must have 16 double values describing the transformation matrix soleHshoe.";
            return false;
        }

        if (!wbk.addWearableSensorizedShoes(shoeWearableName, modelLinkName,
                                            iDynTree::Transform(footRshoe, footpshoe)))
        {
            yError() << printPrefix << " [addFTShoesFromConfig] "
                     << " Could not add sensorized shoe: "
                     << shoeWearableName << ".";
            return false;
        }

    }
    return true;
}

bool WholeBodyKinematicsYarpHelper::addTargetsFromConfig(yarp::os::Searchable& config,
                                                         WholeBodyKinematics& wbk,
                                                         const std::string& printPrefix)
{
    yarp::os::Bottle& linksGroup = config.findGroup("WEARABLE_SENSOR_TARGETS");
    if (linksGroup.isNull())
    {
        yError() << printPrefix
                 << " [addTargetsFromConfig] Failed to find group WEARABLE_SENSOR_TARGETS.";
        return false;
    }

    for (size_t i = 1; i < linksGroup.size(); ++i)
    {
        if (!(linksGroup.get(i).isList() &&
              linksGroup.get(i).asList()->size() == 2))
        {
            yError() << printPrefix
                     << " [addTargetsFromConfig] Children of WEARABLE_SENSOR_TARGETS must be lists of two elements.";
            return false;
        }
        yarp::os::Bottle* list = linksGroup.get(i).asList();
        std::string targetName = list->get(0).asString();
        yarp::os::Bottle* listContent = list->get(1).asList();

        if (!((listContent->size() == 3) &&
              (listContent->get(0).isString()) &&
              (listContent->get(1).isString()) &&
              (listContent->get(2).isString()) ))
        {
            yError() << printPrefix
                     << " [addTargetsFromConfig] Link list must have two strings.";
            return false;
        }

        auto modelLinkName    = listContent->get(0).asString();
        auto wearableName     = listContent->get(1).asString();
        auto targetTypeString = listContent->get(2).asString();

        if (!wbk.addWearableSensorTarget(wearableName,
                                         modelLinkName,
                                         targetTypeString,
                                         targetName))
        {
            yError() << printPrefix << " [addTargetsFromConfig] "
                     << " Could not add target for wearable sensor: "
                     << wearableName << ".";
            return false;
        }

    }
    return true;
}

bool WholeBodyKinematicsYarpHelper::setWearableCalibMatricesFromConfig(yarp::os::Searchable& config,
                                                                       WholeBodyKinematics& wbk,
                                                                       const std::string& printPrefix)
{
    yarp::os::Bottle& fixedRightTransformGroup = config.findGroup("MEASUREMENT_TO_LINK_TRANSFORMS");
    if (!fixedRightTransformGroup.isNull())
    {
        for (std::size_t i = 1; i < fixedRightTransformGroup.size(); ++i)
        {
            if (!(fixedRightTransformGroup.get(i).isList() &&
                  fixedRightTransformGroup.get(i).asList()->size() == 2))
            {
                yError() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                         << " Children of MEASUREMENT_TO_LINK_TRANSFORMS must be lists of 2 elements";
                return false;
            }
            yarp::os::Bottle* list = fixedRightTransformGroup.get(i).asList();
            std::string targetName = list->get(0).asString();
            iDynTree::Rotation fixedRightRotationOffset;
            iDynTree::Position fixedRightPositionOffset;
            bool ok = readTransformFromConfig(list,
                                              fixedRightRotationOffset,
                                              fixedRightPositionOffset);

            if (!ok)
            {
                yError() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                         << " MEASUREMENT_TO_LINK_TRANSFORMS "
                         << targetName << " must have 16 double values describing the transformation matrix sensorHsegment.";
                return false;
            }

            auto werableSensorPtr = wbk.getWearableTarget(targetName);
            if (!werableSensorPtr)
            {
                yError() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                         << targetName << " target not found in WEARABLE_SENSOR_TARGETS group."
                         << " Unable to load transform from MEASUREMENT_TO_LINK_TRANSFORMS group.";
                return false;
            }

           werableSensorPtr->
                getCalibrationStorage().sensor_H_segment.setRotation(fixedRightRotationOffset);
            werableSensorPtr->
                getCalibrationStorage().sensor_H_segment.setPosition(fixedRightPositionOffset);

            yInfo() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                    << " MEASUREMENT_TO_LINK_TRANSFORMS added for target " << targetName
                    << "\n" << werableSensorPtr->getCalibrationStorage().sensor_H_segment.toString();
        }
    }

    yarp::os::Bottle& fixedLeftTransformGroup = config.findGroup("WORLD_TO_MEASUREMENT_TRANSFORMS");
    if (!fixedLeftTransformGroup.isNull())
    {
        for (std::size_t i = 1; i < fixedLeftTransformGroup.size(); ++i)
        {
            if (!(fixedLeftTransformGroup.get(i).isList() && fixedLeftTransformGroup.get(i).asList()->size() == 2))
            {
                yError() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                         << "Children of WORLD_TO_MEASUREMENT_TRANSFORMS must be lists of 2 elements";
                return false;
            }

            yarp::os::Bottle* list = fixedLeftTransformGroup.get(i).asList();
            std::string targetName = list->get(0).asString();
            iDynTree::Rotation fixedLeftRotationOffset;
            iDynTree::Position fixedLeftPositionOffset;
            bool ok = readTransformFromConfig(list,
                                              fixedLeftRotationOffset,
                                              fixedLeftPositionOffset);

            if (!ok)
            {
                yError() << printPrefix  << " [setWearableCalibMatricesFromConfig] "
                         << "WORLD_TO_MEASUREMENT_TRANSFORMS "
                         << targetName << " must have 16 double values describing the transformation matrix describing calibWorldHsensorWorld.";
                return false;
            }

            auto werableSensorPtr = wbk.getWearableTarget(targetName);
            if (!werableSensorPtr)
            {
                yError() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                         << targetName << " target not found in WEARABLE_SENSOR_TARGETS group."
                         << " Unable to load transform from WORLD_TO_MEASUREMENT_TRANSFORMS group.";
                return false;
            }

            werableSensorPtr->
                getCalibrationStorage().calibrationWorld_H_sensorWorld.setRotation(fixedLeftRotationOffset);
            werableSensorPtr->
                getCalibrationStorage().calibrationWorld_H_sensorWorld.setPosition(fixedLeftPositionOffset);

            yInfo() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                    << "WORLD_TO_MEASUREMENT_TRANSFORMS added for target " << targetName
                    << "\n" << werableSensorPtr->getCalibrationStorage().calibrationWorld_H_sensorWorld.toString();
        }
    }

    yarp::os::Bottle& positionScaleFactorGroup = config.findGroup("MEASUREMENT_POSITION_SCALE_FACTOR");
    if (!positionScaleFactorGroup.isNull())
    {
        for (std::size_t i = 1; i < positionScaleFactorGroup.size(); ++i)
        {
            if (!(positionScaleFactorGroup.get(i).isList() &&
                  positionScaleFactorGroup.get(i).asList()->size() == 2))
            {
                yError() << printPrefix  << " [setWearableCalibMatricesFromConfig] "
                         << "Children of MEASUREMENT_POSITION_SCALE_FACTOR must be lists of 2 elements";
                return false;
            }
            yarp::os::Bottle* list = positionScaleFactorGroup.get(i).asList();
            std::string targetName = list->get(0).asString();
            yarp::os::Bottle* listContent = list->get(1).asList();

            if (!( (listContent->size() == 3) &&
                   (listContent->get(0).isFloat64()) &&
                   (listContent->get(1).isFloat64()) &&
                   (listContent->get(2).isFloat64()) ))
            {
                yError() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                         << "MEASUREMENT_POSITION_SCALE_FACTOR "  << targetName
                         << " must have 3 double values describing the scaling factor for x, y, and z axis";
                return false;
            }

            auto werableSensorPtr = wbk.getWearableTarget(targetName);
            if (!werableSensorPtr)
            {
                yError() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                         << targetName << " target not found in WEARABLE_SENSOR_TARGETS group."
                         << " Unable to load scaling factor from MEASUREMENT_POSITION_SCALE_FACTOR group.";
                return false;
            }

            auto& positionScaleFactor = werableSensorPtr->
                                            getCalibrationStorage().positionScalingFactor;
            for (auto jdx = 0; jdx < 2; jdx++)
            {
                positionScaleFactor.setVal(jdx,
                                           listContent->get(0).asFloat64());
            }

            yInfo() << printPrefix << " [setWearableCalibMatricesFromConfig] "
                    << "MEASUREMENT_POSITION_SCALE_FACTOR added for target " << targetName
                    << "\n" << werableSensorPtr->getCalibrationStorage().positionScalingFactor.toString();
        }
    }

    return true;
}

bool WholeBodyKinematicsYarpHelper::readTransformFromConfig(const yarp::os::Bottle* list,
                                                            iDynTree::Rotation& rot,
                                                            iDynTree::Position& pos)
{
    if (!list)
    {
        return false;
    }

    yarp::os::Bottle* listContent = list->get(1).asList();

    bool ok{true};
    ok = (listContent->size() == 16);
    for (auto jdx =0; jdx < 16; jdx ++)
    {
        ok = ok && listContent->get(jdx).isFloat64();
    }

    if (!ok)
    {
        return false;
    }

    rot = iDynTree::Rotation(listContent->get(0).asFloat64(), listContent->get(1).asFloat64(), listContent->get(2).asFloat64(),
                             listContent->get(4).asFloat64(), listContent->get(5).asFloat64(), listContent->get(6).asFloat64(),
                             listContent->get(8).asFloat64(), listContent->get(9).asFloat64(), listContent->get(10).asFloat64());
    pos = iDynTree::Position(listContent->get(3).asFloat64(), listContent->get(7).asFloat64(), listContent->get(11).asFloat64());

    return true;
}

bool WholeBodyKinematicsYarpHelper::setFeetDataFromConfig(yarp::os::Searchable& config,
                                                          FootMetaData& lFoot,
                                                          FootMetaData& rFoot)
{
    std::string printPrefix{"[WholeBodyKinematicsYarpHelper::setFootDataFromConfig] "};
    yarp::os::Bottle& footDataGroup = config.findGroup("FOOT_DATA");
    if (footDataGroup.isNull())
    {
        yError() << printPrefix << "Missing group \"FOOT_DATA\".";
        return false;
    }

    yarp::os::Bottle* feet = footDataGroup.find("foot_link_names").asList();
    if (feet->size() != 2)
    {
        yError() << printPrefix << " Expecting only two elements in \"foot_link_names\".";
        return false;
    }

    for (std::size_t idx = 0; idx < 2; idx++)
    {
        std::string footLinkName = feet->get(idx).asString();
        yarp::os::Bottle& footGroup = footDataGroup.findGroup(footLinkName);
        if (footGroup.isNull())
        {
            yError() << printPrefix
                    << "Failed to load "<< footLinkName <<" data.";
            return false;
        }

        if (footLinkName == "LeftFoot")
        {
            if (!setFootDataFromConfig(footGroup, footLinkName, lFoot))
            {
                yError() << printPrefix
                         << "Failed to load "<< footLinkName <<" data.";
                return false;
            }
        }
        else if (footLinkName == "RightFoot")
        {
            if (!setFootDataFromConfig(footGroup, footLinkName, rFoot))
            {
                yError() << printPrefix
                         << "Failed to load "<< footLinkName <<" data.";
                return false;
            }
        }
    }

    return true;
}

bool WholeBodyKinematicsYarpHelper::setFootDataFromConfig(yarp::os::Searchable& config,
                                                          const std::string& linkName,
                                                          FootMetaData& foot)
{
    std::string printPrefix{"[WholeBodyKinematicsYarpHelper::setFootDataFromConfig] "};
    bool ok{true};
    ok = ok && (config.check("wearable_name") &&
                config.find("wearable_name").isString());
    ok = ok && (config.check("sole_frame") &&
                config.find("sole_frame").isString());
    ok = ok && (config.check("sole_position_in_foot") &&
                config.find("sole_position_in_foot").isList());
    ok = ok && (config.check("sole_rpy_degrees_in_foot") &&
                config.find("sole_rpy_degrees_in_foot").isList());
    ok = ok && (config.check("top_left_vertex_in_sole") &&
                config.find("top_left_vertex_in_sole").isList());
    ok = ok && (config.check("foot_length") &&
                config.find("foot_length").isFloat64());
    ok = ok && (config.check("foot_width") &&
                config.find("foot_width").isFloat64());
    ok = ok && (config.check("fz_threshold_for_cop") &&
                config.find("fz_threshold_for_cop").isFloat64());
    ok = ok && (config.check("schmitt_make_threshold") &&
                config.find("schmitt_make_threshold").isFloat64());
    ok = ok && (config.check("schmitt_break_threshold") &&
                config.find("schmitt_break_threshold").isFloat64());
    ok = ok && (config.check("schmitt_make_switch_time") &&
                config.find("schmitt_make_switch_time").isFloat64());
    ok = ok && (config.check("schmitt_break_switch_time") &&
                config.find("schmitt_break_switch_time").isFloat64());

    yarp::os::Bottle* pos = config.find("sole_position_in_foot").asList();
    yarp::os::Bottle* rpy = config.find("sole_rpy_degrees_in_foot").asList();
    yarp::os::Bottle* topLeft = config.find("top_left_vertex_in_sole").asList();
    ok = ok && (pos!= nullptr) && (pos->size() == 3);
    ok = ok && (rpy!= nullptr) && (rpy->size() == 3);
    ok = ok && (topLeft!= nullptr) && (topLeft->size() == 3);
    if (!ok)
    {
        yError() << printPrefix
                    << "Failed to load parameters for "<< linkName <<".";
        return false;
    }


    foot.footLinkName = linkName;
    foot.wearableName = config.find("wearable_name").asString();
    foot.soleFrame = config.find("sole_frame").asString();
    foot.footLength = config.find("foot_length").asFloat64();
    foot.footWidth  = config.find("foot_width").asFloat64();
    foot.contactForceThresholdForCOPComputation  = config.find("fz_threshold_for_cop").asFloat64();
    foot.schmittParams.onThreshold  = config.find("schmitt_make_threshold").asFloat64();
    foot.schmittParams.offThreshold  = config.find("schmitt_break_threshold").asFloat64();
    foot.schmittParams.switchOnAfter  = config.find("schmitt_make_switch_time").asFloat64();
    foot.schmittParams.switchOffAfter  = config.find("schmitt_break_switch_time").asFloat64();
    foot.solePositionInFoot = iDynTree::Position(pos->get(0).asFloat64(),
                                                 pos->get(1).asFloat64(),
                                                 pos->get(2).asFloat64());
    foot.topLeftPositionInSole = iDynTree::Position(topLeft->get(0).asFloat64(),
                                                    topLeft->get(1).asFloat64(),
                                                    topLeft->get(2).asFloat64());
    for (std::size_t idx = 0; idx < 3; idx++)
    {
        foot.soleRPYInDegInFoot.setVal(idx, rpy->get(idx).asFloat64());
    }
    return true;
}

