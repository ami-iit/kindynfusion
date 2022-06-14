/**
 * @file InverseKinematicsYarpHelper.h
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_INVERSE_KINEMATICS_INVERSE_KINEMATICS_YARP_HELPER_H
#define KINDYNFUSION_INVERSE_KINEMATICS_INVERSE_KINEMATICS_YARP_HELPER_H

#include <KinDynFusion/InverseKinematics/IKDataTypes.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Model/Model.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/LogStream.h>

#include <unordered_map>
#include <vector>
#include <string>

namespace KinDynFusion
{

namespace IK
{

class YarpHelper
{
public:
    static bool configure(const yarp::os::Searchable& config,
                          ModelData& modelData,
                          IKParameters& ikParams);
    static bool loadModelData(const yarp::os::Searchable& config,
                              ModelData& modelData);
protected:
    static void printModelConfigurationReport(const ModelData& modelData);

    // loads only top-level parameters of the struct
    // and does not load any of the parameters of nested structs
    static bool loadIKParameters(const yarp::os::Searchable& config,
                                 IKParameters& ikParams);
    static void printIKConfigurationReport(const IKParameters& ikParams);

    static bool loadDynamicalIKParameters(const yarp::os::Searchable& config,
                                          DynamicalIKParameters& dynIKParams);
    static void printDynamicalIKConfigurationReport(const DynamicalIKParameters& dynIKParams);

    static bool loadCustomIKConstraintParameters(const yarp::os::Searchable& config,
                                                 const ModelData& modelData,
                                                 IKParameters& ikParams,
                                                 CustomIKConstraints& custom);
    static void printCustomIKConstraintsConfigurationReport(const CustomIKConstraints& custom);

public:

    template <typename Scalar>
    static bool checkAndLoadScalar(const yarp::os::Searchable& config,
                                   const std::string& paramName,
                                   Scalar& param,
                                   const Scalar& defaultValue,
                                   bool required = true)
    {
        std::string printPrefix;
        bool ok = config.check(paramName);

        if constexpr (std::is_same<Scalar, double>::value)
        {
            printPrefix = "YarpHelper::checkAndLoadFloat64: ";
            ok = ok && config.find(paramName).isFloat64();
        }
        else if constexpr (std::is_same<Scalar, bool>::value)
        {
            printPrefix = "YarpHelper::checkAndLoadBoolean: ";
            ok = ok && config.find(paramName).isBool();
        }
        else if constexpr (std::is_same<Scalar, std::string>::value)
        {
            printPrefix = "YarpHelper::checkAndLoadString: ";
            ok = ok && config.find(paramName).isString();
        }

        if (!ok)
        {
            yError() << printPrefix + paramName << " option not found or not valid";
            if (required)
            {
                return false;
            }
            param = defaultValue;
            yInfo() << printPrefix << "Using default value"
                    << defaultValue << " for parameter " << paramName << ".";
            return true;
        }

        if constexpr (std::is_same<Scalar, double>::value)
        {
            param = config.find(paramName).asFloat64();
        }
        else if constexpr (std::is_same<Scalar, bool>::value)
        {
            param = config.find(paramName).asBool();
        }
        else if constexpr (std::is_same<Scalar, std::string>::value)
        {
            param = config.find(paramName).asString();
        }

        return true;
    }


    template <typename Scalar>
    static bool checkAndLoadList(const yarp::os::Searchable& config,
                                 const std::string& paramName,
                                 std::vector<Scalar>& param,
                                 const std::vector<Scalar>& defaultValue,
                                 int checklistSize = -1,
                                 bool required = true)
    {
        std::string printPrefix{"YarpHelper::checkAndLoadList: "};
        bool ok = config.check(paramName) &&
                  config.find(paramName).isList();
        if (checklistSize > 0)
        {
            ok = ok &&
                 (config.find(paramName).asList()->size() == checklistSize);
        }

        if (!ok)
        {
            yError() << printPrefix + paramName << " option not found or not valid";
            if (required)
            {
                return false;
            }

            param = defaultValue;
            yInfo() << printPrefix << "Using default values"
                    << " for parameter " << paramName << ".";
            return true;
        }

        param.clear();
        auto listBottle = config.find(paramName).asList();

        for (std::size_t it = 0; it < listBottle->size(); it++)
        {
            if constexpr (std::is_same<Scalar, std::string>::value)
            {
                param.emplace_back(listBottle->get(it).asString());
            }
            else if constexpr (std::is_same<Scalar, double>::value)
            {
                param.emplace_back(listBottle->get(it).asFloat64());
            }
        }

        return true;
    }
};





} // IK
} // KinDynFusion
#endif // KINDYNFUSION_INVERSE_KINEMATICS_INVERSE_KINEMATICS_YARP_HELPER_H

