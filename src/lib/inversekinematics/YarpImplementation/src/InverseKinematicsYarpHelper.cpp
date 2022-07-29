/**
 * @file InverseKinematicsYarpHelper.cpp
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynFusion/InverseKinematics/InverseKinematicsYarpHelper.h>
#include <iostream>

namespace KinDynFusion
{
namespace IK
{

const std::string LogPrefix = "InverseKinematicsYarpHelper : ";

bool YarpHelper::configure(const yarp::os::Searchable& config,
                           ModelData& modelData,
                           IKParameters& ikParams)
{
    std::string printPrefix{"YarpHelper::configure: "};
    // Load Model Data
    if (!IK::YarpHelper::loadModelData(config, modelData))
    {
        yError() << LogPrefix + printPrefix
                 << " Unable to load model data.";
        return false;
    }

    IK::YarpHelper::printModelConfigurationReport(modelData);

    // Load IK Parameters
    if (!IK::YarpHelper::loadIKParameters(config, ikParams))
    {
        yError() << LogPrefix + printPrefix
                 << " Unable to load IK parameters.";
        return false;
    }

    IK::YarpHelper::printIKConfigurationReport(ikParams);

    // Load Dynamical IK Parameters
    if (ikParams.ikSolver == IK::SolverIK::dynamical)
    {
        if (!IK::YarpHelper::loadDynamicalIKParameters(config, ikParams.dynIKParams))
        {
            yError() << LogPrefix + printPrefix
                     << " Unable to load dynamical IK parameters.";
            return false;
        }

        IK::YarpHelper::printDynamicalIKConfigurationReport(ikParams.dynIKParams);

        if (!IK::YarpHelper::loadCustomIKConstraintParameters(config,
                                                              modelData,
                                                              ikParams,
                                                              ikParams.customIKConstraints))
        {
            yError() << LogPrefix + printPrefix << " Unable to load custom IK constraint parameters.";
            return false;
        }

        IK::YarpHelper::printCustomIKConstraintsConfigurationReport(ikParams.customIKConstraints);
    }

    return true;
}

bool YarpHelper::loadModelData(const yarp::os::Searchable& config,
                               ModelData& modelData)
{
    std::string printPrefix{"YarpHelper::loadModelData: "};

    bool ok{true};
    // ==========
    // load URDF
    // ==========
    ok = ok && checkAndLoadScalar(config,
                                  std::string{"urdf"},
                                  modelData.urdfFileName,
                                  /*default=*/ std::string{"model.urdf"});
    // =================
    // load base frame
    // =================
    ok = ok && checkAndLoadScalar(config,
                                  std::string{"baseFrame"},
                                  modelData.baseFrame,
                                  /*default=*/ std::string{"Pelvis"});
    // ==================
    // load joints list
    // ==================
    std::vector<std::string> emptyList;
    emptyList.clear();
    ok = ok && checkAndLoadList(config,
                                "jointList",
                                modelData.jointList,
                                /*default=*/ emptyList,
                                /*checkListSize=*/ -1,
                                /*required=*/false); // -1 means there will be no size check
    if (!ok)
    {
        yError() << LogPrefix + printPrefix
                 << "Failed to read one or more model-related configuration parameters";
        return false;
    }

    // ==========================
    // INITIALIZE THE HUMAN MODEL
    // ==========================
    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(modelData.urdfFileName);
    if (urdfFilePath.empty())
    {
        yError() << LogPrefix + printPrefix << "Failed to find file"
                 << config.find("urdf").asString();
        return false;
    }

    iDynTree::ModelLoader modelLoader;
    if ( modelData.jointList.empty())
    {
        if (!modelLoader.loadModelFromFile(urdfFilePath) ||
            !modelLoader.isValid())
        {
            yError() << LogPrefix + printPrefix << "Failed to load model" << urdfFilePath;
            return false;
        }
    }
    else
    {
        if (!modelLoader.loadReducedModelFromFile(urdfFilePath, modelData.jointList) ||
            !modelLoader.isValid())
        {
            yError() << LogPrefix + printPrefix << "Failed to load model" << urdfFilePath;
            return false;
        }
    }

    yInfo() << LogPrefix + printPrefix << "----------------------------------------> Is Model valid?"
            << modelLoader.isValid();
    modelData.model = modelLoader.model();
    modelData.model.setDefaultBaseLink(modelData.model.getLinkIndex(modelData.baseFrame));

    yInfo() << LogPrefix + printPrefix << "base link: "
            << modelData.model.getLinkName(modelData.model.getDefaultBaseLink());
    return true;
}

void YarpHelper::printModelConfigurationReport(const ModelData& modelData)
{
    yInfo() << LogPrefix << "*** ===================================";
    yInfo() << LogPrefix << "*** =========== MODEL DATA ============";
    yInfo() << LogPrefix << "*** Urdf file name                    :" << modelData.urdfFileName;
    yInfo() << LogPrefix << "*** Base frame                        :" << modelData.baseFrame;
    yInfo() << LogPrefix << "*** Nr. of  DoFs                      :" << modelData.model.getNrOfDOFs();

    yInfo() << LogPrefix << modelData.model.toString();
    yInfo() << LogPrefix << "Number of links: " << modelData.model.getNrOfLinks()
            << " , joints: " << modelData.model.getNrOfJoints();
    yInfo() << LogPrefix << "*** ===================================";
}

bool YarpHelper::loadIKParameters(const yarp::os::Searchable& config,
                                  IKParameters& ikParams)
{
    std::string printPrefix{"YarpHelper::loadIKParameters: "};
    bool ok{true};

    std::string solverName;
    ok = ok && checkAndLoadScalar(config,
                                  std::string{"ikSolver"},
                                  solverName,
                                  /*default=*/ std::string{"dynamical"});

    ok = ok && checkAndLoadScalar(config,
                                  "useFixedBase",
                                  ikParams.useFixedBase,
                                  /*default=*/ false);

    if (!ok)
    {
        yError() << LogPrefix + printPrefix
                 << "Failed to read one or more IK configuration parameters";
        return false;
    }

    // check for validity against available solvers
    ikParams.ikSolver = ikParams.getSolverType(solverName);
    if (ikParams.ikSolver == SolverIK::invalid)
    {
        yError() << LogPrefix + printPrefix << "ikSolver " << solverName << " not found";
        return false;
    }

    return true;
}

void YarpHelper::printIKConfigurationReport(const IKParameters& ikParams)
{
    yInfo() << LogPrefix << "*** ===================================";
    yInfo() << LogPrefix << "*** ==========  IK Parameters =========";
    yInfo() << LogPrefix << "*** Inverse Kinematics solver         :" << ikParams.getSolverString(ikParams.ikSolver);
    yInfo() << LogPrefix << "*** Using fixed base configuration    :" << ikParams.useFixedBase;
    yInfo() << LogPrefix << "*** ===================================";
}

bool YarpHelper::loadDynamicalIKParameters(const yarp::os::Searchable& config,
                                           DynamicalIKParameters& dynIKParams)
{
    std::string printPrefix{"YarpHelper::loadDynamicalIKParameters: "};

    bool ok{true};
    ok = ok && checkAndLoadScalar(config,
                                  std::string{"inverseVelocityKinematicsSolver"},
                                  dynIKParams.inverseVelocityKinematicsSolver,
                                  /*default=*/ std::string{"moorePenrose"},
                                  /*required=*/ false);
    ok = ok && checkAndLoadScalar(config,
                                  "useDirectBaseMeasurement",
                                  dynIKParams.useDirectBaseMeasurement,
                                  /*default=*/ false);
    ok = ok && checkAndLoadScalar(config,
                                  "costRegularization",
                                  dynIKParams.costRegularization,
                                  /*default=*/ 1.0);
    ok = ok && checkAndLoadScalar(config,
                                  "linVelTargetWeight",
                                  dynIKParams.linVelTargetWeight,
                                  /*default=*/ 1.0);
    ok = ok && checkAndLoadScalar(config,
                                  "angVelTargetWeight",
                                  dynIKParams.angVelTargetWeight,
                                  /*default=*/ 1.0);
    ok = ok && checkAndLoadScalar(config,
                                  "dynamicalIKJointVelocityLimit",
                                  dynIKParams.jointVelocityLimit,
                                  /*defaultValue=*/ 1000.0,
                                  /*required=*/ false); // if no limits given
                                                        // for a joint we put
                                                        // 1000.0 rad/sec,
                                                        // which is very high
    std::vector<double> measuredVelocityGains;
    ok = ok && checkAndLoadList(config,
                                "dynamicalIKMeasuredVelocityGainLinRot",
                                measuredVelocityGains,
                                /*default=*/ std::vector<double>{1.0, 1.0},
                                /*checkListSize=*/ 2);
    std::vector<double> correctionVelocityGains;
    ok = ok && checkAndLoadList(config,
                                "dynamicalIKCorrectionGainsLinRot",
                                correctionVelocityGains,
                                /*default=*/ std::vector<double>{1.0, 1.0},
                                /*checkListSize=*/ 2);


    ok = ok && checkAndLoadScalar(config,
                                  "k_u",
                                  dynIKParams.k_u,
                                  /*default=*/ 0.);
    ok = ok && checkAndLoadScalar(config,
                                  "k_l",
                                  dynIKParams.k_l,
                                  /*default=*/ 0.);
    if (!ok)
    {
        yError() << LogPrefix + printPrefix
                 << "Failed to read one or more dynamical IK configuration parameters";
        return false;
    }

    dynIKParams.measuredLinearVelocityGain = measuredVelocityGains[0];
    dynIKParams.measuredAngularVelocityGain = measuredVelocityGains[1];
    dynIKParams.linearCorrectionGain = correctionVelocityGains[0];
    dynIKParams.angularCorrectionGain = correctionVelocityGains[1];

    return true;
}

void YarpHelper::printDynamicalIKConfigurationReport(const DynamicalIKParameters& dynIKParams)
{
    yInfo() << LogPrefix << "*** ===================================";
    yInfo() << LogPrefix << "*** ===== Dynamical IK Parameters =====";
    yInfo() << LogPrefix << "*** Inverse Velocity Kinematics solver:" << dynIKParams.inverseVelocityKinematicsSolver;
    yInfo() << LogPrefix << "*** Using direct base measurement     :" << dynIKParams.useDirectBaseMeasurement;
    yInfo() << LogPrefix << "*** Cost regularization               :" << dynIKParams.costRegularization;
    yInfo() << LogPrefix << "*** Linear velocity target weight     :" << dynIKParams.linVelTargetWeight;
    yInfo() << LogPrefix << "*** Angular velocity target weight    :" << dynIKParams.angVelTargetWeight;
    yInfo() << LogPrefix << "*** Measured linear velocity Gain     :" << dynIKParams.measuredLinearVelocityGain;
    yInfo() << LogPrefix << "*** Measured angular velocity gain    :" << dynIKParams.measuredAngularVelocityGain;
    yInfo() << LogPrefix << "*** Linear velocity correction gain   :" << dynIKParams.linearCorrectionGain;
    yInfo() << LogPrefix << "*** Angular velocity correction gain  :" << dynIKParams.angularCorrectionGain;
    yInfo() << LogPrefix << "*** Joint velocity limit              :" << dynIKParams.jointVelocityLimit;
    yInfo() << LogPrefix << "*** k_u                               :" << dynIKParams.k_u;
    yInfo() << LogPrefix << "*** k_l                               :" << dynIKParams.k_l;
    yInfo() << LogPrefix << "*** ===================================";
}


bool YarpHelper::loadCustomIKConstraintParameters(const yarp::os::Searchable& config,
                                                  const ModelData& modelData,
                                                  IKParameters& ikParams,
                                                  CustomIKConstraints& custom)
{
    std::string printPrefix{"YarpHelper::loadCustomIKConstraintParameters: "};

    custom.constraintMatrix.resize(0, 0);
    custom.constraintVariables.resize(0);
    custom.constraintLowerBound.resize(0);
    custom.constraintUpperBound.resize(0);
    custom.constraintVariablesIndex.resize(0);
    custom.jointsVelocityLimitsNames.resize(0);
    custom.jointsVelocityLimitsValues.resize(0);
    custom.jointsVelocityLimitsIndexes.resize(0);

    custom.baseVelocityLowerLimit.resize(6);
    custom.baseVelocityUpperLimit.resize(6);
    for (std::size_t i = 0; i < custom.baseVelocityUpperLimit.size(); i++)
    {
        // setting default velocity limit values to
        // very high values of 1000 rad/s
        custom.baseVelocityUpperLimit.setVal(i, 1000.0);
        custom.baseVelocityLowerLimit.setVal(i, -1000.0);
    }

    if (ikParams.ikSolver != SolverIK::dynamical ||
        ikParams.dynIKParams.inverseVelocityKinematicsSolver != "QP")
    {
        yWarning() << LogPrefix + printPrefix
                   << "'CUSTOM_CONSTRAINTS' group option is available only if "
                      "'ikSolver==dynamical' & 'inverseVelocityKinematicsSolver==QP'. \n "
                      "Currently, you are NOT using the customized constraint group.";
        ikParams.customConstraintsLoaded = false;
        return true;
    }

    if (config.check("CUSTOM_CONSTRAINTS"))
    {
        yarp::os::Bottle& constraintGroup = config.findGroup("CUSTOM_CONSTRAINTS");
        if (constraintGroup.isNull())
        {
            yError() << LogPrefix + printPrefix << "Failed to find group CUSTOM_CONSTRAINTS";
            ikParams.customConstraintsLoaded = false;
            return false;
        }

        yInfo() << LogPrefix + printPrefix << " Number of custom IK constraint groups to read: "
                << constraintGroup.size() - 1;

        // loop counter starts from 1,
        // since the first value of bottle name is group name.
        for (std::size_t i = 1; i < constraintGroup.size(); i++)
        {
            if (!(constraintGroup.get(i).isList() &&
                  constraintGroup.get(i).asList()->size() == 2))
            {
                yError() << LogPrefix + printPrefix
                         << "Child groups of CUSTOM_CONSTRAINTS must be lists of two elements";
                ikParams.customConstraintsLoaded = false;
                return false;
            }

            yarp::os::Bottle* constraintList = constraintGroup.get(i).asList();
            std::string constraintKey = constraintList->get(0).asString();
            yarp::os::Bottle* constraintListContent = constraintList->get(1).asList();
            yInfo() << "Parsing custom IK constraints group# " << i << "with key " << constraintKey;

            if (constraintKey == "custom_joints_velocity_limits_names")
            {
                for (std::size_t i = 0; i < constraintListContent->size(); i++)
                {
                    custom.jointsVelocityLimitsNames.emplace_back(constraintListContent->get(i).asString());
                }

            }
            else if (constraintKey == "custom_joints_velocity_limits_values")
            {
                custom.jointsVelocityLimitsValues.resize(constraintListContent->size());
                for (std::size_t i = 0; i < constraintListContent->size(); i++)
                {
                    custom.jointsVelocityLimitsValues.setVal(i, constraintListContent->get(i).asFloat64());
                }
            }
            else if (constraintKey == "custom_constraint_variables")
            {
                for (std::size_t i = 0; i < constraintListContent->size(); i++)
                {
                    custom.constraintVariables.push_back(
                        constraintListContent->get(i).asString());
                }
            }
            else if (constraintKey == "custom_constraint_matrix")
            {
                for (std::size_t i = 0; i < constraintListContent->size(); i++)
                {
                    yarp::os::Bottle* innerLoop = constraintListContent->get(i).asList();
                    if (i == 0)
                    {
                        custom.constraintMatrix.resize(constraintListContent->size(),
                                                       innerLoop->size());
                    }
                    for (std::size_t j = 0; j < innerLoop->size(); j++)
                    {
                        custom.constraintMatrix.setVal(i, j, innerLoop->get(j).asFloat64());
                    }
                }
            }
            else if (constraintKey == "custom_constraint_upper_bound")
            {
                custom.constraintUpperBound.resize(constraintListContent->size());
                for (std::size_t i = 0; i < constraintListContent->size(); i++)
                {
                    custom.constraintUpperBound.setVal(
                        i, constraintListContent->get(i).asFloat64());
                }
            }
            else if (constraintKey == "custom_constraint_lower_bound")
            {
                custom.constraintLowerBound.resize(constraintListContent->size());
                for (std::size_t i = 0; i < constraintListContent->size(); i++)
                {
                    custom.constraintLowerBound.setVal(
                        i, constraintListContent->get(i).asFloat64());
                }
            }
            else if (constraintKey == "base_velocity_limit_upper_bound")
            {
                if (constraintListContent->size() != 6)
                {
                    yError() << LogPrefix + printPrefix
                             << " base velocity limit should have size of 6.";
                    return false;
                }

                for (std::size_t i = 0; i < constraintListContent->size(); i++)
                {
                    custom.baseVelocityUpperLimit.setVal(i, constraintListContent->get(i).asFloat64());
                }
            }
            else if (constraintKey == "base_velocity_limit_lower_bound")
            {
                if (constraintListContent->size() != 6)
                {
                    yError() << LogPrefix + printPrefix
                             << " base velocity limit should have size of 6.";
                    ikParams.customConstraintsLoaded = false;
                    return false;
                }

                for (std::size_t i = 0; i < constraintListContent->size(); i++)
                {
                    custom.baseVelocityLowerLimit.setVal(i, constraintListContent->get(i).asFloat64());
                }
            }
            else
            {
                yError() << LogPrefix + printPrefix
                         << constraintKey << " parameter key is not defined";
                ikParams.customConstraintsLoaded = false;
                return false;
            }
        }
        ikParams.customConstraintsLoaded = true;
    }
    else
    {
        yInfo() << LogPrefix + printPrefix
                << "CUSTOM_CONSTRAINTS group is not defined in the configuration file.";
        ikParams.customConstraintsLoaded = false;
    }

    // set base velocity constraint to zero if the base is fixed
    if (ikParams.useFixedBase)
    {
        custom.baseVelocityLowerLimit.zero();
        custom.baseVelocityUpperLimit.zero();

        yInfo() << LogPrefix + printPrefix
                << "Using fixed base model, base velocity limits are set to zero";
    }

    if (ikParams.customConstraintsLoaded)
    {
        // check sizes
        if (custom.jointsVelocityLimitsNames.size() != custom.jointsVelocityLimitsValues.size())
        {
            yError() << LogPrefix  + printPrefix
                    << " joint velocity limits name and value size are not equal";
            return false;
        }

        if ((custom.constraintUpperBound.size() != custom.constraintLowerBound.size()) &&
            (custom.constraintLowerBound.size() != custom.constraintMatrix.rows()))
        {
            yError() << LogPrefix  + printPrefix
                    << "the number of lower bound (" << custom.constraintLowerBound.size()
                    << "), upper bound(" << custom.constraintUpperBound.size()
                    << "), and cosntraint matrix rows(" << custom.constraintMatrix.rows()
                    << ") are not equal";

            return false;
        }

        if ((custom.constraintVariables.size() != custom.constraintMatrix.cols()))
        {
            yError() << LogPrefix  + printPrefix
                    << "the number of constraint variables ("
                    << custom.constraintVariables.size() << "), and cosntraint matrix columns ("
                    << custom.constraintMatrix.cols() << ") are not equal";
            return false;
        }

        for (std::size_t i = 0; i < custom.jointsVelocityLimitsNames.size(); i++)
        {
            auto jIdx = modelData.model.getJointIndex(custom.jointsVelocityLimitsNames[i]);
            if (jIdx == iDynTree::FRAME_INVALID_INDEX)
            {
                yError() << LogPrefix  + printPrefix
                        << "Joint " << custom.jointsVelocityLimitsNames[i]
                        << "not available in the model";
                return false;
            }
            custom.jointsVelocityLimitsIndexes.push_back(jIdx);
        }

        for (std::size_t i = 0; i < custom.constraintVariables.size(); i++)
        {
            auto jIdx = modelData.model.getJointIndex(custom.constraintVariables[i]);
            if (jIdx == iDynTree::FRAME_INVALID_INDEX)
            {
                yError() << LogPrefix  + printPrefix
                        << "Joint " << custom.constraintVariables[i]
                        << "not available in the model";
                return false;
            }

            custom.constraintVariablesIndex.push_back(jIdx);
        }
    }

    return true;
}

void YarpHelper::printCustomIKConstraintsConfigurationReport(const CustomIKConstraints& custom)
{
    yInfo() << LogPrefix << "*** ===================================";
    yInfo() << LogPrefix << "*** = Custom IK constraint parameters =";
    yInfo() << LogPrefix << "*** Custom joint velocity limits::" ;
    for (std::size_t i = 0; i < custom.jointsVelocityLimitsNames.size(); i++)
    {
        std::cout << "Custom joint velocity limit for "
                  << custom.jointsVelocityLimitsNames[i] << ": "
                  << custom.jointsVelocityLimitsValues.getVal(i) << "rad/s."<< std::endl;
    }

    yInfo() << LogPrefix << "*** Custom constraint variables::";
    for (std::size_t i = 0; i < custom.constraintVariables.size(); i++)
    {
        std::cout << "Custom constraint variable:  "
                  << custom.constraintVariables[i] << ", "
                  << "index: " << custom.constraintVariablesIndex[i] << std::endl;
    }

    yInfo() << LogPrefix << "*** Custom constraint matrix::";
    for (std::size_t i = 0; i < custom.constraintMatrix.rows(); i++)
    {
        for (std::size_t j = 0; j < custom.constraintMatrix.cols(); j++)
        {
            std::cout << custom.constraintMatrix.getVal(i, j) << " ";
        }
        std::cout << std::endl;
    }

    yInfo() << LogPrefix << "*** Custom constraint upper bound::";
    for (std::size_t i = 0; i < custom.constraintUpperBound.size(); i++)
    {
        std::cout << custom.constraintUpperBound.getVal(i)<< std::endl;
    }

    yInfo() << LogPrefix << "*** Custom constraint lower bound::";
    for (std::size_t i = 0; i < custom.constraintLowerBound.size(); i++)
    {
        std::cout << custom.constraintLowerBound.getVal(i)<< std::endl;
    }

    yInfo() << LogPrefix << "*** Custom base velocity upper bound::";
    for (std::size_t i = 0; i < custom.baseVelocityUpperLimit.size(); i++)
    {
        std::cout << custom.baseVelocityUpperLimit.getVal(i)<< std::endl;
    }

    yInfo() << LogPrefix << "*** Custom base velocity lower bound::";
    for (std::size_t i = 0; i < custom.baseVelocityLowerLimit.size(); i++)
    {
        std::cout << custom.baseVelocityLowerLimit.getVal(i)<< std::endl;
    }

    yInfo() << LogPrefix << "*** ===================================";
}




} // namespace IK
} // namespace KinDynFusion


