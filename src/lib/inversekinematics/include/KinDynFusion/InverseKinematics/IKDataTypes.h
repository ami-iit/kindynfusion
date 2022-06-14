/**
 * @file IKDataTypes.h
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNFUSION_INVERSE_KINEMATICS_IK_DATA_TYPES_H
#define KINDYNFUSION_INVERSE_KINEMATICS_IK_DATA_TYPES_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Span.h>

#include <unordered_map>
#include <vector>
#include <string>

namespace KinDynFusion
{
namespace IK
{

struct ModelData
{
    std::string baseFrame;
    std::vector<std::string> jointList;
    iDynTree::Model model;
    std::string urdfFileName;
};

struct CustomIKConstraints
{
public:
    std::vector<std::string> jointsVelocityLimitsNames;
    std::vector<iDynTree::JointIndex> jointsVelocityLimitsIndexes;
    iDynTree::VectorDynSize jointsVelocityLimitsValues;
    // Custom Constraint Form: lowerBound<=A*X<=upperBuond
    iDynTree::MatrixDynSize
        constraintMatrix; // A, CxN matrix; C: number of Constraints, N: number of
                          // system states: Dofs+6 in floating-based robot
    std::vector<std::string> constraintVariables; // X, Nx1  Vector : variables names
    std::vector<iDynTree::JointIndex>
        constraintVariablesIndex; // X, Nx1  Vector : variables index
    iDynTree::VectorDynSize constraintUpperBound; // upperBound, Cx1 Vector
    iDynTree::VectorDynSize constraintLowerBound; // lowerBound, Cx1 Vector
    iDynTree::VectorDynSize baseVelocityUpperLimit;
    iDynTree::VectorDynSize baseVelocityLowerLimit;
};

enum SolverIK
{
    dynamical,
    invalid
};

struct DynamicalIKParameters
{
    std::string inverseVelocityKinematicsSolver{"moorePenrose"};
    double costRegularization{1.0};
    bool useDirectBaseMeasurement{false};
    double linVelTargetWeight{1.0};
    double angVelTargetWeight{1.0};
    double measuredLinearVelocityGain{1.0};
    double measuredAngularVelocityGain{1.0};
    double linearCorrectionGain{1.0};
    double angularCorrectionGain{1.0};
    double jointVelocityLimit{1000.0};
    double k_u{0.5}, k_l{0.5};
};

struct IKParameters
{
    // IK parameters
    SolverIK ikSolver{SolverIK::dynamical};
    bool useFixedBase{false};
    bool customConstraintsLoaded{false};
    DynamicalIKParameters dynIKParams;
    CustomIKConstraints customIKConstraints;

    // Helper methods
    std::string getSolverString(const SolverIK& solver) const
    {
        if (stringMap.find(solver) == stringMap.end())
        {
            return " ";
        }

        return stringMap.at(solver);
    }

    SolverIK getSolverType(const std::string& str) const
    {
        if (solverMap.find(str) == solverMap.end())
        {
            return SolverIK::invalid;
        }

        return solverMap.at(str);
    }

private:
    using str2solver = std::unordered_map<std::string, SolverIK> ;
    using solver2str = std::unordered_map<SolverIK, std::string> ;
    str2solver solverMap{{"dynamical", SolverIK::dynamical} };
    solver2str stringMap{{SolverIK::dynamical, "dynamical"},
                         {SolverIK::invalid, "invalid"}};
};


} // namespace IK
} // namespace KinDynFusion

#endif // KINDYNFUSION_INVERSE_KINEMATICS_IK_DATA_TYPES_H
