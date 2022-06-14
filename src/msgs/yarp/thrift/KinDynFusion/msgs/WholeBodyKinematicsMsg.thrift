namespace yarp KinDynFusion.msgs

/**
 * Representation of a 3D vector
 */
struct Vector3 {
    1: double x;
    2: double y;
    3: double z;
}

/**
 * Representation of a Quaternion
 */
struct Quaternion {
    1: double w;
    2: Vector3 imaginary;
}

/**
  *
  */
struct ContactNormalForce {
    1: string name;
    2: Vector3 positionWRTLink;
    3: bool isActive;
    4: double magnitude;
    5: string linkName;
}

struct EstimatedContactPosition {
    1: string name;
    2: Vector3 positionWRTGlobal;
}

/**
 * Representation of the IWholeBodyKinematics interface
 */
struct WholeBodyKinematicsOutput {
    1: list<string> jointNames;
    2: list<double> positions;
    3: list<double> velocities;

    4: string baseName;
    5: Vector3 baseOriginWRTGlobal;
    6: Quaternion baseOrientationWRTGlobal;
    7: list<double> baseVelocityWRTGlobal;
    8: Vector3 lFootCoPPositionWRTLink;
    9: Vector3 rFootCoPPositionWRTLink;

    10: list<ContactNormalForce> vertexContacts;
    11: list<string> vertexContactNames;

    12: string lFootLinkName;
    13: string rFootLinkName;

    14: Quaternion lfOrientationWRTGlobal;
    15: Quaternion rfOrientationWRTGlobal;
    16: list<EstimatedContactPosition> contactPositionWRTGlobal;
    17: Vector3 CoPPositionWRTGlobal;
}

