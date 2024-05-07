from rdflib.namespace import DefinedNamespace, Namespace
from rdflib.term import URIRef


class QUDT(DefinedNamespace):

    unit: URIRef
    value: URIRef
    hasQuantityKind: URIRef

    _NS = Namespace("http://qudt.org/schema/qudt/")

class MONITOR(DefinedNamespace):

    Monitor: URIRef

    constraint: URIRef

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/monitors/monitor#")


class CONTROLLER(DefinedNamespace):

    Controller: URIRef

    constraint: URIRef
    signal: URIRef

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/controllers/controller#"
    )


class PID_CONTROLLER(DefinedNamespace):

    PIDController: URIRef

    _extras = [
        "p-gain",
        "i-gain",
        "d-gain",
        "time-step",
    ]

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/controllers/pid_controller#"
    )


class IMPEDANCE_CONTROLLER(DefinedNamespace):

    ImpedanceController: URIRef

    stiffness: URIRef
    damping: URIRef

    _extras = [
        "position-constraint",
        "velocity-constraint",
    ]

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/controllers/impedance_controller#"
    )


class THRESHOLD(DefinedNamespace):

    Threshold: URIRef

    _extras = [
        "reference-value",
        "threshold",
        "upper-threshold",
        "lower-threshold"
    ]

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/thresholds#")


class CONSTRAINT(DefinedNamespace):

    Constraint: URIRef
    DistanceConstraint: URIRef
    VelocityConstraint: URIRef
    VelocityTwistConstraint: URIRef
    AccelerationConstraint: URIRef
    ForceConstraint: URIRef

    LessThan: URIRef
    LessThanOrEqual: URIRef
    GreaterThan: URIRef
    GreaterThanOrEqual: URIRef
    Equal: URIRef
    NotEqual: URIRef
    InInterval: URIRef

    operator: URIRef

    quantity: URIRef

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/relations/constraints#"
    )


class GEOM_COORD(DefinedNamespace):

    Coordinates: URIRef

    PositionCoordinate: URIRef
    OrientationCoordinate: URIRef
    PoseCoordinate: URIRef

    DistanceCoordinate: URIRef

    VelocityTwistCoordinate: URIRef

    LinearVelocityVectorX: URIRef
    LinearVelocityVectorY: URIRef
    LinearVelocityVectorZ: URIRef
    LinearVelocityVectorXY: URIRef
    LinearVelocityVectorXZ: URIRef
    LinearVelocityVectorYZ: URIRef
    LinearVelocityVectorXYZ: URIRef

    AngularVelocityVectorX: URIRef
    AngularVelocityVectorY: URIRef
    AngularVelocityVectorZ: URIRef
    AngularVelocityVectorXY: URIRef
    AngularVelocityVectorXZ: URIRef
    AngularVelocityVectorYZ: URIRef
    AngularVelocityVectorXYZ: URIRef

    AccelerationTwistCoordinate: URIRef
    AngularAccelerationVector: URIRef
    LinearAccelerationVector: URIRef

    VectorXYZ: URIRef

    of: URIRef

    _extras = [
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration",
        "as-seen-by",
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/coordinates#")


class GEOM_REL(DefinedNamespace):
    Pose: URIRef

    EuclideanDistance: URIRef
    PointToPointDistance: URIRef
    LinearDistance: URIRef
    AngularDistance: URIRef

    VelocityTwist: URIRef

    _extras = [
        "of-entity",
        "with-respect-to",
        "between-entities",
        "reference-point",
    ]

    _NS = Namespace(
        "https://comp-rob2b.github.io/metamodels/geometry/spatial-relations#"
    )


class NEWTONIAN_RBD_COORD(DefinedNamespace):

    ForceReference: URIRef
    TorqueReference: URIRef
    WrenchReference: URIRef

    ForceCoordinate: URIRef
    TorqueCoordinate: URIRef
    WrenchCoordinate: URIRef

    ForceVectorX: URIRef
    ForceVectorY: URIRef
    ForceVectorZ: URIRef

    of: URIRef

    _extras = [
        "as-seen-by",
    ]

    _NS = Namespace(
        "https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/coordinates#"
    )


class NEWTONIAN_RBD_REL(DefinedNamespace):

    Force: URIRef
    ContactForce: URIRef
    Torque: URIRef
    Wrench: URIRef

    Stiffness: URIRef
    LinearStiffness: URIRef
    Damping: URIRef
    LinearDamping: URIRef

    _extras = [
        "applied-to",
        "applied-by",
        "stiffness-diagonal-matrix",
        "damping-diagonal-matrix",
    ]

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/newtonian-rigid-body-dynamics/spatial-relations#"
    )


class GEOM_ENT(DefinedNamespace):

    Point: URIRef
    Frame: URIRef

    origin: URIRef

    _NS = Namespace(
        "https://comp-rob2b.github.io/metamodels/geometry/structural-entities#"
    )


class SOLVER(DefinedNamespace):

    Solver: URIRef

    _extras = [
        "root-acceleration",
    ]

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/solvers/solver#")


class ACHD_SOLVER(DefinedNamespace):

    ACHDSolver: URIRef
    ACHDSolverFext: URIRef

    _extras = [
        "alpha-constraints",
        "acceleration-energy",
        "external-wrench",

        "root-link",
        "tip-link"
    ]

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/solvers/vereshchagin_solver#"
    )


class BASE_FD_SOLVER(DefinedNamespace):

    BaseFDSolver: URIRef

    wrench: URIRef

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/solvers/base_fd_solver#"
    )


class EMBED_MAP(DefinedNamespace):

    EmbeddingMap: URIRef
    DirectionVector: URIRef

    controller: URIRef
    solver: URIRef
    input: URIRef
    vector: URIRef

    _extras = [
        "output-external-wrench",
        "output-acceleration-energy",
        "stiffness-vector",
        "damping-vector",
        "vector-direction-from",
        "vector-direction-to",
        "vector-direction-asb"
    ]

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/controllers/embedding-map#"
    )


class MOTION_SPEC(DefinedNamespace):

    MotionSpec: URIRef

    _extras = ["pre-conditions", "per-conditions", "post-conditions"]

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/motion_specification#")


class ROBOTS(DefinedNamespace):

    Robot: URIRef
    Manipulator: URIRef
    MobileBase: URIRef
    
    solvers: URIRef

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/robots#")