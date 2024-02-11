from rdflib.namespace import DefinedNamespace, Namespace
from rdflib.term import URIRef


class PIDController(DefinedNamespace):

    PIDController: URIRef

    constraint: URIRef

    output: URIRef

    _extras = [
        "p-gain",
        "i-gain",
        "d-gain",
        "time-step",
    ]

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/controllers/pid_controller#")


class THRESHOLD(DefinedNamespace):

    Threshold: URIRef

    _extras = [
        "threshold-value",
    ]

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/thresholds#")


class CONSTRAINT(DefinedNamespace):

    Constraint: URIRef
    DistanceConstraint: URIRef
    VelocityConstraint: URIRef
    ForceConstraint: URIRef

    LessThan: URIRef
    LessThanOrEqual: URIRef
    GreaterThan: URIRef
    GreaterThanOrEqual: URIRef
    Equal: URIRef
    NotEqual: URIRef
    InInterval: URIRef

    threshold: URIRef
    operator: URIRef
    coordinate: URIRef

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/relations/constraints#")


class GEOM_COORD(DefinedNamespace):

    Coordinates: URIRef

    PositionCoordinate: URIRef
    OrientationCoordinate: URIRef
    PoseCoordinate: URIRef

    VelocityTwistCoordinate: URIRef
    AngularVelocityVector: URIRef
    LinearVelocityVector: URIRef

    AccelerationTwistCoordinate: URIRef
    AngularAccelerationVector: URIRef
    LinearAccelerationVector: URIRef

    VectorX: URIRef
    VectorY: URIRef
    VectorZ: URIRef

    VectorXY: URIRef
    VectorXZ: URIRef
    VectorYZ: URIRef

    VectorXYZ: URIRef

    of: URIRef

    _extras = [
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration",
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/coordinates#")


class ACHD_SOLVER(DefinedNamespace):

    VereshchaginSolver: URIRef

    AccelerationEnergy: URIRef
    ExternalWrench: URIRef

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/solvers/vereshchagin_solver#"
    )


class CONTROLLER_OUTPUT(DefinedNamespace):

    ControllerOutput: URIRef

    AccelerationEnergy: URIRef
    ExternalWrench: URIRef

    Linear: URIRef
    Angular: URIRef

    vector: URIRef

    _extras = ["6D"]

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/controllers/controller_output#"
    )


class MOTION_SPEC(DefinedNamespace):

    MotionSpec: URIRef

    _extras = [
        "pre-conditions",
        "per-conditions",
        "post-conditions"
    ]

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/motion_specification#")