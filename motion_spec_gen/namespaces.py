from rdflib.namespace import DefinedNamespace, Namespace
from rdflib.term import URIRef


class Monitor(DefinedNamespace):

    Monitor: URIRef

    constraint: URIRef

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/monitors/monitor#")


class Controller(DefinedNamespace):

    Controller: URIRef

    constraint: URIRef
    signal: URIRef

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/controllers/controller#"
    )


class PIDController(DefinedNamespace):

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

    threshold: URIRef
    operator: URIRef
    coordinate: URIRef

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


class GEOM_ENT(DefinedNamespace):

    Point: URIRef
    Frame: URIRef

    origin: URIRef

    _NS = Namespace(
        "https://comp-rob2b.github.io/metamodels/geometry/structural-entities#"
    )


class SOLVER(DefinedNamespace):

    Solver: URIRef

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/solvers/solver#")


class ACHD_SOLVER(DefinedNamespace):

    VereshchaginSolver: URIRef

    _extras = [
        "root-acceleration",
        "alpha-constraints",
        "acceleration-energy",
        "external-wrench",
    ]

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/solvers/vereshchagin_solver#"
    )


class EMBED_MAP(DefinedNamespace):

    EmbeddingMap: URIRef

    controller: URIRef
    solver: URIRef
    input: URIRef
    vector: URIRef

    _extras = [
        "output-external-wrench",
        "output-acceleration-energy",
    ]

    _NS = Namespace(
        "https://roboticscosmos.github.io/metamodels/controllers/embedding-map#"
    )


class MOTION_SPEC(DefinedNamespace):

    MotionSpec: URIRef

    _extras = ["pre-conditions", "per-conditions", "post-conditions"]

    _NS = Namespace("https://roboticscosmos.github.io/metamodels/motion_specification#")
