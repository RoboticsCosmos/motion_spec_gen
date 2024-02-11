import uuid
import rdflib
from rdflib.collection import Collection
from motion_spec_gen.namespaces import (
    PIDController,
    THRESHOLD,
    CONSTRAINT,
    GEOM_COORD,
    ACHD_SOLVER,
    CONTROLLER_OUTPUT,
)

vector_type_to_num = {
    GEOM_COORD.VectorX: [1, 0, 0],
    GEOM_COORD.VectorY: [0, 1, 0],
    GEOM_COORD.VectorZ: [0, 0, 1],
    GEOM_COORD.VectorXY: [1, 1, 0],
    GEOM_COORD.VectorXZ: [1, 0, 1],
    GEOM_COORD.VectorYZ: [0, 1, 1],
    GEOM_COORD.VectorXYZ: [1, 1, 1],
}


class PIDControllerStep:

    def emit(self, g: rdflib.Graph, node) -> dict:
        """
        Add an output node to the graph for pid controller
        """

        constraint = g.value(node, PIDController.constraint)

        coordinate = g.value(constraint, CONSTRAINT.coordinate)

        output_data = {}

        # chceck if coordinate is of type GEOM_COORD
        if coordinate.startswith(GEOM_COORD):
            # type = AccelerationEnergy
            output_data["interface_type"] = CONTROLLER_OUTPUT.AccelerationEnergy

        else:
            # type = ExternalWrench
            output_data["interface_type"] = CONTROLLER_OUTPUT.ExternalWrench

        for vector_type, vector in vector_type_to_num.items():
            if (coordinate, rdflib.RDF.type, vector_type) in g:
                output_data["vector"] = vector
                break

        # check linear or angular
        for type in [
            GEOM_COORD.LinearVelocityVector,
            GEOM_COORD.LinearAccelerationVector,
            GEOM_COORD.PositionCoordinate,
        ]:
            if (coordinate, rdflib.RDF.type, type) in g:
                output_data["coord_type"] = CONTROLLER_OUTPUT.Linear
                break

        for type in [
            GEOM_COORD.AngularVelocityVector,
            GEOM_COORD.AngularAccelerationVector,
            GEOM_COORD.OrientationCoordinate,
        ]:
            if (coordinate, rdflib.RDF.type, type) in g:
                output_data["coord_type"] = CONTROLLER_OUTPUT.Angular
                break

        # Assumption: when specified as Pose, Twist, or Wrench alone,
        # the vector is assumed to be in 6D
        for type in [
            GEOM_COORD.PoseCoordinate,
            GEOM_COORD.VelocityTwistCoordinate,
            GEOM_COORD.AccelerationTwistCoordinate,
        ]:
            if (coordinate, rdflib.RDF.type, type) in g:
                output_data["vector"] = [1, 1, 1, 1, 1, 1]
                output_data["coord_type"] = CONTROLLER_OUTPUT["6D"]
                break

        # add the output node to the graph
        id_ = rdflib.URIRef(uuid.uuid4().urn)
        g.add((id_, rdflib.RDF.type, CONTROLLER_OUTPUT.ControllerOutput))
        g.add((id_, rdflib.RDF.type, output_data["coord_type"]))
        g.add((id_, rdflib.RDF.type, output_data["interface_type"]))
        # add vector as a collection
        vector_collection = rdflib.BNode()
        l = [rdflib.Literal(i) for i in output_data["vector"]]
        Collection(g, vector_collection, l)
        g.add((id_, CONTROLLER_OUTPUT.vector, vector_collection))

        # add the output node to the controller node
        g.add((node, PIDController.output, id_))
