import uuid
import rdflib
from rdflib.collection import Collection
from motion_spec_gen.namespaces import (
    PIDController,
    THRESHOLD,
    CONSTRAINT,
    GEOM_COORD,
    ACHD_SOLVER,
    EMBED_MAP,
)

vector_components = {
    "X": (1, 0, 0),
    "Y": (0, 1, 0),
    "Z": (0, 0, 1),
    "XY": (1, 1, 0),
    "XZ": (1, 0, 1),
    "YZ": (0, 1, 1),
    "XYZ": (1, 1, 1),
}


linear_vector_type_to_num = {
    **{
        getattr(GEOM_COORD, f"LinearVelocityVector{suffix}"): vector
        for suffix, vector in vector_components.items()
    }
}

angular_vector_type_to_num = {
    **{
        getattr(GEOM_COORD, f"AngularVelocityVector{suffix}"): vector
        for suffix, vector in vector_components.items()
    }
}


class PIDControllerStep:

    def emit(self, g: rdflib.Graph, node: rdflib.URIRef, **kwargs) -> dict:
        """
        Add an output node to the graph for pid controller
        """

        constraint = g.value(node, PIDController.constraint)

        coordinate = g.value(constraint, CONSTRAINT.coordinate)

        output_data = {}

        qname = g.compute_qname(node)
        prefix = qname[1]
        name = qname[2]

        # get achd_solver from kwargs
        achd_solver = kwargs["achd_solver"]

        # TOOD: check for a better way
        is_geom_coord = (
            g[coordinate : rdflib.RDF.type : GEOM_COORD.PositionCoordinate]
            or g[coordinate : rdflib.RDF.type : GEOM_COORD.DistanceCoordinate]
            or g[coordinate : rdflib.RDF.type : GEOM_COORD.VelocityTwistCoordinate]
            or g[coordinate : rdflib.RDF.type : GEOM_COORD.AccelerationTwistCoordinate]
        )

        # *Assumption: one controller signal can be mapped to only one type of output
        if is_geom_coord:
            # type = AccelerationEnergy
            output_data["output"] = {
                "type": "output-acceleration-energy",
                "var_name": rdflib.URIRef(f"{prefix}{name}_output_acceleration_energy"),
            }

            g.add(
                (
                    achd_solver,
                    ACHD_SOLVER["acceleration-energy"],
                    output_data["output"]["var_name"],
                )
            )

        else:
            # type = ExternalWrench
            output_data["output"] = {
                "type": "output-external-wrench",
                "var_name": rdflib.URIRef(f"{prefix}{name}_output_external_wrench"),
            }

            g.add(
                (
                    achd_solver,
                    ACHD_SOLVER["external-wrench"],
                    (output_data["output"]["var_name"]),
                )
            )

        for vector_type, vector in linear_vector_type_to_num.items():
            if (coordinate, rdflib.RDF.type, vector_type) in g:
                output_data["vector"] = vector
                break

        for vector_type, vector in angular_vector_type_to_num.items():
            if (coordinate, rdflib.RDF.type, vector_type) in g:
                output_data["vector"] += vector
                break

        signal = g.value(node, PIDController.signal)

        # add the output node to the graph
        id_ = rdflib.URIRef(uuid.uuid4().urn)
        g.add((id_, rdflib.RDF.type, EMBED_MAP.EmbeddingMap))
        # add vector as a collection
        vector_collection = rdflib.BNode()
        l = [rdflib.Literal(i) for i in output_data["vector"]]
        Collection(g, vector_collection, l)
        g.add((id_, EMBED_MAP.vector, vector_collection))
        # add input
        g.add((id_, EMBED_MAP.input, signal))
        # add the output data
        g.add(
            (
                id_,
                EMBED_MAP[output_data["output"]["type"]],
                output_data["output"]["var_name"],
            )
        )
