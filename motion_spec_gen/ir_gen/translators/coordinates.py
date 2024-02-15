from motion_spec_gen.namespaces import (
    PIDController,
    THRESHOLD,
    CONSTRAINT,
    GEOM_COORD,
    GEOM_REL
)
import rdflib
from rdflib.collection import Collection


class CoordinatesTranslator:
    def translate(self, g: rdflib.Graph, node) -> dict:
        state = []
        variables = []

        data = {}

        is_geom_coord = (
            g[node : rdflib.RDF.type : GEOM_COORD.PositionCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.DistanceCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.VelocityTwistCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.AccelerationTwistCoordinate]
        )

        if is_geom_coord:

            if (node, rdflib.RDF.type, GEOM_COORD.PositionCoordinate) in g:
                pass

            elif (node, rdflib.RDF.type, GEOM_COORD.VelocityTwistCoordinate) in g:
                of_vel = g.value(node, GEOM_COORD.of)
                # TODO: as-seen-by is assumed to be base_link for now
                as_seen_by = g.value(node, GEOM_COORD["as-seen-by"])

                linear_vel = g.value(node, GEOM_COORD["linear-velocity"])
                angular_vel = g.value(node, GEOM_COORD["angular-velocity"])

                # TODO: units are not considered for now as it is assumed to be m/s and rad/s

                # get the veloicty 
                vel_of = g.value(of_vel, GEOM_REL["of-entity"])
                vel_wrt = g.value(of_vel, GEOM_REL["with-respect-to"])

                data["velocity-of"] = g.compute_qname(vel_of)[2]
                data["velocity-wrt"] = g.compute_qname(vel_wrt)[2]

            elif (node, rdflib.RDF.type, GEOM_COORD.AccelerationTwistCoordinate) in g:
                pass

        else:
            pass
        
        return {
            "data": data,
            "state": state,
            "variables": variables,
        }