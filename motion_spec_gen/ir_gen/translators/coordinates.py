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
    def translate(self, g: rdflib.Graph, node, **kwargs) -> dict:
        state = {}
        variables = {}

        data = {}

        prefix = kwargs.get("prefix", "")

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

                # get the veloicty 
                vel_of = g.value(of_vel, GEOM_REL["of-entity"])
                # TODO: do something with the with-respect-to
                vel_wrt = g.value(of_vel, GEOM_REL["with-respect-to"])
                
                of_vel_qname = g.compute_qname(of_vel)[2]
                vel_of_qname = g.compute_qname(vel_of)[2]

                variables[vel_of_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": vel_of_qname
                }

                data["of"] = {
                    "id": of_vel_qname,
                    "entity": vel_of_qname,
                }

                linear_vel = g.value(node, GEOM_COORD["linear-velocity"])
                angular_vel = g.value(node, GEOM_COORD["angular-velocity"])

                # TODO: units are not considered for now as it is assumed to be m/s and rad/s

                # state[of_vel_qname] = {
                #     "type": "Twist",
                #     "value": None
                # }

                if linear_vel or angular_vel:
                    data["sp"] = f'{prefix}_{of_vel_qname}_sp'

                    variables_sp = {
                        "type": "array",
                        "dtype": "double",
                        "value": None
                    }

                    if linear_vel:
                        linear = list(Collection(g, linear_vel))
                        linear_value = [float(q) for q in linear]
                        variables_sp["value"] = linear_value
                    if angular_vel:
                        angular = list(Collection(g, angular_vel))
                        angular_value = [float(q) for q in angular]
                        variables_sp["value"] = angular_value if not variables_sp["value"] else variables_sp["value"] + angular_value

                    variables_sp["size"] = len(variables_sp["value"])

                    variables[f'{prefix}_{of_vel_qname}_sp'] = variables_sp

            elif (node, rdflib.RDF.type, GEOM_COORD.AccelerationTwistCoordinate) in g:
                pass

        else:
            pass
        
        return {
            "data": data,
            "state": state,
            "variables": variables,
        }