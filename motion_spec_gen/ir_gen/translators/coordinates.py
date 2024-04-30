from motion_spec_gen.namespaces import (
    PID_CONTROLLER,
    THRESHOLD,
    CONSTRAINT,
    GEOM_COORD,
    GEOM_REL,
    NEWTONIAN_RBD_REL,
    NEWTONIAN_RBD_COORD,
)
import rdflib
from rdflib.collection import Collection


def get_vector_value(input_string):
    # Dictionary of vector components
    vector_components = {
        "X": (1, 0, 0),
        "Y": (0, 1, 0),
        "Z": (0, 0, 1),
        "XY": (1, 1, 0),
        "XZ": (1, 0, 1),
        "YZ": (0, 1, 1),
        "XYZ": (1, 1, 1),
    }

    # Extract the suffix after "Vector" in the input string
    suffix_start = input_string.find("Vector") + len("Vector")
    suffix = input_string[suffix_start:]

    vec_comp = vector_components.get(suffix, None)

    if vec_comp is None:
        raise ValueError("Invalid suffix")

    if "linear" in input_string.lower():
        return vec_comp + (0, 0, 0), f"lin_{suffix.lower()}"
    elif "angular" in input_string.lower():
        return (0, 0, 0) + vec_comp, f"ang_{suffix.lower()}"
    else:
        return vec_comp, suffix.lower()


class CoordinatesTranslator:
    def translate(self, g: rdflib.Graph, node, **kwargs) -> dict:
        variables = {}

        data = {}

        prefix = kwargs.get("prefix", "")

        is_geom_coord = (
            g[node : rdflib.RDF.type : GEOM_COORD.PositionCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.DistanceCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.VelocityTwistCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.AccelerationTwistCoordinate]
        )

        # get the types of coordinates
        coord_types = g.objects(node, rdflib.RDF.type)
        # get the type with vector in it
        vector_type = [t for t in coord_types if "vector" in str(t).lower()][0]
        vec_comp, suffix = get_vector_value(str(vector_type))

        if is_geom_coord:

            if g[node : rdflib.RDF.type : GEOM_COORD.PositionCoordinate]:
                data["type"] = "Position"

                of_pos = g.value(node, GEOM_COORD.of)
                # TODO: do something with this
                asb = g.value(node, GEOM_COORD["as-seen-by"])

                # get position
                of = g.value(of_pos, GEOM_REL["of-entity"])
                wrt = g.value(of_pos, GEOM_REL["with-respect-to"])

                of_pos_qname = g.compute_qname(of_pos)[2]
                asb_qname = g.compute_qname(asb)[2]
                of_qname = g.compute_qname(of)[2]
                wrt_qname = g.compute_qname(wrt)[2]

                variables[of_pos_qname] = {
                    "type": "array",
                    "size": 3,
                    "dtype": "double",
                    "value": None,
                }

                variables[of_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": of_qname,
                }

                variables[wrt_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": wrt_qname,
                }

                variables[asb_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": asb_qname,
                }

                if g[node : rdflib.RDF.type : GEOM_COORD.VectorXYZ]:
                    x = g.value(of_pos, GEOM_COORD.x).toPython()
                    y = g.value(of_pos, GEOM_COORD.y).toPython()
                    z = g.value(of_pos, GEOM_COORD.z).toPython()

                    variables[of_pos_qname]["value"] = [x, y, z]

                data["of"] = {
                    "id": of_pos_qname,
                    "entity": of_qname,
                }
                data["asb"] = asb_qname
                data["wrt"] = wrt_qname

            elif g[node : rdflib.RDF.type : GEOM_COORD.DistanceCoordinate]:
                data["type"] = "Distance"

                # TODO: validate the entities

                of_dist = g.value(node, GEOM_COORD.of)
                # TODO: as-seen-by is assumed to be base_link for now
                asb = g.value(node, GEOM_COORD["as-seen-by"])
                # TODO: get units

                # distance entities
                dist_bw = g.objects(of_dist, GEOM_REL["between-entities"])
                dist_bw = [g.compute_qname(e)[2] for e in dist_bw]

                of_dist_qname = g.compute_qname(of_dist)[2]
                asb_qname = g.compute_qname(asb)[2]

                variables[of_dist_qname] = {
                    "type": None,
                    "dtype": "double",
                    "value": None,
                }

                variables[asb_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": asb_qname,
                }

                for entity in dist_bw:
                    variables[entity] = {
                        "type": None,
                        "dtype": "string",
                        "value": entity,
                    }

                data["of"] = {
                    "id": of_dist_qname,
                    "entities": dist_bw,
                }
                data["asb"] = asb_qname

            elif g[node : rdflib.RDF.type : GEOM_COORD.VelocityTwistCoordinate]:
                data["type"] = "VelocityTwist"

                # TODO: validate the entities

                of_vel = g.value(node, GEOM_COORD.of)
                # TODO: as-seen-by is assumed to be base_link for now
                as_seen_by = g.value(node, GEOM_COORD["as-seen-by"])
                as_seen_by = g.compute_qname(as_seen_by)[2]

                variables[as_seen_by] = {
                    "type": None,
                    "dtype": "string",
                    "value": as_seen_by,
                }

                # get the veloicty
                vel_of = g.value(of_vel, GEOM_REL["of-entity"])
                vel_wrt = g.value(of_vel, GEOM_REL["with-respect-to"])
                vel_wrt = g.compute_qname(vel_wrt)[2]

                variables[vel_wrt] = {
                    "type": None,
                    "dtype": "string",
                    "value": vel_wrt,
                }

                of_vel_qname = f"{g.compute_qname(of_vel)[2]}{prefix}"
                vel_of_qname = g.compute_qname(vel_of)[2]

                variables[vel_of_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": vel_of_qname,
                }

                variables[f"{of_vel_qname}_vector_{suffix}"] = {
                    "type": "array",
                    "size": 6,
                    "dtype": "double",
                    "value": vec_comp,
                }

                data["of"] = {
                    "id": f"of_vel_qname_{suffix}",
                    "entity": vel_of_qname,
                    "type": "twist",
                    "vector": f"{of_vel_qname}_vector_{suffix}",
                }

                # TODO: units are not considered for now as it is assumed to be m/s and rad/s

                variables[f"of_vel_qname_{suffix}"] = {
                    "type": None,
                    "dtype": "double",
                    "value": None,
                }

                data["wrt"] = vel_wrt
                data["asb"] = as_seen_by

            elif (node, rdflib.RDF.type, GEOM_COORD.AccelerationTwistCoordinate) in g:
                pass

        else:
            if g[node : rdflib.RDF.type : NEWTONIAN_RBD_COORD.ForceCoordinate]:
                data["type"] = "Force"

                of_force = g.value(node, NEWTONIAN_RBD_COORD.of)
                asb = g.value(node, NEWTONIAN_RBD_COORD["as-seen-by"])

                # get the force
                if not g[of_force : rdflib.RDF.type : NEWTONIAN_RBD_REL.ContactForce]:
                    raise ValueError("Invalid force type")

                assert g.value(of_force, NEWTONIAN_RBD_REL["applied-by"]) != g.value(
                    of_force, NEWTONIAN_RBD_REL["applied-to"]
                ), "Applied by and applied to cannot be the same"

                applied_by = g.value(of_force, NEWTONIAN_RBD_REL["applied-by"])
                applied_to = g.value(of_force, NEWTONIAN_RBD_REL["applied-to"])

                asb_qname = g.compute_qname(asb)[2]
                ab_qname = g.compute_qname(applied_by)[2]
                at_qname = g.compute_qname(applied_to)[2]

                variables[asb_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": asb_qname,
                }

                variables[ab_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": ab_qname,
                }

                variables[at_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": at_qname,
                }

                of_force_qname = g.compute_qname(of_force)[2]

                variables[f'{of_force_qname}_vector_{suffix}'] = {
                    "type": "array",
                    "size": 6,
                    "dtype": "double",
                    "value": vec_comp,
                }

                variables[of_force_qname] = {
                    "type": None,
                    "dtype": "double",
                    "value": None,
                }

                data["of"] = {
                    "id": of_force_qname,
                    "applied-by-entity": ab_qname,
                    "applied-to-entity": at_qname,
                    "type": "force",
                    "vector": f"{of_force_qname}_vector_{suffix}",
                }

                data["asb"] = asb_qname

        return {
            "data": data,
            "variables": variables,
        }
