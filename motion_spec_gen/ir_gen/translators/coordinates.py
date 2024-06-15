import json
from motion_spec_gen.namespaces import (
    PID_CONTROLLER,
    THRESHOLD,
    CONSTRAINT,
    GEOM_COORD,
    GEOM_REL,
    GEOM_ENT,
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
    if "Vector" in input_string:
        suffix_start = input_string.find("Vector") + len("Vector")
    elif "AngleAbout" in input_string:
        suffix_start = input_string.find("AngleAbout") + len("AngleAbout")
    elif "OrientationAbout" in input_string:
        suffix_start = input_string.find("OrientationAbout") + len("OrientationAbout")
    else:
        raise ValueError("Invalid input string")

    suffix = input_string[suffix_start:]

    vec_comp = vector_components.get(suffix, None)

    input_string = input_string.lower()

    if vec_comp is None:
        raise ValueError("Invalid suffix")

    if "linear" in input_string or "position" in input_string:
        return vec_comp + (0, 0, 0), f"lin_{suffix.lower()}"
    elif (
        "angular" in input_string
        or "orientation" in input_string
        or "angle" in input_string
    ):
        return (0, 0, 0) + vec_comp, f"ang_{suffix.lower()}"
    else:
        return vec_comp, suffix.lower()


class CoordinatesTranslator:
    def translate(self, g: rdflib.Graph, node, verbose=False, **kwargs) -> dict:
        verbose_padding: int = 0
        # Get the verbose padding from the kwargs
        if "verbose_padding" in kwargs:
            verbose_padding = kwargs["verbose_padding"]
        if verbose:
            print(
                f"{'-'*verbose_padding} Translating coordinates: {g.compute_qname(node)[2]}"
            )

        variables = {}
        data = {}

        prefix = kwargs.get("prefix", "")

        node_qname = g.compute_qname(node)[2]

        is_geom_coord = (
            g[node : rdflib.RDF.type : GEOM_COORD.PoseCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.PositionCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.OrientationCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.DistanceCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.AngularDistanceCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.VelocityTwistCoordinate]
            or g[node : rdflib.RDF.type : GEOM_COORD.AccelerationTwistCoordinate]
        )

        if is_geom_coord:
            if g[node : rdflib.RDF.type : GEOM_COORD.PoseCoordinate]:
                data["type"] = "Pose"

                # get the types of coordinates
                coord_types = g.objects(node, rdflib.RDF.type)
                # get the type with vector in it
                vector_type = [
                    t
                    for t in coord_types
                    if "vector" in str(t).lower() or "angleabout" in str(t).lower()
                ][0]
                vec_comp, suffix = get_vector_value(str(vector_type))

                of_pose = g.value(node, GEOM_COORD.of)
                # TODO: do something with this
                asb = g.value(node, GEOM_COORD["as-seen-by"])

                # get position
                of = g.value(of_pose, GEOM_REL["of-entity"])
                wrt = g.value(of_pose, GEOM_REL["with-respect-to"])

                of_pose_qname = g.compute_qname(of_pose)[2]
                asb_qname = g.compute_qname(asb)[2]
                of_qname = g.compute_qname(of)[2]
                wrt_qname = g.compute_qname(wrt)[2]

                # *assumption*: pose is a 1D vector
                variables[node_qname] = {
                    "type": None,
                    "dtype": "double",
                    "value": None,
                }

                variables[f"{node_qname}_vector"] = {
                    "type": "array",
                    "size": 6,
                    "dtype": "double",
                    "value": vec_comp,
                }

                variables[of_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": of_qname.replace("_origin_point", ""),
                }

                variables[wrt_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": wrt_qname.replace("_origin_point", ""),
                }

                variables[asb_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": asb_qname,
                }

                # if g[node : rdflib.RDF.type : GEOM_COORD.VectorXYZ]:
                #     x = g.value(of_pose, GEOM_COORD.x).toPython()
                #     y = g.value(of_pose, GEOM_COORD.y).toPython()
                #     z = g.value(of_pose, GEOM_COORD.z).toPython()

                #     variables[of_pose_qname]["value"] = [x, y, z]

                data["of"] = {
                    "id": node_qname,
                    "entity": of_qname,
                    "type": "pose",
                    "vector": f"{node_qname}_vector",
                }
                data["asb"] = asb_qname
                data["wrt"] = wrt_qname

            elif g[node : rdflib.RDF.type : GEOM_COORD.PositionCoordinate]:
                data["type"] = "Position"

                # get the types of coordinates
                coord_types = g.objects(node, rdflib.RDF.type)
                # get the type with vector in it
                vector_type = [
                    t
                    for t in coord_types
                    if "vector" in str(t).lower()
                ][0]
                vec_comp, suffix = get_vector_value(str(vector_type))

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

                variables[node_qname] = {
                    "type": None,
                    "size": 1,
                    "dtype": "double",
                    "value": None,
                }

                variables[of_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": of_qname.replace("_origin_point", ""),
                }

                variables[wrt_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": wrt_qname.replace("_origin_point", ""),
                }

                variables[asb_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": asb_qname,
                }

                variables[f"{node_qname}_vector"] = {
                    "type": "array",
                    "size": 6,
                    "dtype": "double",
                    "value": vec_comp,
                }

                if g[node : rdflib.RDF.type : GEOM_COORD.VectorXYZ]:
                    x = g.value(of_pos, GEOM_COORD.x).toPython()
                    y = g.value(of_pos, GEOM_COORD.y).toPython()
                    z = g.value(of_pos, GEOM_COORD.z).toPython()

                    variables[node_qname]["value"] = [x, y, z]

                data["of"] = {
                    "id": node_qname,
                    "entity": of_qname,
                    "type": "position",
                    "vector": f'{node_qname}_vector',
                }
                data["asb"] = asb_qname
                data["wrt"] = wrt_qname

            elif g[node : rdflib.RDF.type : GEOM_COORD.OrientationCoordinate]:
                data["type"] = "Orientation"

                # get the types of coordinates
                coord_types = g.objects(node, rdflib.RDF.type)
                # get the type with vector in it
                vector_type = [
                    t
                    for t in coord_types
                    if "vector" in str(t).lower() or "about" in str(t).lower()
                ][0]
                vec_comp, suffix = get_vector_value(str(vector_type))

                of_ori = g.value(node, GEOM_COORD.of)
                # TODO: do something with this
                asb = g.value(node, GEOM_COORD["as-seen-by"])

                # get Orientation
                of = g.value(of_ori, GEOM_REL["of-entity"])
                wrt = g.value(of_ori, GEOM_REL["with-respect-to"])

                of_ori_qname = g.compute_qname(of_ori)[2]
                asb_qname = g.compute_qname(asb)[2]
                of_qname = g.compute_qname(of)[2]
                wrt_qname = g.compute_qname(wrt)[2]

                variables[node_qname] = {
                    "type": None,
                    "size": 1, # quaternion TODO: this is not correct
                    "dtype": "double",
                    "value": None,
                }

                variables[of_ori_qname] = {
                    "type": None,
                    "size": 1, # quaternion
                    "dtype": "double",
                    "value": None,
                }

                variables[of_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": of_qname.replace("_origin_point", ""),
                }

                variables[wrt_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": wrt_qname.replace("_origin_point", ""),
                }

                variables[asb_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": asb_qname,
                }

                variables[f"{node_qname}_vector"] = {
                    "type": "array",
                    "size": 6,
                    "dtype": "double",
                    "value": vec_comp
                }

                if g[node : rdflib.RDF.type : GEOM_COORD.QuaterniontXYZW]:
                    x = g.value(of_pos, GEOM_COORD.x).toPython()
                    y = g.value(of_pos, GEOM_COORD.y).toPython()
                    z = g.value(of_pos, GEOM_COORD.z).toPython()
                    w = g.value(of_pos, GEOM_COORD.w).toPython()

                    variables[node_qname]["value"] = [x, y, z, w]

                data["of"] = {
                    "id": node_qname,
                    "entity": of_qname,
                    "type": "orientation",
                    "vector": f'{node_qname}_vector',
                }
                data["asb"] = asb_qname
                data["wrt"] = wrt_qname

            elif g[node : rdflib.RDF.type : GEOM_COORD.DistanceCoordinate]:
                # get the types of coordinates
                coord_types = g.objects(node, rdflib.RDF.type)
                # get the type with vector in it
                vector_type = [
                    t
                    for t in coord_types
                    if "vector" in str(t).lower() or "angleabout" in str(t).lower()
                ][0]
                vec_comp, suffix = get_vector_value(str(vector_type))

                of_dist = g.value(node, GEOM_COORD.of)
                of_dist_qname = g.compute_qname(of_dist)[2]

                if g[node : rdflib.RDF.type : GEOM_ENT["1D"]]:
                    data["type"] = "Distance1D"

                    axis_name = f"{node_qname}_axis"

                    variables[axis_name] = {
                        "type": "array",
                        "size": 6,
                        "dtype": "double",
                        "value": vec_comp,
                    }

                    data["axis"] = axis_name
                else:
                    data["type"] = "Distance"

                # TODO: validate the entities

                # TODO: as-seen-by is assumed to be base_link for now
                asb = g.value(node, GEOM_COORD["as-seen-by"])
                # TODO: get units

                # distance entities
                dist_bw = g.objects(of_dist, GEOM_REL["between-entities"])
                dist_bw = [g.compute_qname(e)[2] for e in dist_bw]

                asb_qname = g.compute_qname(asb)[2]

                variables[node_qname] = {
                    "type": None,
                    "dtype": "double",
                    "value": None,
                }

                variables[asb_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": asb_qname.replace("_origin_point", ""),
                }

                for entity in dist_bw:
                    variables[entity] = {
                        "type": None,
                        "dtype": "string",
                        "value": entity.replace("_origin_point", ""),
                    }

                data["of"] = {
                    "id": node_qname,
                    "entities": dist_bw,
                }
                data["asb"] = asb_qname

            elif g[node : rdflib.RDF.type : GEOM_COORD.AngularDistanceCoordinate]:
                # get the types of coordinates
                coord_types = g.objects(node, rdflib.RDF.type)
                # get the type with vector in it
                vector_type = [
                    t
                    for t in coord_types
                    if "vector" in str(t).lower() or "angleabout" in str(t).lower()
                ][0]
                vec_comp, suffix = get_vector_value(str(vector_type))

                of_ang_dist = g.value(node, GEOM_COORD.of)
                of_ang_dist_qname = g.compute_qname(of_ang_dist)[2]

                asb = g.value(node, GEOM_COORD["as-seen-by"])
                asb_qname = g.compute_qname(asb)[2]

                if not g[
                    of_ang_dist : rdflib.RDF.type : GEOM_REL.AngularDistanceBetweenLines
                ]:
                    raise ValueError("Invalid angular distance type")

                print(f"of_ang_dist: {of_ang_dist}")

                # get the lines
                lines = g.objects(of_ang_dist, GEOM_REL["between-entities"])

                # TODO: validate the entities

                query = f"""
                        SELECT ?pos ?pos_coord
                        WHERE {{
                            ?point a geom:Point .
                            
                            ?pos a ?type ;
                                geom-rel:of-entity ?point .
                            
                            FILTER (?type IN (geom-rel:Position, geom-rel:Pose))

                            ?pos_coord a ?type2 ;
                                geom-coord:of ?pos .

                            FILTER (?type2 IN (geom-coord:PositionCoordinate, geom-coord:PoseCoordinate))
                        }}
                        """

                lines_coords = {}

                for line in lines:
                    # get points
                    points = g.objects(line, GEOM_ENT.points)
                    line_qname = g.compute_qname(line)[2]

                    lines_coords[line_qname] = []

                    # check if a position or pose relation exists
                    for point in points:
                        res = g.query(query, initBindings={"point": point})

                        res = res.bindings[0]

                        if not res:
                            raise ValueError("Invalid entities")

                        # pos = res["pos"]
                        pos_coord = res["pos_coord"]

                        coord_ir = self.translate(g, pos_coord, prefix=prefix)

                        variables.update(coord_ir["variables"])

                        lines_coords[line_qname].append(coord_ir["data"])

                data["of"] = {
                    "id": node_qname,
                    "entities": lines_coords,
                }
                data["asb"] = asb_qname
                data["angle_about"] = f"{node_qname}_{suffix}"

            elif g[node : rdflib.RDF.type : GEOM_COORD.VelocityTwistCoordinate]:
                # get the types of coordinates
                coord_types = g.objects(node, rdflib.RDF.type)
                # get the type with vector in it
                vector_type = [
                    t
                    for t in coord_types
                    if "vector" in str(t).lower() or "angleabout" in str(t).lower()
                ][0]
                vec_comp, suffix = get_vector_value(str(vector_type))

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
                    "value": vel_wrt.replace("_origin_point", ""),
                }

                of_vel_qname = f"{g.compute_qname(of_vel)[2]}{prefix}"
                vel_of_qname = g.compute_qname(vel_of)[2]

                variables[vel_of_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": vel_of_qname,
                }

                variables[f"{node_qname}_vector"] = {
                    "type": "array",
                    "size": 6,
                    "dtype": "double",
                    "value": vec_comp,
                }

                data["of"] = {
                    "id": node_qname,
                    "entity": vel_of_qname,
                    "type": "twist",
                    "vector": f"{node_qname}_vector",
                }

                # TODO: units are not considered for now as it is assumed to be m/s and rad/s

                variables[node_qname] = {
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
                # get the types of coordinates
                coord_types = g.objects(node, rdflib.RDF.type)
                # get the type with vector in it
                vector_type = [
                    t
                    for t in coord_types
                    if "vector" in str(t).lower() or "angleabout" in str(t).lower()
                ][0]
                vec_comp, suffix = get_vector_value(str(vector_type))

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
                    "value": asb_qname.replace("_origin_point", ""),
                }

                variables[ab_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": ab_qname.replace("_origin_point", ""),
                }

                variables[at_qname] = {
                    "type": None,
                    "dtype": "string",
                    "value": at_qname.replace("_origin_point", ""),
                }

                of_force_qname = g.compute_qname(of_force)[2]

                variables[f"{of_force_qname}_vector_{suffix}"] = {
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
                    "applied_by_entity": ab_qname,
                    "applied_to_entity": at_qname,
                    "type": "force",
                    "vector": f"{of_force_qname}_vector_{suffix}",
                }

                data["asb"] = asb_qname

        return {
            "data": data,
            "variables": variables,
        }
