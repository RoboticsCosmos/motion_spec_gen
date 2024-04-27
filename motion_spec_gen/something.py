import uuid
import rdflib
from rdflib.collection import Collection
from motion_spec_gen.utility.helpers import for_type
from motion_spec_gen.namespaces import (
    CONTROLLER,
    GEOM_REL,
    PID_CONTROLLER,
    IMPEDANCE_CONTROLLER,
    THRESHOLD,
    CONSTRAINT,
    GEOM_COORD,
    ACHD_SOLVER,
    BASE_FD_SOLVER,
    EMBED_MAP,
)


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

    input_string = input_string.lower()

    if "linear" in input_string or "force" in input_string:
        return vec_comp + (0, 0, 0), f"lin_{suffix.lower()}"
    elif "angular" in input_string or "torque" in input_string:
        return (0, 0, 0) + vec_comp, f"ang_{suffix.lower()}"
    else:
        return vec_comp, suffix.lower()


@for_type(PID_CONTROLLER.PIDController)
class PIDControllerStep:

    def emit(self, g: rdflib.Graph, node: rdflib.URIRef, **kwargs) -> dict:
        """
        Add an output node to the graph for pid controller
        """

        constraint = g.value(node, CONTROLLER.constraint)

        # *Assumption: compute using setpoint
        coordinate = g.value(constraint, CONSTRAINT.quantity)

        output_data = {}

        qname = g.compute_qname(node)
        prefix = qname[1]
        name = qname[2]

        # get solver from embed map
        embed_map = g.value(predicate=EMBED_MAP.controller, object=node)
        solver = g.value(embed_map, EMBED_MAP.solver)
        solver_name = g.compute_qname(solver)[2]

        # TOOD: check for a better way
        is_geom_coord = (
            g[coordinate : rdflib.RDF.type : GEOM_COORD.PositionCoordinate]
            or g[coordinate : rdflib.RDF.type : GEOM_COORD.DistanceCoordinate]
            or g[coordinate : rdflib.RDF.type : GEOM_COORD.VelocityTwistCoordinate]
            or g[coordinate : rdflib.RDF.type : GEOM_COORD.AccelerationTwistCoordinate]
        )

        # *Assumption: one controller signal can be mapped to only one type of output
        if is_geom_coord:
            if g[solver : rdflib.RDF.type : ACHD_SOLVER.ACHDSolver]:
                output_data["output"] = {
                    "type": "output-acceleration-energy",
                    "var_name": rdflib.URIRef(
                        f"{prefix}{solver_name}_output_acceleration_energy"
                    ),
                }

                g.add(
                    (
                        solver,
                        ACHD_SOLVER["acceleration-energy"],
                        output_data["output"]["var_name"],
                    )
                )

        else:
            if g[solver : rdflib.RDF.type : ACHD_SOLVER.ACHDSolverFext]:
                output_data["output"] = {
                    "type": "output-external-wrench",
                    "var_name": rdflib.URIRef(
                        f"{prefix}{solver_name}_output_external_wrench"
                    ),
                }

                g.add(
                    (
                        solver,
                        ACHD_SOLVER["external-wrench"],
                        (output_data["output"]["var_name"]),
                    )
                )

        types_of_coord = list(g.objects(coordinate, rdflib.RDF.type))
        # get the type with "vector" in it
        vec_type = [t for t in types_of_coord if "vector" in str(t).lower()][0]
        vec_comp, suffix = get_vector_value(str(vec_type))

        output_data["vector"] = vec_comp

        signal = rdflib.URIRef(f"{prefix}{name}_signal")
        # add the signal to the controller
        g.add((node, CONTROLLER.signal, rdflib.URIRef(signal)))

        # add the output node to the graph
        # add vector as a collection
        vector_collection = rdflib.BNode()
        l = [rdflib.Literal(i) for i in output_data["vector"]]
        Collection(g, vector_collection, l)
        g.add((embed_map, EMBED_MAP.vector, vector_collection))
        # add input
        g.add((embed_map, EMBED_MAP.input, signal))
        # add the output data
        g.add(
            (
                embed_map,
                EMBED_MAP[output_data["output"]["type"]],
                output_data["output"]["var_name"],
            )
        )


@for_type(IMPEDANCE_CONTROLLER.ImpedanceController)
class ImpedanceControllerStep:

    def emit(self, g: rdflib.Graph, node: rdflib.URIRef, **kwargs) -> dict:
        """
        Add an output node to the graph for impedance controller
        """

        output_data = {
            "stiffness": {},
            "damping": {},
        }

        qname = g.compute_qname(node)
        prefix = qname[1]
        name = qname[2]

        signal = rdflib.URIRef(f"{prefix}{name}_signal")
        # add the signal to the controller
        g.add((node, CONTROLLER.signal, rdflib.URIRef(signal)))

        # get solver from embed map
        embed_map = g.value(predicate=EMBED_MAP.controller, object=node)
        solver = g.value(embed_map, EMBED_MAP.solver)
        solver_name = g.compute_qname(solver)[2]

        output_data["output"] = {
            "type": "output-external-wrench",
            "var_name": rdflib.URIRef(f"{prefix}{solver_name}_output_external_wrench"),
        }

        if g[solver : rdflib.RDF.type : ACHD_SOLVER.ACHDSolverFext]:
            pass

        if g[solver : rdflib.RDF.type : BASE_FD_SOLVER.BaseFDSolver]:
            g.add(
                (
                    solver,
                    BASE_FD_SOLVER.wrench,
                    output_data["output"]["var_name"],
                )
            )

        stiffness = g.value(node, IMPEDANCE_CONTROLLER.stiffness)
        damping = g.value(node, IMPEDANCE_CONTROLLER.damping)

        if stiffness is None and damping is None:
            raise ValueError("Stiffness and Damping not defined")

        if stiffness:
            position_constraint = g.value(stiffness, CONTROLLER.constraint)

            coordinate = g.value(position_constraint, CONSTRAINT.quantity)

            if g[coordinate : rdflib.RDF.type : GEOM_COORD.PositionCoordinate]:
                types_of_coord = list(g.objects(coordinate, rdflib.RDF.type))
                # get the type with "vector" in it
                vec_type = [t for t in types_of_coord if "vector" in str(t).lower()][0]
                vec_comp, suffix = get_vector_value(str(vec_type))

                output_data["stiffness"]["vector"] = vec_comp

            if g[coordinate : rdflib.RDF.type : GEOM_COORD.DistanceCoordinate]:
                of_dist = g.value(coordinate, GEOM_COORD.of)
                asb = g.value(coordinate, GEOM_COORD["as-seen-by"])
                asb_qn = g.compute_qname(asb)[2]
                bw_ents = g.objects(of_dist, GEOM_REL["between-entities"])
                bw_ents = [g.compute_qname(e)[2] for e in bw_ents]

                # if asb_qn in bw_ents: consider it as from for direction
                if asb_qn in bw_ents:
                    dir_from = asb_qn
                    dir_to = [e for e in bw_ents if e != asb_qn][0]
                else:
                    raise ValueError("Dist. coord direction not supported")

                output_data["stiffness"]["vector_direction"] = {
                    "from": rdflib.URIRef(f"{prefix}{dir_from}"),
                    "to": rdflib.URIRef(f"{prefix}{dir_to}"),
                }
                output_data["stiffness"]["vector"] = (1, 1, 1)

            output_data["stiffness"]["var_name"] = rdflib.URIRef(
                f"{prefix}{solver_name}_output_stiffness"
            )

            # add the output node to the graph
            # add vector as a collection
            vector_collection = rdflib.BNode()
            l = [rdflib.Literal(i) for i in output_data["stiffness"]["vector"]]
            Collection(g, vector_collection, l)
            g.add((embed_map, EMBED_MAP["vector"], vector_collection))
            # vector direction
            if "vector_direction" in output_data["stiffness"]:
                # add type 
                g.add((embed_map, rdflib.RDF.type, EMBED_MAP.DirectionVector))
                g.add(
                    (
                        embed_map,
                        EMBED_MAP["vector-direction-from"],
                        output_data["stiffness"]["vector_direction"]["from"],
                    )
                )
                g.add(
                    (
                        embed_map,
                        EMBED_MAP["vector-direction-to"],
                        output_data["stiffness"]["vector_direction"]["to"],
                    )
                )
            # add input
            g.add((embed_map, EMBED_MAP.input, signal))
            # add the output data
            g.add(
                (
                    embed_map,
                    EMBED_MAP[output_data["output"]["type"]],
                    output_data["output"]["var_name"],
                )
            )
