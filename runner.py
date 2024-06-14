import os
import json
import argparse
import rdflib

from motion_spec_gen.utility import resolver, loader

from motion_spec_gen.namespaces import (
    MOTION_SPEC,
    EMBED_MAP,
    CONSTRAINT,
    CONTROLLER,
    IMPEDANCE_CONTROLLER,
    MONITOR,
    ROBOTS,
    SOLVER,
    GEOM_COORD,
    NEWTONIAN_RBD_REL,
)

from motion_spec_gen.something import PIDControllerStep, ImpedanceControllerStep
from motion_spec_gen.ir_gen.translators import (
    MonitorTranslator,
    PIDControllerTranslator,
    ImpedanceControllerTranslator,
    EmbedMapTranslator,
    ACHDSolverTranslator,
    BaseFDSolverTranslator,
    ACHDSolverFextTranslator,
    RobotsTranslator,
    CoordinatesTranslator
)


def main(motion_spec_name: str = None, ir_out_file_name: str = "ir.json", verbose: bool = False, print_graph: bool = False):

    if motion_spec_name is None:
        raise ValueError("Motion specification name is required")

    ROB = rdflib.Namespace("https://roboticscosmos.github.io/rob")

    METAMODELS = rdflib.Namespace("https://roboticscosmos.github.io/metamodels")
    MODELS = rdflib.Namespace("https://roboticscosmos.github.io/models")

    metamodels_path = os.path.join(os.path.dirname(__file__), "../metamodels/")

    url_map = {METAMODELS: metamodels_path}

    resolver.install(resolver.IriToFileResolver(url_map))

    g = rdflib.ConjunctiveGraph()
    g.bind("uuid", rdflib.Namespace("urn:uuid:"))
    # g.bind("embedding-map", EMBED_MAP)
    g.bind("controller", CONTROLLER)

    motion_spec_name = motion_spec_name
    models_path = os.path.join(
        os.path.dirname(__file__), f"../models/{motion_spec_name}/"
    )

    for file in os.listdir(models_path):
        if file.endswith(".jsonld"):
            g.parse(models_path + file, format="json-ld")

    if print_graph:
        print(g.serialize(format="turtle"))
        print("**" * 20)

    controller_steps = [PIDControllerStep, ImpedanceControllerStep]
    controller_translators = [PIDControllerTranslator, ImpedanceControllerTranslator]
    solver_translators = [
        ACHDSolverTranslator,
        ACHDSolverFextTranslator,
        BaseFDSolverTranslator,
    ]

    data = {
        "variables": {},
        "compute_variables": {},
        "d": {
            "monitors": {
                "pre": {},
                "post": {},
            },
            "controllers": {},
            "embed_maps": {},
            "solvers": {},
            "robots": {},
        },
    }

    for motion_spec in g.subjects(rdflib.RDF.type, MOTION_SPEC.MotionSpec):

        pre_conditions = g.objects(motion_spec, MOTION_SPEC["pre-conditions"])
        per_conditions = g.objects(motion_spec, MOTION_SPEC["per-conditions"])
        post_conditions = g.objects(motion_spec, MOTION_SPEC["post-conditions"])

        # for pre_condition in pre_conditions:

        #     monitor = g.value(predicate=Monitor.constraint, object=pre_condition)

        #     ir = MonitorTranslator().translate(g, monitor)

        #     ir["data"]["name"] = "monitor_pre"
        #     ir["data"]["return"] = None

        #     data["variables"].update(ir["variables"])
        #     data["d"]["monitors"]["pre"][ir["id"]] = ir["data"]

        for per_condition in per_conditions:
            if verbose:
                print(f"Translating per condition: {g.compute_qname(per_condition)[2]}")
            
            val = g.value(predicate=CONTROLLER.constraint, object=per_condition)

            if g[val : rdflib.RDF.type : CONTROLLER.Controller]:
                controller = val
            elif g[val : rdflib.RDF.type : NEWTONIAN_RBD_REL.Stiffness]:
                controller = g.value(
                    predicate=IMPEDANCE_CONTROLLER.stiffness,
                    object=val,
                )
            else:
                raise ValueError("Controller not found")

            # list(g[per_condition : CONSTRAINT.quantity / GEOM_COORD["as-seen-by"]])

            for step in controller_steps:
                if step.is_applicable(g, controller):
                    step().emit(g, controller, verbose=verbose, verbose_padding=2)

            for translator in controller_translators:
                if translator.is_applicable(g, controller):
                    ir = translator().translate(g, controller, verbose=verbose, verbose_padding=2)

                    data["variables"].update(ir["variables"])
                    data["d"]["controllers"][ir["id"]] = ir["data"]

                    if "compute_variables" in ir:
                        data["compute_variables"].update(ir["compute_variables"])

            embed_map = g.value(predicate=EMBED_MAP.controller, object=controller)
            embed_map_ir = EmbedMapTranslator().translate(g, embed_map)

            data["d"]["embed_maps"][embed_map_ir["id"]] = (
                [embed_map_ir["data"]]
                if embed_map_ir["id"] not in data["d"]["embed_maps"]
                else data["d"]["embed_maps"][embed_map_ir["id"]]
                + [embed_map_ir["data"]]
            )
            data["variables"].update(embed_map_ir["variables"])

            if verbose:
                print()

        # for coord in g.subjects(rdflib.RDF.type, GEOM_COORD.AngularDistanceCoordinate):
        #     CoordinatesTranslator().translate(g, coord)

        # get solvers
        solvers = g.subjects(rdflib.RDF.type, SOLVER.Solver)

        for solver in solvers:
            if verbose:
                print(f"Translating solver: {g.compute_qname(solver)[2]}")
            
            if g.compute_qname(solver)[2] not in data["d"]["embed_maps"]:
                continue

            for translator in solver_translators:
                if translator.is_applicable(g, solver):
                    ir = translator().translate(
                        g,
                        solver,
                        embed_maps=data["d"]["embed_maps"],
                        variables=data["variables"],
                        verbose=verbose,
                        verbose_padding=2,
                    )

                    data["variables"].update(ir["variables"])
                    data["d"]["solvers"][ir["id"]] = ir["data"]

            if verbose:
                print()

        for robot in g.subjects(rdflib.RDF.type, ROBOTS.Robot):

            robot_ir = RobotsTranslator().translate(
                g, robot, solvers_data=data["d"]["solvers"]
            )

            data["variables"].update(robot_ir["variables"])
            data["d"]["robots"][robot_ir["id"]] = robot_ir["data"]

        # for post_condition in post_conditions:

        #     monitor = g.value(predicate=Monitor.constraint, object=post_condition)

        #     ir = MonitorTranslator().translate(g, monitor)

        #     ir["data"]["name"] = "monitor_post"
        #     ir["data"]["return"] = None

        #     data["variables"].update(ir["variables"])
        #     data["d"]["monitors"]["post"][ir["id"]] = ir["data"]

    # print(g.serialize(format="turtle"))

    # print("--" * 20)

    json_obj = json.dumps(data, indent=2)

    # print(json_obj)

    # write to file
    if not ir_out_file_name.endswith(".json"):
        ir_out_file_name += ".json"
    file_path = os.path.join(os.path.dirname(__file__), "irs", ir_out_file_name)
    with open(file_path, "w") as f:
        f.write(json_obj)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate motion specification IR",
        usage="python -m motion_spec_gen.runner -m <motion_spec_name> -o <output_file_name> [-v] [-g]",
    )
    # define arguments
    parser.add_argument(
        "-o", "--output", type=str, help="IR output file name", default="ir.json"
    )
    parser.add_argument(
        "-m", "--motion-spec", type=str, help="Motion specification name"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Print verbose output"
    )
    parser.add_argument(
        "-g", "--graph", action="store_true", help="Print graph output"
    )

    args = parser.parse_args()

    main(args.motion_spec, args.output, args.verbose, args.graph)