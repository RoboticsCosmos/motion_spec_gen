import os
import json
import pprint
import rdflib

from motion_spec_gen.utility import resolver, loader

from motion_spec_gen.namespaces import (
    MOTION_SPEC,
    EMBED_MAP,
    CONSTRAINT,
    CONTROLLER,
    IMPEDANCE_CONTROLLER,
    MONITOR,
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
    SolverTranslator,
)


def main():

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

    motion_spec_name = "freddy_uc1"
    models_path = os.path.join(
        os.path.dirname(__file__), f"../models/{motion_spec_name}/"
    )

    for file in os.listdir(models_path):
        if file.endswith(".jsonld"):
            g.parse(models_path + file, format="json-ld")

    print(g.serialize(format="turtle"))

    print("**" * 20)

    controller_steps = [PIDControllerStep, ImpedanceControllerStep]
    controller_translators = [PIDControllerTranslator, ImpedanceControllerTranslator]

    data = {
        "variables": {},
        "d": {
            "monitors": {
                "pre": {},
                "post": {},
            },
            "controllers": {},
            "embed_maps": {},
            "solvers": {},
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

            val = g.value(predicate=CONTROLLER.constraint, object=per_condition)

            print()
            print(f'val: {val}')

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
                    step().emit(g, controller)

            for translator in controller_translators:
                if translator.is_applicable(g, controller):
                    translator().translate(g, controller)

            # intermediate representation generator
        #     ir = PIDControllerTranslator().translate(g, controller)

        #     data["variables"].update(ir["variables"])
        #     data["d"]["controllers"][ir["id"]] = ir["data"]

        #     embed_map = g.value(predicate=EMBED_MAP.controller, object=controller)
        #     embed_map_ir = EmbedMapTranslator().translate(g, embed_map)

        #     data["d"]["embed_maps"][embed_map_ir["id"]] = (
        #         [embed_map_ir["data"]]
        #         if embed_map_ir["id"] not in data["d"]["embed_maps"]
        #         else data["d"]["embed_maps"][embed_map_ir["id"]]
        #         + [embed_map_ir["data"]]
        #     )
        #     data["variables"].update(embed_map_ir["variables"])

        # # command torques
        # command_torques = []
        # # predicted accelerations
        # predicted_accelerations = []

        # # get solvers
        # solvers = g.subjects(rdflib.RDF.type, SOLVER.Solver)

        # for solver in solvers:
        #     # solver translator
        #     solver_ir = SolverTranslator().translate(
        #         g,
        #         solver,
        #         embed_maps=data["d"]["embed_maps"],
        #         variables=data["variables"],
        #     )

        #     data["variables"].update(solver_ir["variables"])
        #     data["d"]["solvers"][solver_ir["id"]] = solver_ir["data"]

        #     command_torques.append(solver_ir["data"]["output_torques"])
        #     predicted_accelerations.append(solver_ir["data"]["predicted_accelerations"])

        # data["d"]["commands"] = {
        #     "torques": {
        #         "name": "command_torques",
        #         "data": command_torques,
        #     },
        #     "accelerations": {
        #         "name": "command_accelerations",
        #         "data": predicted_accelerations,
        #     },
        # }

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
    file_path = os.path.join(os.path.dirname(__file__), "irs", "ir.json")
    with open(file_path, "w") as f:
        f.write(json_obj)


if __name__ == "__main__":
    main()
