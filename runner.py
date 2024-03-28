import os
import json
import pprint
import rdflib

from motion_spec_gen.utility import resolver, loader

from motion_spec_gen.namespaces import (
    MOTION_SPEC,
    EMBED_MAP,
    Controller,
    Monitor,
    SOLVER,
)

from motion_spec_gen.something import PIDControllerStep
from motion_spec_gen.ir_gen.translators import (
    MonitorTranslator,
    PIDControllerTranslator,
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
    g.bind("embedding-map", EMBED_MAP)

    models_path = os.path.join(os.path.dirname(__file__), "../models/")

    for file in os.listdir(models_path):
        if file.endswith(".jsonld"):
            g.parse(models_path + file, format="json-ld")

    print(g.serialize(format="turtle"))

    print("--" * 20)

    steps = [PIDControllerStep]

    data = {
        "state": {},
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

        for pre_condition in pre_conditions:

            monitor = g.value(predicate=Monitor.constraint, object=pre_condition)

            ir = MonitorTranslator().translate(g, monitor)

            ir["data"]["name"] = "monitor_pre"
            ir["data"]["return"] = None

            data["state"].update(ir["state"])
            data["variables"].update(ir["variables"])
            data["d"]["monitors"]["pre"][ir["id"]] = ir["data"]

        for per_condition in per_conditions:

            controller = g.value(predicate=Controller.constraint, object=per_condition)

            for step in steps:
                # apply steps to suitable controller
                step().emit(g, controller)

            # intermediate representation generator
            ir = PIDControllerTranslator().translate(g, controller)

            data["state"].update(ir["state"])
            data["variables"].update(ir["variables"])
            data["d"]["controllers"][ir["id"]] = ir["data"]

            embed_map = g.value(predicate=EMBED_MAP.controller, object=controller)
            embed_map_ir = EmbedMapTranslator().translate(g, embed_map)

            data["d"]["embed_maps"][embed_map_ir["id"]] = (
                [embed_map_ir["data"]]
                if embed_map_ir["id"] not in data["d"]["embed_maps"]
                else data["d"]["embed_maps"][embed_map_ir["id"]]
                + [embed_map_ir["data"]]
            )
            data["variables"].update(embed_map_ir["variables"])
            data["state"].update(embed_map_ir["state"])

        # get solvers
        solvers = g.subjects(rdflib.RDF.type, SOLVER.Solver)

        for solver in solvers:
            # solver translator
            solver_ir = SolverTranslator().translate(
                g,
                solver,
                embed_maps=data["d"]["embed_maps"],
                variables=data["variables"],
            )

            data["state"].update(solver_ir["state"])
            data["variables"].update(solver_ir["variables"])
            data["d"]["solvers"][solver_ir["id"]] = solver_ir["data"]

        for post_condition in post_conditions:

            monitor = g.value(predicate=Monitor.constraint, object=post_condition)

            ir = MonitorTranslator().translate(g, monitor)

            ir["data"]["name"] = "monitor_post"
            ir["data"]["return"] = None

            data["state"].update(ir["state"])
            data["variables"].update(ir["variables"])
            data["d"]["monitors"]["post"][ir["id"]] = ir["data"]

    print(g.serialize(format="turtle"))

    print("--" * 20)

    json_obj = json.dumps(data, indent=2)

    # print(json_obj)

    # write to file
    with open("ir.json", "w") as f:
        f.write(json_obj)


if __name__ == "__main__":
    main()
