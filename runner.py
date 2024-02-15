import os
import json
import rdflib

from motion_spec_gen.utility import resolver, loader

from motion_spec_gen.namespaces import MOTION_SPEC, EMBED_MAP, PIDController

from motion_spec_gen.something import PIDControllerStep
from motion_spec_gen.ir_gen.translators import PIDControllerTranslator


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

    query = f"""
    SELECT ?achd_solver
    WHERE {{
        ?achd_solver a vereshchaginSolver:VereshchaginSolver .
    }}
    """

    qres = g.query(query)
    qb = qres.bindings

    achd_solver = qb[0]["achd_solver"]

    steps = [PIDControllerStep]

    for motion_spec in g.subjects(rdflib.RDF.type, MOTION_SPEC.MotionSpec):
        per_conditions = g.objects(motion_spec, MOTION_SPEC["per-conditions"])

        for per_condition in per_conditions:

            controller = g.value(
                predicate=PIDController.constraint, object=per_condition
            )

            for step in steps:
                step().emit(g, controller, achd_solver=achd_solver)

            # intermediate representation generator
            ir = PIDControllerTranslator().translate(g, controller)

            json_obj = json.dumps(ir, indent=4)

            print(json_obj)

            # write to file
            # with open("pid_controller.json", "w") as f:
            #     f.write(json_obj)

    # print(g.serialize(format="turtle"))

    # print("==" * 20)


if __name__ == "__main__":
    main()
