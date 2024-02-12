import os
import json
import rdflib

from motion_spec_gen.utility import resolver, loader

from motion_spec_gen.namespaces import MOTION_SPEC, CONTROLLER_OUTPUT

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
    g.bind("controller-output", CONTROLLER_OUTPUT)

    models_path = os.path.join(os.path.dirname(__file__), "../models/")

    for file in os.listdir(models_path):
        if file.endswith(".jsonld"):
            g.parse(models_path + file, format="json-ld")

    print(g.serialize(format="turtle"))

    print("--" * 20)

    steps = [PIDControllerStep]

    for motion_spec in g.subjects(rdflib.RDF.type, MOTION_SPEC.MotionSpec):
        per_conditions = g.objects(motion_spec, MOTION_SPEC["per-conditions"])

        query = f"""
        SELECT ?controller 
        WHERE {{
            ?constraint a constraints:Constraint ;
                ^pidcontroller:constraint ?controller .
        }}
        """

        for per_condition in per_conditions:

            init_bindings = {"constraint": per_condition}

            qres = g.query(query, initBindings=init_bindings)

            qb = qres.bindings

            # print(qb[0]['controller'])

            for step in steps:
                step().emit(g, qb[0]["controller"])

            # intermediate representation generator
            ir = PIDControllerTranslator().translate(g, qb[0]["controller"])

            print(json.dumps(ir, indent=2))
            

    # print(g.serialize(format="turtle"))

    # print("==" * 20)

    
    



if __name__ == "__main__":
    main()
