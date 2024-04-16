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
    THRESHOLD,
    QUDT,
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

    motion_spec_name = "test"
    models_path = os.path.join(
        os.path.dirname(__file__), f"../models/{motion_spec_name}/"
    )

    for file in os.listdir(models_path):
        if file.endswith(".jsonld"):
            g.parse(models_path + file, format="json-ld")

    print(g.serialize(format="turtle"))

    print("**" * 20)

    # print triplets
    for s, p, o in g:
        print(s, p, o)

    print("**" * 20)

    # get constraint
    constraint = g.value(predicate=rdflib.RDF.type, object=CONSTRAINT.Constraint)
    ref_val = g.value(constraint, THRESHOLD["reference-value"])
    unit = g.value(ref_val, QUDT["unit"])
    print(constraint, ref_val, unit)
    

if __name__ == "__main__":
    main()
