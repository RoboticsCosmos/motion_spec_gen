from motion_spec_gen.namespaces import (
    Controller,
    PIDController,
    EMBED_MAP,
)
import rdflib
from rdflib.collection import Collection


class EmbedMapTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        variables = {}
        state = {}

        # input
        em_input = g.value(node, EMBED_MAP.input)

        # solver
        em_solver = g.value(node, EMBED_MAP.solver)

        # output
        em_output_ae = g.value(node, EMBED_MAP["output-acceleration-energy"])
        em_output_ew = g.value(node, EMBED_MAP["output-external-wrench"])

        if em_output_ae:
            output = g.compute_qname(em_output_ae)[2]
            output_type = "acceleration-energy"
        elif em_output_ew:
            output = g.compute_qname(em_output_ew)[2]
            output_type = "external-wrench"

        embed_map_vector_collec = g.value(node, EMBED_MAP.vector)
        embed_map_vector = list(Collection(g, embed_map_vector_collec))
        embed_map_vector = [float(q) for q in embed_map_vector]

        vector_id = f'{g.compute_qname(node)[2]}_vector'

        variables[vector_id] = {
            "type": "array",
            "size": len(embed_map_vector),
            "dtype": "double",
            "value": embed_map_vector,
        }

        return {
            "id": g.compute_qname(em_solver)[2],
            "data":{
                "name": "embed_map",
                "input": g.compute_qname(em_input)[2],
                "output": output,
                "output_type": output_type,
                "vector": vector_id,
                "return": None,
            },
            "variables": variables,
            "state": state,
        }
