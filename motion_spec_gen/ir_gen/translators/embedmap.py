from motion_spec_gen.namespaces import (
    EMBED_MAP,
    BASE_FD_SOLVER
)
import rdflib
from rdflib.collection import Collection


class EmbedMapTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        variables = {}
        data = {}

        id = g.compute_qname(node)[2]

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

        # vector
        vector_id = None
        vector_info = None

        embed_map_vector_collec = g.value(node, EMBED_MAP.vector)
        embed_map_vector = list(Collection(g, embed_map_vector_collec))
        embed_map_vector = [float(q) for q in embed_map_vector]

        vector_id = f"{g.compute_qname(node)[2]}_vector"

        variables[vector_id] = {
            "type": "array",
            "size": len(embed_map_vector),
            "dtype": "double",
            "value": embed_map_vector,
        }

        # check if vector direction is defined
        if g[node : rdflib.RDF.type : EMBED_MAP.DirectionVector]:
            vector_direction_from = g.value(node, EMBED_MAP["vector-direction-from"])
            vector_direction_to = g.value(node, EMBED_MAP["vector-direction-to"])
            vector_direction_asb = g.value(node, EMBED_MAP["vector-direction-asb"])

            vector_id = None
            
            vector_info = {
                "from": g.compute_qname(vector_direction_from)[2],
                "to": g.compute_qname(vector_direction_to)[2],
                "asb": g.compute_qname(vector_direction_asb)[2],
            }

        data["name"] = "embed_map"
        data["input"] = g.compute_qname(em_input)[2]
        data["output"] = f'{id}_{output}'
        data["output_type"] = output_type
        data["vector"] = vector_id
        data["vector_info"] = vector_info
        
        data["return"] = None

        return {
            "id": g.compute_qname(em_solver)[2],
            "data": data,
            "variables": variables,
        }
