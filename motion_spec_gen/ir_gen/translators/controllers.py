from motion_spec_gen.namespaces import (
    PIDController,
    THRESHOLD,
    CONSTRAINT,
    EMBED_MAP,
)
import rdflib
from rdflib.collection import Collection

from motion_spec_gen.ir_gen.translators.coordinates import CoordinatesTranslator


class PIDControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        state = []
        variables = []

        id = g.compute_qname(node)[2]

        p_gain = g.value(node, PIDController["p-gain"])
        i_gain = g.value(node, PIDController["i-gain"])
        d_gain = g.value(node, PIDController["d-gain"])
        time_step = g.value(node, PIDController["time-step"])

        signal = g.value(node, PIDController.signal)

        # find a embedded_map associated with the signal
        for s in g.subjects(EMBED_MAP.input, signal):
            embedded_map = s

        embed_map_vector_collec = g.value(embedded_map, EMBED_MAP.vector)
        embed_map_vector = list(Collection(g, embed_map_vector_collec))
        embed_map_vector = [float(q) for q in embed_map_vector]

        # append singal to variables
        variables.append(
            {
                "type": f"Vector{len(embed_map_vector)}",
                "name": g.compute_qname(signal)[2],
            }
        )

        constraint = g.value(node, PIDController.constraint)
        threshold = g.value(constraint, CONSTRAINT["threshold"])
        threshold_value = g.value(threshold, THRESHOLD["threshold-value"])

        coord = g.value(constraint, CONSTRAINT.coordinate)
        coord_trans_ir = CoordinatesTranslator().translate(g, coord)
        # extend state and variables with the ones from coord
        state.extend(coord_trans_ir["state"])
        variables.extend(coord_trans_ir["variables"])

        # skip any of p, i and d gains if they are not present
        gains = {}
        state = []
        controller_type = []
        if p_gain:
            gains["kp"] = p_gain
            controller_type.append("p_controller")
        if i_gain:
            gains["ki"] = i_gain
            controller_type.append("i_controller")
            state.append(
                {
                    "type": f"Vector{len(embed_map_vector)}",
                    "name": f"{id}_error_sum",
                }
            )
        if d_gain:
            gains["kd"] = d_gain
            controller_type.append("d_controller")
            state.append(
                {
                    "type": f"Vector{len(embed_map_vector)}",
                    "name": "prev_error",
                }
            )

        return {
            "data": {
                "id": id,
                "type": controller_type,
                "dt": time_step,
                "gains": gains if len(gains) > 0 else None,
                "threshold_value": threshold_value,
                "current": coord_trans_ir["data"]["velocity-of"],
                "target": coord_trans_ir["data"]["velocity-wrt"],
                "signal": g.compute_qname(signal)[2]
            },
            "state": state if len(state) > 0 else None,
            "variables": variables if len(variables) > 0 else None,
        }
