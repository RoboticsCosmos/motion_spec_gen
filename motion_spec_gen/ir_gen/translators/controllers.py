from motion_spec_gen.namespaces import (
    Controller,
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

        state = {}
        variables = {}

        id = g.compute_qname(node)[2]

        p_gain = g.value(node, PIDController["p-gain"])
        i_gain = g.value(node, PIDController["i-gain"])
        d_gain = g.value(node, PIDController["d-gain"])
        time_step = g.value(node, PIDController["time-step"])

        signal = g.value(node, Controller.signal)

        # find a embedded_map associated with the signal
        embedded_map = g.value(predicate=EMBED_MAP.input, object=signal)

        embed_map_vector_collec = g.value(embedded_map, EMBED_MAP.vector)
        embed_map_vector = list(Collection(g, embed_map_vector_collec))
        embed_map_vector = [float(q) for q in embed_map_vector]

        # append signal to variables
        variables[g.compute_qname(signal)[2]] = {
            "type": "array",
            "size": len(embed_map_vector),
            "dtype": "double",
            "value": None,
        }

        constraint = g.value(node, Controller.constraint)
        threshold = g.value(constraint, CONSTRAINT["threshold"])
        threshold_value = g.value(threshold, THRESHOLD["threshold-value"])

        coord = g.value(constraint, CONSTRAINT.coordinate)
        coord_trans_ir = CoordinatesTranslator().translate(g, coord, prefix=id)
        # extend state and variables with the ones from coord
        state.update(coord_trans_ir["state"])
        variables.update(coord_trans_ir["variables"])

        # time-step
        variables[f"{id}_time_step"] = {
            "type": None,
            "dtype": "double",
            "value": time_step,
        }

        # threshold value
        variables[f"{id}_threshold_value"] = {
            "type": None,
            "dtype": "double",
            "value": threshold_value,
        }

        # skip any of p, i and d gains if they are not present
        gains = {
            "kp": None,
            "ki": None,
            "kd": None,
        }
        error_sum = None
        prev_error = None
        if p_gain:
            variables[f'{id}_kp'] = {
                "type": None,
                "dtype": "double",
                "value": p_gain,
            }
            gains["kp"] = f'{id}_kp'
        if i_gain:
            variables[f'{id}_ki'] = {
                "type": None,
                "dtype": "double",
                "value": i_gain,
            }
            gains["ki"] = f'{id}_ki'
            error_sum = f"{id}_error_sum"
            variables[error_sum] = {
                "type": "array",
                "size": len(embed_map_vector),
                "dtype": "double",
                "value": None,
            }

        if d_gain:
            variables[f'{id}_kd'] = {
                "type": None,
                "dtype": "double",
                "value": d_gain,
            }
            gains["kd"] = f'{id}_kd'
            prev_error = f"{id}_prev_error"
            variables[prev_error] = {
                "type": "array",
                "size": len(embed_map_vector),
                "dtype": "double",
                "value": None,
            }

        # vector
        vector_id = f'{g.compute_qname(embedded_map)[2]}_vector'
        variables[vector_id] = {
            "type": "array",
            "size": len(embed_map_vector),
            "dtype": "double",
            "value": embed_map_vector,
        }

        return {
            "id": id,
            "data": {
                "name": "pid_controller",
                "dt": f'{id}_time_step',
                "gains": gains if len(gains) > 0 else None,
                "threshold_value": f"{id}_threshold_value",
                "measured": coord_trans_ir["data"]["of"],
                "setpoint": coord_trans_ir["data"]["sp"],
                "signal": g.compute_qname(signal)[2],
                "vector": vector_id,
                "error_sum": error_sum,
                "last_error": prev_error,
            },
            "state": state,
            "variables": variables,
        }
