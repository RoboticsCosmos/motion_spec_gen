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

        variables = {}
        data = {}

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
            "type": None,
            "dtype": "double",
            "value": None,
        }

        constraint = g.value(node, Controller.constraint)
        threshold = g.value(constraint, CONSTRAINT["threshold"])

        operator = g.value(constraint, CONSTRAINT.operator)

        operator_type = g.value(operator, rdflib.RDF.type)
        operator_type = g.compute_qname(operator_type)[2]

        data["operator"] = operator_type
        if operator_type == "Equal":
            reference_value = g.value(constraint, CONSTRAINT["reference-value"])

            if reference_value is None:
                # TODO: maybe check for reference (@id)
                raise ValueError("Reference value not found")

            ref_val_id = f"{id}_reference_value"
            variables[ref_val_id] = {
                "type": None,
                "dtype": "double",
                "value": reference_value,
            }
            data["setpoint_value"] = ref_val_id

        elif operator_type == "LessThan" or operator_type == "GreaterThan":
            threshold_value = g.value(threshold, THRESHOLD["threshold-value"])

            if threshold_value is None:
                raise ValueError("Threshold value not found")

            threshold_id = f"{id}_threshold_value"
            variables[threshold_id] = {
                "type": None,
                "dtype": "double",
                "value": threshold_value,
            }
            data["threshold_value"] = threshold_id

        else:
            raise ValueError("Operator type not supported")

        # measured coordinate
        measured_coord = g.value(constraint, CONSTRAINT["quantity"])
        measured_coord_ir = CoordinatesTranslator().translate(
            g, measured_coord, prefix=""
        )
        variables.update(measured_coord_ir["variables"])

        coord_type = measured_coord_ir["data"]["type"]

        if coord_type == "VelocityTwist":
            measure_variable = "computeForwardVelocityKinematics"

        # time-step
        variables[f"{id}_time_step"] = {
            "type": None,
            "dtype": "double",
            "value": time_step,
        }

        # skip any of p, i and d gains if they are not present
        variables[f"{id}_kp"] = {
            "type": None,
            "dtype": "double",
            "value": 0.0,
        }
        variables[f"{id}_ki"] = {
            "type": None,
            "dtype": "double",
            "value": 0.0,
        }
        variables[f"{id}_kd"] = {
            "type": None,
            "dtype": "double",
            "value": 0.0,
        }

        variables[f"{id}_error_sum"] = {
            "type": None,
            "dtype": "double",
            "value": None,
        }

        variables[f"{id}_prev_error"] = {
            "type": None,
            "dtype": "double",
            "value": None,
        }

        gains = {
            "kp": f"{id}_kp",
            "ki": f"{id}_ki",
            "kd": f"{id}_kd",
        }

        if p_gain:
            variables[f"{id}_kp"]["value"] = p_gain
        if i_gain:
            variables[f"{id}_ki"]["value"] = i_gain
        if d_gain:
            variables[f"{id}_kd"]["value"] = d_gain

        # vector
        vector_id = f"{g.compute_qname(embedded_map)[2]}_vector"
        variables[vector_id] = {
            "type": "array",
            "size": len(embed_map_vector),
            "dtype": "double",
            "value": embed_map_vector,
        }

        data["name"] = "pid_controller"
        data["measure_variable"] = measure_variable
        data["dt"] = f"{id}_time_step"
        data["gains"] = gains if len(gains) > 0 else None
        data["measured"] = measured_coord_ir["data"]["of"]
        data["measured"]["asb"] = measured_coord_ir["data"]["asb"]
        data["measured"]["wrt"] = measured_coord_ir["data"]["wrt"]
        data["signal"] = g.compute_qname(signal)[2]
        data["vector"] = vector_id
        data["error_sum"] = f"{id}_error_sum"
        data["last_error"] = f"{id}_prev_error"

        return {
            "id": id,
            "data": data,
            "variables": variables,
        }
