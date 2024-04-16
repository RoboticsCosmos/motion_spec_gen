from motion_spec_gen.namespaces import (
    CONTROLLER,
    PID_CONTROLLER,
    IMPEDANCE_CONTROLLER,
    THRESHOLD,
    CONSTRAINT,
    EMBED_MAP,
    QUDT,
    GEOM_REL,
    GEOM_COORD,
    NEWTONIAN_RBD_COORD,
    NEWTONIAN_RBD_REL
)
import rdflib
from rdflib.collection import Collection

from motion_spec_gen.ir_gen.translators.coordinates import CoordinatesTranslator

from motion_spec_gen.utility.helpers import for_type


@for_type(PID_CONTROLLER.PIDController)
class PIDControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        variables = {}
        data = {}

        id = g.compute_qname(node)[2]

        p_gain = g.value(node, PID_CONTROLLER["p-gain"])
        i_gain = g.value(node, PID_CONTROLLER["i-gain"])
        d_gain = g.value(node, PID_CONTROLLER["d-gain"])
        time_step = g.value(node, PID_CONTROLLER["time-step"])

        signal = g.value(node, CONTROLLER.signal)

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

        constraint = g.value(node, CONTROLLER.constraint)
        quantity = g.value(constraint, CONSTRAINT.quantity)

        operator = g.value(constraint, CONSTRAINT.operator)

        operator_type = g.value(operator, rdflib.RDF.type)
        operator_type = g.compute_qname(operator_type)[2]

        data["operator"] = operator_type
        if operator_type == "Equal":
            reference_value = g.value(constraint, THRESHOLD["reference-value"])

            if reference_value is None:
                raise ValueError("Reference value not found")
            
            #TODO: check quantity kind of reference value is same as that of the quantity
            rv_quant_kind = g.value(reference_value, QUDT.hasQuantityKind)

            unit = g.value(reference_value, QUDT.unit)
            value = g.value(reference_value, QUDT["value"])

            ref_val_id = g.compute_qname(reference_value)[2]
            variables[ref_val_id] = {
                "type": None,
                "dtype": "double",
                "value": value,
            }
            data["reference_value"] = ref_val_id

        else:
            raise ValueError("Operator type not supported")

        # measured coordinate
        measured_coord_ir = CoordinatesTranslator().translate(
            g, quantity, prefix=""
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


@for_type(IMPEDANCE_CONTROLLER.ImpedanceController)
class ImpedanceControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        pass

        variables = {}
        data = {}

        id = g.compute_qname(node)[2]

        stiffness = g.value(node, IMPEDANCE_CONTROLLER.stiffness)
        damping = g.value(node, IMPEDANCE_CONTROLLER.damping)

        signal = g.value(node, CONTROLLER.signal)

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

        constraint = g.value(node, CONTROLLER.constraint)
        threshold = g.value(constraint, CONSTRAINT["threshold"])

        operator = g.value(constraint, CONSTRAINT.operator)

        operator_type = g.value(operator, rdflib.RDF.type)
        operator_type = g.compute_qname(operator_type)[2]

        data["operator"] = operator_type
        if operator_type == "Equal":
            reference_value = g.value(constraint, CONSTRAINT["reference-value"])

            pass