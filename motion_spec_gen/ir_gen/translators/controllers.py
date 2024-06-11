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
    NEWTONIAN_RBD_REL,
)
import rdflib
from rdflib.collection import Collection

from motion_spec_gen.ir_gen.translators.coordinates import CoordinatesTranslator

from motion_spec_gen.utility.helpers import for_type


@for_type(PID_CONTROLLER.PIDController)
class PIDControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        compute_variables = {}
        variables = {}
        data = {
            "measured": {},
        }

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

            if g[reference_value : rdflib.RDF.type : QUDT.Quantity]:

                # TODO: check quantity kind of reference value is same as that of the quantity
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
                # *assumption*: will be a coordinate
                ref_coord_ir = CoordinatesTranslator().translate(
                    g, reference_value, prefix=""
                )

                # print(f'ref {reference_value} coord vars: {ref_coord_ir["variables"]}')
                ref_coord_var_id = ref_coord_ir["data"]["of"]["id"]
                if ref_coord_ir["variables"][ref_coord_var_id]["value"] is None:

                    # pop the value from variables
                    ref_coord_var = ref_coord_ir["variables"].pop(ref_coord_var_id)

                    ref_coord_var = {
                        "type": None,
                        "dtype": "double",
                        "value": None,
                    }

                    variables[ref_coord_var_id] = ref_coord_var

                    if ref_coord_ir["data"]["type"] == "Pose":
                        measure_variable = "computeForwardPoseKinematics"

                    # add it to compute_variables
                    compute_variables[ref_coord_var_id] = {
                        "measured": {
                            "of": ref_coord_ir["data"]["of"],
                            "asb": ref_coord_ir["data"]["asb"],
                            "wrt": ref_coord_ir["data"]["wrt"],
                        },
                        "measure_variable": measure_variable,
                    }

                    data["reference_value"] = ref_coord_var_id

                variables.update(ref_coord_ir["variables"])

        else:
            raise ValueError("Operator type not supported")

        # measured coordinate
        measured_coord_ir = CoordinatesTranslator().translate(g, quantity, prefix="")
        variables.update(measured_coord_ir["variables"])

        coord_type = measured_coord_ir["data"]["type"]

        if coord_type == "Pose":
            measure_variable = "computeForwardPoseKinematics"
            data["measured"]["wrt"] = measured_coord_ir["data"]["wrt"]

        elif coord_type == "VelocityTwist":
            measure_variable = "computeForwardVelocityKinematics"
            data["measured"]["wrt"] = measured_coord_ir["data"]["wrt"]

        elif coord_type == "Force":
            measure_variable = "computeForce"

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
        data["error"] = f"{id}_error"
        data["measure_variable"] = measure_variable
        data["dt"] = f"{id}_time_step"
        data["gains"] = gains if len(gains) > 0 else None
        data["measured"]["of"] = measured_coord_ir["data"]["of"]
        data["measured"]["asb"] = measured_coord_ir["data"]["asb"]
        data["signal"] = g.compute_qname(signal)[2]
        data["vector"] = vector_id
        data["error_sum"] = f"{id}_error_sum"
        data["last_error"] = f"{id}_prev_error"

        return {
            "id": id,
            "data": data,
            "variables": variables,
            "compute_variables": compute_variables,
        }


@for_type(IMPEDANCE_CONTROLLER.ImpedanceController)
class ImpedanceControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        variables = {}
        data = {}

        id = g.compute_qname(node)[2]

        stiffness = g.value(node, IMPEDANCE_CONTROLLER.stiffness)
        damping = g.value(node, IMPEDANCE_CONTROLLER.damping)

        signal = g.value(node, CONTROLLER.signal)
        signal_qname = g.compute_qname(signal)[2]

        # find a embedded_map associated with the signal
        embedded_map = g.value(predicate=EMBED_MAP.input, object=signal)

        embed_map_vector_collec = g.value(embedded_map, EMBED_MAP.vector)
        embed_map_vector = list(Collection(g, embed_map_vector_collec))
        embed_map_vector = [float(q) for q in embed_map_vector]

        # append signal to variables
        variables[signal_qname] = {
            "type": None,
            "dtype": "double",
            "value": None,
        }

        if stiffness:
            stiffness_constraint = g.value(stiffness, CONTROLLER.constraint)
            stiffness_diag_mat = g.value(
                stiffness, NEWTONIAN_RBD_REL["stiffness-diagonal-matrix"]
            )
            stiffness_diag_mat = list(Collection(g, stiffness_diag_mat))
            stiffness_diag_mat = [float(q) for q in stiffness_diag_mat]

            variables[f"{id}_stiffness_diag_mat"] = {
                "type": "array",
                "size": len(stiffness_diag_mat),
                "dtype": "double",
                "value": stiffness_diag_mat,
            }

            quantity = g.value(stiffness_constraint, CONSTRAINT.quantity)

            operator = g.value(stiffness_constraint, CONSTRAINT.operator)

            operator_type = g.value(operator, rdflib.RDF.type)
            operator_type = g.compute_qname(operator_type)[2]

            data["operator"] = operator_type
            if operator_type == "Equal":
                reference_value = g.value(
                    stiffness_constraint, THRESHOLD["reference-value"]
                )

                if reference_value is None:
                    raise ValueError("Reference value not found")

                # TODO: check quantity kind of reference value is same as that of the quantity
                rv_quant_kind = g.value(reference_value, QUDT.hasQuantityKind)

                unit = g.value(reference_value, QUDT.unit)
                value = g.value(reference_value, QUDT["value"])

                ref_val_id = g.compute_qname(reference_value)[2]
                variables[ref_val_id] = {
                    "type": None,
                    "dtype": "double",
                    "value": value,
                }

            else:
                raise ValueError("Operator type not supported")

            # measured coordinate
            measured_coord_ir = CoordinatesTranslator().translate(
                g, quantity, prefix=""
            )
            variables.update(measured_coord_ir["variables"])

            coord_type = measured_coord_ir["data"]["type"]

            measured = {}

            measured = {
                "of": measured_coord_ir["data"]["of"],
                "asb": measured_coord_ir["data"]["asb"],
            }

            if coord_type == "Distance":
                measure_variable = "computeDistance"
            elif coord_type == "Distance1D":
                measure_variable = "computeDistance1D"
                measured["of"]["axis"] = measured_coord_ir["data"]["axis"]

            # vector
            vector_id = f"{g.compute_qname(embedded_map)[2]}_vector"
            variables[vector_id] = {
                "type": "array",
                "size": len(embed_map_vector),
                "dtype": "double",
                "value": embed_map_vector,
            }

            data["stiffness"] = {
                "error": f"{id}_stiffness_error",
                "reference_value": ref_val_id,
                "measure_variable": measure_variable,
                "diag_mat": f"{id}_stiffness_diag_mat",
                "measured": measured,
            }

        force = g.value(node, IMPEDANCE_CONTROLLER.force)

        if g[force : rdflib.RDF.type : NEWTONIAN_RBD_REL.VirtualForce]:
            force_applied_to = g.value(force, NEWTONIAN_RBD_REL["applied-to"])
            force_applied_to = g.compute_qname(force_applied_to)[2]

            variables[force_applied_to] = {
                "type": None,
                "dtype": "string",
                "value": force_applied_to,
            }

            data["force"] = {
                "id": g.compute_qname(force)[2],
                "applied_to": force_applied_to,
            }

        data["name"] = "impedance_controller"
        data["damping"] = None
        data["signal"] = signal_qname
        data["vector"] = vector_id

        return {
            "id": id,
            "data": data,
            "variables": variables,
        }
