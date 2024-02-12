from motion_spec_gen.namespaces import (
    PIDController,
    THRESHOLD,
    CONSTRAINT,
    CONTROLLER_OUTPUT,
)
import rdflib
from rdflib.collection import Collection


class PIDControllerTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        p_gain = g.value(node, PIDController["p-gain"])
        i_gain = g.value(node, PIDController["i-gain"])
        d_gain = g.value(node, PIDController["d-gain"])
        time_step = g.value(node, PIDController["time-step"])

        constraint = g.value(node, PIDController.constraint)
        threshold = g.value(constraint, CONSTRAINT["threshold"])
        threshold_value = g.value(threshold, THRESHOLD["threshold-value"])

        # skip any of p, i and d gains if they are not present
        gains = {}
        state = {}
        if p_gain:
            gains["p"] = p_gain
        if i_gain:
            gains["i"] = i_gain
            state["error_sum"] = [0.0]  # TODO: should depend on constraint
        if d_gain:
            gains["d"] = d_gain
            state["last_error"] = [0.0]  # TODO: should depend on constraint

        # controller output
        output = g.value(node, PIDController.output)

        if output is None:
            raise ValueError("Controller output is not defined")

        # get types of output
        output_types = list(g.objects(output, rdflib.RDF.type))
        if len(output_types) == 0:
            raise ValueError("Controller output type is not defined")

        output_vector_collec = g.value(output, CONTROLLER_OUTPUT.vector)

        output_vector_list = list(Collection(g, output_vector_collec))
        output_vector_list = [float(q) for q in output_vector_list]

        return {
            "type": "PIDController",
            "timeStep": time_step,
            "gains": gains if len(gains) > 0 else None,
            "threshold_value": threshold_value,
            "state": state if len(state) > 0 else None,
            "output_types": output_types,
            "output_vector": output_vector_list,
        }
