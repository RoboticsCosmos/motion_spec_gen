from motion_spec_gen.namespaces import (
    Monitor,
    THRESHOLD,
    CONSTRAINT,
    EMBED_MAP,
)
import rdflib
from rdflib.collection import Collection

from motion_spec_gen.ir_gen.translators.coordinates import CoordinatesTranslator


class MonitorTranslator:

    def translate(self, g: rdflib.Graph, node) -> dict:

        state = {}
        variables = {}

        id = g.compute_qname(node)[2]

        constraint = g.value(node, Monitor.constraint)
        threshold = g.value(constraint, CONSTRAINT["threshold"])
        threshold_value = g.value(threshold, THRESHOLD["threshold-value"])
        operator = g.value(constraint, CONSTRAINT.operator)

        coord = g.value(constraint, CONSTRAINT.coordinate)
        coord_trans_ir = CoordinatesTranslator().translate(g, coord, prefix=id)
        # extend state and variables with the ones from coord
        state.update(coord_trans_ir["state"])
        variables.update(coord_trans_ir["variables"])

        coord_type = coord_trans_ir["data"]["type"]

        
        if coord_type == "VelocityTwist":
            measure_variable = "computeForwardVelocityKinematics"

        variables[f"{id}_threshold_value"] = {
            "type": None,
            "dtype": "double",
            "value": threshold_value,
        }

        # monitor flag
        variables[f"{id}_flag"] = {
            "type": None,
            "dtype": "bool",
            "value": False,
        }

        return {
            "id": id,
            "data": {
                "measure_variable": measure_variable,
                "operator": g.compute_qname(operator)[2],
                "threshold": f"{id}_threshold_value",
                "measured": coord_trans_ir["data"]["of"],
                "setpoint": coord_trans_ir["data"]["sp"],
                "flag": f"{id}_flag",
                "return": None,
            },
            "state": state,
            "variables": variables,
        }
