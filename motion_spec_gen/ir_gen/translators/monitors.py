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

        variables = {}
        data = {}

        id = g.compute_qname(node)[2]

        constraint = g.value(node, Monitor.constraint)
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
            
            ref_val_id = f'{id}_reference_value'
            variables[ref_val_id] = {
                "type": None,
                "dtype": "double",
                "value": reference_value,
            }
            data["reference_value"] = ref_val_id

        elif operator_type == "LessThan" or operator_type == "GreaterThan":
            threshold_value = g.value(threshold, THRESHOLD["threshold-value"])

            if threshold_value is None:
                raise ValueError("Threshold value not found")
            
            threshold_id = f'{id}_threshold_value'
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
        measured_coord_ir = CoordinatesTranslator().translate(g, measured_coord, prefix="")
        variables.update(measured_coord_ir["variables"])
        
        
        coord_type = measured_coord_ir["data"]["type"]

                
        # if coord_type == "Position":
        #     data["measure_variable"] = "computeForwardPositionKinematics"
        #     data["measured"] = coord_trans_ir["data"]["of"]
        #     data["asb"] = coord_trans_ir["data"]["asb"]
        #     data["wrt"] = coord_trans_ir["data"]["wrt"]

        # if coord_type == "Distance":
        #     data["measure_variable"] = "computeDistance"
        #     of = coord_trans_ir["data"]["of"]
        #     asb = coord_trans_ir["data"]["asb"]

        #     data["measured"] = {
        #         "id": of["id"],
        #         "entities": [g.compute_qname(e)[2] for e in of["entities"]],
        #     }
        #     data["asb"] = asb
        
        if coord_type == "VelocityTwist":
            data["measure_variable"] = "computeForwardVelocityKinematics"
            data["measured"] = measured_coord_ir["data"]["of"]
            data["measured"]["asb"] = measured_coord_ir["data"]["asb"]
            data["measured"]["wrt"] = measured_coord_ir["data"]["wrt"]

        # monitor flag
        variables[f"{id}_flag"] = {
            "type": None,
            "dtype": "bool",
            "value": False,
        }

        data["flag"] = f"{id}_flag"
        data["return"] = None

        return {
            "id": id,
            "data": data,
            "variables": variables,
        }
