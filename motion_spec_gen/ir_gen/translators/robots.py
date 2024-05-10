import rdflib
from motion_spec_gen.namespaces import ROBOTS


class RobotsTranslator:

    def translate(self, g: rdflib.Graph, node, **kwargs):

        solvers_data = kwargs.get("solvers_data")

        robot_data = {}
        variables = {}

        if g[node : rdflib.RDF.type : ROBOTS.Manipulator]:
            robot_data["type"] = "Manipulator"

            robot_data["kinematic_chain_start"] = g.compute_qname(
                g.value(node, ROBOTS["kinematic-chain-start"])
            )[2]
            robot_data["kinematic_chain_end"] = g.compute_qname(
                g.value(node, ROBOTS["kinematic-chain-end"])
            )[2]
        elif g[node : rdflib.RDF.type : ROBOTS.MobileBase]:
            robot_data["type"] = "MobileBase"
        else:
            raise ValueError("Unknown robot type")

        solvers = g.objects(node, ROBOTS.solvers)
        solvers = [g.compute_qname(solver)[2] for solver in solvers]

        robot_data["input_command_torques"] = []

        for solver in solvers:
            robot_data["input_command_torques"].append(
                solvers_data[solver]["output_torques"]
            )

        return {
            "id": g.compute_qname(node)[2],
            "data": robot_data,
            "variables": variables,
        }
