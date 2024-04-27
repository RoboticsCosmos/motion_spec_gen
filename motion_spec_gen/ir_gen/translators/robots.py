import rdflib
from motion_spec_gen.namespaces import ROBOTS


class RobotsTranslator:

    def translate(self, g: rdflib.Graph, node, **kwargs):

        solvers_data = kwargs.get("solvers_data")

        data = {}
        variables = {}

        if g[node : rdflib.RDF.type : ROBOTS.Robot]:
            solvers = g.objects(node, ROBOTS.solvers)
            solvers = [g.compute_qname(solver)[2] for solver in solvers]

            robot_data = {
                "input_command_torques": [],
            }
            for solver in solvers:
                robot_data["input_command_torques"].append(
                    solvers_data[solver]["output_torques"]
                )

        return {
            "id": g.compute_qname(node)[2],
            "data": robot_data,
            "variables": variables,
        }
