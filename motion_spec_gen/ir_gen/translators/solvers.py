import numpy as np
from motion_spec_gen.namespaces import (
    Monitor,
    THRESHOLD,
    CONSTRAINT,
    EMBED_MAP,
    ACHD_SOLVER
)
import rdflib
from rdflib.collection import Collection

from motion_spec_gen.ir_gen.translators.coordinates import CoordinatesTranslator


class SolverTranslator:

    def translate(self, g: rdflib.Graph, node, **kwargs) -> dict:

        state = {}
        variables = {}

        embed_maps = kwargs.get("embed_maps")
        d_vars = kwargs.get("variables")

        assert embed_maps is not None and len(embed_maps) > 0, "No embed maps provided"

        id = g.compute_qname(node)[2]

        # get embed maps for this solver
        embed_maps_for_solver = embed_maps[id]

        # achd solver
        if (g[node : rdflib.RDF.type : ACHD_SOLVER.VereshchaginSolver]):

            alpha = np.array([])
            beta = []
            ext_wrench = []

            for embed_map in embed_maps_for_solver:
                vec = d_vars[embed_map["vector"]]["value"]
                local_alpha = None
                # if vec has values of 0 or 1, then form a diagonal matrix
                if all(v == 0 or v == 1 for v in vec):
                    local_alpha = np.zeros((6, len(vec)))
                    np.fill_diagonal(local_alpha, vec)
                else:
                    local_alpha = np.array(vec).reshape(6, -1)

                # remove zero columns
                local_alpha = local_alpha[:, ~np.all(local_alpha == 0, axis=0)]
                
                if alpha.size == 0:
                    alpha = local_alpha
                else:
                    alpha = np.hstack((alpha, local_alpha))

                # output
                output = embed_map["output"]

                if output["acceleration-energy"]:
                    variables[output["acceleration-energy"]] = {
                        "type": "array",
                        "size": 6,
                        "dtype": "double",
                        "value": None
                    }
                    beta += [output["acceleration-energy"]]
                
                if output["external-wrench"]:
                    pass
                    
            nc = alpha.shape[1]

            # num of joints TODO: get from the robot model
            nj = 7

            # solver variables
            # number of constraints
            variables[f'{id}_nc'] = {
                "type": None,
                "dtype": "int",
                "value": nc
            }
            # number of joints
            variables[f'{id}_nj'] = {
                "type": None,
                "dtype": "int",
                "value": nj
            }
            # alpha matrix
            variables[f'{id}_alpha'] = {
                "type": "array_2d",
                "size": 6,
                "dim": 2,
                "dtype": "double",
                "value": alpha.tolist()
            }
            # joint positions
            variables[f'{id}_joint_positions'] = {
                "type": "array",
                "size": nj,
                "dim": 1,
                "dtype": "double",
                "value": None
            }
            # joint velocities
            variables[f'{id}_joint_velocities'] = {
                "type": "array",
                "size": nj,
                "dim": 1,
                "dtype": "double",
                "value": None
            }
            # feed forward torques
            variables[f'{id}_feed_forward_torques'] = {
                "type": "array",
                "size": nj,
                "dim": 1,
                "dtype": "double",
                "value": None
            }

            # output variables
            # predicted joint accelerations
            variables[f'{id}_joint_accelerations'] = {
                "type": "array",
                "size": nj,
                "dim": 1,
                "dtype": "double",
                "value": None
            }
            # output constraint torques
            variables[f'{id}_output_torques'] = {
                "type": "array",
                "size": nj,
                "dim": 1,
                "dtype": "double",
                "value": None
            }

        return {
            "id": id,
            "data": {
                "name": "vereshchagin_solver",
                "alpha": f'{id}_alpha',
                "beta": beta,
                "ext_wrench": ext_wrench,
                "nc": f'{id}_nc',
                "nj": f'{id}_nj',
                "q": f'{id}_joint_positions',
                "qdot": f'{id}_joint_velocities',
                "tau_ff": f'{id}_feed_forward_torques',
                "output_torques": f'{id}_output_torques',
                "predicted_accelerations": f'{id}_predicted_accelerations',
                "return": None,
            },
            "state": state,
            "variables": variables,
        }
        
