import numpy as np
from motion_spec_gen.namespaces import (
    MONITOR,
    THRESHOLD,
    CONSTRAINT,
    EMBED_MAP,
    SOLVER,
    ACHD_SOLVER,
    BASE_FD_SOLVER,
)
import rdflib
from rdflib.collection import Collection

from motion_spec_gen.ir_gen.translators.coordinates import CoordinatesTranslator

from motion_spec_gen.utility.helpers import for_type

import warnings


@for_type(ACHD_SOLVER.ACHDSolver)
class ACHDSolverTranslator:

    def translate(self, g: rdflib.Graph, node, verbose=False, **kwargs) -> dict:
        verbose_padding: int = 0
        # Get the verbose padding from the kwargs
        if "verbose_padding" in kwargs:
            verbose_padding = kwargs["verbose_padding"]
        if verbose:
            print(
                f"{'-'*verbose_padding} Translating ACHD Solver: {g.compute_qname(node)[2]}"
            )

        variables = {}

        embed_maps = kwargs.get("embed_maps")
        d_vars = kwargs.get("variables")

        assert embed_maps is not None and len(embed_maps) > 0, "No embed maps provided"

        id = g.compute_qname(node)[2]

        # get embed maps for this solver
        embed_maps_for_solver = embed_maps[id]

        # root acceleration
        root_acc_vector_collec = g.value(node, SOLVER["root-acceleration"])
        root_acc_vector = list(Collection(g, root_acc_vector_collec))
        root_acc_vector = [float(q) for q in root_acc_vector]

        variables[f"{id}_root_acceleration"] = {
            "type": "array",
            "size": 6,
            "dtype": "double",
            "value": root_acc_vector,
        }

        # root and tip links
        root_link = g.value(node, ACHD_SOLVER["root-link"])
        tip_link = g.value(node, ACHD_SOLVER["tip-link"])

        root_link_name = g.compute_qname(root_link)[2]
        tip_link_name = g.compute_qname(tip_link)[2]

        variables[root_link_name] = {
            "type": None,
            "dtype": "string",
            "value": root_link_name,
        }

        variables[tip_link_name] = {
            "type": None,
            "dtype": "string",
            "value": tip_link_name,
        }

        alpha = np.array([])
        beta = []
        ext_wrench = []

        for embed_map in embed_maps_for_solver:
            vec = d_vars[embed_map["vector"]]["value"]
            local_alpha = None
            # if vec has values of 0 or 1, then form a diagonal matrix
            if all(v == 0 or v == 1 for v in vec):
                local_alpha = np.diag(vec)
            else:
                local_alpha = np.array(vec).reshape(6, -1)

            # remove zero columns
            local_alpha = local_alpha[:, ~np.all(local_alpha == 0, axis=0)]
            # convert each column as one array
            local_alpha = local_alpha.T

            if alpha.size == 0:
                alpha = local_alpha
            else:
                alpha = np.vstack((alpha, local_alpha))

            # output
            output = embed_map["output"]

            if embed_map["output_type"] == "acceleration-energy":
                variables[embed_map["output"]] = {
                    "type": "array",
                    "size": 6,
                    "dtype": "double",
                    "value": None,
                }
                # append to beta if not already present
                if output not in beta:
                    beta.append(output)

            if embed_map["output_type"] == "external-wrench":
                pass

        nc = alpha.shape[0]

        # num of joints TODO: get from the robot model
        nj = 7

        # solver variables
        # number of constraints
        variables[f"{id}_nc"] = {"type": None, "dtype": "int", "value": nc}
        # number of joints
        variables[f"{id}_nj"] = {"type": None, "dtype": "int", "value": nj}
        # alpha matrix
        variables[f"{id}_alpha"] = {
            "type": "array_2d",
            "rows": alpha.shape[1],
            "cols": alpha.shape[0],
            "dtype": "double",
            "value": alpha.tolist(),
        }
        # feed forward torques
        variables[f"{id}_feed_forward_torques"] = {
            "type": "array",
            "size": nj,
            "dim": 1,
            "dtype": "double",
            "value": None,
        }

        # output variables
        # predicted joint accelerations
        variables[f"{id}_predicted_accelerations"] = {
            "type": "array",
            "size": nj,
            "dim": 1,
            "dtype": "double",
            "value": None,
        }
        # output constraint torques
        variables[f"{id}_output_torques"] = {
            "type": "array",
            "size": nj,
            "dim": 1,
            "dtype": "double",
            "value": None,
        }

        data = {
            "name": "achd_solver",
            "root_acceleration": f"{id}_root_acceleration",
            "alpha": f"{id}_alpha",
            "beta": beta,
            "ext_wrench": ext_wrench,
            "nc": f"{id}_nc",
            "nj": f"{id}_nj",
            "ns": f"{id}_ns",
            "tau_ff": f"{id}_feed_forward_torques",
            "output_torques": f"{id}_output_torques",
            "predicted_accelerations": f"{id}_predicted_accelerations",
            "root_link": root_link_name,
            "tip_link": tip_link_name,
            "return": None,
        }

        return {
            "id": id,
            "data": data,
            "variables": variables,
        }


@for_type(ACHD_SOLVER.ACHDSolverFext)
class ACHDSolverFextTranslator:

    def translate(self, g: rdflib.Graph, node, verbose=False, **kwargs) -> dict:
        verbose_padding: int = 0
        # Get the verbose padding from the kwargs
        if "verbose_padding" in kwargs:
            verbose_padding = kwargs["verbose_padding"]
        if verbose:
            print(
                f"{'-'*verbose_padding} Translating ACHD Solver Fext: {g.compute_qname(node)[2]}"
            )

        variables = {}

        embed_maps = kwargs.get("embed_maps")
        d_vars = kwargs.get("variables")

        assert embed_maps is not None and len(embed_maps) > 0, "No embed maps provided"

        id = g.compute_qname(node)[2]

        # get embed maps for this solver
        embed_maps_for_solver = embed_maps[id]

        # root acceleration
        root_acc_vector_collec = g.value(node, SOLVER["root-acceleration"])
        root_acc_vector = list(Collection(g, root_acc_vector_collec))
        root_acc_vector = [float(q) for q in root_acc_vector]

        # variables[f"{id}_root_acceleration"] = {
        #     "type": "array",
        #     "size": 6,
        #     "dtype": "double",
        #     "value": root_acc_vector,
        # }

        # root and tip links
        root_link = g.value(node, ACHD_SOLVER["root-link"])
        tip_link = g.value(node, ACHD_SOLVER["tip-link"])

        root_link_name = g.compute_qname(root_link)[2]
        tip_link_name = g.compute_qname(tip_link)[2]

        variables[root_link_name] = {
            "type": None,
            "dtype": "string",
            "value": root_link_name,
        }

        variables[tip_link_name] = {
            "type": None,
            "dtype": "string",
            "value": tip_link_name,
        }

        ext_wrench = []

        for embed_map in embed_maps_for_solver:
            # *Assumption: ext_wrench is only for end-effector
            if embed_map["output_type"] == "external-wrench":
                variables[embed_map["output"]] = {
                    "type": "array",
                    "size": 6,
                    "dtype": "double",
                    "value": None,
                }

                link = None
                if "output_wrench_applied_to" in embed_map:
                    link = embed_map["output_wrench_applied_to"]
                elif "output_wrench_applied_by" in embed_map:
                    link = embed_map["output_wrench_applied_by"]

                asb = embed_map["asb"]

                ext_wrench.append(
                    {
                        "link": link,
                        "wrench": embed_map["output"],
                        "asb": asb,
                    }
                )

        # TODO: get num of joints from the robot model
        nj = 7

        # solver variables
        # number of joints
        variables[f"{id}_nj"] = {"type": None, "dtype": "int", "value": nj}

        # output variables
        # # predicted joint accelerations
        # variables[f"{id}_predicted_accelerations"] = {
        #     "type": "array",
        #     "size": nj,
        #     "dim": 1,
        #     "dtype": "double",
        #     "value": None,
        # }
        # output constraint torques
        variables[f"{id}_output_torques"] = {
            "type": "array",
            "size": nj,
            "dim": 1,
            "dtype": "double",
            "value": None,
        }

        data = {
            "name": "achd_solver_fext",
            # "root_acceleration": f"{id}_root_acceleration",
            "root_link": root_link_name,
            "tip_link": tip_link_name,
            "ext_wrench": ext_wrench,
            "nj": f"{id}_nj",
            "ns": f"{id}_ns",
            "output_torques": f"{id}_output_torques",
            "predicted_accelerations": None,
            "return": None,
        }

        return {
            "id": id,
            "data": data,
            "variables": variables,
        }


@for_type(BASE_FD_SOLVER.BaseFDSolver)
class BaseFDSolverTranslator:

    def translate(self, g: rdflib.Graph, node, verbose=False, **kwargs) -> dict:
        verbose_padding: int = 0
        # Get the verbose padding from the kwargs
        if "verbose_padding" in kwargs:
            verbose_padding = kwargs["verbose_padding"]
        if verbose:
            print(
                f"{'-'*verbose_padding} Translating Base FD Solver: {g.compute_qname(node)[2]}"
            )

        variables = {}
        data = {}

        embed_maps = kwargs.get("embed_maps")
        d_vars = kwargs.get("variables")

        assert embed_maps is not None and len(embed_maps) > 0, "No embed maps provided"

        id = g.compute_qname(node)[2]

        # get embed maps for this solver
        embed_maps_for_solver = embed_maps[id]

        # root acceleration
        root_acc_vector_collec = g.value(node, SOLVER["root-acceleration"])
        root_acc_vector = list(Collection(g, root_acc_vector_collec))
        root_acc_vector = [float(q) for q in root_acc_vector]

        variables[f"{id}_root_acceleration"] = {
            "type": "array",
            "size": 6,
            "dtype": "double",
            "value": root_acc_vector,
        }

        platform_force = []

        for embed_map in embed_maps_for_solver:
            if embed_map["output_type"] != "external-wrench":
                # raise warning and continue
                warnings.warn(
                    f"Base FD solver expects external-wrench as output, but got {embed_map['output_type']}"
                )
                continue

            variables[embed_map["output"]] = {
                "type": "array",
                "size": 6,
                "dtype": "double",
                "value": None,
            }

            # check transform
            tf = None
            if embed_map["vector_info"] is not None:
                if embed_map["vector_info"]["asb"] != "base_link":
                    tf = {
                        "from": embed_map["vector_info"]["asb"],
                        "to": "base_link",
                    }

            pf = {
                "wrench": embed_map["output"],
                "transform": tf,
            }

            platform_force.append(pf)

        # TODO: get the number of joints from the robot model
        variables[f"{id}_output_torques"] = {
            "type": "array",
            "size": 8,
            "dim": 1,
            "dtype": "double",
            "value": None,
        }

        data = {
            "robot": "freddy_base",
            "name": "base_fd_solver",
            "root_acceleration": f"{id}_root_acceleration",
            "platform_force": platform_force,
            "output_torques": f"{id}_output_torques",
            "predicted_accelerations": None,
            "return": None,
        }

        return {
            "id": id,
            "data": data,
            "variables": variables,
        }
