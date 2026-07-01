import os
import numpy as np

from utils import driving_cycle_spd_profile_reader, traffic_online_MPC_control_step_nVeh
from _controller import (
    PCC_MPC_controller,
    USING_ONLINE_MPC,
    SIM_STEP,
    MPC_DT,
    MPC_REF_STAGES,
    REF_CYCLE_DT,
    REF_CYCLE_STAGES,
    BOOL_USE_FRONT_PREVIEW,
)


class LiveMPCController:
    """
    Thin adapter around the existing sumoOnly_v3.py MPC controller logic.

    Expected veh_states_matrix format is the same as sumoOnly_v3.py:
        [lane, acceleration, speed, position, ...]
    for vehicles ordered consistently with vehicle_list.
    """

    def __init__(self, scripts_dir=None, verbose=False):
        if scripts_dir is None:
            scripts_dir = os.path.dirname(__file__)

        self.scripts_dir = scripts_dir
        self.project_dir = os.path.abspath(os.path.join(scripts_dir, os.pardir))
        self.verbose = verbose

        spd_filename = os.path.join(
            self.project_dir,
            "speed_profile",
            "US06_CMI_Urban_speed_profile.csv",
        )

        leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)

        self.record_t = np.array(leading_vehicle_speed_profile[:, 0])
        self.front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
        self.front_s_t = np.array(leading_vehicle_speed_profile[:, 3]) + 20.0

        if USING_ONLINE_MPC:
            self.online_mpc_control = PCC_MPC_controller(dirname=scripts_dir)
        else:
            self.online_mpc_control = None

    def compute_accel(self, veh_states_matrix, sim_time, vehicle_id="nv1"):
        """
        Compute acceleration command for nv1 using the existing MPC logic.

        Returns:
            acc_cmd: float
            debug: dict with predictions and reference info
        """

        if not USING_ONLINE_MPC:
            return 0.0, {
                "acc": {vehicle_id: 0.0},
                "preds_s": {},
                "preds_v": {},
                "cycle_ss": None,
                "cycle_vs": None,
            }

        acc, preds_s, preds_v, cycle_ss, cycle_vs = traffic_online_MPC_control_step_nVeh(
            veh_states_matrix,
            sim_t=sim_time,
            record_t=self.record_t,
            front_v_t=self.front_v_t,
            online_MPC_control=self.online_mpc_control,
            simStep=SIM_STEP,
            mpc_dt=MPC_DT,
            mpc_ref_stages=MPC_REF_STAGES,
            cycle_dt=REF_CYCLE_DT,
            cycle_stages=REF_CYCLE_STAGES,
            PassIntention=BOOL_USE_FRONT_PREVIEW,
            outputUsedCycleforFront=True,
            verbose=self.verbose,
        )

        # Resolve acceleration command robustly.
        # In sumoOnly_v3.py, acc is usually keyed by vehicle ID, e.g. acc["nv1"].
        # In isolated/fake tests, the controller may return a different key structure.
        if isinstance(acc, dict):
            if vehicle_id in acc:
                acc_cmd = acc[vehicle_id]
            elif len(acc) == 1:
                acc_cmd = next(iter(acc.values()))
            else:
                # Try numeric suffix fallback: "nv1" -> 1
                try:
                    numeric_id = int("".join(ch for ch in vehicle_id if ch.isdigit()))
                except ValueError:
                    numeric_id = None

                if numeric_id is not None and numeric_id in acc:
                    acc_cmd = acc[numeric_id]
                else:
                    raise KeyError(
                        f"Could not find acceleration command for {vehicle_id}. "
                        f"Available acc keys: {list(acc.keys())}"
                    )
        else:
            acc_cmd = float(acc)

        return acc_cmd, {
            "acc": acc,
            "preds_s": preds_s,
            "preds_v": preds_v,
            "cycle_ss": cycle_ss,
            "cycle_vs": cycle_vs,
        }
