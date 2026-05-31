from __future__ import annotations

from typing import List, TYPE_CHECKING
from f110_msgs.msg import Wpnt
from state_machine.state_types import StateType
if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine

def DefaultStateLogic(state_machine: StateMachine) -> List[Wpnt]:
    """
    This is a global state that incorporates the other states
    """
    match state_machine.state:
        case StateType.GB_TRACK:
            return GlobalTracking(state_machine)
        case StateType.TRAILING:
            return Trailing(state_machine)
        case StateType.OVERTAKE:
            return Overtaking(state_machine)
        case StateType.FTGONLY:
            return FTGOnly(state_machine)

        case StateType.TRAILING_TO_GBTRACK:
            return Trailing_to_gbtrack(state_machine)

        case _:
            raise NotImplementedError(f"State {state_machine.state} not recognized")

"""
Here we define the behaviour in the different states.
Every function should be fairly concise, and output an array of f110_msgs.Wpnt
"""
def GlobalTracking(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Trailing_to_gbtrack(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Trailing(state_machine: StateMachine) -> List[Wpnt]:
    # This allows us to trail on the last valid spline if necessary
    if state_machine.last_valid_avoidance_wpnts is not None:
        print("exist valid_avoidance_wpts")
        splini_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]
    else:
        print("No valid_avoidance_wpts")
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

# def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
#     splini_wpts = state_machine.get_splini_wpts()
#     s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
#     return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]


def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
    # 1) 회피용 스플라인 웨이포인트 가져오기
    splini_wpts = state_machine.get_splini_wpts()

    # 2) 현재 s에서부터 로컬 웨이포인트 개수만큼 인덱싱
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)

    out: List[Wpnt] = []
    for i in range(state_machine.params.n_loc_wpnts):
        idx = (s + i) % state_machine.num_glb_wpnts

        ot_speed_scaling = 0.8

        # 스플라인에서 기하/트랙 정보 가져오고
        src = splini_wpts[idx]
        # 전역 웨이포인트에서 속도만 가져온다
        vx_from_glb = state_machine.glb_wpnts[idx].vx_mps

        wp = Wpnt()
        # ----- splini_wpts 값들로 채우기 -----
        wp.id = src.id

        # frenet
        wp.s_m = src.s_m
        wp.d_m = src.d_m

        # map
        wp.x_m = src.x_m
        wp.y_m = src.y_m

        # track bound distance
        wp.d_right = src.d_right
        wp.d_left  = src.d_left

        # track info (psi, kappa, ax 등은 스플라인 값 유지)
        wp.psi_rad      = src.psi_rad
        wp.kappa_radpm  = src.kappa_radpm
        wp.ax_mps2      = src.ax_mps2

        # ----- 속도만 glb_wpnts 값으로 덮어쓰기 ----- (ot_speed_scaling은 아직 파라미터로 안만들었음. 일단 코드 내에서 변경해보면서 관찰)
        wp.vx_mps = vx_from_glb * ot_speed_scaling

        out.append(wp)

    return out

def FTGOnly(state_machine: StateMachine) -> List[Wpnt]:
    """No waypoints are generated in this follow the gap only state, all the 
    control inputs are generated in the control node.
    """
    return []