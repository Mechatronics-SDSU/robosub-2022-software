"""Scion's decision making process code.
"""
import ai.mission_planner as scion_mis


class Heuristics:
    def __init__(self, missions: str):
        self.missions = scion_mis.MissionSystem(load_from_file=missions)


def heuristics_test():
    """Build up the mission planner, loop over mission planner taking parameters.
    """
    h = Heuristics('ai/heuristics.cfg')
    print(h.missions.tree)


if __name__ == '__main__':
    heuristics_test()
