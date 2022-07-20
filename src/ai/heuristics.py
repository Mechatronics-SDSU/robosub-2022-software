"""Scion's decision making process code.
"""
import ai.mission_planner as scion_mis

# Parser instructions for the mission planner.
COMMENT = '!'
COMPARE = '#'
BREAK = '*'
ITERABLE = '@'
BEGIN_REGISTER = '{'
END_REGISTER = '}'
BEGIN_ROS_PARAM = '('
END_ROS_PARAM = ')'
BEGIN_VAR = "\""
END_VAR = "\""
PYTHON_FLOAT = 'f64'
PYTHON_INT = 'i64'
PYTHON_LIST = 'list'
PYTHON_LIST_SEP = ','
BEGIN_PYTHON_LIST = '['
END_PYTHON_LIST = ']'
COMP_EQ = '=='
COMP_NE = '!='
COMP_LE = '<='
COMP_GE = '>='
COMP_LT = '<<'
COMP_GT = '>>'


class Heuristics:
    def __init__(self, missions: str):
        self.missions = scion_mis.MissionSystem(load_from_file=missions)
        self.instructions = self.parse_instructions()

    def parse_instructions(self) -> list:
        pass

    def run_instruction(self, instruction: str) -> bool:
        return True


def heuristics_test():
    """Build up the mission planner, loop over mission planner taking parameters.
    """
    h = Heuristics('ai/heuristics.cfg')
    print(h.missions.tree)


if __name__ == '__main__':
    heuristics_test()
