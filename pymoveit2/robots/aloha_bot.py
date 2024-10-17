from typing import List

MOVE_GROUP_ARM: str = "arm_group"

prefix: str = ""



def joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "arm_joint_1",
        prefix + "arm_joint_2",
    ]


def base_link_name(prefix: str = prefix) -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = prefix) -> str:
    return prefix + "end_link"


