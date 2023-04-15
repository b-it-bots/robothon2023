#!/usr/bin/env python3
#
import rospy
from abc import ABC, abstractmethod
from typing import Type, TypeVar
from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils
from robothon2023.abstract_action import AbstractAction


class PlugRemoveSlidAction(AbstractAction):
    """
    Assumption: We have approximate pose of the plug .

    - Move arm to the top of the buttons
    - Do visual servoing and adjust arm
    - Open gripper
    - Move arm with velocity control
    - stope when the force is higher 
    - retreate above
    - close gripper
    - move up 
    - go to fixed insert position 
    - Move downwards with velocity control
    - Check force for 2 levels - (board collision and slid force)  needs to be identified
    - Stop if board collision is hit
    - retreat above and random sample x and y pose
    - if slid force is there for some amount of time then collision force comes then open gripper
    - 
    """
    def __init__(self, arm: FullArmMovement, transform_utils: TransformUtils) -> None:

    def pre_perceive(self) -> bool:
        pass

    def act(self) -> bool:
        pass

    def verify(self) -> bool:
        pass

