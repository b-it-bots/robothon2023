
#!/usr/bin/env python3
#
import rospy
from robothon2023.abstract_action import AbstractAction

class SliderAction(AbstractAction):

    def pre_perceive(self) -> bool:
        print ("in pre perceive")
        return True

    def act(self) -> bool:
        print ("in act")
        return True

    def verify(self) -> bool:
        print ("in verify")
        return True


