#!/usr/bin/env python3
"""
This module contains a qui component for moving kinova arm

"""
# -*- encoding: utf-8 -*-

import os
import json
import math
import threading
import tkinter as Tkinter

import geometry_msgs.msg
import rospy
import tf
import visualization_msgs.msg

#from robothon2023.full_arm_movement import FullArmMovement
#from robothon2023.transform_utils import TransformUtils


class RobothonTask(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.joint_angles = rospy.get_param("~joint_angles", None)
        self.trajectories = rospy.get_param("~trajectories", None)
        self.wind_cable_poses = rospy.get_param("~wind_cable_poses", None)
        #self.arm = FullArmMovement()


    def send_joint_angle_listing(self, event):
        '''
        Get users selection and print to terminal.
        '''
        # Get the selected item in the listbox
        selected_item = event.widget.get(event.widget.curselection())
        print(selected_item)
        print (self.selected_value)
        pass

    def send_joint_angle_button(self):
        print ("Button ", self.selected_value)
        pass
        


    def create_window(self):
        master = Tkinter.Tk()

        master.title("Pose mock-up")

        joint_angle_label = Tkinter.Label(master, text="JOINT ANGLES")
        joint_angle_label.pack()
        print (self.joint_angles)


        # create listbox to hold names
        lb_of_joint_angles = Tkinter.Listbox(master, selectmode=Tkinter.BROWSE, width = 24)  # width is equal to number of characters
        lb_of_joint_angles.pack()

        # add items to listbox
        for joint_angle_name, value in self.joint_angles.items():
            lb_of_joint_angles.insert(Tkinter.END, joint_angle_name)

        # set binding on item select in listbox
        # when item of listbox is selected, call the function get_selection
        lb_of_joint_angles.bind("&lt;&lt;ListboxSelect&gt;&gt;", lambda event: self.send_joint_angle_listing())
        # Bind the <<ListboxSelect>> event to the on_select function
        lb_of_joint_angles.bind("<<ListboxSelect>>", self.send_joint_angle_listing)

        # Create an IntVar to store the selected value
        self.selected_value = Tkinter.IntVar()

        # Create the radio buttons
        for i, joint_angle_name in enumerate(self.joint_angles):
            print (joint_angle_name)
            radio_button_1 = Tkinter.Radiobutton(master, text=joint_angle_name, variable=self.selected_value, value=i)
            # Pack the radio buttons
            radio_button_1.pack()


        # Create a button widget
        button = Tkinter.Button(master, text="Send Joint Angle", command=self.send_joint_angle_button)

        # Pack the button
        button.pack()

        trajectory_label = Tkinter.Label(master, text="TRAJECTORIES")
        trajectory_label.pack()
        # create listbox to hold names
        lb_of_trajectories = Tkinter.Listbox(master, selectmode=Tkinter.BROWSE, width = 24)  # width is equal to number of characters
        lb_of_trajectories.pack()

        # add items to listbox
        for traj, value in self.trajectories.items():
            lb_of_trajectories.insert(Tkinter.END, traj)

        # Bind the <<ListboxSelect>> event to the on_select function
        lb_of_trajectories.bind("<<ListboxSelect>>", self.send_joint_angle_listing)

        wind_cabel_label = Tkinter.Label(master, text="WIND CABLE POSES")
        wind_cabel_label.pack()
        # create listbox to hold names
        lb_of_wind_cable_poses = Tkinter.Listbox(master, selectmode=Tkinter.BROWSE, width = 24)  # width is equal to number of characters
        lb_of_wind_cable_poses.pack()

        # add items to listbox
        for traj, value in self.wind_cable_poses.items():
            lb_of_wind_cable_poses.insert(Tkinter.END, traj)

        # Bind the <<ListboxSelect>> event to the on_select function
        lb_of_wind_cable_poses.bind("<<ListboxSelect>>", self.send_joint_angle_listing)

        master.mainloop()
        rospy.signal_shutdown("GUI closed")

if __name__ == "__main__":
    rospy.init_node('robothon_task')
    task = RobothonTask()
    task.create_window()
    print ("crate window finished")
    #task.test()
    #PAP.test_go_to_board()
    #PAP.test_press_button()
    rospy.spin()

