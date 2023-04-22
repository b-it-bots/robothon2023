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

import tkinter as tk
from tkinter import ttk

import geometry_msgs.msg
import rospy
import tf
import visualization_msgs.msg

#from robothon2023.full_arm_movement import FullArmMovement
#from robothon2023.transform_utils import TransformUtils

class ListWindow(tk.Frame):
    def __init__(self, master, items, callback):
        super().__init__(master)
        self.master = master
        self.items = items
        self.callback = callback

        # Create listbox with a scrollbar
        self.listbox = tk.Listbox(self, width=30)
        self.listbox.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.scrollbar = ttk.Scrollbar(self.listbox)
        self.listbox.config(yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.listbox.yview)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Add items to listbox
        for item in self.items:
            self.listbox.insert(tk.END, item)

        # Create button with modern style
        self.button = ttk.Button(self, text="Button", command=self._on_button_click)
        self.button.pack(padx=5, pady=5)

        # Set listbox style
        style = ttk.Style()
        style.configure("TListbox", font=("Helvetica", 12), background="#f0f0f0", foreground="#000000", borderwidth=0,
                        highlightthickness=0)
        style.map("TListbox", background=[("selected", "#0078d7")], foreground=[("selected", "#ffffff")])

        # Set button style
        style.configure("TButton", font=("Helvetica", 12))
        style.map("TButton",
                  background=[("active", "#0056b3"), ("selected", "#0056b3")],
                  foreground=[("active", "#ffffff"), ("selected", "#ffffff")])

    def _on_button_click(self):
        # get the current selection
        cs = self.listbox.curselection()

        if cs == ():
            pass
        else:
            # Get selected item from listbox
            selected_item = self.listbox.get(cs)

            # Call the callback function with the selected item as argument
            self.callback(selected_item)


class RobothonTask(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.joint_angles = rospy.get_param("/joint_angles", None)
        self.trajectories = rospy.get_param("/trajectories", None)
        self.wind_cable_poses = rospy.get_param("/wind_poses", None)

        self.lists = [self.joint_angles, self.trajectories, self.wind_cable_poses]

        #self.arm = FullArmMovement()

    def render_lists(self, root):
        joint_angles_window = ListWindow(root, self.joint_angles, self.joint_angles_cb)
        joint_angles_window.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        trajectories_window = ListWindow(root, self.trajectories, self.trajectories_cb)
        trajectories_window.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        wind_cable_poses_window = ListWindow(root, self.wind_cable_poses, self.wind_cable_poses_cb)
        wind_cable_poses_window.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)    

    def send_joint_angle_listing(self, event):
        '''
        Get users selection and print to terminal.
        '''
        # Get the selected item in the listbox
        selected_item = event.widget.get(event.widget.curselection())
        print(selected_item)
        print (self.selected_value)
        pass

    def joint_angles_cb(self, item):
        print ("Joint angles button ", item)
        pass

    def trajectories_cb(self, item):
        print ("Trajectories button ", item)
        pass

    def wind_cable_poses_cb(self, item):
        print ("Wind cable poses button ", item)
        pass

    def create_window(self):
        master = Tkinter.Tk()
        master.title("Kinova Arm GUI")
        master.geometry("1000x600")

        # Create and configure the window's contents
        self.render_lists(master)

        master.mainloop()
        rospy.signal_shutdown("GUI closed")

if __name__ == "__main__":
    rospy.init_node('robothon_task')
    task = RobothonTask()
    task.create_window()
    print ("crate window finished")
    rospy.spin()

