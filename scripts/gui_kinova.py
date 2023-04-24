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
from tkinter import messagebox

import geometry_msgs.msg
import rospy
import tf
import visualization_msgs.msg

from typing import List, Tuple, Dict, Any, Union, Optional

from robothon2023.full_arm_movement import FullArmMovement
from robothon2023.transform_utils import TransformUtils

from utils.kinova_pose import KinovaPose, get_kinovapose_from_list, get_kinovapose_from_pose_stamped

from kortex_driver.srv import *
from kortex_driver.msg import *

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from typing import List, Optional

class Item:
    """A class that represents an item with a name and a value."""
    def __init__(self, name: str, value: List):
        """
        Initializes an instance of the Item class with a given name and value.

        Args:
            name (str): The name of the item.
            value (List): The value of the item.
        """
        self.name = name
        self.value = value

    def __str__(self):
        return f'Item(name={self.name}, value={self.value})'

class ItemList:
    """A class that represents a list of items and provides methods to interact with the list."""
    def __init__(self):
        """
        Initializes an instance of the ItemList class with an empty list of items.
        """
        self.items: List[Item] = []

    def add_item(self, item: Item) -> None:
        """
        Adds an Item object to the item list.

        Args:
            item (Item): The Item object to be added to the item list.
        """
        self.items.append(item)

    def add_items(self, items: List[Tuple[str, Any]]) -> None:
        """
        Adds a list of Item objects to the item list.

        Args:
            items (list): A list of Item objects to be added to the item list.
        """
        for name, value in items:
            item = Item(name, value)
            self.items.append(item)

    def get_item_by_name(self, name: str) -> Optional[Item]:
        """
        Retrieves an Item object from the item list by name.

        Args:
            name (str): The name of the item to be retrieved.

        Returns:
            Item or None: The Item object with the given name, or None if not found.
        """
        for item in self.items:
            if item.name == name:
                return item
        return None

    def get_all_items(self) -> List[Item]:
        """
        Retrieves all Item objects from the item list.

        Returns:
            list: A list of all Item objects in the item list.
        """
        return self.items
    
    def __iter__(self):
        """
        Returns an iterator over the items in the item list.

        Returns:
            iterator: An iterator over the items in the item list.
        """
        return iter(self.items)

class ListWindow(tk.Frame):
    def __init__(self, master, items: ItemList, callback):
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
            self.listbox.insert(tk.END, item.name)

        # Create button with modern style
        self.button = ttk.Button(self, text="Go", command=self._on_button_click)
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
            selected_item_name = self.listbox.get(cs)

            # Call the callback function with the selected item as argument
            self.callback(self.items.get_item_by_name(selected_item_name))

class RobothonTask(object):

    """pick and place code using full arm movement and 3d segmentation perception"""

    def __init__(self):
        self.joint_angles = rospy.get_param("~joint_angles", None)
        self.trajectories = rospy.get_param("~trajectories", None)
        self.wind_cable_poses = rospy.get_param("~wind_poses", None)

        self.lists = [self.joint_angles, self.trajectories, self.wind_cable_poses]

        self.arm = FullArmMovement()
        self.transform_utils = TransformUtils()

    def render_lists(self, frame: tk.Frame):
        joint_angles = [(joint_angle, self.joint_angles[joint_angle]) for joint_angle in self.joint_angles.keys()]

        joint_anlges_list = ItemList()
        joint_anlges_list.add_items(joint_angles)

        joint_angles_window = ListWindow(frame, joint_anlges_list, self.joint_angles_cb)
        joint_angles_window.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        trajectories = [(trajectory, self.trajectories[trajectory]) for trajectory in self.trajectories.keys()]

        trajectories_list = ItemList()
        trajectories_list.add_items(trajectories)

        trajectories_window = ListWindow(frame, trajectories_list, self.trajectories_cb)
        trajectories_window.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # get the poses from trajectories
        trajs = [self.wind_cable_poses[trajectory]['poses'] for trajectory in self.wind_cable_poses]
        
        wind_cable_poses = [(pose, traj[pose]) for traj in trajs for pose in traj.keys()]
        
        wind_cable_poses_list = ItemList()
        wind_cable_poses_list.add_items(wind_cable_poses)
        
        wind_cable_poses_window = ListWindow(frame, wind_cable_poses_list, self.wind_cable_poses_cb)
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

    def joint_angles_cb(self, item: Item):
        
        self.arm.send_joint_angles(item.value)

        messagebox.showinfo("Joint Angles", "Joint Angles sent to robot")

    def trajectories_cb(self, item: Item):
        print ("Trajectories button ", item)
        pass

    def wind_cable_poses_cb(self, item: Item):
        
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = item.value['position']['x']
        msg.pose.position.y = item.value['position']['y']
        msg.pose.position.z = item.value['position']['z']
        msg.pose.orientation.x = item.value['orientation']['x']
        msg.pose.orientation.y = item.value['orientation']['y']
        msg.pose.orientation.z = item.value['orientation']['z']
        msg.pose.orientation.w = item.value['orientation']['w']

        kp = get_kinovapose_from_pose_stamped(msg)

        success = self.arm.send_cartesian_pose(kp)

        # display popup window to inform user
        messagebox.showinfo("Wind Cable Pose", "Wind Cable Pose sent to robot")


    def current_pose_window(self, frame: tk.Frame):
        # Create a heading for the frame
        self.heading = tk.Label(frame, text="Current Tool Tip Pose", font=("Helvetica", 14))
        self.heading.grid(row=0, column=0, padx=10, pady=10)

        # Create a text box to display the current tool tip pose in base frame
        # make the text box expand width based on the window size
        self.base_frame_text = tk.Text(frame, height=2, width=30, font=("Helvetica", 12))
        self.base_frame_text.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        self.base_frame_text.config(state='disabled')

        # Create a text box to display the current tool tip pose in board frame
        # make the text box expand width based on the window size
        self.board_frame_text = tk.Text(frame, height=2, width=30, font=("Helvetica", 12))
        self.board_frame_text.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        self.board_frame_text.config(state='disabled')

        # Create a text box to display the current tool tip pose in joint frame
        # make the text box expand width based on the window size
        self.joint_angles_text = tk.Text(frame, height=2, width=30, font=("Helvetica", 12))
        self.joint_angles_text.grid(row=3, column=0, padx=10, pady=10, sticky="ew")
        self.joint_angles_text.config(state='disabled')

        # Create an update button
        self.update_button = ttk.Button(frame, text="Update", command=self.update_tool_tip_pose)
        self.update_button.grid(row=4, column=0, padx=10, pady=10, columnspan=2) 

        # Set button style
        style = ttk.Style()
        style.configure("TButton", font=("Helvetica", 12))
        style.map("TButton",
                background=[("active", "#0056b3"), ("selected", "#0056b3")],
                foreground=[("active", "#ffffff"), ("selected", "#ffffff")])
        
        frame.grid_columnconfigure(0, weight=1)

    def update_tool_tip_pose(self):
        # Function to update the current tool tip pose in three formats
        # Get the current tool tip pose
        current_pose = self.arm.get_current_pose()

        cp_kp = get_kinovapose_from_list(current_pose)
        cp_in_board_frame = self.transform_utils.transformed_pose_with_retries(cp_kp, "board_link")

        # get joint angles from joint state publisher
        msg = rospy.wait_for_message("/joint_states", JointState)
        joint_angles = msg.position
        
        # Update the text boxes with the current pose
        self.base_frame_text.config(state='normal')
        self.base_frame_text.delete('1.0', tk.END)
        self.base_frame_text.insert(tk.END, current_pose)
        self.base_frame_text.config(state='disabled')

        self.board_frame_text.config(state='normal')
        self.board_frame_text.delete('1.0', tk.END)
        self.board_frame_text.insert(tk.END, cp_in_board_frame)
        self.board_frame_text.config(state='disabled')

        self.joint_angles_text.config(state='normal')
        self.joint_angles_text.delete('1.0', tk.END)
        self.joint_angles_text.insert(tk.END, joint_angles)
        self.joint_angles_text.config(state='disabled')

    def create_window(self):
        master = tk.Tk()  # Updated to use tk instead of Tkinter
        master.title("Kinova Arm GUI")
        master.geometry("1000x1000")

        lists_frame = tk.Frame(master)
        lists_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")  # Updated to use grid manager
        master.grid_rowconfigure(0, weight=2)  # Configure row 0 to take twice the height

        # Create and configure the window's contents
        self.render_lists(lists_frame)

        pose_frame = tk.Frame(master)
        pose_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")  # Updated to use grid manager
        master.grid_rowconfigure(1, weight=1)  # Configure row 1 to take normal height

        # Call the current_pose_window method with the pose_frame as the parent
        self.current_pose_window(pose_frame)

        # Configure the columns to take 100% width
        master.grid_columnconfigure(0, weight=1)

        master.mainloop()
        rospy.signal_shutdown("GUI closed")


if __name__ == "__main__":
    rospy.init_node('robothon_task')
    task = RobothonTask()
    task.create_window()
    print ("crate window finished")
    rospy.spin()

