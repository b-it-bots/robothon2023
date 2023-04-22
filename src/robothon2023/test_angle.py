
import math
import numpy as np

# angle = 0

# radius = 0.055
# arc_length = (2 * math.pi * radius) /4
# travel_time = 1
# angular_velocity_door = arc_length / travel_time
# # angular_velocity_door *= 10
# travel_time = 15

# prev_theta = 0

# for t in range(travel_time):
    
#     print("time==>: ", t, "s")

#     # angle += np.deg2rad(30/travel_time)

#     angle_feedback = np.rad2deg(angular_velocity_door * t)


#     # print("angluar velocity: ", angular_velocity_door, "rad/s")
#     # print("angle_feedback: ", angle_feedback)
#     # print("angle: ", angle)

#     angle = np.deg2rad(angle_feedback)

#     print("angle: ", angle)

#     # angular roataion calculation
#     z = radius * math.cos(angle)
#     y = radius * math.sin(angle)

#     theta = np.arctan2(z, y)
#     print("theta: ", theta)
#     current_theta = theta

#     #find the rate of change of theta
#     d_theta = current_theta - prev_theta
#     prev_theta = current_theta
#     print("d_theta: ", d_theta)

#     #angular velocity in x calculation
#     angular_velocity_x = d_theta 


#     # update step
#     linear_velocity_z = radius * angular_velocity_door * math.cos(angle)
#     linear_velocity_x = radius * angular_velocity_door * math.sin(angle) + (radius * (angular_velocity_door)**2)
#             # if(t > 5 and t < 10):



#     # print("linear_velocity_x: ", linear_velocity_x)
#     # print("linear_velocity_z: ", linear_velocity_z)
    
#     print("++++++++++++++++++++++++++++++"+"\n")
    
#     print("++++++++++++++++++++++++++++++"+"\n")


#     # if(t > 5 and t < 10):


def garbage():


    angle = 0

    radius = 0.07

    radius_y = 0.07
    radius_z = 0.15

    arc_length = (2 * math.pi * radius) /4
    travel_time = 1
    angular_velocity_door = arc_length / travel_time

    # angular_velocity_door *= 10
    travel_time = 15

    prev_theta = 0

    for t in range(travel_time):
        
        print("time==>: ", t, "s")
        angle_feedback = np.rad2deg(angular_velocity_door * t)

        print("angluar velocity of the door : ", angular_velocity_door, "rad/s")
        print("angle_feedback of the motion : ", angle_feedback)
        angle = np.deg2rad(angle_feedback)


        #ellipse motion

        velocity_door_ellipse = angular_velocity_door * math.sqrt( radius_z**2 * math.cos(angle)**2 +
                                                                    radius_y**2 * math.sin(angle)**2)

        omega_ =  (angular_velocity_door /  math.sqrt( radius_z**2 * math.cos(angle)**2 +
                                                    radius_y**2 * math.sin(angle)**2))


        # y and z calculation
        y = omega_ * radius_y * math.cos(angle)
        z = omega_ * radius_z * math.sin(angle)

        #theta calculation
        theta = omega_ * t  


        # velocity in y and z calculation
        vy = -radius_y * math.sin(angle) * velocity_door_ellipse
        vz = radius_z * math.cos(angle) * velocity_door_ellipse

        # assignment step

        linear_velocity_y = vy
        linear_velocity_z = vz



        # # angular roataion calculation
        # z = radius * math.cos(angle)
        # y = radius * math.sin(angle)

        # theta = np.arctan2(z, y)
        # print("theta of perpendicular vector: ", theta)
        # current_theta = theta

        # #find the rate of change of theta
        # d_theta = current_theta - prev_theta
        # prev_theta = current_theta
        # print("rate of change in angle: ", d_theta)

        # #angular velocity in x calculation
        # angular_velocity_x = d_theta /4 

        # # update step
        # linear_velocity_z = radius * angular_velocity_door * math.cos(angle)
        # linear_velocity_y = radius * angular_velocity_door * math.sin(angle) + (radius * (angular_velocity_door)**2)

        # #conditions 
        # if t == 0:
        #     angular_velocity_x = 0
        
        print("++++++++++++++++++++++++++++++")
        # printing step 
        print("linear_velocity_y: ", linear_velocity_y, "m/s")
        print("linear_velocity_z: ", linear_velocity_z, "m/s")
        # print("angular_velocity_x: ", angular_velocity_x, "rad/s")
        
        print("++++++++++++++++++++++++++++++"+"\n")


def ellipse_test():

    angle = 0
    radius = 0.07
    radius_y = 0.07
    radius_z = 0.15
    arc_length = (2 * math.pi * radius) /4
    travel_time = 1
    angular_velocity_door = arc_length / travel_time
    angular_velocity_door *= 10
    travel_time = 15

    for t in range(travel_time):
        
        print("time==>: ", t, "s")
        # angle_feedback = np.rad2deg(angular_velocity_door * t)

        print("angluar velocity of the door : ", angular_velocity_door, "rad/s")
        # print("angle_feedback of the motion : ", angle_feedback)
        # angle = np.deg2rad(angle_feedback)

        angle += np.deg2rad(90/travel_time)

        #ellipse motion

        velocity_door_ellipse = angular_velocity_door * math.sqrt( radius_z**2 * math.cos(angle)**2 +
                                                                    radius_y**2 * math.sin(angle)**2)

        omega_ =  (angular_velocity_door /  math.sqrt( radius_z**2 * math.cos(angle)**2 +
                                                    radius_y**2 * math.sin(angle)**2))


        # y and z calculation
        y = omega_ * radius_y * math.cos(angle)
        z = omega_ * radius_z * math.sin(angle)

        #theta calculation
        theta = omega_ * t  


        # velocity in y and z calculation
        vy = -radius_y * math.sin(angle) * velocity_door_ellipse
        vz = radius_z * math.cos(angle) * velocity_door_ellipse

        # assignment step

        linear_velocity_y = vy
        linear_velocity_z = vz


        print("++++++++++++++++++++++++++++++")
        # printing step 
        print("linear_velocity_y: ", linear_velocity_y, "m/s")
        print("linear_velocity_z: ", linear_velocity_z, "m/s")

        print("++++++++++++++++++++++++++++++"+"\n")


if __name__ == "__main__":
    ellipse_test()