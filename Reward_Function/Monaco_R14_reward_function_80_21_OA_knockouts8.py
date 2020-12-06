import math


def dist_2_points(x1, x2, y1, y2):
    return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

def reward_function(params):

    x = params['x']
    y = params['y']

    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    objects_left_of_center = params['objects_left_of_center']
    is_left_of_center = params['is_left_of_center']

    previous_object_index, next_object_index = params['closest_objects']
    objects_location = params['objects_location']

    # Initialize reward with a small number but not zero
    # because zero means off-track or crashed
    reward = 1e-3

    # Reward if the agent stays inside the two borders of the track
    if all_wheels_on_track and (0.5 * track_width - distance_from_center) >= 0.05:
        reward_lane = 1.0
    else:
        reward_lane = 1e-3

    # Penalize if the agent is too close to the next object
    reward_avoid = 1.0

    distance_closest_previous_object = dist_2_points(x, objects_location[previous_object_index][0], y, objects_location[previous_object_index][1])
    distance_closest_next_object = dist_2_points(x, objects_location[next_object_index][0], y, objects_location[next_object_index][1])
    is_same_lane = objects_left_of_center[next_object_index] == is_left_of_center

    # Decide if the agent and the next object is on the same lane
    is_same_lane = objects_left_of_center[next_object_index] == is_left_of_center
    
    if is_same_lane:
        if 0.5 <= distance_closest_next_object < 0.8: 
            reward_avoid *= 0.5
        elif 0.3 <= distance_closest_next_object < 0.5:
            reward_avoid *= 0.2
        elif distance_closest_next_object < 0.3:
            reward_avoid = 1e-3 # Likely crashed

    # Calculate reward by putting different weights on 
    # the two aspects above
    reward += 1.0 * reward_lane + 4.0 * reward_avoid

    return reward
