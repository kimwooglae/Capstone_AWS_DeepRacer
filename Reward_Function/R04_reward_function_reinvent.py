import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0  # None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        # import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum(
                [times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time /
                                  current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[2.87651, 0.7062, 2.2, 0.13257],
                        [3.16555, 0.69146, 2.2, 0.13155],
                        [3.44788, 0.68538, 2.2, 0.12836],
                        [3.73496, 0.68376, 2.2, 0.13049],
                        [4.07281, 0.68361, 2.2, 0.15357],
                        [4.5, 0.68376, 2.2, 0.19418],
                        [4.55, 0.68378, 2.0446, 0.02445],
                        [5.15168, 0.68865, 1.76297, 0.3413],
                        [5.474, 0.705, 1.5408, 0.20946],
                        [5.73199, 0.73075, 1.34524, 0.19273],
                        [5.96435, 0.7683, 1.18756, 0.1982],
                        [6.17932, 0.81992, 1.18756, 0.18617],
                        [6.37537, 0.88623, 1.18756, 0.17427],
                        [6.5488, 0.96778, 1.18756, 0.16138],
                        [6.69791, 1.06487, 1.12362, 0.15836],
                        [6.8211, 1.17876, 1.12362, 0.14931],
                        [6.91517, 1.31125, 1.1, 0.14772],
                        [6.98072, 1.45979, 1.1, 0.1476],
                        [7.01996, 1.6202, 1.1, 0.15013],
                        [7.02809, 1.79078, 1.1, 0.15525],
                        [6.99253, 1.96745, 1.1, 0.16382],
                        [6.91465, 2.14156, 1.1, 0.1734],
                        [6.77939, 2.29829, 1.35897, 0.15234],
                        [6.60667, 2.43101, 1.58787, 0.13718],
                        [6.4097, 2.53862, 1.86142, 0.12058],
                        [6.19847, 2.62412, 2.2, 0.10358],
                        [5.9799, 2.69364, 2.2, 0.10426],
                        [5.75821, 2.75456, 2.13521, 0.10768],
                        [5.55489, 2.80852, 2.13521, 0.09852],
                        [5.35299, 2.86607, 2.13521, 0.09832],
                        [5.15329, 2.92954, 2.13521, 0.09814],
                        [4.95645, 3.00143, 2.13521, 0.09814],
                        [4.76315, 3.08535, 2.13521, 0.09869],
                        [4.57389, 3.1868, 2.2, 0.09761],
                        [4.38736, 3.30265, 2.2, 0.09981],
                        [4.20281, 3.4295, 2.2, 0.10179],
                        [4.01964, 3.56387, 2.2, 0.10326],
                        [3.83713, 3.70184, 1.94456, 0.11766],
                        [3.68323, 3.81315, 1.94456, 0.09767],
                        [3.52812, 3.91858, 1.94456, 0.09645],
                        [3.37121, 4.01652, 1.94456, 0.09512],
                        [3.21171, 4.10525, 1.94456, 0.09386],
                        [3.04867, 4.18325, 1.94227, 0.09305],
                        [2.87998, 4.2471, 1.78494, 0.10105],
                        [2.70521, 4.29824, 1.63993, 0.11104],
                        [2.52331, 4.33633, 1.49272, 0.1245],
                        [2.33289, 4.36041, 1.3121, 0.14628],
                        [2.1318, 4.36807, 1.26378, 0.15924],
                        [1.91728, 4.3553, 1.26378, 0.17004],
                        [1.68411, 4.31169, 1.26378, 0.18771],
                        [1.4292, 4.21799, 1.26378, 0.2149],
                        [1.17078, 4.04749, 1.26378, 0.24498],
                        [0.96353, 3.77587, 1.26378, 0.27034],
                        [0.86997, 3.41697, 1.71396, 0.21639],
                        [0.85662, 3.06919, 2.10216, 0.16556],
                        [0.88074, 2.79223, 2.08336, 0.13344],
                        [0.91565, 2.56169, 1.86637, 0.12493],
                        [0.96322, 2.30877, 1.65821, 0.1552],
                        [1.01166, 2.10048, 1.47982, 0.14451],
                        [1.06864, 1.9001, 1.2846, 0.16217],
                        [1.13675, 1.70894, 1.2846, 0.15797],
                        [1.21698, 1.53087, 1.2846, 0.15203],
                        [1.30964, 1.3691, 1.2846, 0.14513],
                        [1.41585, 1.22571, 1.2846, 0.13891],
                        [1.53843, 1.10178, 1.2846, 0.1357],
                        [1.68762, 0.99961, 1.53899, 0.11749],
                        [1.86418, 0.91227, 1.77124, 0.11121],
                        [2.0718, 0.83841, 2.0478, 0.10761],
                        [2.31329, 0.77856, 2.2, 0.11309],
                        [2.58621, 0.73439, 2.2, 0.12567]]


        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 16
        FASTEST_TIME = 8
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME,
                               reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        REWARD_FOR_FASTEST_TIME = 500
        STANDARD_TIME = 16  # seconds (time that is easily done by model)
        FASTEST_TIME = 8  # seconds (best time of 1st place on the track)
        if progress > 99.5 :
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
