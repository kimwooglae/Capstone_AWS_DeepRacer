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

        # Optimal racing line for the Austin track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-8.71717, -1.36222, 5.2, 0.05636],
                        [-8.58469, -1.62367, 4.99115, 0.05872],
                        [-8.45199, -1.88501, 4.07717, 0.07189],
                        [-8.31933, -2.14636, 3.52741, 0.08309],
                        [-8.18681, -2.40777, 3.1506, 0.09302],
                        [-8.05419, -2.66872, 2.86892, 0.10203],
                        [-7.91711, -2.92268, 2.64823, 0.10898],
                        [-7.77187, -3.16316, 2.47012, 0.11373],
                        [-7.61619, -3.38424, 2.3189, 0.11661],
                        [-7.44927, -3.58107, 2.18543, 0.11809],
                        [-7.27153, -3.75004, 2.06925, 0.11852],
                        [-7.08423, -3.88866, 2.06925, 0.11261],
                        [-6.88913, -3.99516, 2.06925, 0.10742],
                        [-6.68826, -4.06812, 2.06925, 0.10327],
                        [-6.48392, -4.1058, 2.06925, 0.10042],
                        [-6.27879, -4.10565, 2.06925, 0.09913],
                        [-6.07651, -4.06369, 2.26879, 0.09106],
                        [-5.87979, -3.98591, 2.41356, 0.08764],
                        [-5.69093, -3.87445, 2.58796, 0.08474],
                        [-5.51215, -3.73139, 2.80546, 0.08162],
                        [-5.34557, -3.5596, 3.08188, 0.07764],
                        [-5.1929, -3.36341, 3.44686, 0.07212],
                        [-5.05478, -3.14866, 3.74188, 0.06824],
                        [-4.92323, -2.97317, 3.91787, 0.05598],
                        [-4.78334, -2.80819, 4.1163, 0.05255],
                        [-4.63519, -2.65146, 4.34531, 0.04963],
                        [-4.47861, -2.50111, 4.6045, 0.04714],
                        [-4.3131, -2.35554, 4.91641, 0.04483],
                        [-4.13786, -2.21314, 5.2, 0.04342],
                        [-3.95149, -2.07212, 5.2, 0.04494],
                        [-3.75149, -1.93012, 5.2, 0.04717],
                        [-3.53286, -1.78336, 5.2, 0.05064],
                        [-3.28636, -1.62644, 5.2, 0.05619],
                        [-3.06004, -1.49064, 5.2, 0.05076],
                        [-2.84267, -1.3679, 5.2, 0.04801],
                        [-2.62938, -1.25502, 5.09613, 0.04735],
                        [-2.41789, -1.15066, 4.94832, 0.04766],
                        [-2.20695, -1.05429, 4.81214, 0.04819],
                        [-1.99581, -0.96573, 4.67596, 0.04896],
                        [-1.78402, -0.88501, 4.54372, 0.04988],
                        [-1.57129, -0.81229, 4.41067, 0.05097],
                        [-1.35748, -0.74781, 4.28336, 0.05214],
                        [-1.14255, -0.69192, 4.16379, 0.05334],
                        [-0.92656, -0.64499, 4.03687, 0.05475],
                        [-0.70961, -0.60747, 3.9131, 0.05626],
                        [-0.4919, -0.57985, 3.9131, 0.05608],
                        [-0.27366, -0.56267, 3.9131, 0.05594],
                        [-0.05514, -0.55653, 3.9131, 0.05587],
                        [0.16346, -0.56206, 3.9131, 0.05588],
                        [0.38214, -0.58006, 3.9131, 0.05607],
                        [0.60145, -0.61166, 4.48216, 0.04943],
                        [0.82234, -0.65416, 4.85136, 0.04637],
                        [1.04599, -0.70676, 5.2, 0.04418],
                        [1.27358, -0.76869, 5.2, 0.04536],
                        [1.50601, -0.83914, 5.2, 0.04671],
                        [1.74353, -0.9169, 5.2, 0.04806],
                        [1.98542, -1.00021, 5.2, 0.0492],
                        [2.22996, -1.08665, 5.2, 0.04988],
                        [2.49964, -1.18103, 5.2, 0.05494],
                        [2.77012, -1.27361, 5.2, 0.05498],
                        [3.0416, -1.3634, 5.2, 0.05499],
                        [3.31409, -1.44937, 5.2, 0.05495],
                        [3.58748, -1.53051, 5.2, 0.05484],
                        [3.86159, -1.60586, 5.2, 0.05467],
                        [4.13625, -1.67453, 5.2, 0.05444],
                        [4.41134, -1.73577, 5.2, 0.0542],
                        [4.68681, -1.78882, 5.2, 0.05395],
                        [4.96266, -1.83302, 5.05744, 0.05524],
                        [5.23891, -1.86773, 4.92017, 0.05659],
                        [5.51557, -1.89234, 4.76914, 0.05824],
                        [5.79265, -1.90623, 4.62811, 0.05994],
                        [6.07011, -1.90875, 4.49032, 0.06179],
                        [6.3479, -1.8992, 4.35708, 0.06379],
                        [6.62593, -1.8768, 4.22679, 0.06599],
                        [6.90407, -1.84063, 4.09208, 0.06854],
                        [7.18215, -1.78954, 3.57061, 0.07918],
                        [7.45987, -1.72222, 3.19516, 0.08944],
                        [7.7368, -1.63709, 2.91203, 0.09949],
                        [8.01215, -1.5324, 2.6925, 0.10941],
                        [8.2792, -1.40893, 2.51359, 0.11705],
                        [8.52699, -1.27161, 2.36446, 0.11981],
                        [8.75043, -1.11943, 2.23016, 0.12122],
                        [8.94567, -0.95256, 2.11309, 0.12155],
                        [9.10996, -0.77211, 2.11309, 0.11549],
                        [9.24131, -0.57981, 2.11309, 0.11021],
                        [9.33797, -0.37774, 2.11309, 0.10601],
                        [9.39793, -0.16826, 2.11309, 0.10311],
                        [9.4182, 0.04577, 2.11309, 0.10174],
                        [9.39449, 0.2603, 2.25493, 0.09572],
                        [9.33122, 0.47106, 2.38065, 0.09243],
                        [9.23068, 0.67465, 2.52375, 0.08997],
                        [9.09473, 0.86801, 2.6946, 0.08772],
                        [8.92542, 1.04837, 2.9012, 0.08527],
                        [8.72533, 1.21335, 3.16116, 0.08204],
                        [8.49806, 1.36133, 3.50428, 0.07739],
                        [8.2483, 1.49195, 3.98761, 0.07068],
                        [7.98169, 1.60648, 4.74276, 0.06118],
                        [7.70409, 1.70802, 5.2, 0.05684],
                        [7.42077, 1.80116, 5.2, 0.05735],
                        [7.13575, 1.89153, 5.2, 0.0575],
                        [6.85067, 1.98169, 5.2, 0.0575],
                        [6.56562, 2.07196, 5.2, 0.0575],
                        [6.28054, 2.16217, 5.2, 0.0575],
                        [5.99549, 2.25242, 5.2, 0.0575],
                        [5.71042, 2.34264, 5.2, 0.0575],
                        [5.42536, 2.43288, 5.2, 0.0575],
                        [5.14034, 2.52321, 5.2, 0.0575],
                        [4.85544, 2.61382, 5.2, 0.05749],
                        [4.57065, 2.70468, 5.2, 0.05749],
                        [4.28595, 2.79577, 5.2, 0.05748],
                        [4.00137, 2.88711, 5.2, 0.05748],
                        [3.7169, 2.9787, 5.09385, 0.05867],
                        [3.43253, 3.07054, 4.45751, 0.06704],
                        [3.14828, 3.16264, 4.00943, 0.07452],
                        [2.86812, 3.25208, 3.669, 0.08016],
                        [2.60114, 3.33197, 3.39912, 0.08199],
                        [2.35094, 3.39861, 3.17661, 0.08151],
                        [2.11779, 3.45067, 2.98839, 0.07994],
                        [1.90038, 3.48817, 2.82641, 0.07806],
                        [1.69692, 3.51163, 2.67516, 0.07656],
                        [1.50568, 3.52168, 2.67516, 0.07159],
                        [1.32508, 3.51886, 2.67516, 0.06752],
                        [1.1538, 3.50348, 2.67516, 0.06428],
                        [0.99077, 3.47558, 2.67516, 0.06183],
                        [0.83507, 3.43483, 2.67516, 0.06016],
                        [0.68598, 3.38033, 3.14963, 0.0504],
                        [0.54097, 3.3159, 3.3718, 0.04706],
                        [0.39924, 3.24258, 3.64811, 0.04374],
                        [0.26016, 3.1614, 3.26557, 0.04932],
                        [0.12318, 3.07338, 2.67952, 0.06076],
                        [-0.01216, 2.97965, 2.32617, 0.07077],
                        [-0.1463, 2.88143, 2.08118, 0.07988],
                        [-0.32955, 2.74121, 1.89842, 0.12155],
                        [-0.5125, 2.61431, 1.89842, 0.11728],
                        [-0.69489, 2.50975, 1.89842, 0.11075],
                        [-0.87655, 2.43285, 1.89842, 0.10391],
                        [-1.05722, 2.38667, 1.89842, 0.09823],
                        [-1.23645, 2.37389, 1.89842, 0.09465],
                        [-1.41312, 2.39867, 2.1798, 0.08184],
                        [-1.58675, 2.45235, 2.2348, 0.08132],
                        [-1.75615, 2.53565, 2.18712, 0.08631],
                        [-1.91957, 2.65092, 2.08312, 0.09601],
                        [-2.0743, 2.80245, 1.99268, 0.10868],
                        [-2.21572, 2.99756, 1.91088, 0.12611],
                        [-2.32895, 3.23194, 1.83767, 0.14165],
                        [-2.46328, 3.41133, 1.76523, 0.12696],
                        [-2.60917, 3.54486, 1.7, 0.11633],
                        [-2.76109, 3.63954, 1.7, 0.1053],
                        [-2.91558, 3.70005, 1.7, 0.0976],
                        [-3.07016, 3.72909, 1.7, 0.09252],
                        [-3.22266, 3.72779, 1.7, 0.08971],
                        [-3.37073, 3.69539, 1.7, 0.08916],
                        [-3.51113, 3.62922, 1.87103, 0.08296],
                        [-3.64232, 3.533, 2.0529, 0.07925],
                        [-3.76303, 3.40827, 2.29639, 0.07559],
                        [-3.87246, 3.25633, 2.64803, 0.07071],
                        [-3.97062, 3.0794, 3.23193, 0.06261],
                        [-4.05913, 2.88215, 2.89282, 0.07474],
                        [-4.14171, 2.6728, 2.60514, 0.08639],
                        [-4.22258, 2.47322, 2.38774, 0.09019],
                        [-4.30723, 2.28282, 2.21349, 0.09413],
                        [-4.39763, 2.10649, 2.0704, 0.09571],
                        [-4.49498, 1.9472, 1.94657, 0.0959],
                        [-4.59978, 1.80648, 1.94657, 0.09014],
                        [-4.7121, 1.68504, 1.94657, 0.08498],
                        [-4.83172, 1.58333, 1.94657, 0.08067],
                        [-4.95824, 1.50191, 1.94657, 0.07729],
                        [-5.09105, 1.44172, 1.94657, 0.07491],
                        [-5.22924, 1.40437, 2.17844, 0.06571],
                        [-5.37059, 1.38496, 2.19705, 0.06494],
                        [-5.51425, 1.38339, 2.21116, 0.06497],
                        [-5.65941, 1.40009, 2.22349, 0.06572],
                        [-5.80521, 1.43598, 2.23457, 0.0672],
                        [-5.95064, 1.49259, 2.23461, 0.06984],
                        [-6.09434, 1.57241, 2.46227, 0.06676],
                        [-6.23523, 1.67407, 2.66731, 0.06514],
                        [-6.37241, 1.79724, 2.92492, 0.06303],
                        [-6.50523, 1.94122, 3.27374, 0.05984],
                        [-6.63344, 2.10456, 3.77644, 0.05499],
                        [-6.75727, 2.2847, 4.61571, 0.04736],
                        [-6.8776, 2.4775, 4.60947, 0.04931],
                        [-6.99593, 2.67733, 4.22695, 0.05494],
                        [-7.11676, 2.87604, 3.92373, 0.05927],
                        [-7.23972, 3.06933, 3.67846, 0.06228],
                        [-7.36556, 3.25536, 3.48221, 0.0645],
                        [-7.495, 3.43258, 3.32169, 0.06607],
                        [-7.62863, 3.59988, 3.1795, 0.06734],
                        [-7.76703, 3.7564, 3.0511, 0.06848],
                        [-7.91081, 3.90149, 2.93922, 0.0695],
                        [-8.06065, 4.03469, 2.83278, 0.07077],
                        [-8.21733, 4.15557, 2.73795, 0.07228],
                        [-8.38185, 4.2637, 2.64319, 0.07448],
                        [-8.55555, 4.35834, 2.55078, 0.07755],
                        [-8.74021, 4.43819, 2.46559, 0.0816],
                        [-8.938, 4.50112, 2.38242, 0.08712],
                        [-9.15118, 4.54343, 2.30181, 0.09442],
                        [-9.38039, 4.55944, 2.22616, 0.10321],
                        [-9.62055, 4.5419, 2.1508, 0.11196],
                        [-9.85741, 4.48633, 2.1508, 0.11312],
                        [-10.07578, 4.39459, 2.1508, 0.11012],
                        [-10.26759, 4.27133, 2.1508, 0.10601],
                        [-10.42935, 4.1204, 2.1508, 0.10286],
                        [-10.55821, 3.9445, 2.1508, 0.10138],
                        [-10.64991, 3.74563, 2.3451, 0.09338],
                        [-10.70771, 3.52979, 2.50252, 0.08929],
                        [-10.73261, 3.30043, 2.69142, 0.08572],
                        [-10.72553, 3.0604, 2.92436, 0.08211],
                        [-10.68776, 2.81229, 3.22662, 0.07778],
                        [-10.62149, 2.55846, 3.64102, 0.07205],
                        [-10.53018, 2.30103, 4.26423, 0.06405],
                        [-10.41884, 2.04165, 5.2, 0.05428],
                        [-10.29379, 1.78119, 5.2, 0.05556],
                        [-10.16215, 1.5194, 5.2, 0.05635],
                        [-10.03094, 1.2574, 5.2, 0.05635],
                        [-9.9, 0.99526, 5.2, 0.05635],
                        [-9.76927, 0.73303, 5.2, 0.05635],
                        [-9.63861, 0.47073, 5.2, 0.05635],
                        [-9.50769, 0.20855, 5.2, 0.05636],
                        [-9.37652, -0.05352, 5.2, 0.05636],
                        [-9.2451, -0.31548, 5.2, 0.05636],
                        [-9.11346, -0.57732, 5.2, 0.05636],
                        [-8.98159, -0.83906, 5.2, 0.05636],
                        [-8.84949, -1.10069, 5.2, 0.05636]]
        
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
