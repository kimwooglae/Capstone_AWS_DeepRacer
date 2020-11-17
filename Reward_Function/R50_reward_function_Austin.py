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
        racing_track = [[-8.71807, -1.36264, 2.24866, 0.13034],
                        [-8.58558, -1.62408, 2.02341, 0.14485],
                        [-8.45273, -1.88523, 1.77133, 0.16541],
                        [-8.31812, -2.14361, 1.57579, 0.18488],
                        [-8.18033, -2.39664, 1.4085, 0.20456],
                        [-8.03799, -2.64151, 1.24555, 0.22739],
                        [-7.89003, -2.87537, 1.24555, 0.22219],
                        [-7.73562, -3.09539, 1.24555, 0.2158],
                        [-7.5743, -3.29884, 1.24555, 0.20846],
                        [-7.40598, -3.48327, 1.24555, 0.20047],
                        [-7.23085, -3.64638, 1.24555, 0.19215],
                        [-7.0494, -3.78606, 1.24555, 0.18385],
                        [-6.86222, -3.89958, 1.24555, 0.17576],
                        [-6.67006, -3.98236, 1.24555, 0.16799],
                        [-6.47448, -4.03004, 1.24555, 0.16162],
                        [-6.2779, -4.03701, 1.24555, 0.15793],
                        [-6.08478, -3.99322, 1.45018, 0.13655],
                        [-5.89794, -3.91041, 1.65449, 0.12352],
                        [-5.7186, -3.79498, 1.94272, 0.10978],
                        [-5.54671, -3.65341, 2.4, 0.09279],
                        [-5.38065, -3.4938, 2.4, 0.09597],
                        [-5.21788, -3.32468, 2.4, 0.09781],
                        [-5.0549, -3.16405, 2.4, 0.09534],
                        [-4.88765, -3.00629, 2.4, 0.0958],
                        [-4.71648, -2.85114, 2.4, 0.09626],
                        [-4.54163, -2.69856, 2.4, 0.09669],
                        [-4.36325, -2.5486, 2.4, 0.0971],
                        [-4.18147, -2.40138, 2.4, 0.09747],
                        [-3.99622, -2.25721, 2.4, 0.0978],
                        [-3.80767, -2.11631, 2.4, 0.09808],
                        [-3.61598, -1.97889, 2.4, 0.09828],
                        [-3.42117, -1.84536, 2.4, 0.0984],
                        [-3.22336, -1.71618, 2.4, 0.09844],
                        [-3.02269, -1.59185, 2.4, 0.09836],
                        [-2.81933, -1.47289, 2.4, 0.09816],
                        [-2.61356, -1.35983, 2.4, 0.09783],
                        [-2.40565, -1.25319, 2.4, 0.09736],
                        [-2.19589, -1.15347, 2.4, 0.09677],
                        [-1.98456, -1.06118, 2.4, 0.09608],
                        [-1.77192, -0.97685, 2.4, 0.09531],
                        [-1.55819, -0.90097, 2.4, 0.0945],
                        [-1.34355, -0.83404, 2.4, 0.09368],
                        [-1.12815, -0.77651, 2.4, 0.0929],
                        [-0.91205, -0.72881, 2.4, 0.09221],
                        [-0.69531, -0.6913, 2.4, 0.09165],
                        [-0.4779, -0.66424, 2.4, 0.09129],
                        [-0.25978, -0.64733, 2.4, 0.09116],
                        [-0.04087, -0.64029, 2.4, 0.09126],
                        [0.17898, -0.64311, 2.4, 0.09161],
                        [0.4001, -0.65675, 2.4, 0.09231],
                        [0.62315, -0.68291, 2.4, 0.09358],
                        [0.84829, -0.71925, 2.4, 0.09502],
                        [1.07568, -0.76447, 2.4, 0.0966],
                        [1.30542, -0.81743, 2.4, 0.09823],
                        [1.53759, -0.87727, 2.4, 0.0999],
                        [1.77223, -0.94317, 2.4, 0.10155],
                        [2.00937, -1.01445, 2.4, 0.10318],
                        [2.24896, -1.09042, 2.4, 0.10473],
                        [2.49091, -1.17049, 2.4, 0.10619],
                        [2.76506, -1.25738, 2.4, 0.11983],
                        [3.0393, -1.34015, 2.4, 0.11936],
                        [3.31365, -1.41854, 2.4, 0.11889],
                        [3.58809, -1.49226, 2.4, 0.1184],
                        [3.86263, -1.56085, 2.4, 0.11791],
                        [4.13726, -1.62387, 2.4, 0.1174],
                        [4.41197, -1.68042, 2.4, 0.11686],
                        [4.68675, -1.72946, 2.4, 0.1163],
                        [4.96154, -1.76946, 2.4, 0.1157],
                        [5.23624, -1.79863, 2.4, 0.1151],
                        [5.51061, -1.814, 2.4, 0.1145],
                        [5.78397, -1.81192, 2.4, 0.11391],
                        [6.05603, -1.79677, 2.4, 0.11353],
                        [6.32644, -1.76959, 2.18027, 0.12465],
                        [6.59481, -1.73091, 1.93416, 0.14019],
                        [6.86076, -1.68119, 1.71021, 0.1582],
                        [7.12384, -1.62086, 1.51044, 0.1787],
                        [7.38348, -1.55004, 1.32642, 0.2029],
                        [7.63897, -1.46876, 1.21963, 0.21982],
                        [7.88906, -1.37635, 1.21963, 0.21861],
                        [8.13219, -1.27217, 1.21963, 0.21687],
                        [8.36615, -1.15541, 1.21963, 0.21439],
                        [8.5879, -1.02514, 1.21963, 0.21087],
                        [8.79268, -0.88008, 1.21963, 0.20576],
                        [8.97471, -0.71994, 1.21963, 0.19878],
                        [9.12781, -0.54564, 1.21963, 0.19022],
                        [9.24582, -0.35896, 1.21963, 0.18108],
                        [9.32243, -0.16238, 1.21963, 0.17299],
                        [9.34931, 0.0405, 1.21963, 0.1678],
                        [9.31907, 0.24316, 1.46773, 0.13961],
                        [9.24826, 0.44103, 1.60898, 0.13062],
                        [9.14121, 0.63167, 1.76672, 0.12376],
                        [9.0012, 0.81314, 1.9722, 0.11622],
                        [8.83213, 0.98418, 2.18951, 0.10984],
                        [8.63745, 1.14392, 2.4, 0.10493],
                        [8.4203, 1.29173, 2.4, 0.10945],
                        [8.18467, 1.42794, 2.4, 0.1134],
                        [7.93437, 1.55343, 2.4, 0.11667],
                        [7.6728, 1.66948, 2.4, 0.11923],
                        [7.40288, 1.77764, 2.4, 0.12116],
                        [7.12706, 1.87955, 2.4, 0.12252],
                        [6.84718, 1.97673, 2.4, 0.12345],
                        [6.56459, 2.07045, 2.4, 0.12405],
                        [6.2804, 2.16196, 2.4, 0.1244],
                        [5.99549, 2.25242, 2.4, 0.12455],
                        [5.71042, 2.34264, 2.4, 0.12459],
                        [5.4254, 2.43297, 2.4, 0.12458],
                        [5.14049, 2.52356, 2.4, 0.12457],
                        [4.8557, 2.61443, 2.4, 0.12455],
                        [4.57104, 2.70557, 2.4, 0.12454],
                        [4.28648, 2.79697, 2.4, 0.12453],
                        [4.00204, 2.88861, 2.4, 0.12452],
                        [3.72164, 2.97768, 2.4, 0.12258],
                        [3.44739, 3.06209, 2.4, 0.11956],
                        [3.18111, 3.14011, 2.30863, 0.12019],
                        [2.92399, 3.21042, 2.26962, 0.11745],
                        [2.67672, 3.27205, 2.26962, 0.11228],
                        [2.43998, 3.32397, 2.26962, 0.10679],
                        [2.21342, 3.36597, 2.26962, 0.10152],
                        [1.99683, 3.39771, 2.26962, 0.09645],
                        [1.78984, 3.41899, 2.26962, 0.09168],
                        [1.59191, 3.4297, 2.26962, 0.08734],
                        [1.40229, 3.43004, 2.26962, 0.08355],
                        [1.22048, 3.41981, 2.26962, 0.08023],
                        [1.04588, 3.39894, 2.26962, 0.07748],
                        [0.87771, 3.36747, 1.98415, 0.08623],
                        [0.71509, 3.32566, 1.5917, 0.10549],
                        [0.557, 3.27393, 1.35453, 0.1228],
                        [0.40242, 3.21291, 1.15761, 0.14357],
                        [0.2503, 3.14355, 1.15761, 0.14443],
                        [0.09971, 3.06709, 1.15761, 0.14589],
                        [-0.05, 2.98452, 1.15761, 0.1477],
                        [-0.19935, 2.8967, 1.15761, 0.14967],
                        [-0.34872, 2.80371, 1.15761, 0.152],
                        [-0.52217, 2.68919, 1.15761, 0.17954],
                        [-0.69649, 2.5868, 1.15761, 0.17464],
                        [-0.87203, 2.50644, 1.15761, 0.16677],
                        [-1.04831, 2.45601, 1.15761, 0.15839],
                        [-1.22398, 2.44196, 1.15761, 0.15224],
                        [-1.39645, 2.47459, 1.38628, 0.12662],
                        [-1.56495, 2.54096, 1.29438, 0.13991],
                        [-1.72895, 2.63615, 1.14205, 0.16604],
                        [-1.8885, 2.75508, 1.0, 0.199],
                        [-2.04442, 2.89139, 1.0, 0.2071],
                        [-2.19826, 3.03712, 1.0, 0.21191],
                        [-2.34378, 3.17161, 1.0, 0.19815],
                        [-2.48991, 3.29746, 1.0, 0.19286],
                        [-2.63693, 3.40908, 1.0, 0.18459],
                        [-2.78482, 3.50228, 1.0, 0.17481],
                        [-2.93326, 3.57355, 1.0, 0.16466],
                        [-3.08149, 3.61921, 1.0, 0.15511],
                        [-3.22821, 3.63623, 1.0, 0.1477],
                        [-3.37124, 3.6202, 1.0, 0.14392],
                        [-3.50614, 3.562, 1.20713, 0.12171],
                        [-3.63258, 3.47218, 1.37173, 0.11306],
                        [-3.74982, 3.35357, 1.60218, 0.1041],
                        [-3.85781, 3.20929, 1.61768, 0.1114],
                        [-3.95679, 3.04208, 1.45015, 0.13399],
                        [-4.04831, 2.85688, 1.28448, 0.16083],
                        [-4.13558, 2.66174, 1.28448, 0.16642],
                        [-4.22438, 2.48078, 1.28448, 0.15693],
                        [-4.31729, 2.31044, 1.28448, 0.15106],
                        [-4.41458, 2.15165, 1.28448, 0.14498],
                        [-4.51671, 2.00602, 1.28448, 0.13848],
                        [-4.62388, 1.87457, 1.28448, 0.13203],
                        [-4.73617, 1.75825, 1.28448, 0.12587],
                        [-4.85357, 1.65797, 1.28448, 0.1202],
                        [-4.97628, 1.57582, 1.28448, 0.11497],
                        [-5.10426, 1.51397, 1.28448, 0.11066],
                        [-5.23734, 1.47603, 1.40081, 0.09878],
                        [-5.37356, 1.45684, 1.40081, 0.0982],
                        [-5.51215, 1.45542, 1.40081, 0.09894],
                        [-5.65252, 1.47206, 1.40081, 0.10091],
                        [-5.79407, 1.5077, 1.40081, 0.1042],
                        [-5.93591, 1.56796, 1.53033, 0.1007],
                        [-6.0766, 1.65218, 1.77775, 0.09224],
                        [-6.21519, 1.75705, 1.9989, 0.08695],
                        [-6.35106, 1.88072, 2.29468, 0.08007],
                        [-6.48401, 2.02065, 2.4, 0.08042],
                        [-6.61412, 2.17452, 2.26715, 0.08888],
                        [-6.74174, 2.33961, 1.99738, 0.10447],
                        [-6.86753, 2.51259, 1.99738, 0.10708],
                        [-6.99236, 2.68967, 1.99738, 0.10847],
                        [-7.12216, 2.86913, 1.99738, 0.11089],
                        [-7.25391, 3.0442, 1.99738, 0.1097],
                        [-7.38824, 3.21346, 1.99738, 0.10819],
                        [-7.52582, 3.37543, 1.99738, 0.1064],
                        [-7.66739, 3.52847, 1.99738, 0.10437],
                        [-7.81366, 3.67098, 1.93931, 0.10531],
                        [-7.96536, 3.8014, 1.77399, 0.11277],
                        [-8.12357, 3.91724, 1.5927, 0.12312],
                        [-8.28966, 4.01527, 1.43056, 0.13482],
                        [-8.46179, 4.09929, 1.26215, 0.15176],
                        [-8.63908, 4.17089, 1.26215, 0.15149],
                        [-8.82113, 4.23051, 1.26215, 0.15177],
                        [-9.00753, 4.27849, 1.26215, 0.1525],
                        [-9.19836, 4.31354, 1.26215, 0.15373],
                        [-9.3937, 4.33361, 1.26215, 0.15558],
                        [-9.59328, 4.3359, 1.26215, 0.15814],
                        [-9.79615, 4.31609, 1.26215, 0.1615],
                        [-9.99939, 4.26846, 1.26215, 0.16538],
                        [-10.19548, 4.18584, 1.26215, 0.16859],
                        [-10.36999, 4.0639, 1.26215, 0.16868],
                        [-10.50368, 3.90235, 1.29003, 0.16254],
                        [-10.5924, 3.71129, 1.55566, 0.13541],
                        [-10.64705, 3.5024, 1.6972, 0.12722],
                        [-10.66965, 3.27918, 1.87606, 0.11959],
                        [-10.66228, 3.04449, 2.1014, 0.11173],
                        [-10.62753, 2.80096, 2.36248, 0.10413],
                        [-10.56815, 2.55083, 2.4, 0.10712],
                        [-10.48831, 2.29621, 2.4, 0.11119],
                        [-10.39102, 2.03849, 2.4, 0.11478],
                        [-10.28007, 1.77885, 2.4, 0.11765],
                        [-10.15918, 1.51807, 2.4, 0.11977],
                        [-10.03181, 1.25661, 2.4, 0.12118],
                        [-9.90113, 0.99471, 2.4, 0.12195],
                        [-9.77002, 0.73266, 2.4, 0.12209],
                        [-9.63916, 0.47048, 2.4, 0.12209],
                        [-9.50822, 0.2083, 2.4, 0.12211],
                        [-9.37711, -0.0538, 2.4, 0.12211],
                        [-9.24579, -0.3158, 2.4, 0.12211],
                        [-9.1142, -0.57767, 2.4, 0.12211],
                        [-8.98238, -0.83943, 2.4, 0.12212],
                        [-8.85034, -1.10109, 2.4, 0.12212]]

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
