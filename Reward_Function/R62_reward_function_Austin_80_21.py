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
        racing_track = [[-8.71807, -1.36264, 8.0, 0.03664],
                        [-8.58558, -1.62408, 7.28891, 0.04021],
                        [-8.45273, -1.88523, 6.21732, 0.04713],
                        [-8.31812, -2.14361, 5.44003, 0.05355],
                        [-8.18033, -2.39664, 4.84232, 0.0595],
                        [-8.03799, -2.64151, 4.36346, 0.06491],
                        [-7.89003, -2.87537, 3.95512, 0.06997],
                        [-7.73562, -3.09539, 3.59785, 0.07471],
                        [-7.5743, -3.29884, 3.23745, 0.0802],
                        [-7.40598, -3.48327, 2.83413, 0.0881],
                        [-7.23085, -3.64638, 2.52126, 0.09492],
                        [-7.0494, -3.78606, 2.2536, 0.10161],
                        [-6.86222, -3.89958, 1.96842, 0.11121],
                        [-6.67006, -3.98236, 1.96842, 0.1063],
                        [-6.47448, -4.03004, 1.96842, 0.10227],
                        [-6.2779, -4.03701, 1.96842, 0.09993],
                        [-6.08499, -3.99193, 2.30594, 0.08591],
                        [-5.89866, -3.9073, 2.68711, 0.07616],
                        [-5.71975, -3.79094, 3.24829, 0.0657],
                        [-5.54759, -3.65065, 4.0742, 0.05451],
                        [-5.38065, -3.4938, 5.50602, 0.0416],
                        [-5.21706, -3.32743, 6.24057, 0.03739],
                        [-5.05025, -3.16796, 6.6688, 0.0346],
                        [-4.87905, -3.0129, 7.01131, 0.03295],
                        [-4.70362, -2.86163, 7.4015, 0.0313],
                        [-4.52446, -2.71388, 7.60105, 0.03055],
                        [-4.34194, -2.56965, 7.61814, 0.03054],
                        [-4.15636, -2.42901, 7.61814, 0.03057],
                        [-3.96796, -2.29208, 7.4755, 0.03116],
                        [-3.77686, -2.15917, 7.29319, 0.03192],
                        [-3.58338, -2.03025, 7.11135, 0.03269],
                        [-3.38763, -1.90557, 6.94599, 0.03341],
                        [-3.18972, -1.78542, 6.79782, 0.03406],
                        [-2.98974, -1.6701, 6.6811, 0.03455],
                        [-2.7878, -1.55992, 6.61139, 0.03479],
                        [-2.58401, -1.45518, 6.60122, 0.03471],
                        [-2.37848, -1.35614, 6.60122, 0.03456],
                        [-2.17136, -1.26298, 6.60122, 0.0344],
                        [-1.96276, -1.17584, 6.60122, 0.03425],
                        [-1.75284, -1.09468, 6.66667, 0.03376],
                        [-1.54175, -1.01938, 6.47355, 0.03462],
                        [-1.32965, -0.94964, 5.92231, 0.0377],
                        [-1.11665, -0.88534, 5.43031, 0.04097],
                        [-0.9028, -0.82677, 4.83182, 0.04589],
                        [-0.68813, -0.77426, 4.33385, 0.05099],
                        [-0.47259, -0.72894, 3.85218, 0.05718],
                        [-0.25614, -0.6921, 3.40879, 0.06441],
                        [-0.03867, -0.6659, 3.40879, 0.06426],
                        [0.17996, -0.65291, 3.40879, 0.06425],
                        [0.4001, -0.65675, 3.40879, 0.06459],
                        [0.62315, -0.68291, 5.20605, 0.04314],
                        [0.84829, -0.71925, 5.73727, 0.03975],
                        [1.07568, -0.76447, 6.32747, 0.03664],
                        [1.30542, -0.81743, 6.93234, 0.03401],
                        [1.53759, -0.87727, 7.63053, 0.03142],
                        [1.77223, -0.94317, 8.0, 0.03046],
                        [2.00937, -1.01445, 8.0, 0.03095],
                        [2.24896, -1.09042, 8.0, 0.03142],
                        [2.49091, -1.17049, 8.0, 0.03186],
                        [2.76506, -1.25738, 8.0, 0.03595],
                        [3.0393, -1.34015, 8.0, 0.03581],
                        [3.31365, -1.41854, 8.0, 0.03567],
                        [3.58809, -1.49226, 7.41192, 0.03834],
                        [3.86263, -1.56085, 6.71047, 0.04217],
                        [4.13726, -1.62387, 6.08839, 0.04628],
                        [4.41197, -1.68042, 5.35669, 0.05236],
                        [4.68675, -1.72946, 4.72992, 0.05901],
                        [4.96154, -1.76946, 4.72992, 0.05871],
                        [5.23624, -1.79863, 4.72992, 0.0584],
                        [5.51061, -1.814, 4.72992, 0.0581],
                        [5.78397, -1.81192, 5.42925, 0.05035],
                        [6.05603, -1.79677, 5.62878, 0.04841],
                        [6.32644, -1.76959, 5.72552, 0.04747],
                        [6.59481, -1.73091, 5.78108, 0.0469],
                        [6.86076, -1.68119, 5.49268, 0.04926],
                        [7.12384, -1.62086, 5.20503, 0.05186],
                        [7.38348, -1.55004, 4.84626, 0.05553],
                        [7.63897, -1.46876, 4.43916, 0.06039],
                        [7.88906, -1.37635, 3.93571, 0.06774],
                        [8.13219, -1.27217, 3.48844, 0.07582],
                        [8.36615, -1.15541, 3.09466, 0.08449],
                        [8.5879, -1.02514, 2.73634, 0.09399],
                        [8.79268, -0.88008, 2.4167, 0.10384],
                        [8.97471, -0.71994, 2.12227, 0.11424],
                        [9.12781, -0.54564, 1.95141, 0.11889],
                        [9.24582, -0.35896, 1.95141, 0.11317],
                        [9.32243, -0.16238, 1.95141, 0.10812],
                        [9.34931, 0.0405, 1.95141, 0.10488],
                        [9.31907, 0.24316, 2.34836, 0.08725],
                        [9.24826, 0.44103, 2.57436, 0.08164],
                        [9.14121, 0.63167, 2.82675, 0.07735],
                        [9.0012, 0.81314, 3.15552, 0.07264],
                        [8.83213, 0.98418, 3.50321, 0.06865],
                        [8.63745, 1.14392, 3.86223, 0.0652],
                        [8.4203, 1.29173, 4.34448, 0.06046],
                        [8.18467, 1.42794, 4.91048, 0.05542],
                        [7.93437, 1.55343, 5.58185, 0.05016],
                        [7.6728, 1.66948, 6.40485, 0.04468],
                        [7.40288, 1.77764, 7.47084, 0.03892],
                        [7.12706, 1.87955, 8.0, 0.03676],
                        [6.84718, 1.97673, 8.0, 0.03703],
                        [6.56459, 2.07045, 8.0, 0.03722],
                        [6.2804, 2.16196, 8.0, 0.03732],
                        [5.99549, 2.25242, 8.0, 0.03737],
                        [5.71042, 2.34264, 8.0, 0.03738],
                        [5.4254, 2.43297, 8.0, 0.03737],
                        [5.14049, 2.52356, 8.0, 0.03737],
                        [4.85779, 2.61313, 8.0, 0.03707],
                        [4.57696, 2.7014, 8.0, 0.0368],
                        [4.29908, 2.78748, 8.0, 0.03636],
                        [4.02497, 2.87059, 8.0, 0.0358],
                        [3.7554, 2.94998, 8.0, 0.03513],
                        [3.49103, 3.02498, 8.0, 0.03435],
                        [3.2324, 3.09497, 7.39548, 0.03623],
                        [2.97997, 3.15943, 6.69279, 0.03893],
                        [2.73413, 3.21784, 6.02658, 0.04193],
                        [2.49526, 3.2697, 5.32031, 0.04594],
                        [2.2637, 3.31448, 4.76044, 0.04954],
                        [2.03982, 3.35157, 4.25482, 0.05333],
                        [1.824, 3.38033, 3.79165, 0.05742],
                        [1.61684, 3.39974, 3.36543, 0.06183],
                        [1.41857, 3.40905, 2.95966, 0.06707],
                        [1.22931, 3.40741, 2.95966, 0.06395],
                        [1.04915, 3.3938, 2.95966, 0.06105],
                        [0.87807, 3.36681, 2.95966, 0.05852],
                        [0.71581, 3.32408, 3.60194, 0.04658],
                        [0.55808, 3.27106, 3.93862, 0.04225],
                        [0.40349, 3.20932, 4.35302, 0.03824],
                        [0.25105, 3.14021, 4.7299, 0.03539],
                        [0.10011, 3.06456, 5.17004, 0.03266],
                        [-0.04988, 2.98309, 4.50322, 0.0379],
                        [-0.19934, 2.89618, 3.17463, 0.05446],
                        [-0.34872, 2.80371, 2.54673, 0.06898],
                        [-0.52217, 2.68919, 2.16725, 0.0959],
                        [-0.69649, 2.5868, 1.85217, 0.10915],
                        [-0.87203, 2.50644, 1.85217, 0.10423],
                        [-1.04831, 2.45601, 1.85217, 0.09899],
                        [-1.22398, 2.44196, 1.85217, 0.09515],
                        [-1.39645, 2.47459, 2.21806, 0.07914],
                        [-1.56495, 2.54096, 2.5274, 0.07166],
                        [-1.72895, 2.63615, 2.96597, 0.06393],
                        [-1.8885, 2.75508, 3.67887, 0.05409],
                        [-2.04442, 2.89139, 3.98623, 0.05195],
                        [-2.19826, 3.03712, 3.27409, 0.06472],
                        [-2.34378, 3.17161, 2.77108, 0.07151],
                        [-2.48991, 3.29746, 2.3618, 0.08166],
                        [-2.63693, 3.40908, 2.071, 0.08913],
                        [-2.78482, 3.50228, 1.82728, 0.09567],
                        [-2.93326, 3.57355, 1.6, 0.10291],
                        [-3.08149, 3.61921, 1.6, 0.09694],
                        [-3.22821, 3.63623, 1.6, 0.09231],
                        [-3.37124, 3.6202, 1.6, 0.08995],
                        [-3.50614, 3.562, 1.93141, 0.07607],
                        [-3.63258, 3.47218, 2.19477, 0.07066],
                        [-3.74982, 3.35357, 2.56349, 0.06506],
                        [-3.85781, 3.20929, 2.99906, 0.06009],
                        [-3.95679, 3.04208, 3.71069, 0.05236],
                        [-4.04831, 2.85688, 4.42487, 0.04669],
                        [-4.13558, 2.66174, 3.90763, 0.0547],
                        [-4.22438, 2.48078, 3.52745, 0.05715],
                        [-4.31729, 2.31044, 3.2041, 0.06056],
                        [-4.41458, 2.15165, 2.92082, 0.06376],
                        [-4.51671, 2.00602, 2.58828, 0.06873],
                        [-4.62388, 1.87457, 2.32023, 0.07309],
                        [-4.73617, 1.75825, 2.05517, 0.07867],
                        [-4.85357, 1.65797, 2.05517, 0.07513],
                        [-4.97628, 1.57582, 2.05517, 0.07185],
                        [-5.10426, 1.51397, 2.05517, 0.06916],
                        [-5.23734, 1.47603, 2.28004, 0.06069],
                        [-5.37356, 1.45684, 2.2413, 0.06138],
                        [-5.51215, 1.45542, 2.2413, 0.06184],
                        [-5.65252, 1.47206, 2.2413, 0.06307],
                        [-5.79407, 1.5077, 2.2413, 0.06512],
                        [-5.93591, 1.56796, 2.44852, 0.06294],
                        [-6.0766, 1.65218, 2.8444, 0.05765],
                        [-6.21519, 1.75705, 3.19824, 0.05434],
                        [-6.35106, 1.88072, 3.67149, 0.05004],
                        [-6.48401, 2.02065, 4.2034, 0.04592],
                        [-6.61412, 2.17452, 4.92674, 0.0409],
                        [-6.74174, 2.33961, 6.11126, 0.03414],
                        [-6.86753, 2.51259, 6.62442, 0.03229],
                        [-6.99236, 2.68967, 5.77921, 0.03749],
                        [-7.12216, 2.86913, 5.07874, 0.04361],
                        [-7.25391, 3.0442, 4.53961, 0.04827],
                        [-7.38824, 3.21346, 4.10419, 0.05265],
                        [-7.52582, 3.37543, 3.62744, 0.05859],
                        [-7.66739, 3.52847, 3.1958, 0.06523],
                        [-7.81366, 3.67098, 3.1958, 0.0639],
                        [-7.96536, 3.8014, 3.1958, 0.0626],
                        [-8.12357, 3.91724, 3.1958, 0.06136],
                        [-8.28966, 4.01527, 3.55022, 0.05432],
                        [-8.46179, 4.09929, 3.75919, 0.05095],
                        [-8.63908, 4.17089, 3.57605, 0.05347],
                        [-8.82113, 4.23051, 3.35986, 0.05701],
                        [-9.00753, 4.27849, 3.1029, 0.06203],
                        [-9.19836, 4.31354, 2.83839, 0.06836],
                        [-9.3937, 4.33361, 2.54832, 0.07706],
                        [-9.59328, 4.3359, 2.2889, 0.0872],
                        [-9.79615, 4.31609, 2.01944, 0.10094],
                        [-9.99939, 4.26846, 2.01944, 0.10337],
                        [-10.19548, 4.18584, 2.01944, 0.10537],
                        [-10.36999, 4.0639, 2.01944, 0.10542],
                        [-10.50368, 3.90235, 2.06405, 0.10159],
                        [-10.5924, 3.71129, 2.48906, 0.08463],
                        [-10.64705, 3.5024, 2.71551, 0.07951],
                        [-10.66965, 3.27918, 3.0017, 0.07475],
                        [-10.66228, 3.04449, 3.36224, 0.06983],
                        [-10.62753, 2.80096, 3.77997, 0.06508],
                        [-10.56815, 2.55083, 4.3829, 0.05866],
                        [-10.48831, 2.29621, 4.96197, 0.05378],
                        [-10.39102, 2.03849, 5.81188, 0.0474],
                        [-10.28007, 1.77885, 6.98675, 0.04041],
                        [-10.15918, 1.51807, 8.0, 0.03593],
                        [-10.03181, 1.25661, 8.0, 0.03635],
                        [-9.90113, 0.99471, 8.0, 0.03659],
                        [-9.77002, 0.73266, 8.0, 0.03663],
                        [-9.63916, 0.47048, 8.0, 0.03663],
                        [-9.50822, 0.2083, 8.0, 0.03663],
                        [-9.37711, -0.0538, 8.0, 0.03663],
                        [-9.24579, -0.3158, 8.0, 0.03663],
                        [-9.1142, -0.57767, 8.0, 0.03663],
                        [-8.98238, -0.83943, 8.0, 0.03664],
                        [-8.85034, -1.10109, 8.0, 0.03664]]

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
