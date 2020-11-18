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
        racing_track = [[-8.71807, -1.36264, 4.49732, 0.06517],
                        [-8.58558, -1.62408, 4.04681, 0.07243],
                        [-8.45273, -1.88523, 3.54267, 0.0827],
                        [-8.31812, -2.14361, 3.15158, 0.09244],
                        [-8.18033, -2.39664, 2.817, 0.10228],
                        [-8.03799, -2.64151, 2.43073, 0.11652],
                        [-7.89003, -2.87537, 2.43073, 0.11385],
                        [-7.73562, -3.09539, 2.43073, 0.11058],
                        [-7.5743, -3.29884, 2.43073, 0.10682],
                        [-7.40598, -3.48327, 2.43073, 0.10272],
                        [-7.23085, -3.64638, 2.43073, 0.09846],
                        [-7.0494, -3.78606, 2.43073, 0.09421],
                        [-6.86222, -3.89958, 2.43073, 0.09006],
                        [-6.67006, -3.98236, 2.43073, 0.08608],
                        [-6.47448, -4.03004, 2.43073, 0.08282],
                        [-6.2779, -4.03701, 2.43073, 0.08093],
                        [-6.08521, -3.99063, 2.86773, 0.06911],
                        [-5.89939, -3.90424, 3.38121, 0.06061],
                        [-5.72103, -3.78643, 4.19687, 0.05093],
                        [-5.54899, -3.64617, 5.93804, 0.03738],
                        [-5.38065, -3.4938, 6.0, 0.03784],
                        [-5.20459, -3.34385, 6.0, 0.03854],
                        [-5.02511, -3.19876, 6.0, 0.03846],
                        [-4.84287, -3.05798, 6.0, 0.03838],
                        [-4.65836, -2.92102, 6.0, 0.0383],
                        [-4.47192, -2.78744, 6.0, 0.03823],
                        [-4.28388, -2.65682, 6.0, 0.03816],
                        [-4.09442, -2.52891, 6.0, 0.0381],
                        [-3.9037, -2.4035, 6.0, 0.03804],
                        [-3.71178, -2.2805, 6.0, 0.03799],
                        [-3.51874, -2.15983, 6.0, 0.03794],
                        [-3.32464, -2.04141, 6.0, 0.0379],
                        [-3.1295, -1.92523, 6.0, 0.03785],
                        [-2.93333, -1.81136, 6.0, 0.0378],
                        [-2.73604, -1.70002, 6.0, 0.03776],
                        [-2.53751, -1.59157, 6.0, 0.0377],
                        [-2.33758, -1.48652, 5.69176, 0.03968],
                        [-2.13602, -1.38562, 5.05202, 0.04462],
                        [-1.9325, -1.28996, 4.38623, 0.05127],
                        [-1.72678, -1.20097, 3.86753, 0.05796],
                        [-1.51997, -1.11565, 3.86753, 0.05784],
                        [-1.31207, -1.0343, 3.86753, 0.05772],
                        [-1.10302, -0.95745, 3.86753, 0.05759],
                        [-0.89274, -0.88584, 3.86753, 0.05744],
                        [-0.68115, -0.82051, 3.86753, 0.05726],
                        [-0.46814, -0.76282, 3.86753, 0.05706],
                        [-0.25359, -0.71471, 3.86753, 0.05685],
                        [-0.03744, -0.67857, 3.86753, 0.05667],
                        [0.18038, -0.65756, 3.86753, 0.05658],
                        [0.4001, -0.65675, 3.86753, 0.05681],
                        [0.62315, -0.68291, 6.0, 0.03743],
                        [0.84829, -0.71925, 6.0, 0.03801],
                        [1.07568, -0.76447, 6.0, 0.03864],
                        [1.30542, -0.81743, 6.0, 0.03929],
                        [1.53759, -0.87727, 6.0, 0.03996],
                        [1.77223, -0.94317, 6.0, 0.04062],
                        [2.00937, -1.01445, 6.0, 0.04127],
                        [2.24896, -1.09042, 6.0, 0.04189],
                        [2.49091, -1.17049, 6.0, 0.04248],
                        [2.76506, -1.25738, 5.91241, 0.04864],
                        [3.0393, -1.34015, 5.91241, 0.04845],
                        [3.31365, -1.41854, 5.91241, 0.04826],
                        [3.58809, -1.49226, 5.91241, 0.04806],
                        [3.86263, -1.56085, 5.91241, 0.04786],
                        [4.13726, -1.62387, 5.91241, 0.04766],
                        [4.41197, -1.68042, 5.91241, 0.04744],
                        [4.68675, -1.72946, 5.91241, 0.04721],
                        [4.96154, -1.76946, 5.91241, 0.04697],
                        [5.23624, -1.79863, 5.91241, 0.04672],
                        [5.51061, -1.814, 5.91241, 0.04648],
                        [5.78397, -1.81192, 5.54895, 0.04927],
                        [6.05603, -1.79677, 4.91964, 0.05539],
                        [6.32644, -1.76959, 4.36055, 0.06232],
                        [6.59481, -1.73091, 3.86832, 0.07009],
                        [6.86076, -1.68119, 3.42043, 0.0791],
                        [7.12384, -1.62086, 3.02087, 0.08935],
                        [7.38348, -1.55004, 2.65283, 0.10145],
                        [7.63897, -1.46876, 2.43927, 0.10991],
                        [7.88906, -1.37635, 2.43927, 0.1093],
                        [8.13219, -1.27217, 2.43927, 0.10844],
                        [8.36615, -1.15541, 2.43927, 0.1072],
                        [8.5879, -1.02514, 2.43927, 0.10544],
                        [8.79268, -0.88008, 2.43927, 0.10288],
                        [8.97471, -0.71994, 2.43927, 0.09939],
                        [9.12781, -0.54564, 2.43927, 0.09511],
                        [9.24582, -0.35896, 2.43927, 0.09054],
                        [9.32243, -0.16238, 2.43927, 0.08649],
                        [9.34931, 0.0405, 2.43927, 0.0839],
                        [9.31907, 0.24316, 2.93546, 0.0698],
                        [9.24826, 0.44103, 3.21795, 0.06531],
                        [9.14121, 0.63167, 3.53343, 0.06188],
                        [9.0012, 0.81314, 3.9444, 0.05811],
                        [8.83213, 0.98418, 4.37901, 0.05492],
                        [8.63745, 1.14392, 4.82779, 0.05216],
                        [8.4203, 1.29173, 5.4306, 0.04837],
                        [8.18467, 1.42794, 6.0, 0.04536],
                        [7.93437, 1.55343, 6.0, 0.04667],
                        [7.6728, 1.66948, 6.0, 0.04769],
                        [7.40288, 1.77764, 6.0, 0.04846],
                        [7.12706, 1.87955, 6.0, 0.04901],
                        [6.84718, 1.97673, 6.0, 0.04938],
                        [6.56459, 2.07045, 6.0, 0.04962],
                        [6.2804, 2.16196, 6.0, 0.04976],
                        [5.99549, 2.25242, 6.0, 0.04982],
                        [5.71042, 2.34264, 6.0, 0.04983],
                        [5.4254, 2.43297, 6.0, 0.04983],
                        [5.14049, 2.52356, 6.0, 0.04983],
                        [4.85779, 2.61313, 6.0, 0.04943],
                        [4.57696, 2.7014, 6.0, 0.04906],
                        [4.29908, 2.78748, 6.0, 0.04848],
                        [4.02497, 2.87059, 5.95055, 0.04814],
                        [3.7554, 2.94998, 5.31853, 0.05284],
                        [3.49103, 3.02498, 4.73957, 0.05798],
                        [3.2324, 3.09497, 4.20678, 0.06369],
                        [2.97997, 3.15943, 3.69957, 0.07042],
                        [2.73413, 3.21784, 3.69957, 0.0683],
                        [2.49526, 3.2697, 3.69957, 0.06607],
                        [2.2637, 3.31448, 3.69957, 0.06375],
                        [2.03982, 3.35157, 3.69957, 0.06134],
                        [1.824, 3.38033, 3.69957, 0.05885],
                        [1.61684, 3.39974, 3.69957, 0.05624],
                        [1.41857, 3.40905, 3.69957, 0.05365],
                        [1.22931, 3.40741, 3.69957, 0.05116],
                        [1.04915, 3.3938, 3.69957, 0.04884],
                        [0.87807, 3.36681, 3.69957, 0.04682],
                        [0.71581, 3.32408, 3.18341, 0.05271],
                        [0.55808, 3.27106, 2.70906, 0.06142],
                        [0.40349, 3.20932, 2.31521, 0.0719],
                        [0.25105, 3.14021, 2.31521, 0.07229],
                        [0.10011, 3.06456, 2.31521, 0.07293],
                        [-0.04988, 2.98309, 2.31521, 0.07372],
                        [-0.19934, 2.89618, 2.31521, 0.07468],
                        [-0.34872, 2.80371, 2.31521, 0.07588],
                        [-0.52217, 2.68919, 2.31521, 0.08977],
                        [-0.69649, 2.5868, 2.31521, 0.08732],
                        [-0.87203, 2.50644, 2.31521, 0.08339],
                        [-1.04831, 2.45601, 2.31521, 0.0792],
                        [-1.22398, 2.44196, 2.31521, 0.07612],
                        [-1.39645, 2.47459, 2.77257, 0.06331],
                        [-1.56495, 2.54096, 2.58875, 0.06996],
                        [-1.72895, 2.63615, 2.2841, 0.08302],
                        [-1.8885, 2.75508, 2.0, 0.0995],
                        [-2.04442, 2.89139, 2.0, 0.10355],
                        [-2.19826, 3.03712, 2.0, 0.10595],
                        [-2.34378, 3.17161, 2.0, 0.09907],
                        [-2.48991, 3.29746, 2.0, 0.09643],
                        [-2.63693, 3.40908, 2.0, 0.09229],
                        [-2.78482, 3.50228, 2.0, 0.08741],
                        [-2.93326, 3.57355, 2.0, 0.08233],
                        [-3.08149, 3.61921, 2.0, 0.07755],
                        [-3.22821, 3.63623, 2.0, 0.07385],
                        [-3.37124, 3.6202, 2.0, 0.07196],
                        [-3.50614, 3.562, 2.41426, 0.06086],
                        [-3.63258, 3.47218, 2.74347, 0.05653],
                        [-3.74982, 3.35357, 3.20436, 0.05205],
                        [-3.85781, 3.20929, 3.23535, 0.0557],
                        [-3.95679, 3.04208, 2.90029, 0.067],
                        [-4.04831, 2.85688, 2.56896, 0.08041],
                        [-4.13558, 2.66174, 2.56896, 0.08321],
                        [-4.22438, 2.48078, 2.56896, 0.07847],
                        [-4.31729, 2.31044, 2.56896, 0.07553],
                        [-4.41458, 2.15165, 2.56896, 0.07249],
                        [-4.51671, 2.00602, 2.56896, 0.06924],
                        [-4.62388, 1.87457, 2.56896, 0.06602],
                        [-4.73617, 1.75825, 2.56896, 0.06293],
                        [-4.85357, 1.65797, 2.56896, 0.0601],
                        [-4.97628, 1.57582, 2.56896, 0.05748],
                        [-5.10426, 1.51397, 2.56896, 0.05533],
                        [-5.23734, 1.47603, 2.80162, 0.04939],
                        [-5.37356, 1.45684, 2.80162, 0.0491],
                        [-5.51215, 1.45542, 2.80162, 0.04947],
                        [-5.65252, 1.47206, 2.80162, 0.05046],
                        [-5.79407, 1.5077, 2.80162, 0.0521],
                        [-5.93591, 1.56796, 3.06065, 0.05035],
                        [-6.0766, 1.65218, 3.5555, 0.04612],
                        [-6.21519, 1.75705, 3.9978, 0.04347],
                        [-6.35106, 1.88072, 4.58936, 0.04003],
                        [-6.48401, 2.02065, 5.13024, 0.03762],
                        [-6.61412, 2.17452, 4.5343, 0.04444],
                        [-6.74174, 2.33961, 3.99475, 0.05224],
                        [-6.86753, 2.51259, 3.99475, 0.05354],
                        [-6.99236, 2.68967, 3.99475, 0.05423],
                        [-7.12216, 2.86913, 3.99475, 0.05544],
                        [-7.25391, 3.0442, 3.99475, 0.05485],
                        [-7.38824, 3.21346, 3.99475, 0.05409],
                        [-7.52582, 3.37543, 3.99475, 0.0532],
                        [-7.66739, 3.52847, 3.99475, 0.05219],
                        [-7.81366, 3.67098, 3.87863, 0.05265],
                        [-7.96536, 3.8014, 3.54798, 0.05638],
                        [-8.12357, 3.91724, 3.18541, 0.06156],
                        [-8.28966, 4.01527, 2.86112, 0.06741],
                        [-8.46179, 4.09929, 2.5243, 0.07588],
                        [-8.63908, 4.17089, 2.5243, 0.07575],
                        [-8.82113, 4.23051, 2.5243, 0.07589],
                        [-9.00753, 4.27849, 2.5243, 0.07625],
                        [-9.19836, 4.31354, 2.5243, 0.07686],
                        [-9.3937, 4.33361, 2.5243, 0.07779],
                        [-9.59328, 4.3359, 2.5243, 0.07907],
                        [-9.79615, 4.31609, 2.5243, 0.08075],
                        [-9.99939, 4.26846, 2.5243, 0.08269],
                        [-10.19548, 4.18584, 2.5243, 0.08429],
                        [-10.36999, 4.0639, 2.5243, 0.08434],
                        [-10.50368, 3.90235, 2.58007, 0.08127],
                        [-10.5924, 3.71129, 3.11132, 0.06771],
                        [-10.64705, 3.5024, 3.39439, 0.06361],
                        [-10.66965, 3.27918, 3.75213, 0.0598],
                        [-10.66228, 3.04449, 4.2028, 0.05587],
                        [-10.62753, 2.80096, 4.72496, 0.05206],
                        [-10.56815, 2.55083, 5.47862, 0.04693],
                        [-10.48831, 2.29621, 6.0, 0.04447],
                        [-10.39102, 2.03849, 6.0, 0.04591],
                        [-10.28007, 1.77885, 6.0, 0.04706],
                        [-10.15918, 1.51807, 6.0, 0.04791],
                        [-10.03181, 1.25661, 6.0, 0.04847],
                        [-9.90113, 0.99471, 6.0, 0.04878],
                        [-9.77002, 0.73266, 6.0, 0.04884],
                        [-9.63916, 0.47048, 6.0, 0.04884],
                        [-9.50822, 0.2083, 6.0, 0.04884],
                        [-9.37711, -0.0538, 6.0, 0.04884],
                        [-9.24579, -0.3158, 6.0, 0.04884],
                        [-9.1142, -0.57767, 6.0, 0.04885],
                        [-8.98238, -0.83943, 5.45432, 0.05373],
                        [-8.85034, -1.10109, 4.9439, 0.05928]]
        
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
