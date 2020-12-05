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
        racing_track = [[-8.71807, -1.36264, 3.14812, 0.0931],
                        [-8.58558, -1.62408, 2.83277, 0.10347],
                        [-8.45273, -1.88523, 2.47987, 0.11815],
                        [-8.31812, -2.14361, 2.2061, 0.13206],
                        [-8.18033, -2.39664, 1.9719, 0.14611],
                        [-8.03799, -2.64151, 1.72237, 0.16444],
                        [-7.89003, -2.87537, 1.72237, 0.16068],
                        [-7.73562, -3.09539, 1.72237, 0.15606],
                        [-7.5743, -3.29884, 1.72237, 0.15075],
                        [-7.40598, -3.48327, 1.72237, 0.14497],
                        [-7.23085, -3.64638, 1.72237, 0.13895],
                        [-7.0494, -3.78606, 1.72237, 0.13295],
                        [-6.86222, -3.89958, 1.72237, 0.1271],
                        [-6.67006, -3.98236, 1.72237, 0.12148],
                        [-6.47448, -4.03004, 1.72237, 0.11688],
                        [-6.2779, -4.03701, 1.72237, 0.11421],
                        [-6.08499, -3.99193, 2.01769, 0.09818],
                        [-5.89866, -3.9073, 2.35122, 0.08704],
                        [-5.71975, -3.79094, 2.84226, 0.07509],
                        [-5.54759, -3.65065, 3.56493, 0.0623],
                        [-5.38065, -3.4938, 4.81776, 0.04755],
                        [-5.21706, -3.32743, 5.2, 0.04487],
                        [-5.05025, -3.16796, 5.2, 0.04438],
                        [-4.87905, -3.0129, 5.2, 0.04442],
                        [-4.70362, -2.86163, 5.2, 0.04455],
                        [-4.52446, -2.71388, 5.2, 0.04466],
                        [-4.34194, -2.56965, 5.2, 0.04474],
                        [-4.15636, -2.42901, 5.2, 0.04478],
                        [-3.96796, -2.29208, 5.2, 0.04479],
                        [-3.77686, -2.15917, 5.2, 0.04476],
                        [-3.58338, -2.03025, 5.2, 0.04471],
                        [-3.38763, -1.90557, 5.2, 0.04463],
                        [-3.18972, -1.78542, 5.2, 0.04453],
                        [-2.98974, -1.6701, 5.2, 0.04439],
                        [-2.7878, -1.55992, 5.18202, 0.04439],
                        [-2.58401, -1.45518, 4.75152, 0.04822],
                        [-2.37848, -1.35614, 4.22784, 0.05396],
                        [-2.17136, -1.26298, 3.79212, 0.05989],
                        [-1.96276, -1.17584, 3.37065, 0.06707],
                        [-1.75284, -1.09468, 2.98269, 0.07546],
                        [-1.54175, -1.01938, 2.98269, 0.07514],
                        [-1.32965, -0.94964, 2.98269, 0.07485],
                        [-1.11665, -0.88534, 2.98269, 0.07459],
                        [-0.9028, -0.82677, 2.98269, 0.07434],
                        [-0.68813, -0.77426, 2.98269, 0.07409],
                        [-0.47259, -0.72894, 2.98269, 0.07384],
                        [-0.25614, -0.6921, 2.98269, 0.07361],
                        [-0.03867, -0.6659, 2.98269, 0.07344],
                        [0.17996, -0.65291, 2.98269, 0.07343],
                        [0.4001, -0.65675, 2.98269, 0.07382],
                        [0.62315, -0.68291, 4.5553, 0.0493],
                        [0.84829, -0.71925, 5.02011, 0.04543],
                        [1.07568, -0.76447, 5.2, 0.04458],
                        [1.30542, -0.81743, 5.2, 0.04534],
                        [1.53759, -0.87727, 5.2, 0.04611],
                        [1.77223, -0.94317, 5.2, 0.04687],
                        [2.00937, -1.01445, 5.2, 0.04762],
                        [2.24896, -1.09042, 5.2, 0.04834],
                        [2.49091, -1.17049, 4.6871, 0.05438],
                        [2.76506, -1.25738, 4.13868, 0.06949],
                        [3.0393, -1.34015, 4.13868, 0.06922],
                        [3.31365, -1.41854, 4.13868, 0.06894],
                        [3.58809, -1.49226, 4.13868, 0.06866],
                        [3.86263, -1.56085, 4.13868, 0.06837],
                        [4.13726, -1.62387, 4.13868, 0.06808],
                        [4.41197, -1.68042, 4.13868, 0.06777],
                        [4.68675, -1.72946, 4.13868, 0.06744],
                        [4.96154, -1.76946, 4.13868, 0.0671],
                        [5.23624, -1.79863, 4.13868, 0.06675],
                        [5.51061, -1.814, 4.13868, 0.0664],
                        [5.78397, -1.81192, 3.88426, 0.07038],
                        [6.05603, -1.79677, 3.44375, 0.07912],
                        [6.32644, -1.76959, 3.05238, 0.08904],
                        [6.59481, -1.73091, 2.70783, 0.10014],
                        [6.86076, -1.68119, 2.3943, 0.113],
                        [7.12384, -1.62086, 2.11461, 0.12764],
                        [7.38348, -1.55004, 1.85698, 0.14493],
                        [7.63897, -1.46876, 1.70749, 0.15701],
                        [7.88906, -1.37635, 1.70749, 0.15615],
                        [8.13219, -1.27217, 1.70749, 0.15491],
                        [8.36615, -1.15541, 1.70749, 0.15314],
                        [8.5879, -1.02514, 1.70749, 0.15062],
                        [8.79268, -0.88008, 1.70749, 0.14697],
                        [8.97471, -0.71994, 1.70749, 0.14199],
                        [9.12781, -0.54564, 1.70749, 0.13587],
                        [9.24582, -0.35896, 1.70749, 0.12934],
                        [9.32243, -0.16238, 1.70749, 0.12356],
                        [9.34931, 0.0405, 1.70749, 0.11986],
                        [9.31907, 0.24316, 2.05482, 0.09972],
                        [9.24826, 0.44103, 2.25257, 0.0933],
                        [9.14121, 0.63167, 2.4734, 0.0884],
                        [9.0012, 0.81314, 2.76108, 0.08301],
                        [8.83213, 0.98418, 3.06531, 0.07846],
                        [8.63745, 1.14392, 3.37946, 0.07452],
                        [8.4203, 1.29173, 3.80142, 0.0691],
                        [8.18467, 1.42794, 4.29667, 0.06334],
                        [7.93437, 1.55343, 4.88412, 0.05733],
                        [7.6728, 1.66948, 5.2, 0.05503],
                        [7.40288, 1.77764, 5.2, 0.05592],
                        [7.12706, 1.87955, 5.2, 0.05655],
                        [6.84718, 1.97673, 5.2, 0.05698],
                        [6.56459, 2.07045, 5.2, 0.05726],
                        [6.2804, 2.16196, 5.2, 0.05742],
                        [5.99549, 2.25242, 5.2, 0.05749],
                        [5.71042, 2.34264, 5.2, 0.0575],
                        [5.4254, 2.43297, 5.2, 0.0575],
                        [5.14049, 2.52356, 5.2, 0.05749],
                        [4.85779, 2.61313, 5.2, 0.05703],
                        [4.57696, 2.7014, 5.2, 0.05661],
                        [4.29908, 2.78748, 4.65527, 0.06249],
                        [4.02497, 2.87059, 4.16539, 0.06876],
                        [3.7554, 2.94998, 3.72297, 0.07548],
                        [3.49103, 3.02498, 3.3177, 0.08283],
                        [3.2324, 3.09497, 2.94475, 0.09099],
                        [2.97997, 3.15943, 2.5897, 0.1006],
                        [2.73413, 3.21784, 2.5897, 0.09757],
                        [2.49526, 3.2697, 2.5897, 0.09439],
                        [2.2637, 3.31448, 2.5897, 0.09107],
                        [2.03982, 3.35157, 2.5897, 0.08763],
                        [1.824, 3.38033, 2.5897, 0.08407],
                        [1.61684, 3.39974, 2.5897, 0.08035],
                        [1.41857, 3.40905, 2.5897, 0.07665],
                        [1.22931, 3.40741, 2.5897, 0.07308],
                        [1.04915, 3.3938, 2.5897, 0.06977],
                        [0.87807, 3.36681, 2.5897, 0.06688],
                        [0.71581, 3.32408, 2.22839, 0.0753],
                        [0.55808, 3.27106, 1.89634, 0.08775],
                        [0.40349, 3.20932, 1.62065, 0.10271],
                        [0.25105, 3.14021, 1.62065, 0.10327],
                        [0.10011, 3.06456, 1.62065, 0.10418],
                        [-0.04988, 2.98309, 1.62065, 0.10532],
                        [-0.19934, 2.89618, 1.62065, 0.10669],
                        [-0.34872, 2.80371, 1.62065, 0.1084],
                        [-0.52217, 2.68919, 1.62065, 0.12824],
                        [-0.69649, 2.5868, 1.62065, 0.12474],
                        [-0.87203, 2.50644, 1.62065, 0.11912],
                        [-1.04831, 2.45601, 1.62065, 0.11314],
                        [-1.22398, 2.44196, 1.62065, 0.10874],
                        [-1.39645, 2.47459, 1.9408, 0.09044],
                        [-1.56495, 2.54096, 1.81213, 0.09994],
                        [-1.72895, 2.63615, 1.59887, 0.1186],
                        [-1.8885, 2.75508, 1.4, 0.14214],
                        [-2.04442, 2.89139, 1.4, 0.14793],
                        [-2.19826, 3.03712, 1.4, 0.15136],
                        [-2.34378, 3.17161, 1.4, 0.14153],
                        [-2.48991, 3.29746, 1.4, 0.13776],
                        [-2.63693, 3.40908, 1.4, 0.13185],
                        [-2.78482, 3.50228, 1.4, 0.12486],
                        [-2.93326, 3.57355, 1.4, 0.11761],
                        [-3.08149, 3.61921, 1.4, 0.11079],
                        [-3.22821, 3.63623, 1.4, 0.1055],
                        [-3.37124, 3.6202, 1.4, 0.1028],
                        [-3.50614, 3.562, 1.68998, 0.08694],
                        [-3.63258, 3.47218, 1.92043, 0.08076],
                        [-3.74982, 3.35357, 2.24305, 0.07435],
                        [-3.85781, 3.20929, 2.26475, 0.07957],
                        [-3.95679, 3.04208, 2.03021, 0.09571],
                        [-4.04831, 2.85688, 1.79827, 0.11488],
                        [-4.13558, 2.66174, 1.79827, 0.11887],
                        [-4.22438, 2.48078, 1.79827, 0.1121],
                        [-4.31729, 2.31044, 1.79827, 0.1079],
                        [-4.41458, 2.15165, 1.79827, 0.10355],
                        [-4.51671, 2.00602, 1.79827, 0.09892],
                        [-4.62388, 1.87457, 1.79827, 0.09431],
                        [-4.73617, 1.75825, 1.79827, 0.08991],
                        [-4.85357, 1.65797, 1.79827, 0.08586],
                        [-4.97628, 1.57582, 1.79827, 0.08212],
                        [-5.10426, 1.51397, 1.79827, 0.07905],
                        [-5.23734, 1.47603, 1.96114, 0.07056],
                        [-5.37356, 1.45684, 1.96114, 0.07015],
                        [-5.51215, 1.45542, 1.96114, 0.07067],
                        [-5.65252, 1.47206, 1.96114, 0.07208],
                        [-5.79407, 1.5077, 1.96114, 0.07443],
                        [-5.93591, 1.56796, 2.14246, 0.07193],
                        [-6.0766, 1.65218, 2.48885, 0.06588],
                        [-6.21519, 1.75705, 2.79846, 0.0621],
                        [-6.35106, 1.88072, 3.21255, 0.05719],
                        [-6.48401, 2.02065, 3.59117, 0.05375],
                        [-6.61412, 2.17452, 3.17401, 0.06348],
                        [-6.74174, 2.33961, 2.79633, 0.07462],
                        [-6.86753, 2.51259, 2.79633, 0.07649],
                        [-6.99236, 2.68967, 2.79633, 0.07748],
                        [-7.12216, 2.86913, 2.79633, 0.0792],
                        [-7.25391, 3.0442, 2.79633, 0.07836],
                        [-7.38824, 3.21346, 2.79633, 0.07728],
                        [-7.52582, 3.37543, 2.79633, 0.076],
                        [-7.66739, 3.52847, 2.79633, 0.07455],
                        [-7.81366, 3.67098, 2.71504, 0.07522],
                        [-7.96536, 3.8014, 2.48359, 0.08055],
                        [-8.12357, 3.91724, 2.22978, 0.08794],
                        [-8.28966, 4.01527, 2.00278, 0.0963],
                        [-8.46179, 4.09929, 1.76701, 0.1084],
                        [-8.63908, 4.17089, 1.76701, 0.10821],
                        [-8.82113, 4.23051, 1.76701, 0.10841],
                        [-9.00753, 4.27849, 1.76701, 0.10893],
                        [-9.19836, 4.31354, 1.76701, 0.10981],
                        [-9.3937, 4.33361, 1.76701, 0.11113],
                        [-9.59328, 4.3359, 1.76701, 0.11296],
                        [-9.79615, 4.31609, 1.76701, 0.11535],
                        [-9.99939, 4.26846, 1.76701, 0.11813],
                        [-10.19548, 4.18584, 1.76701, 0.12042],
                        [-10.36999, 4.0639, 1.76701, 0.12048],
                        [-10.50368, 3.90235, 1.80605, 0.1161],
                        [-10.5924, 3.71129, 2.17792, 0.09672],
                        [-10.64705, 3.5024, 2.37607, 0.09087],
                        [-10.66965, 3.27918, 2.62649, 0.08542],
                        [-10.66228, 3.04449, 2.94196, 0.07981],
                        [-10.62753, 2.80096, 3.30747, 0.07438],
                        [-10.56815, 2.55083, 3.83504, 0.06704],
                        [-10.48831, 2.29621, 4.34173, 0.06146],
                        [-10.39102, 2.03849, 5.0854, 0.05417],
                        [-10.28007, 1.77885, 5.2, 0.0543],
                        [-10.15918, 1.51807, 5.2, 0.05528],
                        [-10.03181, 1.25661, 5.2, 0.05593],
                        [-9.90113, 0.99471, 5.2, 0.05629],
                        [-9.77002, 0.73266, 5.2, 0.05635],
                        [-9.63916, 0.47048, 5.2, 0.05635],
                        [-9.50822, 0.2083, 5.2, 0.05636],
                        [-9.37711, -0.0538, 5.2, 0.05636],
                        [-9.24579, -0.3158, 4.76002, 0.06157],
                        [-9.1142, -0.57767, 4.23703, 0.06917],
                        [-8.98238, -0.83943, 3.81802, 0.07676],
                        [-8.85034, -1.10109, 3.46073, 0.08469]]
        
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
