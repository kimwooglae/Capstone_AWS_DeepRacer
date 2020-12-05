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
        racing_track = [[-8.71807, -1.36264, 6.79126, 0.04316],
                        [-8.58558, -1.62408, 5.46668, 0.05362],
                        [-8.45273, -1.88523, 4.66299, 0.06283],
                        [-8.31812, -2.14361, 4.08002, 0.07141],
                        [-8.18033, -2.39664, 3.63174, 0.07933],
                        [-8.03799, -2.64151, 3.27259, 0.08655],
                        [-7.89003, -2.87537, 2.96634, 0.0933],
                        [-7.73562, -3.09539, 2.69839, 0.09961],
                        [-7.5743, -3.29884, 2.42809, 0.10693],
                        [-7.40598, -3.48327, 2.1256, 0.11747],
                        [-7.23085, -3.64638, 1.89095, 0.12657],
                        [-7.0494, -3.78606, 1.6902, 0.13548],
                        [-6.86222, -3.89958, 1.47632, 0.14828],
                        [-6.67006, -3.98236, 1.47632, 0.14173],
                        [-6.47448, -4.03004, 1.47632, 0.13635],
                        [-6.2779, -4.03701, 1.47632, 0.13324],
                        [-6.08499, -3.99193, 1.72945, 0.11455],
                        [-5.89866, -3.9073, 2.01533, 0.10154],
                        [-5.71975, -3.79094, 2.43622, 0.0876],
                        [-5.54759, -3.65065, 3.05565, 0.07268],
                        [-5.38065, -3.4938, 4.12951, 0.05547],
                        [-5.21706, -3.32743, 4.68043, 0.04985],
                        [-5.05025, -3.16796, 5.0016, 0.04614],
                        [-4.87905, -3.0129, 5.25848, 0.04393],
                        [-4.70362, -2.86163, 5.55113, 0.04173],
                        [-4.52446, -2.71388, 5.70079, 0.04073],
                        [-4.34194, -2.56965, 5.7136, 0.04071],
                        [-4.15636, -2.42901, 5.7136, 0.04075],
                        [-3.96796, -2.29208, 5.60663, 0.04154],
                        [-3.77686, -2.15917, 5.46989, 0.04256],
                        [-3.58338, -2.03025, 5.33352, 0.04359],
                        [-3.38763, -1.90557, 5.20949, 0.04455],
                        [-3.18972, -1.78542, 5.09836, 0.04541],
                        [-2.98974, -1.6701, 5.01083, 0.04607],
                        [-2.7878, -1.55992, 4.95854, 0.04639],
                        [-2.58401, -1.45518, 4.95091, 0.04628],
                        [-2.37848, -1.35614, 4.95091, 0.04608],
                        [-2.17136, -1.26298, 4.95091, 0.04587],
                        [-1.96276, -1.17584, 4.95091, 0.04566],
                        [-1.75284, -1.09468, 5.0, 0.04501],
                        [-1.54175, -1.01938, 4.85517, 0.04616],
                        [-1.32965, -0.94964, 4.44173, 0.05027],
                        [-1.11665, -0.88534, 4.07274, 0.05463],
                        [-0.9028, -0.82677, 3.62386, 0.06118],
                        [-0.68813, -0.77426, 3.25039, 0.06799],
                        [-0.47259, -0.72894, 2.88913, 0.07624],
                        [-0.25614, -0.6921, 2.55659, 0.08588],
                        [-0.03867, -0.6659, 2.55659, 0.08568],
                        [0.17996, -0.65291, 2.55659, 0.08567],
                        [0.4001, -0.65675, 2.55659, 0.08612],
                        [0.62315, -0.68291, 3.90454, 0.05752],
                        [0.84829, -0.71925, 4.30295, 0.053],
                        [1.07568, -0.76447, 4.7456, 0.04885],
                        [1.30542, -0.81743, 5.19925, 0.04535],
                        [1.53759, -0.87727, 5.7229, 0.04189],
                        [1.77223, -0.94317, 6.27651, 0.03883],
                        [2.00937, -1.01445, 6.95213, 0.03562],
                        [2.24896, -1.09042, 7.50815, 0.03348],
                        [2.49091, -1.17049, 7.22831, 0.03526],
                        [2.76506, -1.25738, 6.85933, 0.04193],
                        [3.0393, -1.34015, 6.53667, 0.04382],
                        [3.31365, -1.41854, 6.0349, 0.04728],
                        [3.58809, -1.49226, 5.55894, 0.05112],
                        [3.86263, -1.56085, 5.03286, 0.05623],
                        [4.13726, -1.62387, 4.56629, 0.06171],
                        [4.41197, -1.68042, 4.01752, 0.06981],
                        [4.68675, -1.72946, 3.54744, 0.07868],
                        [4.96154, -1.76946, 3.54744, 0.07828],
                        [5.23624, -1.79863, 3.54744, 0.07787],
                        [5.51061, -1.814, 3.54744, 0.07746],
                        [5.78397, -1.81192, 4.07194, 0.06714],
                        [6.05603, -1.79677, 4.22158, 0.06454],
                        [6.32644, -1.76959, 4.29414, 0.06329],
                        [6.59481, -1.73091, 4.33581, 0.06254],
                        [6.86076, -1.68119, 4.11951, 0.06568],
                        [7.12384, -1.62086, 3.90377, 0.06914],
                        [7.38348, -1.55004, 3.6347, 0.07404],
                        [7.63897, -1.46876, 3.32937, 0.08053],
                        [7.88906, -1.37635, 2.95178, 0.09033],
                        [8.13219, -1.27217, 2.61633, 0.1011],
                        [8.36615, -1.15541, 2.32099, 0.11266],
                        [8.5879, -1.02514, 2.05226, 0.12532],
                        [8.79268, -0.88008, 1.81252, 0.13846],
                        [8.97471, -0.71994, 1.5917, 0.15232],
                        [9.12781, -0.54564, 1.46356, 0.15851],
                        [9.24582, -0.35896, 1.46356, 0.1509],
                        [9.32243, -0.16238, 1.46356, 0.14416],
                        [9.34931, 0.0405, 1.46356, 0.13983],
                        [9.31907, 0.24316, 1.76127, 0.11634],
                        [9.24826, 0.44103, 1.93077, 0.10885],
                        [9.14121, 0.63167, 2.12006, 0.10313],
                        [9.0012, 0.81314, 2.36664, 0.09685],
                        [8.83213, 0.98418, 2.62741, 0.09153],
                        [8.63745, 1.14392, 2.89668, 0.08694],
                        [8.4203, 1.29173, 3.25836, 0.08062],
                        [8.18467, 1.42794, 3.68286, 0.0739],
                        [7.93437, 1.55343, 4.18639, 0.06688],
                        [7.6728, 1.66948, 4.80364, 0.05957],
                        [7.40288, 1.77764, 5.60313, 0.0519],
                        [7.12706, 1.87955, 6.6041, 0.04452],
                        [6.84718, 1.97673, 7.88119, 0.03759],
                        [6.56459, 2.07045, 8.0, 0.03722],
                        [6.2804, 2.16196, 8.0, 0.03732],
                        [5.99549, 2.25242, 8.0, 0.03737],
                        [5.71042, 2.34264, 8.0, 0.03738],
                        [5.4254, 2.43297, 8.0, 0.03737],
                        [5.14049, 2.52356, 8.0, 0.03737],
                        [4.85779, 2.61313, 8.0, 0.03707],
                        [4.57696, 2.7014, 8.0, 0.0368],
                        [4.29908, 2.78748, 8.0, 0.03636],
                        [4.02497, 2.87059, 7.33946, 0.03903],
                        [3.7554, 2.94998, 6.6895, 0.04201],
                        [3.49103, 3.02498, 6.09961, 0.04505],
                        [3.2324, 3.09497, 5.54661, 0.04831],
                        [2.97997, 3.15943, 5.0196, 0.0519],
                        [2.73413, 3.21784, 4.51993, 0.0559],
                        [2.49526, 3.2697, 3.99023, 0.06126],
                        [2.2637, 3.31448, 3.57033, 0.06606],
                        [2.03982, 3.35157, 3.19112, 0.07111],
                        [1.824, 3.38033, 2.84374, 0.07656],
                        [1.61684, 3.39974, 2.52407, 0.08243],
                        [1.41857, 3.40905, 2.21974, 0.08942],
                        [1.22931, 3.40741, 2.21974, 0.08526],
                        [1.04915, 3.3938, 2.21974, 0.08139],
                        [0.87807, 3.36681, 2.21974, 0.07803],
                        [0.71581, 3.32408, 2.70145, 0.06211],
                        [0.55808, 3.27106, 2.95397, 0.05633],
                        [0.40349, 3.20932, 3.26476, 0.05099],
                        [0.25105, 3.14021, 3.54743, 0.04718],
                        [0.10011, 3.06456, 3.87753, 0.04354],
                        [-0.04988, 2.98309, 3.37741, 0.05054],
                        [-0.19934, 2.89618, 2.38098, 0.07262],
                        [-0.34872, 2.80371, 1.91005, 0.09198],
                        [-0.52217, 2.68919, 1.62544, 0.12787],
                        [-0.69649, 2.5868, 1.38913, 0.14553],
                        [-0.87203, 2.50644, 1.38913, 0.13898],
                        [-1.04831, 2.45601, 1.38913, 0.13199],
                        [-1.22398, 2.44196, 1.38913, 0.12686],
                        [-1.39645, 2.47459, 1.66354, 0.10551],
                        [-1.56495, 2.54096, 1.89555, 0.09554],
                        [-1.72895, 2.63615, 2.22448, 0.08525],
                        [-1.8885, 2.75508, 2.75915, 0.07212],
                        [-2.04442, 2.89139, 2.98967, 0.06927],
                        [-2.19826, 3.03712, 2.45557, 0.0863],
                        [-2.34378, 3.17161, 2.07831, 0.09534],
                        [-2.48991, 3.29746, 1.77135, 0.10888],
                        [-2.63693, 3.40908, 1.55325, 0.11884],
                        [-2.78482, 3.50228, 1.37046, 0.12756],
                        [-2.93326, 3.57355, 1.2, 0.13721],
                        [-3.08149, 3.61921, 1.2, 0.12926],
                        [-3.22821, 3.63623, 1.2, 0.12309],
                        [-3.37124, 3.6202, 1.2, 0.11994],
                        [-3.50614, 3.562, 1.44855, 0.10143],
                        [-3.63258, 3.47218, 1.64608, 0.09422],
                        [-3.74982, 3.35357, 1.92262, 0.08675],
                        [-3.85781, 3.20929, 2.2493, 0.08012],
                        [-3.95679, 3.04208, 2.78302, 0.06982],
                        [-4.04831, 2.85688, 3.31866, 0.06225],
                        [-4.13558, 2.66174, 2.93072, 0.07294],
                        [-4.22438, 2.48078, 2.64559, 0.07619],
                        [-4.31729, 2.31044, 2.40307, 0.08074],
                        [-4.41458, 2.15165, 2.19061, 0.08501],
                        [-4.51671, 2.00602, 1.94121, 0.09163],
                        [-4.62388, 1.87457, 1.74018, 0.09746],
                        [-4.73617, 1.75825, 1.54138, 0.10489],
                        [-4.85357, 1.65797, 1.54138, 0.10017],
                        [-4.97628, 1.57582, 1.54138, 0.09581],
                        [-5.10426, 1.51397, 1.54138, 0.09222],
                        [-5.23734, 1.47603, 1.71003, 0.08092],
                        [-5.37356, 1.45684, 1.68097, 0.08184],
                        [-5.51215, 1.45542, 1.68097, 0.08245],
                        [-5.65252, 1.47206, 1.68097, 0.08409],
                        [-5.79407, 1.5077, 1.68097, 0.08683],
                        [-5.93591, 1.56796, 1.83639, 0.08392],
                        [-6.0766, 1.65218, 2.1333, 0.07686],
                        [-6.21519, 1.75705, 2.39868, 0.07245],
                        [-6.35106, 1.88072, 2.75361, 0.06672],
                        [-6.48401, 2.02065, 3.15255, 0.06123],
                        [-6.61412, 2.17452, 3.69505, 0.05453],
                        [-6.74174, 2.33961, 4.58345, 0.04553],
                        [-6.86753, 2.51259, 4.96831, 0.04305],
                        [-6.99236, 2.68967, 4.33441, 0.04998],
                        [-7.12216, 2.86913, 3.80905, 0.05815],
                        [-7.25391, 3.0442, 3.40471, 0.06435],
                        [-7.38824, 3.21346, 3.07814, 0.0702],
                        [-7.52582, 3.37543, 2.72058, 0.07811],
                        [-7.66739, 3.52847, 2.39685, 0.08698],
                        [-7.81366, 3.67098, 2.39685, 0.0852],
                        [-7.96536, 3.8014, 2.39685, 0.08346],
                        [-8.12357, 3.91724, 2.39685, 0.08181],
                        [-8.28966, 4.01527, 2.66267, 0.07243],
                        [-8.46179, 4.09929, 2.81939, 0.06794],
                        [-8.63908, 4.17089, 2.68204, 0.07129],
                        [-8.82113, 4.23051, 2.5199, 0.07602],
                        [-9.00753, 4.27849, 2.32718, 0.08271],
                        [-9.19836, 4.31354, 2.12879, 0.09115],
                        [-9.3937, 4.33361, 1.91124, 0.10274],
                        [-9.59328, 4.3359, 1.71667, 0.11627],
                        [-9.79615, 4.31609, 1.51458, 0.13458],
                        [-9.99939, 4.26846, 1.51458, 0.13782],
                        [-10.19548, 4.18584, 1.51458, 0.14049],
                        [-10.36999, 4.0639, 1.51458, 0.14057],
                        [-10.50368, 3.90235, 1.54804, 0.13545],
                        [-10.5924, 3.71129, 1.86679, 0.11285],
                        [-10.64705, 3.5024, 2.03664, 0.10602],
                        [-10.66965, 3.27918, 2.25128, 0.09966],
                        [-10.66228, 3.04449, 2.52168, 0.09311],
                        [-10.62753, 2.80096, 2.83497, 0.08677],
                        [-10.56815, 2.55083, 3.28717, 0.07821],
                        [-10.48831, 2.29621, 3.72148, 0.0717],
                        [-10.39102, 2.03849, 4.35891, 0.0632],
                        [-10.28007, 1.77885, 5.24007, 0.05388],
                        [-10.15918, 1.51807, 6.6293, 0.04336],
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
