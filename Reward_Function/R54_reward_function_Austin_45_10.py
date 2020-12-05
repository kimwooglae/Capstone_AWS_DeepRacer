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
        racing_track = [[-8.71807, -1.36264, 3.37299, 0.08689],
                        [-8.58558, -1.62408, 3.03511, 0.09657],
                        [-8.45273, -1.88523, 2.657, 0.11027],
                        [-8.31812, -2.14361, 2.36368, 0.12326],
                        [-8.18033, -2.39664, 2.11275, 0.13637],
                        [-8.03799, -2.64151, 1.82304, 0.15536],
                        [-7.89003, -2.87537, 1.82304, 0.1518],
                        [-7.73562, -3.09539, 1.82304, 0.14744],
                        [-7.5743, -3.29884, 1.82304, 0.14242],
                        [-7.40598, -3.48327, 1.82304, 0.13696],
                        [-7.23085, -3.64638, 1.82304, 0.13128],
                        [-7.0494, -3.78606, 1.82304, 0.12561],
                        [-6.86222, -3.89958, 1.82304, 0.12008],
                        [-6.67006, -3.98236, 1.82304, 0.11477],
                        [-6.47448, -4.03004, 1.82304, 0.11042],
                        [-6.2779, -4.03701, 1.82304, 0.1079],
                        [-6.08521, -3.99063, 2.1508, 0.09215],
                        [-5.89939, -3.90424, 2.53591, 0.08081],
                        [-5.72103, -3.78643, 3.14765, 0.06791],
                        [-5.54899, -3.64617, 4.45353, 0.04984],
                        [-5.38065, -3.4938, 4.5, 0.05046],
                        [-5.20459, -3.34385, 4.5, 0.05139],
                        [-5.02511, -3.19876, 4.5, 0.05129],
                        [-4.84287, -3.05798, 4.5, 0.05117],
                        [-4.65836, -2.92102, 4.5, 0.05107],
                        [-4.47192, -2.78744, 4.5, 0.05097],
                        [-4.28388, -2.65682, 4.5, 0.05088],
                        [-4.09442, -2.52891, 4.5, 0.0508],
                        [-3.9037, -2.4035, 4.5, 0.05072],
                        [-3.71178, -2.2805, 4.5, 0.05066],
                        [-3.51874, -2.15983, 4.5, 0.05059],
                        [-3.32464, -2.04141, 4.5, 0.05053],
                        [-3.1295, -1.92523, 4.5, 0.05047],
                        [-2.93333, -1.81136, 4.5, 0.05041],
                        [-2.73604, -1.70002, 4.5, 0.05034],
                        [-2.53751, -1.59157, 4.5, 0.05027],
                        [-2.33758, -1.48652, 4.26882, 0.05291],
                        [-2.13602, -1.38562, 3.78901, 0.05949],
                        [-1.9325, -1.28996, 3.28967, 0.06836],
                        [-1.72678, -1.20097, 2.90064, 0.07728],
                        [-1.51997, -1.11565, 2.90064, 0.07713],
                        [-1.31207, -1.0343, 2.90064, 0.07697],
                        [-1.10302, -0.95745, 2.90064, 0.07679],
                        [-0.89274, -0.88584, 2.90064, 0.07658],
                        [-0.68115, -0.82051, 2.90064, 0.07634],
                        [-0.46814, -0.76282, 2.90064, 0.07608],
                        [-0.25359, -0.71471, 2.90064, 0.0758],
                        [-0.03744, -0.67857, 2.90064, 0.07555],
                        [0.18038, -0.65756, 2.90064, 0.07544],
                        [0.4001, -0.65675, 2.90064, 0.07575],
                        [0.62315, -0.68291, 4.5, 0.04991],
                        [0.84829, -0.71925, 4.5, 0.05068],
                        [1.07568, -0.76447, 4.5, 0.05152],
                        [1.30542, -0.81743, 4.5, 0.05239],
                        [1.53759, -0.87727, 4.5, 0.05328],
                        [1.77223, -0.94317, 4.5, 0.05416],
                        [2.00937, -1.01445, 4.5, 0.05503],
                        [2.24896, -1.09042, 4.5, 0.05585],
                        [2.49091, -1.17049, 4.5, 0.05664],
                        [2.76506, -1.25738, 4.4343, 0.06485],
                        [3.0393, -1.34015, 4.4343, 0.0646],
                        [3.31365, -1.41854, 4.4343, 0.06434],
                        [3.58809, -1.49226, 4.4343, 0.06408],
                        [3.86263, -1.56085, 4.4343, 0.06382],
                        [4.13726, -1.62387, 4.4343, 0.06354],
                        [4.41197, -1.68042, 4.4343, 0.06325],
                        [4.68675, -1.72946, 4.4343, 0.06295],
                        [4.96154, -1.76946, 4.4343, 0.06262],
                        [5.23624, -1.79863, 4.4343, 0.0623],
                        [5.51061, -1.814, 4.4343, 0.06197],
                        [5.78397, -1.81192, 4.16171, 0.06569],
                        [6.05603, -1.79677, 3.68973, 0.07385],
                        [6.32644, -1.76959, 3.27041, 0.0831],
                        [6.59481, -1.73091, 2.90124, 0.09346],
                        [6.86076, -1.68119, 2.56532, 0.10547],
                        [7.12384, -1.62086, 2.26566, 0.11913],
                        [7.38348, -1.55004, 1.98962, 0.13526],
                        [7.63897, -1.46876, 1.82945, 0.14655],
                        [7.88906, -1.37635, 1.82945, 0.14574],
                        [8.13219, -1.27217, 1.82945, 0.14458],
                        [8.36615, -1.15541, 1.82945, 0.14293],
                        [8.5879, -1.02514, 1.82945, 0.14058],
                        [8.79268, -0.88008, 1.82945, 0.13718],
                        [8.97471, -0.71994, 1.82945, 0.13252],
                        [9.12781, -0.54564, 1.82945, 0.12681],
                        [9.24582, -0.35896, 1.82945, 0.12072],
                        [9.32243, -0.16238, 1.82945, 0.11533],
                        [9.34931, 0.0405, 1.82945, 0.11187],
                        [9.31907, 0.24316, 2.20159, 0.09307],
                        [9.24826, 0.44103, 2.41346, 0.08708],
                        [9.14121, 0.63167, 2.65007, 0.08251],
                        [9.0012, 0.81314, 2.9583, 0.07748],
                        [8.83213, 0.98418, 3.28426, 0.07323],
                        [8.63745, 1.14392, 3.62085, 0.06955],
                        [8.4203, 1.29173, 4.07295, 0.06449],
                        [8.18467, 1.42794, 4.5, 0.06048],
                        [7.93437, 1.55343, 4.5, 0.06222],
                        [7.6728, 1.66948, 4.5, 0.06359],
                        [7.40288, 1.77764, 4.5, 0.06462],
                        [7.12706, 1.87955, 4.5, 0.06534],
                        [6.84718, 1.97673, 4.5, 0.06584],
                        [6.56459, 2.07045, 4.5, 0.06616],
                        [6.2804, 2.16196, 4.5, 0.06635],
                        [5.99549, 2.25242, 4.5, 0.06643],
                        [5.71042, 2.34264, 4.5, 0.06645],
                        [5.4254, 2.43297, 4.5, 0.06644],
                        [5.14049, 2.52356, 4.5, 0.06644],
                        [4.85779, 2.61313, 4.5, 0.0659],
                        [4.57696, 2.7014, 4.5, 0.06542],
                        [4.29908, 2.78748, 4.5, 0.06465],
                        [4.02497, 2.87059, 4.46291, 0.06418],
                        [3.7554, 2.94998, 3.9889, 0.07045],
                        [3.49103, 3.02498, 3.55468, 0.07731],
                        [3.2324, 3.09497, 3.15509, 0.08492],
                        [2.97997, 3.15943, 2.77468, 0.0939],
                        [2.73413, 3.21784, 2.77468, 0.09107],
                        [2.49526, 3.2697, 2.77468, 0.0881],
                        [2.2637, 3.31448, 2.77468, 0.085],
                        [2.03982, 3.35157, 2.77468, 0.08179],
                        [1.824, 3.38033, 2.77468, 0.07847],
                        [1.61684, 3.39974, 2.77468, 0.07499],
                        [1.41857, 3.40905, 2.77468, 0.07154],
                        [1.22931, 3.40741, 2.77468, 0.06821],
                        [1.04915, 3.3938, 2.77468, 0.06512],
                        [0.87807, 3.36681, 2.77468, 0.06242],
                        [0.71581, 3.32408, 2.38756, 0.07028],
                        [0.55808, 3.27106, 2.0318, 0.0819],
                        [0.40349, 3.20932, 1.73641, 0.09586],
                        [0.25105, 3.14021, 1.73641, 0.09639],
                        [0.10011, 3.06456, 1.73641, 0.09724],
                        [-0.04988, 2.98309, 1.73641, 0.0983],
                        [-0.19934, 2.89618, 1.73641, 0.09957],
                        [-0.34872, 2.80371, 1.73641, 0.10118],
                        [-0.52217, 2.68919, 1.73641, 0.11969],
                        [-0.69649, 2.5868, 1.73641, 0.11643],
                        [-0.87203, 2.50644, 1.73641, 0.11118],
                        [-1.04831, 2.45601, 1.73641, 0.10559],
                        [-1.22398, 2.44196, 1.73641, 0.10149],
                        [-1.39645, 2.47459, 2.07943, 0.08441],
                        [-1.56495, 2.54096, 1.94156, 0.09328],
                        [-1.72895, 2.63615, 1.71307, 0.11069],
                        [-1.8885, 2.75508, 1.5, 0.13267],
                        [-2.04442, 2.89139, 1.5, 0.13807],
                        [-2.19826, 3.03712, 1.5, 0.14127],
                        [-2.34378, 3.17161, 1.5, 0.1321],
                        [-2.48991, 3.29746, 1.5, 0.12857],
                        [-2.63693, 3.40908, 1.5, 0.12306],
                        [-2.78482, 3.50228, 1.5, 0.11654],
                        [-2.93326, 3.57355, 1.5, 0.10977],
                        [-3.08149, 3.61921, 1.5, 0.10341],
                        [-3.22821, 3.63623, 1.5, 0.09847],
                        [-3.37124, 3.6202, 1.5, 0.09595],
                        [-3.50614, 3.562, 1.81069, 0.08114],
                        [-3.63258, 3.47218, 2.0576, 0.07537],
                        [-3.74982, 3.35357, 2.40327, 0.0694],
                        [-3.85781, 3.20929, 2.42651, 0.07427],
                        [-3.95679, 3.04208, 2.17522, 0.08933],
                        [-4.04831, 2.85688, 1.92672, 0.10722],
                        [-4.13558, 2.66174, 1.92672, 0.11095],
                        [-4.22438, 2.48078, 1.92672, 0.10462],
                        [-4.31729, 2.31044, 1.92672, 0.10071],
                        [-4.41458, 2.15165, 1.92672, 0.09665],
                        [-4.51671, 2.00602, 1.92672, 0.09232],
                        [-4.62388, 1.87457, 1.92672, 0.08802],
                        [-4.73617, 1.75825, 1.92672, 0.08391],
                        [-4.85357, 1.65797, 1.92672, 0.08013],
                        [-4.97628, 1.57582, 1.92672, 0.07664],
                        [-5.10426, 1.51397, 1.92672, 0.07378],
                        [-5.23734, 1.47603, 2.10122, 0.06586],
                        [-5.37356, 1.45684, 2.10122, 0.06547],
                        [-5.51215, 1.45542, 2.10122, 0.06596],
                        [-5.65252, 1.47206, 2.10122, 0.06727],
                        [-5.79407, 1.5077, 2.10122, 0.06947],
                        [-5.93591, 1.56796, 2.29549, 0.06714],
                        [-6.0766, 1.65218, 2.66662, 0.06149],
                        [-6.21519, 1.75705, 2.99835, 0.05796],
                        [-6.35106, 1.88072, 3.44202, 0.05338],
                        [-6.48401, 2.02065, 3.84768, 0.05017],
                        [-6.61412, 2.17452, 3.40073, 0.05925],
                        [-6.74174, 2.33961, 2.99606, 0.06965],
                        [-6.86753, 2.51259, 2.99606, 0.07139],
                        [-6.99236, 2.68967, 2.99606, 0.07231],
                        [-7.12216, 2.86913, 2.99606, 0.07392],
                        [-7.25391, 3.0442, 2.99606, 0.07313],
                        [-7.38824, 3.21346, 2.99606, 0.07212],
                        [-7.52582, 3.37543, 2.99606, 0.07093],
                        [-7.66739, 3.52847, 2.99606, 0.06958],
                        [-7.81366, 3.67098, 2.90897, 0.0702],
                        [-7.96536, 3.8014, 2.66099, 0.07518],
                        [-8.12357, 3.91724, 2.38905, 0.08208],
                        [-8.28966, 4.01527, 2.14584, 0.08988],
                        [-8.46179, 4.09929, 1.89322, 0.10117],
                        [-8.63908, 4.17089, 1.89322, 0.10099],
                        [-8.82113, 4.23051, 1.89322, 0.10118],
                        [-9.00753, 4.27849, 1.89322, 0.10166],
                        [-9.19836, 4.31354, 1.89322, 0.10249],
                        [-9.3937, 4.33361, 1.89322, 0.10372],
                        [-9.59328, 4.3359, 1.89322, 0.10543],
                        [-9.79615, 4.31609, 1.89322, 0.10766],
                        [-9.99939, 4.26846, 1.89322, 0.11026],
                        [-10.19548, 4.18584, 1.89322, 0.11239],
                        [-10.36999, 4.0639, 1.89322, 0.11245],
                        [-10.50368, 3.90235, 1.93505, 0.10836],
                        [-10.5924, 3.71129, 2.33349, 0.09028],
                        [-10.64705, 3.5024, 2.54579, 0.08481],
                        [-10.66965, 3.27918, 2.8141, 0.07973],
                        [-10.66228, 3.04449, 3.1521, 0.07449],
                        [-10.62753, 2.80096, 3.54372, 0.06942],
                        [-10.56815, 2.55083, 4.10897, 0.06257],
                        [-10.48831, 2.29621, 4.5, 0.0593],
                        [-10.39102, 2.03849, 4.5, 0.06121],
                        [-10.28007, 1.77885, 4.5, 0.06274],
                        [-10.15918, 1.51807, 4.5, 0.06388],
                        [-10.03181, 1.25661, 4.5, 0.06463],
                        [-9.90113, 0.99471, 4.5, 0.06504],
                        [-9.77002, 0.73266, 4.5, 0.06512],
                        [-9.63916, 0.47048, 4.5, 0.06512],
                        [-9.50822, 0.2083, 4.5, 0.06512],
                        [-9.37711, -0.0538, 4.5, 0.06513],
                        [-9.24579, -0.3158, 4.5, 0.06513],
                        [-9.1142, -0.57767, 4.5, 0.06513],
                        [-8.98238, -0.83943, 4.09074, 0.07165],
                        [-8.85034, -1.10109, 3.70792, 0.07904]]
        
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
