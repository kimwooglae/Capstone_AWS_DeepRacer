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
        racing_track = [[-8.71807, -1.36264, 3.0, 0.0977],
                        [-8.58558, -1.62408, 3.0, 0.0977],
                        [-8.45273, -1.88523, 2.657, 0.11027],
                        [-8.31812, -2.14361, 2.36368, 0.12326],
                        [-8.18033, -2.39664, 2.11275, 0.13637],
                        [-8.03799, -2.64151, 1.86832, 0.1516],
                        [-7.89003, -2.87537, 1.86832, 0.14813],
                        [-7.73562, -3.09539, 1.86832, 0.14387],
                        [-7.5743, -3.29884, 1.86832, 0.13897],
                        [-7.40598, -3.48327, 1.86832, 0.13364],
                        [-7.23085, -3.64638, 1.86832, 0.1281],
                        [-7.0494, -3.78606, 1.86832, 0.12256],
                        [-6.86222, -3.89958, 1.86832, 0.11717],
                        [-6.67006, -3.98236, 1.86832, 0.11199],
                        [-6.47448, -4.03004, 1.86832, 0.10774],
                        [-6.2779, -4.03701, 1.86832, 0.10529],
                        [-6.08478, -3.99322, 2.17527, 0.09103],
                        [-5.89794, -3.91041, 2.48174, 0.08235],
                        [-5.7186, -3.79498, 2.91408, 0.07319],
                        [-5.54671, -3.65341, 3.0, 0.07423],
                        [-5.38065, -3.4938, 3.0, 0.07677],
                        [-5.21788, -3.32468, 3.0, 0.07824],
                        [-5.0549, -3.16405, 3.0, 0.07627],
                        [-4.88765, -3.00629, 3.0, 0.07664],
                        [-4.71648, -2.85114, 3.0, 0.07701],
                        [-4.54163, -2.69856, 3.0, 0.07735],
                        [-4.36325, -2.5486, 3.0, 0.07768],
                        [-4.18147, -2.40138, 3.0, 0.07797],
                        [-3.99622, -2.25721, 3.0, 0.07824],
                        [-3.80767, -2.11631, 3.0, 0.07846],
                        [-3.61598, -1.97889, 3.0, 0.07862],
                        [-3.42117, -1.84536, 3.0, 0.07872],
                        [-3.22336, -1.71618, 3.0, 0.07875],
                        [-3.02269, -1.59185, 3.0, 0.07869],
                        [-2.81933, -1.47289, 3.0, 0.07853],
                        [-2.61356, -1.35983, 3.0, 0.07826],
                        [-2.40565, -1.25319, 3.0, 0.07789],
                        [-2.19589, -1.15347, 3.0, 0.07742],
                        [-1.98456, -1.06118, 3.0, 0.07687],
                        [-1.77192, -0.97685, 3.0, 0.07625],
                        [-1.55819, -0.90097, 3.0, 0.0756],
                        [-1.34355, -0.83404, 3.0, 0.07494],
                        [-1.12815, -0.77651, 3.0, 0.07432],
                        [-0.91205, -0.72881, 3.0, 0.07376],
                        [-0.69531, -0.6913, 3.0, 0.07332],
                        [-0.4779, -0.66424, 3.0, 0.07303],
                        [-0.25978, -0.64733, 3.0, 0.07293],
                        [-0.04087, -0.64029, 3.0, 0.07301],
                        [0.17898, -0.64311, 3.0, 0.07329],
                        [0.4001, -0.65675, 3.0, 0.07385],
                        [0.62315, -0.68291, 3.0, 0.07486],
                        [0.84829, -0.71925, 3.0, 0.07602],
                        [1.07568, -0.76447, 3.0, 0.07728],
                        [1.30542, -0.81743, 3.0, 0.07859],
                        [1.53759, -0.87727, 3.0, 0.07992],
                        [1.77223, -0.94317, 3.0, 0.08124],
                        [2.00937, -1.01445, 3.0, 0.08254],
                        [2.24896, -1.09042, 3.0, 0.08378],
                        [2.49091, -1.17049, 3.0, 0.08495],
                        [2.76506, -1.25738, 3.0, 0.09586],
                        [3.0393, -1.34015, 3.0, 0.09549],
                        [3.31365, -1.41854, 3.0, 0.09511],
                        [3.58809, -1.49226, 3.0, 0.09472],
                        [3.86263, -1.56085, 3.0, 0.09433],
                        [4.13726, -1.62387, 3.0, 0.09392],
                        [4.41197, -1.68042, 3.0, 0.09349],
                        [4.68675, -1.72946, 3.0, 0.09304],
                        [4.96154, -1.76946, 3.0, 0.09256],
                        [5.23624, -1.79863, 3.0, 0.09208],
                        [5.51061, -1.814, 3.0, 0.0916],
                        [5.78397, -1.81192, 3.0, 0.09112],
                        [6.05603, -1.79677, 3.0, 0.09083],
                        [6.32644, -1.76959, 3.0, 0.09059],
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
                        [8.83213, 0.98418, 3.0, 0.08017],
                        [8.63745, 1.14392, 3.0, 0.08394],
                        [8.4203, 1.29173, 3.0, 0.08756],
                        [8.18467, 1.42794, 3.0, 0.09072],
                        [7.93437, 1.55343, 3.0, 0.09333],
                        [7.6728, 1.66948, 3.0, 0.09539],
                        [7.40288, 1.77764, 3.0, 0.09693],
                        [7.12706, 1.87955, 3.0, 0.09802],
                        [6.84718, 1.97673, 3.0, 0.09876],
                        [6.56459, 2.07045, 3.0, 0.09924],
                        [6.2804, 2.16196, 3.0, 0.09952],
                        [5.99549, 2.25242, 3.0, 0.09964],
                        [5.71042, 2.34264, 3.0, 0.09967],
                        [5.4254, 2.43297, 3.0, 0.09966],
                        [5.14049, 2.52356, 3.0, 0.09965],
                        [4.8557, 2.61443, 3.0, 0.09964],
                        [4.57104, 2.70557, 3.0, 0.09963],
                        [4.28648, 2.79697, 3.0, 0.09962],
                        [4.00204, 2.88861, 3.0, 0.09961],
                        [3.72164, 2.97768, 3.0, 0.09807],
                        [3.44739, 3.06209, 3.0, 0.09565],
                        [3.18111, 3.14011, 3.0, 0.09249],
                        [2.92399, 3.21042, 3.0, 0.08885],
                        [2.67672, 3.27205, 3.0, 0.08494],
                        [2.43998, 3.32397, 3.0, 0.08079],
                        [2.21342, 3.36597, 3.0, 0.07681],
                        [1.99683, 3.39771, 3.0, 0.07297],
                        [1.78984, 3.41899, 3.0, 0.06936],
                        [1.59191, 3.4297, 3.0, 0.06607],
                        [1.40229, 3.43004, 3.0, 0.06321],
                        [1.22048, 3.41981, 3.0, 0.0607],
                        [1.04588, 3.39894, 3.0, 0.05862],
                        [0.87771, 3.36747, 2.97622, 0.05748],
                        [0.71509, 3.32566, 2.38756, 0.07033],
                        [0.557, 3.27393, 2.0318, 0.08186],
                        [0.40242, 3.21291, 1.73641, 0.09571],
                        [0.2503, 3.14355, 1.73641, 0.09628],
                        [0.09971, 3.06709, 1.73641, 0.09726],
                        [-0.05, 2.98452, 1.73641, 0.09846],
                        [-0.19935, 2.8967, 1.73641, 0.09978],
                        [-0.34872, 2.80371, 1.73641, 0.10134],
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
                        [-6.35106, 1.88072, 3.0, 0.06124],
                        [-6.48401, 2.02065, 3.0, 0.06434],
                        [-6.61412, 2.17452, 3.0, 0.06717],
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
                        [-10.66228, 3.04449, 3.0, 0.07827],
                        [-10.62753, 2.80096, 3.0, 0.082],
                        [-10.56815, 2.55083, 3.0, 0.0857],
                        [-10.48831, 2.29621, 3.0, 0.08895],
                        [-10.39102, 2.03849, 3.0, 0.09182],
                        [-10.28007, 1.77885, 3.0, 0.09412],
                        [-10.15918, 1.51807, 3.0, 0.09582],
                        [-10.03181, 1.25661, 3.0, 0.09694],
                        [-9.90113, 0.99471, 3.0, 0.09756],
                        [-9.77002, 0.73266, 3.0, 0.09767],
                        [-9.63916, 0.47048, 3.0, 0.09767],
                        [-9.50822, 0.2083, 3.0, 0.09769],
                        [-9.37711, -0.0538, 3.0, 0.09769],
                        [-9.24579, -0.3158, 3.0, 0.09769],
                        [-9.1142, -0.57767, 3.0, 0.09769],
                        [-8.98238, -0.83943, 3.0, 0.09769],
                        [-8.85034, -1.10109, 3.0, 0.0977]]
        
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
