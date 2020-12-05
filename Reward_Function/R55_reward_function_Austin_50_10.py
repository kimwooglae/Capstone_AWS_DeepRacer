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
        racing_track = [[-8.71717, -1.36222, 3.11242, 0.09417],
                        [-8.58469, -1.62367, 2.77994, 0.10543],
                        [-8.45199, -1.88501, 2.5314, 0.11579],
                        [-8.31933, -2.14636, 2.33667, 0.12543],
                        [-8.18681, -2.40777, 2.17952, 0.13447],
                        [-8.05419, -2.66872, 2.04609, 0.14307],
                        [-7.91711, -2.92268, 1.92832, 0.14966],
                        [-7.77187, -3.16316, 1.82581, 0.15387],
                        [-7.61619, -3.38424, 1.82581, 0.1481],
                        [-7.44927, -3.58107, 1.82581, 0.14135],
                        [-7.27153, -3.75004, 1.82581, 0.13432],
                        [-7.08423, -3.88866, 1.82581, 0.12762],
                        [-6.88913, -3.99516, 1.82581, 0.12175],
                        [-6.68826, -4.06812, 1.82581, 0.11704],
                        [-6.48392, -4.1058, 1.82581, 0.11381],
                        [-6.27879, -4.10565, 1.82581, 0.11235],
                        [-6.07651, -4.06369, 2.00188, 0.1032],
                        [-5.87979, -3.98591, 2.12961, 0.09933],
                        [-5.69093, -3.87445, 2.28349, 0.09604],
                        [-5.51215, -3.73139, 2.4754, 0.0925],
                        [-5.34557, -3.5596, 2.71931, 0.08799],
                        [-5.1929, -3.36341, 3.04135, 0.08174],
                        [-5.05478, -3.14866, 3.30166, 0.07734],
                        [-4.92323, -2.97317, 3.45695, 0.06344],
                        [-4.78334, -2.80819, 3.63203, 0.05955],
                        [-4.63519, -2.65146, 3.8341, 0.05625],
                        [-4.47861, -2.50111, 4.0628, 0.05343],
                        [-4.3131, -2.35554, 4.33801, 0.05081],
                        [-4.13786, -2.21314, 4.67624, 0.04829],
                        [-3.95149, -2.07212, 4.91745, 0.04753],
                        [-3.75149, -1.93012, 4.76632, 0.05146],
                        [-3.53286, -1.78336, 4.62793, 0.0569],
                        [-3.28636, -1.62644, 4.49659, 0.06498],
                        [-3.06004, -1.49064, 4.36617, 0.06045],
                        [-2.84267, -1.3679, 4.246, 0.05879],
                        [-2.62938, -1.25502, 4.12584, 0.05849],
                        [-2.41789, -1.15066, 4.00916, 0.05882],
                        [-2.20695, -1.05429, 3.89177, 0.05959],
                        [-1.99581, -0.96573, 3.77944, 0.06058],
                        [-1.78402, -0.88501, 3.67394, 0.06169],
                        [-1.57129, -0.81229, 3.56195, 0.06312],
                        [-1.35748, -0.74781, 3.45274, 0.06468],
                        [-1.14255, -0.69192, 3.45274, 0.06432],
                        [-0.92656, -0.64499, 3.45274, 0.06402],
                        [-0.70961, -0.60747, 3.45274, 0.06377],
                        [-0.4919, -0.57985, 3.45274, 0.06356],
                        [-0.27366, -0.56267, 3.45274, 0.0634],
                        [-0.05514, -0.55653, 3.45274, 0.06331],
                        [0.16346, -0.56206, 3.45274, 0.06333],
                        [0.38214, -0.58006, 3.45274, 0.06355],
                        [0.60145, -0.61166, 3.95485, 0.05603],
                        [0.82234, -0.65416, 4.28062, 0.05255],
                        [1.04599, -0.70676, 4.69843, 0.0489],
                        [1.27358, -0.76869, 5.0, 0.04717],
                        [1.50601, -0.83914, 5.0, 0.04857],
                        [1.74353, -0.9169, 5.0, 0.04999],
                        [1.98542, -1.00021, 5.0, 0.05117],
                        [2.22996, -1.08665, 5.0, 0.05187],
                        [2.49964, -1.18103, 5.0, 0.05714],
                        [2.77012, -1.27361, 5.0, 0.05718],
                        [3.0416, -1.3634, 5.0, 0.05719],
                        [3.31409, -1.44937, 4.86534, 0.05873],
                        [3.58748, -1.53051, 4.72198, 0.06039],
                        [3.86159, -1.60586, 4.58918, 0.06195],
                        [4.13625, -1.67453, 4.46245, 0.06344],
                        [4.41134, -1.73577, 4.34133, 0.06492],
                        [4.68681, -1.78882, 4.20807, 0.06667],
                        [4.96266, -1.83302, 4.08363, 0.06841],
                        [5.23891, -1.86773, 3.96204, 0.07027],
                        [5.51557, -1.89234, 3.84448, 0.07225],
                        [5.79265, -1.90623, 3.72952, 0.07439],
                        [6.07011, -1.90875, 3.61066, 0.07685],
                        [6.3479, -1.8992, 3.15054, 0.08822],
                        [6.62593, -1.8768, 2.81926, 0.09894],
                        [6.90407, -1.84063, 2.56944, 0.10916],
                        [7.18215, -1.78954, 2.37574, 0.11901],
                        [7.45987, -1.72222, 2.21787, 0.12885],
                        [7.7368, -1.63709, 2.08629, 0.13887],
                        [8.01215, -1.5324, 1.96779, 0.1497],
                        [8.2792, -1.40893, 1.86449, 0.1578],
                        [8.52699, -1.27161, 1.86449, 0.15194],
                        [8.75043, -1.11943, 1.86449, 0.14499],
                        [8.94567, -0.95256, 1.86449, 0.13775],
                        [9.10996, -0.77211, 1.86449, 0.13088],
                        [9.24131, -0.57981, 1.86449, 0.1249],
                        [9.33797, -0.37774, 1.86449, 0.12014],
                        [9.39793, -0.16826, 1.86449, 0.11686],
                        [9.4182, 0.04577, 1.86449, 0.1153],
                        [9.39449, 0.2603, 1.98965, 0.10848],
                        [9.33122, 0.47106, 2.10057, 0.10476],
                        [9.23068, 0.67465, 2.22684, 0.10197],
                        [9.09473, 0.86801, 2.37759, 0.09941],
                        [8.92542, 1.04837, 2.55988, 0.09664],
                        [8.72533, 1.21335, 2.78926, 0.09297],
                        [8.49806, 1.36133, 3.09201, 0.08771],
                        [8.2483, 1.49195, 3.51848, 0.08011],
                        [7.98169, 1.60648, 4.18479, 0.06934],
                        [7.70409, 1.70802, 5.0, 0.05912],
                        [7.42077, 1.80116, 5.0, 0.05965],
                        [7.13575, 1.89153, 5.0, 0.0598],
                        [6.85067, 1.98169, 5.0, 0.0598],
                        [6.56562, 2.07196, 5.0, 0.0598],
                        [6.28054, 2.16217, 5.0, 0.0598],
                        [5.99549, 2.25242, 5.0, 0.0598],
                        [5.71042, 2.34264, 5.0, 0.0598],
                        [5.42536, 2.43288, 5.0, 0.0598],
                        [5.14034, 2.52321, 5.0, 0.0598],
                        [4.85544, 2.61382, 5.0, 0.05979],
                        [4.57065, 2.70468, 4.49457, 0.06651],
                        [4.28595, 2.79577, 3.9331, 0.076],
                        [4.00137, 2.88711, 3.53774, 0.08448],
                        [3.7169, 2.9787, 3.23735, 0.09231],
                        [3.43253, 3.07054, 2.99922, 0.09963],
                        [3.14828, 3.16264, 2.8029, 0.1066],
                        [2.86812, 3.25208, 2.63682, 0.11153],
                        [2.60114, 3.33197, 2.49389, 0.11174],
                        [2.35094, 3.39861, 2.36043, 0.10969],
                        [2.11779, 3.45067, 2.36043, 0.10121],
                        [1.90038, 3.48817, 2.36043, 0.09347],
                        [1.69692, 3.51163, 2.36043, 0.08676],
                        [1.50568, 3.52168, 2.36043, 0.08113],
                        [1.32508, 3.51886, 2.36043, 0.07652],
                        [1.1538, 3.50348, 2.36043, 0.07285],
                        [0.99077, 3.47558, 2.36043, 0.07007],
                        [0.83507, 3.43483, 2.36043, 0.06818],
                        [0.68598, 3.38033, 2.77909, 0.05712],
                        [0.54097, 3.3159, 2.36428, 0.06712],
                        [0.39924, 3.24258, 2.0525, 0.07774],
                        [0.26016, 3.1614, 1.83634, 0.0877],
                        [0.12318, 3.07338, 1.67508, 0.0972],
                        [-0.01216, 2.97965, 1.67508, 0.09828],
                        [-0.1463, 2.88143, 1.67508, 0.09925],
                        [-0.32955, 2.74121, 1.67508, 0.13775],
                        [-0.5125, 2.61431, 1.67508, 0.13292],
                        [-0.69489, 2.50975, 1.67508, 0.12551],
                        [-0.87655, 2.43285, 1.67508, 0.11776],
                        [-1.05722, 2.38667, 1.67508, 0.11133],
                        [-1.23645, 2.37389, 1.67508, 0.10727],
                        [-1.41312, 2.39867, 1.83805, 0.09706],
                        [-1.58675, 2.45235, 1.75825, 0.10336],
                        [-1.75615, 2.53565, 1.68607, 0.11196],
                        [-1.91957, 2.65092, 1.62148, 0.12334],
                        [-2.0743, 2.80245, 1.55756, 0.13904],
                        [-2.21572, 2.99756, 1.5, 0.16065],
                        [-2.32895, 3.23194, 1.5, 0.17354],
                        [-2.46328, 3.41133, 1.5, 0.14941],
                        [-2.60917, 3.54486, 1.5, 0.13185],
                        [-2.76109, 3.63954, 1.5, 0.11934],
                        [-2.91558, 3.70005, 1.5, 0.11061],
                        [-3.07016, 3.72909, 1.5, 0.10485],
                        [-3.22266, 3.72779, 1.5, 0.10167],
                        [-3.37073, 3.69539, 1.5, 0.10105],
                        [-3.51113, 3.62922, 1.65091, 0.09402],
                        [-3.64232, 3.533, 1.81138, 0.08982],
                        [-3.76303, 3.40827, 2.02623, 0.08567],
                        [-3.87246, 3.25633, 2.29865, 0.08146],
                        [-3.97062, 3.0794, 2.10683, 0.09604],
                        [-4.05913, 2.88215, 1.95308, 0.1107],
                        [-4.14171, 2.6728, 1.82683, 0.12319],
                        [-4.22258, 2.47322, 1.71756, 0.12538],
                        [-4.30723, 2.28282, 1.71756, 0.12131],
                        [-4.39763, 2.10649, 1.71756, 0.11537],
                        [-4.49498, 1.9472, 1.71756, 0.10869],
                        [-4.59978, 1.80648, 1.71756, 0.10216],
                        [-4.7121, 1.68504, 1.71756, 0.09631],
                        [-4.83172, 1.58333, 1.71756, 0.09142],
                        [-4.95824, 1.50191, 1.71756, 0.0876],
                        [-5.09105, 1.44172, 1.71756, 0.08489],
                        [-5.22924, 1.40437, 1.92215, 0.07447],
                        [-5.37059, 1.38496, 1.93857, 0.0736],
                        [-5.51425, 1.38339, 1.95102, 0.07364],
                        [-5.65941, 1.40009, 1.96191, 0.07448],
                        [-5.80521, 1.43598, 1.97168, 0.07616],
                        [-5.95064, 1.49259, 1.97171, 0.07915],
                        [-6.09434, 1.57241, 2.17259, 0.07566],
                        [-6.23523, 1.67407, 2.35351, 0.07382],
                        [-6.37241, 1.79724, 2.58081, 0.07143],
                        [-6.50523, 1.94122, 2.8886, 0.06782],
                        [-6.63344, 2.10456, 3.33215, 0.06232],
                        [-6.75727, 2.2847, 3.46212, 0.06314],
                        [-6.8776, 2.4775, 3.2457, 0.07002],
                        [-6.99593, 2.67733, 3.07254, 0.07558],
                        [-7.11676, 2.87604, 2.9309, 0.07935],
                        [-7.23972, 3.06933, 2.80544, 0.08166],
                        [-7.36556, 3.25536, 2.69215, 0.08343],
                        [-7.495, 3.43258, 2.59343, 0.08462],
                        [-7.62863, 3.59988, 2.49951, 0.08566],
                        [-7.76703, 3.7564, 2.41584, 0.08648],
                        [-7.91081, 3.90149, 2.33223, 0.08759],
                        [-8.06065, 4.03469, 2.25069, 0.08907],
                        [-8.21733, 4.15557, 2.17552, 0.09096],
                        [-8.38185, 4.2637, 2.10214, 0.09366],
                        [-8.55555, 4.35834, 2.03101, 0.09739],
                        [-8.74021, 4.43819, 1.96426, 0.10242],
                        [-8.938, 4.50112, 1.89776, 0.10937],
                        [-9.15118, 4.54343, 1.89776, 0.11452],
                        [-9.38039, 4.55944, 1.89776, 0.12107],
                        [-9.62055, 4.5419, 1.89776, 0.12689],
                        [-9.85741, 4.48633, 1.89776, 0.1282],
                        [-10.07578, 4.39459, 1.89776, 0.12481],
                        [-10.26759, 4.27133, 1.89776, 0.12014],
                        [-10.42935, 4.1204, 1.89776, 0.11658],
                        [-10.55821, 3.9445, 1.89776, 0.1149],
                        [-10.64991, 3.74563, 2.0692, 0.10584],
                        [-10.70771, 3.52979, 2.20811, 0.10119],
                        [-10.73261, 3.30043, 2.37478, 0.09715],
                        [-10.72553, 3.0604, 2.58032, 0.09306],
                        [-10.68776, 2.81229, 2.84702, 0.08815],
                        [-10.62149, 2.55846, 3.21267, 0.08166],
                        [-10.53018, 2.30103, 3.76255, 0.07259],
                        [-10.41884, 2.04165, 4.72484, 0.05974],
                        [-10.29379, 1.78119, 5.0, 0.05778],
                        [-10.16215, 1.5194, 5.0, 0.0586],
                        [-10.03094, 1.2574, 5.0, 0.0586],
                        [-9.9, 0.99526, 5.0, 0.0586],
                        [-9.76927, 0.73303, 5.0, 0.0586],
                        [-9.63861, 0.47073, 5.0, 0.05861],
                        [-9.50769, 0.20855, 5.0, 0.05861],
                        [-9.37652, -0.05352, 5.0, 0.05861],
                        [-9.2451, -0.31548, 5.0, 0.05861],
                        [-9.11346, -0.57732, 5.0, 0.05862],
                        [-8.98159, -0.83906, 4.40396, 0.06655],
                        [-8.84949, -1.10069, 3.5975, 0.08147]]
        
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
