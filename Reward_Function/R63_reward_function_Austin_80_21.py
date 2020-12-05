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
        racing_track = [[-8.71807,-1.36264,8.0,0.03664],
                        [-8.58558,-1.62408,6.83335,0.04289],
                        [-8.45273,-1.88523,5.82874,0.05027],
                        [-8.31812,-2.14361,5.10002,0.05712],
                        [-8.18033,-2.39664,4.53967,0.06347],
                        [-8.03799,-2.64151,4.09074,0.06924],
                        [-7.89003,-2.87537,3.70792,0.07464],
                        [-7.73562,-3.09539,3.37299,0.07969],
                        [-7.5743,-3.29884,3.03511,0.08555],
                        [-7.40598,-3.48327,2.657,0.09397],
                        [-7.23085,-3.64638,2.36368,0.10125],
                        [-7.0494,-3.78606,2.11275,0.10838],
                        [-6.86222,-3.89958,1.8454,0.11863],
                        [-6.67006,-3.98236,1.8454,0.11338],
                        [-6.47448,-4.03004,1.8454,0.10908],
                        [-6.2779,-4.03701,1.8454,0.10659],
                        [-6.08499,-3.99193,2.16181,0.09164],
                        [-5.89866,-3.9073,2.51917,0.08123],
                        [-5.71975,-3.79094,3.04527,0.07008],
                        [-5.54759,-3.65065,3.81956,0.05814],
                        [-5.38065,-3.4938,5.16189,0.04438],
                        [-5.21706,-3.32743,5.85053,0.03988],
                        [-5.05025,-3.16796,6.252,0.03691],
                        [-4.87905,-3.0129,6.5731,0.03514],
                        [-4.70362,-2.86163,6.93891,0.03338],
                        [-4.52446,-2.71388,7.12599,0.03259],
                        [-4.34194,-2.56965,7.142,0.03257],
                        [-4.15636,-2.42901,7.142,0.0326],
                        [-3.96796,-2.29208,7.00828,0.03323],
                        [-3.77686,-2.15917,6.83737,0.03404],
                        [-3.58338,-2.03025,6.18864,0.03757],
                        [-3.38763,-1.90557,6.18864,0.0375],
                        [-3.18972,-1.78542,6.18864,0.03741],
                        [-2.98974,-1.6701,6.06896,0.03804],
                        [-2.7878,-1.55992,5.55217,0.04143],
                        [-2.58401,-1.45518,5.09092,0.04501],
                        [-2.37848,-1.35614,4.52983,0.05037],
                        [-2.17136,-1.26298,4.06298,0.0559],
                        [-1.96276,-1.17584,3.61141,0.0626],
                        [-1.75284,-1.09468,3.19574,0.07043],
                        [-1.54175,-1.01938,3.19574,0.07013],
                        [-1.32965,-0.94964,3.19574,0.06986],
                        [-1.11665,-0.88534,3.19574,0.06962],
                        [-0.9028,-0.82677,3.19574,0.06938],
                        [-0.68813,-0.77426,3.19574,0.06915],
                        [-0.47259,-0.72894,3.19574,0.06892],
                        [-0.25614,-0.6921,3.19574,0.06871],
                        [-0.03867,-0.6659,3.19574,0.06854],
                        [0.17996,-0.65291,3.19574,0.06853],
                        [0.4001,-0.65675,3.19574,0.0689],
                        [0.62315,-0.68291,4.88067,0.04601],
                        [0.84829,-0.71925,5.37869,0.0424],
                        [1.07568,-0.76447,5.932,0.03908],
                        [1.30542,-0.81743,6.49907,0.03628],
                        [1.53759,-0.87727,7.15363,0.03352],
                        [1.77223,-0.94317,6.94867,0.03507],
                        [2.00937,-1.01445,6.29107,0.03936],
                        [2.24896,-1.09042,5.70786,0.04403],
                        [2.49091,-1.17049,5.0219,0.05075],
                        [2.76506,-1.25738,4.4343,0.06485],
                        [3.0393,-1.34015,4.4343,0.0646],
                        [3.31365,-1.41854,4.4343,0.06434],
                        [3.58809,-1.49226,4.4343,0.06408],
                        [3.86263,-1.56085,4.4343,0.06382],
                        [4.13726,-1.62387,4.4343,0.06354],
                        [4.41197,-1.68042,5.0219,0.05585],
                        [4.68675,-1.72946,4.4343,0.06295],
                        [4.96154,-1.76946,4.4343,0.06262],
                        [5.23624,-1.79863,4.4343,0.0623],
                        [5.51061,-1.814,4.4343,0.06197],
                        [5.78397,-1.81192,5.08992,0.05371],
                        [6.05603,-1.79677,5.27698,0.05164],
                        [6.32644,-1.76959,5.36767,0.05063],
                        [6.59481,-1.73091,5.41976,0.05003],
                        [6.86076,-1.68119,5.14939,0.05254],
                        [7.12384,-1.62086,4.87972,0.05531],
                        [7.38348,-1.55004,4.54337,0.05923],
                        [7.63897,-1.46876,4.16171,0.06442],
                        [7.88906,-1.37635,1.82945,0.14574],
                        [8.13219,-1.27217,1.82945,0.14458],
                        [8.36615,-1.15541,1.82945,0.14293],
                        [8.5879,-1.02514,1.82945,0.14058],
                        [8.79268,-0.88008,1.82945,0.13718],
                        [8.97471,-0.71994,1.82945,0.13252],
                        [9.12781,-0.54564,1.82945,0.12681],
                        [9.24582,-0.35896,1.82945,0.12072],
                        [9.32243,-0.16238,1.82945,0.11533],
                        [9.34931,0.0405,1.82945,0.11187],
                        [9.31907,0.24316,2.20159,0.09307],
                        [9.24826,0.44103,2.41346,0.08708],
                        [9.14121,0.63167,2.65007,0.08251],
                        [9.0012,0.81314,2.9583,0.07748],
                        [8.83213,0.98418,3.28426,0.07323],
                        [8.63745,1.14392,3.62085,0.06955],
                        [8.4203,1.29173,4.07295,0.06449],
                        [8.18467,1.42794,4.60357,0.05912],
                        [7.93437,1.55343,5.23299,0.05351],
                        [7.6728,1.66948,6.00455,0.04766],
                        [7.40288,1.77764,7.00391,0.04152],
                        [7.12706,1.87955,8.0,0.03676],
                        [6.84718,1.97673,8.0,0.03703],
                        [6.56459,2.07045,8.0,0.03722],
                        [6.2804,2.16196,8.0,0.03732],
                        [5.99549,2.25242,8.0,0.03737],
                        [5.71042,2.34264,8.0,0.03738],
                        [5.4254,2.43297,8.0,0.03737],
                        [5.14049,2.52356,8.0,0.03737],
                        [4.85779,2.61313,8.0,0.03707],
                        [4.57696,2.7014,8.0,0.0368],
                        [4.29908,2.78748,8.0,0.03636],
                        [4.02497,2.87059,8.0,0.0358],
                        [3.7554,2.94998,8.0,0.03513],
                        [3.49103,3.02498,7.62451,0.03604],
                        [3.2324,3.09497,6.93327,0.03865],
                        [2.97997,3.15943,6.27449,0.04152],
                        [2.73413,3.21784,2.38756,0.10583],
                        [2.49526,3.2697,2.0318,0.12031],
                        [2.2637,3.31448,1.73641,0.13583],
                        [2.03982,3.35157,1.73641,0.13069],
                        [1.824,3.38033,1.73641,0.12539],
                        [1.61684,3.39974,1.73641,0.11983],
                        [1.41857,3.40905,1.73641,0.11431],
                        [1.22931,3.40741,1.73641,0.109],
                        [1.04915,3.3938,1.73641,0.10405],
                        [0.87807,3.36681,1.73641,0.09975],
                        [0.71581,3.32408,1.73641,0.09663],
                        [0.55808,3.27106,1.73641,0.09583],
                        [0.40349,3.20932,1.73641,0.09586],
                        [0.25105,3.14021,1.73641,0.09639],
                        [0.10011,3.06456,1.73641,0.09724],
                        [-0.04988,2.98309,1.71307,0.09964],
                        [-0.19934,2.89618,1.5,0.11527],
                        [-0.34872,2.80371,1.5,0.11712],
                        [-0.52217,2.68919,1.5,0.13856],
                        [-0.69649,2.5868,1.5,0.13478],
                        [-0.87203,2.50644,1.5,0.12871],
                        [-1.04831,2.45601,1.5,0.12224],
                        [-1.22398,2.44196,1.5,0.11749],
                        [-1.39645,2.47459,1.5,0.11702],
                        [-1.56495,2.54096,1.5,0.12073],
                        [-1.72895,2.63615,1.5,0.12642],
                        [-1.8885,2.75508,1.5,0.13267],
                        [-2.04442,2.89139,1.5,0.13807],
                        [-2.19826,3.03712,1.5,0.14127],
                        [-2.34378,3.17161,1.5,0.1321],
                        [-2.48991,3.29746,1.5,0.12857],
                        [-2.63693,3.40908,1.5,0.12306],
                        [-2.78482,3.50228,1.5,0.11654],
                        [-2.93326,3.57355,1.5,0.10977],
                        [-3.08149,3.61921,1.5,0.10341],
                        [-3.22821,3.63623,1.5,0.09847],
                        [-3.37124,3.6202,1.5,0.09595],
                        [-3.50614,3.562,1.81069,0.08114],
                        [-3.63258,3.47218,1.92672,0.08049],
                        [-3.74982,3.35357,1.92672,0.08656],
                        [-3.85781,3.20929,1.92672,0.09353],
                        [-3.95679,3.04208,1.92672,0.10085],
                        [-4.04831,2.85688,1.92672,0.10722],
                        [-4.13558,2.66174,1.92672,0.11095],
                        [-4.22438,2.48078,1.92672,0.10462],
                        [-4.31729,2.31044,1.92672,0.10071],
                        [-4.41458,2.15165,1.92672,0.09665],
                        [-4.51671,2.00602,1.92672,0.09232],
                        [-4.62388,1.87457,1.92672,0.08802],
                        [-4.73617,1.75825,1.92672,0.08391],
                        [-4.85357,1.65797,1.92672,0.08013],
                        [-4.97628,1.57582,1.92672,0.07664],
                        [-5.10426,1.51397,1.92672,0.07378],
                        [-5.23734,1.47603,2.10122,0.06586],
                        [-5.37356,1.45684,2.10122,0.06547],
                        [-5.51215,1.45542,2.10122,0.06596],
                        [-5.65252,1.47206,2.10122,0.06727],
                        [-5.79407,1.5077,2.10122,0.06947],
                        [-5.93591,1.56796,2.29549,0.06714],
                        [-6.0766,1.65218,2.66662,0.06149],
                        [-6.21519,1.75705,2.99606,0.05801],
                        [-6.35106,1.88072,2.99606,0.06132],
                        [-6.48401,2.02065,2.90897,0.06635],
                        [-6.61412,2.17452,2.66099,0.07572],
                        [-6.74174,2.33961,2.38905,0.08734],
                        [-6.86753,2.51259,2.14584,0.09968],
                        [-6.99236,2.68967,1.89322,0.11443],
                        [-7.12216,2.86913,1.89322,0.11699],
                        [-7.25391,3.0442,1.89322,0.11573],
                        [-7.38824,3.21346,3.84768,0.05616],
                        [-7.52582,3.37543,3.40073,0.06249],
                        [-7.66739,3.52847,2.99606,0.06958],
                        [-7.81366,3.67098,2.99606,0.06816],
                        [-7.96536,3.8014,2.99606,0.06677],
                        [-8.12357,3.91724,2.99606,0.06545],
                        [-8.28966,4.01527,3.32833,0.05795],
                        [-8.46179,4.09929,3.52424,0.05435],
                        [-8.63908,4.17089,3.35255,0.05703],
                        [-8.82113,4.23051,3.14987,0.06082],
                        [-9.00753,4.27849,2.90897,0.06617],
                        [-9.19836,4.31354,2.66099,0.07292],
                        [-9.3937,4.33361,2.38905,0.08219],
                        [-9.59328,4.3359,2.14584,0.09302],
                        [-9.79615,4.31609,1.89322,0.10766],
                        [-9.99939,4.26846,1.89322,0.11026],
                        [-10.19548,4.18584,1.89322,0.11239],
                        [-10.36999,4.0639,1.89322,0.11245],
                        [-10.50368,3.90235,1.93505,0.10836],
                        [-10.5924,3.71129,2.33349,0.09028],
                        [-10.64705,3.5024,2.54579,0.08481],
                        [-10.66965,3.27918,2.8141,0.07973],
                        [-10.66228,3.04449,3.1521,0.07449],
                        [-10.62753,2.80096,3.54372,0.06942],
                        [-10.56815,2.55083,4.10897,0.06257],
                        [-10.48831,2.29621,4.65185,0.05736],
                        [-10.39102,2.03849,5.44864,0.05056],
                        [-10.28007,1.77885,6.55008,0.04311],
                        [-10.15918,1.51807,8.0,0.03593],
                        [-10.03181,1.25661,8.0,0.03635],
                        [-9.90113,0.99471,8.0,0.03659],
                        [-9.77002,0.73266,8.0,0.03663],
                        [-9.63916,0.47048,8.0,0.03663],
                        [-9.50822,0.2083,8.0,0.03663],
                        [-9.37711,-0.0538,8.0,0.03663],
                        [-9.24579,-0.3158,8.0,0.03663],
                        [-9.1142,-0.57767,8.0,0.03663],
                        [-8.98238,-0.83943,8.0,0.03664],
                        [-8.85034,-1.10109,8.0,0.03664]]

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
