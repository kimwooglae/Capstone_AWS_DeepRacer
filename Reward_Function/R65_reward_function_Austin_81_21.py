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
        racing_track = [[-8.71807,-1.36264,6.89126,0.04253],
                        [-8.58558,-1.62408,5.56668,0.05266],
                        [-8.45273,-1.88523,4.76299,0.06151],
                        [-8.31812,-2.14361,4.18002,0.0697],
                        [-8.18033,-2.39664,3.73174,0.0772],
                        [-8.03799,-2.64151,3.37259,0.08398],
                        [-7.89003,-2.87537,3.06634,0.09026],
                        [-7.73562,-3.09539,2.79839,0.09605],
                        [-7.5743,-3.29884,2.52809,0.1027],
                        [-7.40598,-3.48327,2.2256,0.11219],
                        [-7.23085,-3.64638,1.99095,0.12021],
                        [-7.0494,-3.78606,1.7902,0.12791],
                        [-6.86222,-3.89958,1.57632,0.13887],
                        [-6.67006,-3.98236,1.57632,0.13274],
                        [-6.47448,-4.03004,1.57632,0.1277],
                        [-6.2779,-4.03701,1.57632,0.12479],
                        [-6.08499,-3.99193,1.82945,0.10829],
                        [-5.89866,-3.9073,2.11533,0.09674],
                        [-5.71975,-3.79094,2.53622,0.08415],
                        [-5.54759,-3.65065,3.15565,0.07038],
                        [-5.38065,-3.4938,4.22951,0.05416],
                        [-5.21706,-3.32743,4.78043,0.04881],
                        [-5.05025,-3.16796,5.1016,0.04524],
                        [-4.87905,-3.0129,5.35848,0.04311],
                        [-4.70362,-2.86163,5.65113,0.04099],
                        [-4.52446,-2.71388,5.80079,0.04003],
                        [-4.34194,-2.56965,5.8136,0.04001],
                        [-4.15636,-2.42901,5.8136,0.04005],
                        [-3.96796,-2.29208,5.70663,0.04081],
                        [-3.77686,-2.15917,5.56989,0.0418],
                        [-3.58338,-2.03025,5.43352,0.04279],
                        [-3.38763,-1.90557,5.30949,0.04371],
                        [-3.18972,-1.78542,5.19836,0.04454],
                        [-2.98974,-1.6701,5.11083,0.04517],
                        [-2.7878,-1.55992,5.05854,0.04547],
                        [-2.58401,-1.45518,5.05091,0.04536],
                        [-2.37848,-1.35614,5.05091,0.04517],
                        [-2.17136,-1.26298,5.05091,0.04496],
                        [-1.96276,-1.17584,5.05091,0.04476],
                        [-1.75284,-1.09468,5.1,0.04413],
                        [-1.54175,-1.01938,4.95517,0.04523],
                        [-1.32965,-0.94964,4.54173,0.04916],
                        [-1.11665,-0.88534,4.17274,0.05332],
                        [-0.9028,-0.82677,3.72386,0.05954],
                        [-0.68813,-0.77426,3.35039,0.06596],
                        [-0.47259,-0.72894,2.98913,0.07369],
                        [-0.25614,-0.6921,2.65659,0.08265],
                        [-0.03867,-0.6659,2.65659,0.08245],
                        [0.17996,-0.65291,2.65659,0.08245],
                        [0.4001,-0.65675,2.65659,0.08288],
                        [0.62315,-0.68291,4.00454,0.05608],
                        [0.84829,-0.71925,4.40295,0.0518],
                        [1.07568,-0.76447,4.8456,0.04784],
                        [1.30542,-0.81743,5.29925,0.04449],
                        [1.53759,-0.87727,5.8229,0.04117],
                        [1.77223,-0.94317,6.37651,0.03822],
                        [2.00937,-1.01445,7.05213,0.03511],
                        [2.24896,-1.09042,7.60815,0.03304],
                        [2.49091,-1.17049,7.32831,0.03478],
                        [2.76506,-1.25738,6.95933,0.04133],
                        [3.0393,-1.34015,6.63667,0.04316],
                        [3.31365,-1.41854,6.1349,0.04651],
                        [3.58809,-1.49226,5.65894,0.05022],
                        [3.86263,-1.56085,5.13286,0.05513],
                        [4.13726,-1.62387,4.66629,0.06039],
                        [4.41197,-1.68042,4.11752,0.06811],
                        [4.68675,-1.72946,3.64744,0.07652],
                        [4.96154,-1.76946,3.64744,0.07613],
                        [5.23624,-1.79863,3.64744,0.07574],
                        [5.51061,-1.814,3.64744,0.07534],
                        [5.78397,-1.81192,4.17194,0.06553],
                        [6.05603,-1.79677,4.32158,0.06305],
                        [6.32644,-1.76959,4.39414,0.06185],
                        [6.59481,-1.73091,4.43581,0.06113],
                        [6.86076,-1.68119,4.21951,0.06412],
                        [7.12384,-1.62086,4.00377,0.06741],
                        [7.38348,-1.55004,3.7347,0.07206],
                        [7.63897,-1.46876,3.42937,0.07818],
                        [7.88906,-1.37635,3.05178,0.08737],
                        [8.13219,-1.27217,2.71633,0.09738],
                        [8.36615,-1.15541,2.42099,0.10801],
                        [8.5879,-1.02514,2.15226,0.1195],
                        [8.79268,-0.88008,1.91252,0.13122],
                        [8.97471,-0.71994,1.6917,0.14332],
                        [9.12781,-0.54564,1.56356,0.14837],
                        [9.24582,-0.35896,1.56356,0.14125],
                        [9.32243,-0.16238,1.56356,0.13494],
                        [9.34931,0.0405,1.56356,0.13089],
                        [9.31907,0.24316,1.86127,0.11009],
                        [9.24826,0.44103,2.03077,0.10349],
                        [9.14121,0.63167,2.22006,0.09848],
                        [9.0012,0.81314,2.46664,0.09292],
                        [8.83213,0.98418,2.72741,0.08817],
                        [8.63745,1.14392,2.99668,0.08404],
                        [8.4203,1.29173,3.35836,0.07822],
                        [8.18467,1.42794,3.78286,0.07195],
                        [7.93437,1.55343,4.28639,0.06532],
                        [7.6728,1.66948,4.90364,0.05836],
                        [7.40288,1.77764,5.70313,0.05099],
                        [7.12706,1.87955,6.7041,0.04386],
                        [6.84718,1.97673,7.98119,0.03712],
                        [6.56459,2.07045,8.1,0.03676],
                        [6.2804,2.16196,8.1,0.03686],
                        [5.99549,2.25242,8.1,0.03691],
                        [5.71042,2.34264,8.1,0.03692],
                        [5.4254,2.43297,8.1,0.03691],
                        [5.14049,2.52356,8.1,0.03691],
                        [4.85779,2.61313,8.1,0.03661],
                        [4.57696,2.7014,8.1,0.03635],
                        [4.29908,2.78748,8.1,0.03591],
                        [4.02497,2.87059,7.43946,0.03851],
                        [3.7554,2.94998,6.7895,0.04139],
                        [3.49103,3.02498,6.19961,0.04432],
                        [3.2324,3.09497,5.64661,0.04745],
                        [2.97997,3.15943,5.1196,0.05089],
                        [2.73413,3.21784,4.61993,0.05469],
                        [2.49526,3.2697,4.09023,0.05976],
                        [2.2637,3.31448,3.67033,0.06426],
                        [2.03982,3.35157,3.29112,0.06895],
                        [1.824,3.38033,2.94374,0.07396],
                        [1.61684,3.39974,2.62407,0.07929],
                        [1.41857,3.40905,2.31974,0.08557],
                        [1.22931,3.40741,2.31974,0.08158],
                        [1.04915,3.3938,2.31974,0.07788],
                        [0.87807,3.36681,2.31974,0.07467],
                        [0.71581,3.32408,2.80145,0.05989],
                        [0.55808,3.27106,3.05397,0.05449],
                        [0.40349,3.20932,3.36476,0.04947],
                        [0.25105,3.14021,3.64743,0.04589],
                        [0.10011,3.06456,3.97753,0.04245],
                        [-0.04988,2.98309,3.47741,0.04909],
                        [-0.19934,2.89618,2.48098,0.06969],
                        [-0.34872,2.80371,2.01005,0.0874],
                        [-0.52217,2.68919,1.72544,0.12046],
                        [-0.69649,2.5868,1.48913,0.13576],
                        [-0.87203,2.50644,1.48913,0.12965],
                        [-1.04831,2.45601,1.48913,0.12313],
                        [-1.22398,2.44196,1.48913,0.11834],
                        [-1.39645,2.47459,1.76354,0.09953],
                        [-1.56495,2.54096,1.99555,0.09075],
                        [-1.72895,2.63615,2.32448,0.08158],
                        [-1.8885,2.75508,2.85915,0.0696],
                        [-2.04442,2.89139,3.08967,0.06703],
                        [-2.19826,3.03712,2.55557,0.08292],
                        [-2.34378,3.17161,2.17831,0.09096],
                        [-2.48991,3.29746,1.87135,0.10306],
                        [-2.63693,3.40908,1.65325,0.11165],
                        [-2.78482,3.50228,1.47046,0.11889],
                        [-2.93326,3.57355,1.3,0.12666],
                        [-3.08149,3.61921,1.3,0.11932],
                        [-3.22821,3.63623,1.3,0.11362],
                        [-3.37124,3.6202,1.3,0.11071],
                        [-3.50614,3.562,1.54855,0.09488],
                        [-3.63258,3.47218,1.74608,0.08882],
                        [-3.74982,3.35357,2.02262,0.08246],
                        [-3.85781,3.20929,2.3493,0.07671],
                        [-3.95679,3.04208,2.88302,0.0674],
                        [-4.04831,2.85688,3.41866,0.06043],
                        [-4.13558,2.66174,3.03072,0.07053],
                        [-4.22438,2.48078,2.74559,0.07342],
                        [-4.31729,2.31044,2.50307,0.07751],
                        [-4.41458,2.15165,2.29061,0.0813],
                        [-4.51671,2.00602,2.04121,0.08714],
                        [-4.62388,1.87457,1.84018,0.09216],
                        [-4.73617,1.75825,1.64138,0.0985],
                        [-4.85357,1.65797,1.64138,0.09407],
                        [-4.97628,1.57582,1.64138,0.08997],
                        [-5.10426,1.51397,1.64138,0.0866],
                        [-5.23734,1.47603,1.81003,0.07645],
                        [-5.37356,1.45684,1.78097,0.07724],
                        [-5.51215,1.45542,1.78097,0.07782],
                        [-5.65252,1.47206,1.78097,0.07937],
                        [-5.79407,1.5077,1.78097,0.08195],
                        [-5.93591,1.56796,1.93639,0.07959],
                        [-6.0766,1.65218,2.2333,0.07342],
                        [-6.21519,1.75705,2.49868,0.06955],
                        [-6.35106,1.88072,2.85361,0.06438],
                        [-6.48401,2.02065,3.25255,0.05935],
                        [-6.61412,2.17452,3.79505,0.05309],
                        [-6.74174,2.33961,4.68345,0.04456],
                        [-6.86753,2.51259,5.06831,0.0422],
                        [-6.99236,2.68967,4.43441,0.04885],
                        [-7.12216,2.86913,3.90905,0.05666],
                        [-7.25391,3.0442,3.50471,0.06251],
                        [-7.38824,3.21346,3.17814,0.06799],
                        [-7.52582,3.37543,2.82058,0.07534],
                        [-7.66739,3.52847,2.49685,0.0835],
                        [-7.81366,3.67098,2.49685,0.08179],
                        [-7.96536,3.8014,2.49685,0.08012],
                        [-8.12357,3.91724,2.49685,0.07853],
                        [-8.28966,4.01527,2.76267,0.06981],
                        [-8.46179,4.09929,2.91939,0.06561],
                        [-8.63908,4.17089,2.78204,0.06873],
                        [-8.82113,4.23051,2.6199,0.07312],
                        [-9.00753,4.27849,2.42718,0.0793],
                        [-9.19836,4.31354,2.22879,0.08706],
                        [-9.3937,4.33361,2.01124,0.09763],
                        [-9.59328,4.3359,1.81667,0.10987],
                        [-9.79615,4.31609,1.61458,0.12624],
                        [-9.99939,4.26846,1.61458,0.12928],
                        [-10.19548,4.18584,1.61458,0.13179],
                        [-10.36999,4.0639,1.61458,0.13186],
                        [-10.50368,3.90235,1.64804,0.12723],
                        [-10.5924,3.71129,1.96679,0.10711],
                        [-10.64705,3.5024,2.13664,0.10106],
                        [-10.66965,3.27918,2.35128,0.09542],
                        [-10.66228,3.04449,2.62168,0.08956],
                        [-10.62753,2.80096,2.93497,0.08381],
                        [-10.56815,2.55083,3.38717,0.0759],
                        [-10.48831,2.29621,3.82148,0.06982],
                        [-10.39102,2.03849,4.45891,0.06178],
                        [-10.28007,1.77885,5.34007,0.05287],
                        [-10.15918,1.51807,6.7293,0.04272],
                        [-10.03181,1.25661,8.1,0.0359],
                        [-9.90113,0.99471,8.1,0.03614],
                        [-9.77002,0.73266,8.1,0.03618],
                        [-9.63916,0.47048,8.1,0.03618],
                        [-9.50822,0.2083,8.1,0.03618],
                        [-9.37711,-0.0538,8.1,0.03618],
                        [-9.24579,-0.3158,8.1,0.03618],
                        [-9.1142,-0.57767,8.1,0.03618],
                        [-8.98238,-0.83943,8.1,0.03619],
                        [-8.85034,-1.10109,8.1,0.03619]]

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
