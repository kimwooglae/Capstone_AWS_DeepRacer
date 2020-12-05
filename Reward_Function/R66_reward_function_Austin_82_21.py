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
        racing_track = [[-8.71807,-1.36264,6.99126,0.04193],
                        [-8.58558,-1.62408,5.66668,0.05173],
                        [-8.45273,-1.88523,4.86299,0.06025],
                        [-8.31812,-2.14361,4.28002,0.06807],
                        [-8.18033,-2.39664,3.83174,0.07519],
                        [-8.03799,-2.64151,3.47259,0.08157],
                        [-7.89003,-2.87537,3.16634,0.08741],
                        [-7.73562,-3.09539,2.89839,0.09274],
                        [-7.5743,-3.29884,2.62809,0.09879],
                        [-7.40598,-3.48327,2.3256,0.10737],
                        [-7.23085,-3.64638,2.09095,0.11446],
                        [-7.0494,-3.78606,1.8902,0.12115],
                        [-6.86222,-3.89958,1.67632,0.13059],
                        [-6.67006,-3.98236,1.67632,0.12482],
                        [-6.47448,-4.03004,1.67632,0.12008],
                        [-6.2779,-4.03701,1.67632,0.11734],
                        [-6.08499,-3.99193,1.92945,0.10268],
                        [-5.89866,-3.9073,2.21533,0.09237],
                        [-5.71975,-3.79094,2.63622,0.08095],
                        [-5.54759,-3.65065,3.25565,0.06822],
                        [-5.38065,-3.4938,4.32951,0.05291],
                        [-5.21706,-3.32743,4.88043,0.04781],
                        [-5.05025,-3.16796,5.2016,0.04437],
                        [-4.87905,-3.0129,5.45848,0.04232],
                        [-4.70362,-2.86163,5.75113,0.04028],
                        [-4.52446,-2.71388,5.90079,0.03935],
                        [-4.34194,-2.56965,5.9136,0.03933],
                        [-4.15636,-2.42901,5.9136,0.03937],
                        [-3.96796,-2.29208,5.80663,0.04011],
                        [-3.77686,-2.15917,5.66989,0.04106],
                        [-3.58338,-2.03025,5.53352,0.04201],
                        [-3.38763,-1.90557,5.40949,0.0429],
                        [-3.18972,-1.78542,5.29836,0.0437],
                        [-2.98974,-1.6701,5.21083,0.0443],
                        [-2.7878,-1.55992,5.15854,0.04459],
                        [-2.58401,-1.45518,5.15091,0.04448],
                        [-2.37848,-1.35614,5.15091,0.04429],
                        [-2.17136,-1.26298,5.15091,0.04409],
                        [-1.96276,-1.17584,5.15091,0.04389],
                        [-1.75284,-1.09468,5.2,0.04328],
                        [-1.54175,-1.01938,5.05517,0.04433],
                        [-1.32965,-0.94964,4.64173,0.0481],
                        [-1.11665,-0.88534,4.27274,0.05207],
                        [-0.9028,-0.82677,3.82386,0.05798],
                        [-0.68813,-0.77426,3.45039,0.06405],
                        [-0.47259,-0.72894,3.08913,0.0713],
                        [-0.25614,-0.6921,2.75659,0.07965],
                        [-0.03867,-0.6659,2.75659,0.07946],
                        [0.17996,-0.65291,2.75659,0.07945],
                        [0.4001,-0.65675,2.75659,0.07987],
                        [0.62315,-0.68291,4.10454,0.05472],
                        [0.84829,-0.71925,4.50295,0.05065],
                        [1.07568,-0.76447,4.9456,0.04687],
                        [1.30542,-0.81743,5.39925,0.04367],
                        [1.53759,-0.87727,5.9229,0.04048],
                        [1.77223,-0.94317,6.47651,0.03763],
                        [2.00937,-1.01445,7.15213,0.03462],
                        [2.24896,-1.09042,7.70815,0.03261],
                        [2.49091,-1.17049,7.42831,0.03431],
                        [2.76506,-1.25738,7.05933,0.04074],
                        [3.0393,-1.34015,6.73667,0.04252],
                        [3.31365,-1.41854,6.2349,0.04576],
                        [3.58809,-1.49226,5.75894,0.04934],
                        [3.86263,-1.56085,5.23286,0.05408],
                        [4.13726,-1.62387,4.76629,0.05912],
                        [4.41197,-1.68042,4.21752,0.0665],
                        [4.68675,-1.72946,3.74744,0.07448],
                        [4.96154,-1.76946,3.74744,0.0741],
                        [5.23624,-1.79863,3.74744,0.07371],
                        [5.51061,-1.814,3.74744,0.07333],
                        [5.78397,-1.81192,4.27194,0.064],
                        [6.05603,-1.79677,4.42158,0.06162],
                        [6.32644,-1.76959,4.49414,0.06047],
                        [6.59481,-1.73091,4.53581,0.05978],
                        [6.86076,-1.68119,4.31951,0.06264],
                        [7.12384,-1.62086,4.10377,0.06577],
                        [7.38348,-1.55004,3.8347,0.07018],
                        [7.63897,-1.46876,3.52937,0.07597],
                        [7.88906,-1.37635,3.15178,0.0846],
                        [8.13219,-1.27217,2.81633,0.09392],
                        [8.36615,-1.15541,2.52099,0.10372],
                        [8.5879,-1.02514,2.25226,0.11419],
                        [8.79268,-0.88008,2.01252,0.1247],
                        [8.97471,-0.71994,1.7917,0.13532],
                        [9.12781,-0.54564,1.66356,0.13945],
                        [9.24582,-0.35896,1.66356,0.13276],
                        [9.32243,-0.16238,1.66356,0.12683],
                        [9.34931,0.0405,1.66356,0.12302],
                        [9.31907,0.24316,1.96127,0.10448],
                        [9.24826,0.44103,2.13077,0.09863],
                        [9.14121,0.63167,2.32006,0.09424],
                        [9.0012,0.81314,2.56664,0.0893],
                        [8.83213,0.98418,2.82741,0.08506],
                        [8.63745,1.14392,3.09668,0.08132],
                        [8.4203,1.29173,3.45836,0.07596],
                        [8.18467,1.42794,3.88286,0.07009],
                        [7.93437,1.55343,4.38639,0.06383],
                        [7.6728,1.66948,5.00364,0.05719],
                        [7.40288,1.77764,5.80313,0.05011],
                        [7.12706,1.87955,6.8041,0.04321],
                        [6.84718,1.97673,8.08119,0.03666],
                        [6.56459,2.07045,8.2,0.03631],
                        [6.2804,2.16196,8.2,0.03641],
                        [5.99549,2.25242,8.2,0.03646],
                        [5.71042,2.34264,8.2,0.03647],
                        [5.4254,2.43297,8.2,0.03646],
                        [5.14049,2.52356,8.2,0.03646],
                        [4.85779,2.61313,8.2,0.03617],
                        [4.57696,2.7014,8.2,0.0359],
                        [4.29908,2.78748,8.2,0.03547],
                        [4.02497,2.87059,7.53946,0.03799],
                        [3.7554,2.94998,6.8895,0.04079],
                        [3.49103,3.02498,6.29961,0.04362],
                        [3.2324,3.09497,5.74661,0.04663],
                        [2.97997,3.15943,5.2196,0.04991],
                        [2.73413,3.21784,4.71993,0.05353],
                        [2.49526,3.2697,4.19023,0.05834],
                        [2.2637,3.31448,3.77033,0.06256],
                        [2.03982,3.35157,3.39112,0.06692],
                        [1.824,3.38033,3.04374,0.07153],
                        [1.61684,3.39974,2.72407,0.07638],
                        [1.41857,3.40905,2.41974,0.08203],
                        [1.22931,3.40741,2.41974,0.07821],
                        [1.04915,3.3938,2.41974,0.07466],
                        [0.87807,3.36681,2.41974,0.07158],
                        [0.71581,3.32408,2.90145,0.05783],
                        [0.55808,3.27106,3.15397,0.05276],
                        [0.40349,3.20932,3.46476,0.04805],
                        [0.25105,3.14021,3.74743,0.04466],
                        [0.10011,3.06456,4.07753,0.0414],
                        [-0.04988,2.98309,3.57741,0.04771],
                        [-0.19934,2.89618,2.58098,0.06699],
                        [-0.34872,2.80371,2.11005,0.08326],
                        [-0.52217,2.68919,1.82544,0.11386],
                        [-0.69649,2.5868,1.58913,0.12721],
                        [-0.87203,2.50644,1.58913,0.12149],
                        [-1.04831,2.45601,1.58913,0.11538],
                        [-1.22398,2.44196,1.58913,0.11089],
                        [-1.39645,2.47459,1.86354,0.09419],
                        [-1.56495,2.54096,2.09555,0.08642],
                        [-1.72895,2.63615,2.42448,0.07822],
                        [-1.8885,2.75508,2.95915,0.06725],
                        [-2.04442,2.89139,3.18967,0.06493],
                        [-2.19826,3.03712,2.65557,0.0798],
                        [-2.34378,3.17161,2.27831,0.08697],
                        [-2.48991,3.29746,1.97135,0.09783],
                        [-2.63693,3.40908,1.75325,0.10528],
                        [-2.78482,3.50228,1.57046,0.11132],
                        [-2.93326,3.57355,1.4,0.11761],
                        [-3.08149,3.61921,1.4,0.11079],
                        [-3.22821,3.63623,1.4,0.10551],
                        [-3.37124,3.6202,1.4,0.10281],
                        [-3.50614,3.562,1.64855,0.08912],
                        [-3.63258,3.47218,1.84608,0.08401],
                        [-3.74982,3.35357,2.12262,0.07858],
                        [-3.85781,3.20929,2.4493,0.07358],
                        [-3.95679,3.04208,2.98302,0.06514],
                        [-4.04831,2.85688,3.51866,0.05871],
                        [-4.13558,2.66174,3.13072,0.06828],
                        [-4.22438,2.48078,2.84559,0.07084],
                        [-4.31729,2.31044,2.60307,0.07454],
                        [-4.41458,2.15165,2.39061,0.0779],
                        [-4.51671,2.00602,2.14121,0.08307],
                        [-4.62388,1.87457,1.94018,0.08741],
                        [-4.73617,1.75825,1.74138,0.09284],
                        [-4.85357,1.65797,1.74138,0.08867],
                        [-4.97628,1.57582,1.74138,0.08481],
                        [-5.10426,1.51397,1.74138,0.08163],
                        [-5.23734,1.47603,1.91003,0.07245],
                        [-5.37356,1.45684,1.88097,0.07314],
                        [-5.51215,1.45542,1.88097,0.07368],
                        [-5.65252,1.47206,1.88097,0.07515],
                        [-5.79407,1.5077,1.88097,0.0776],
                        [-5.93591,1.56796,2.03639,0.07568],
                        [-6.0766,1.65218,2.3333,0.07027],
                        [-6.21519,1.75705,2.59868,0.06687],
                        [-6.35106,1.88072,2.95361,0.0622],
                        [-6.48401,2.02065,3.35255,0.05758],
                        [-6.61412,2.17452,3.89505,0.05173],
                        [-6.74174,2.33961,4.78345,0.04363],
                        [-6.86753,2.51259,5.16831,0.04138],
                        [-6.99236,2.68967,4.53441,0.04778],
                        [-7.12216,2.86913,4.00905,0.05525],
                        [-7.25391,3.0442,3.60471,0.06078],
                        [-7.38824,3.21346,3.27814,0.06592],
                        [-7.52582,3.37543,2.92058,0.07276],
                        [-7.66739,3.52847,2.59685,0.08028],
                        [-7.81366,3.67098,2.59685,0.07864],
                        [-7.96536,3.8014,2.59685,0.07703],
                        [-8.12357,3.91724,2.59685,0.07551],
                        [-8.28966,4.01527,2.86267,0.06737],
                        [-8.46179,4.09929,3.01939,0.06344],
                        [-8.63908,4.17089,2.88204,0.06634],
                        [-8.82113,4.23051,2.7199,0.07043],
                        [-9.00753,4.27849,2.52718,0.07616],
                        [-9.19836,4.31354,2.32879,0.08332],
                        [-9.3937,4.33361,2.11124,0.09301],
                        [-9.59328,4.3359,1.91667,0.10414],
                        [-9.79615,4.31609,1.71458,0.11888],
                        [-9.99939,4.26846,1.71458,0.12174],
                        [-10.19548,4.18584,1.71458,0.1241],
                        [-10.36999,4.0639,1.71458,0.12417],
                        [-10.50368,3.90235,1.74804,0.11995],
                        [-10.5924,3.71129,2.06679,0.10193],
                        [-10.64705,3.5024,2.23664,0.09654],
                        [-10.66965,3.27918,2.45128,0.09153],
                        [-10.66228,3.04449,2.72168,0.08627],
                        [-10.62753,2.80096,3.03497,0.08105],
                        [-10.56815,2.55083,3.48717,0.07372],
                        [-10.48831,2.29621,3.92148,0.06804],
                        [-10.39102,2.03849,4.55891,0.06043],
                        [-10.28007,1.77885,5.44007,0.0519],
                        [-10.15918,1.51807,6.8293,0.04209],
                        [-10.03181,1.25661,8.2,0.03546],
                        [-9.90113,0.99471,8.2,0.0357],
                        [-9.77002,0.73266,8.2,0.03574],
                        [-9.63916,0.47048,8.2,0.03574],
                        [-9.50822,0.2083,8.2,0.03574],
                        [-9.37711,-0.0538,8.2,0.03574],
                        [-9.24579,-0.3158,8.2,0.03574],
                        [-9.1142,-0.57767,8.2,0.03574],
                        [-8.98238,-0.83943,8.2,0.03575],
                        [-8.85034,-1.10109,8.2,0.03575]]

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
        SPEED_MULTIPLE = 3
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        STEP_MULTIPLE = 2
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
        reward += steps_reward * STEP_MULTIPLE

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
