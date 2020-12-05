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
        racing_track = [[-7.17591, 0.52298, 4.10878, 0.07042],
                        [-7.04789, 0.7761, 3.81443, 0.07436],
                        [-6.91386, 1.01896, 3.49017, 0.07948],
                        [-6.77416, 1.25075, 3.16465, 0.08552],
                        [-6.62874, 1.46985, 2.8488, 0.09231],
                        [-6.47767, 1.6749, 2.50403, 0.10171],
                        [-6.3209, 1.86422, 2.20418, 0.11152],
                        [-6.15828, 2.036, 1.93961, 0.12195],
                        [-5.98952, 2.18824, 1.93961, 0.11718],
                        [-5.81384, 2.31795, 1.93961, 0.11259],
                        [-5.62991, 2.42181, 1.93961, 0.1089],
                        [-5.43479, 2.49528, 2.36482, 0.08817],
                        [-5.23089, 2.54881, 2.60203, 0.08102],
                        [-5.01847, 2.58526, 2.86312, 0.07527],
                        [-4.79764, 2.6066, 3.11086, 0.07132],
                        [-4.56815, 2.61397, 3.40455, 0.06744],
                        [-4.32967, 2.60833, 3.7836, 0.06305],
                        [-4.08205, 2.59085, 4.19665, 0.05915],
                        [-3.82492, 2.56241, 4.76526, 0.05429],
                        [-3.55823, 2.52423, 5.46855, 0.04927],
                        [-3.28202, 2.47752, 6.3146, 0.04436],
                        [-2.99642, 2.4234, 7.42618, 0.03914],
                        [-2.70174, 2.36304, 8.0, 0.0376],
                        [-2.39866, 2.29765, 8.0, 0.03876],
                        [-2.08831, 2.22855, 8.0, 0.03974],
                        [-1.77317, 2.15733, 6.83261, 0.04729],
                        [-1.4557, 2.08647, 5.89314, 0.0552],
                        [-1.14182, 2.01897, 5.18713, 0.0619],
                        [-0.83483, 1.95714, 4.62665, 0.06768],
                        [-0.53778, 1.9029, 4.15058, 0.07275],
                        [-0.25308, 1.85778, 3.74419, 0.07699],
                        [0.01774, 1.82281, 3.43633, 0.07946],
                        [0.2739, 1.79859, 3.05477, 0.08423],
                        [0.51516, 1.78546, 2.73865, 0.08823],
                        [0.74168, 1.78358, 2.41331, 0.09386],
                        [0.95413, 1.79271, 2.14151, 0.0993],
                        [1.15237, 1.81335, 1.90867, 0.10442],
                        [1.3365, 1.84587, 1.69546, 0.11028],
                        [1.50616, 1.89112, 1.48929, 0.11791],
                        [1.6611, 1.95001, 1.48929, 0.11129],
                        [1.80084, 2.0238, 1.48929, 0.10611],
                        [1.92424, 2.11477, 1.48929, 0.10294],
                        [2.02878, 2.22832, 1.91298, 0.08068],
                        [2.12242, 2.35792, 2.20883, 0.07238],
                        [2.20774, 2.50253, 2.59829, 0.06462],
                        [2.2868, 2.66107, 3.207, 0.05524],
                        [2.36154, 2.8312, 4.54254, 0.04091],
                        [2.43403, 3.00817, 5.50102, 0.03476],
                        [2.53618, 3.2458, 5.23377, 0.04942],
                        [2.64502, 3.48503, 4.91468, 0.05348],
                        [2.75998, 3.72341, 4.6009, 0.05752],
                        [2.88068, 3.95899, 4.22075, 0.06271],
                        [3.0069, 4.19, 3.86756, 0.06806],
                        [3.13848, 4.41461, 3.48139, 0.07478],
                        [3.27526, 4.63107, 3.15132, 0.08125],
                        [3.41721, 4.83734, 2.83413, 0.08835],
                        [3.56425, 5.03148, 2.53072, 0.09623],
                        [3.71646, 5.21119, 2.24385, 0.10496],
                        [3.87372, 5.37439, 2.01288, 0.1126],
                        [4.03579, 5.51877, 1.75692, 0.12354],
                        [4.20206, 5.64167, 1.53013, 0.13513],
                        [4.37133, 5.74002, 1.53013, 0.12794],
                        [4.54149, 5.81132, 1.48211, 0.12448],
                        [4.70906, 5.85186, 1.48211, 0.11632],
                        [4.86822, 5.8585, 1.48211, 0.10748],
                        [5.01683, 5.84004, 1.47072, 0.10182],
                        [5.15165, 5.79737, 1.40511, 0.10064],
                        [5.27312, 5.73579, 1.40511, 0.09692],
                        [5.38056, 5.65709, 1.40511, 0.09478],
                        [5.47266, 5.56196, 1.40511, 0.09424],
                        [5.5466, 5.44974, 1.49016, 0.09018],
                        [5.60237, 5.32241, 1.6588, 0.0838],
                        [5.64118, 5.18225, 1.82616, 0.07964],
                        [5.66358, 5.03056, 2.01705, 0.07602],
                        [5.67001, 4.8683, 2.28004, 0.07122],
                        [5.66159, 4.69663, 2.64575, 0.06496],
                        [5.6402, 4.517, 3.03562, 0.05959],
                        [5.60743, 4.33049, 3.61434, 0.05239],
                        [5.56559, 4.13852, 3.64861, 0.05385],
                        [5.51715, 3.94258, 3.11056, 0.06489],
                        [5.46384, 3.70634, 2.69764, 0.08978],
                        [5.4197, 3.47608, 2.37879, 0.09856],
                        [5.38802, 3.25428, 2.10831, 0.10627],
                        [5.37113, 3.04311, 1.86234, 0.11375],
                        [5.37072, 2.84455, 1.66132, 0.11952],
                        [5.38761, 2.66027, 1.45706, 0.127],
                        [5.42214, 2.49182, 1.28996, 0.13331],
                        [5.4745, 2.34094, 1.28996, 0.12381],
                        [5.54445, 2.2093, 1.28996, 0.11556],
                        [5.63234, 2.09985, 1.28996, 0.10882],
                        [5.73847, 2.01652, 1.3402, 0.10069],
                        [5.85902, 1.957, 1.4714, 0.09137],
                        [5.99085, 1.91832, 1.61931, 0.08485],
                        [6.1321, 1.8987, 1.74845, 0.08156],
                        [6.28182, 1.89778, 1.91119, 0.07834],
                        [6.43919, 1.91519, 2.10787, 0.07511],
                        [6.60358, 1.95048, 2.298, 0.07317],
                        [6.77507, 2.00395, 2.4763, 0.07254],
                        [6.95492, 2.0771, 2.45452, 0.0791],
                        [7.14736, 2.17367, 2.25292, 0.09557],
                        [7.34749, 2.25428, 2.03202, 0.10618],
                        [7.54488, 2.31427, 1.83139, 0.11265],
                        [7.73813, 2.35398, 1.64136, 0.1202],
                        [7.9258, 2.3724, 1.47173, 0.12813],
                        [8.10596, 2.36784, 1.32694, 0.13582],
                        [8.27617, 2.33854, 1.32694, 0.13016],
                        [8.43304, 2.28228, 1.32694, 0.12559],
                        [8.57166, 2.19643, 1.32694, 0.12288],
                        [8.68431, 2.0779, 1.56327, 0.1046],
                        [8.77518, 1.93743, 1.56327, 0.10702],
                        [8.84207, 1.77658, 1.56327, 0.11143],
                        [8.8773, 1.59414, 1.59525, 0.11648],
                        [8.87339, 1.39265, 1.84129, 0.10945],
                        [8.83254, 1.17983, 2.01489, 0.10755],
                        [8.75471, 0.96127, 2.23356, 0.10387],
                        [8.6422, 0.74215, 2.425, 0.10157],
                        [8.49708, 0.52682, 2.62189, 0.09904],
                        [8.32213, 0.31895, 2.8251, 0.09617],
                        [8.12063, 0.12139, 3.03366, 0.09302],
                        [7.89604, -0.06383, 3.24563, 0.0897],
                        [7.65175, -0.23546, 3.4309, 0.08702],
                        [7.39064, -0.39262, 3.60578, 0.08452],
                        [7.11526, -0.53481, 3.7805, 0.08198],
                        [6.82795, -0.66189, 3.9602, 0.07933],
                        [6.53081, -0.77404, 4.14891, 0.07655],
                        [6.22574, -0.87171, 4.35086, 0.07362],
                        [5.91436, -0.95555, 4.57102, 0.07055],
                        [5.59812, -1.02639, 4.8155, 0.0673],
                        [5.27826, -1.08518, 5.09172, 0.06387],
                        [4.95592, -1.13296, 5.4083, 0.06025],
                        [4.63224, -1.17083, 5.77466, 0.05643],
                        [4.30846, -1.1999, 6.20099, 0.05243],
                        [3.986, -1.22131, 6.7015, 0.04822],
                        [3.66655, -1.2362, 7.30639, 0.04377],
                        [3.35186, -1.24571, 8.0, 0.03935],
                        [3.04377, -1.25096, 8.0, 0.03852],
                        [2.74802, -1.25268, 8.0, 0.03697],
                        [2.48052, -1.25163, 8.0, 0.03344],
                        [2.23813, -1.24859, 8.0, 0.0303],
                        [2.00959, -1.24395, 8.0, 0.02857],
                        [1.78954, -1.2378, 7.71049, 0.02855],
                        [1.57532, -1.23007, 7.40634, 0.02894],
                        [1.36497, -1.22073, 7.20165, 0.02924],
                        [1.15735, -1.20966, 6.90551, 0.03011],
                        [0.95165, -1.19671, 6.74149, 0.03057],
                        [0.74727, -1.18172, 6.67016, 0.03072],
                        [0.54371, -1.16457, 6.67016, 0.03063],
                        [0.34063, -1.14504, 6.67016, 0.03059],
                        [0.13772, -1.12298, 6.67016, 0.0306],
                        [-0.06528, -1.09832, 6.6858, 0.03059],
                        [-0.26855, -1.07101, 6.79102, 0.0302],
                        [-0.47224, -1.04109, 6.99479, 0.02943],
                        [-0.67644, -1.00867, 7.31645, 0.02826],
                        [-0.88118, -0.97393, 7.7937, 0.02665],
                        [-1.08646, -0.93709, 8.0, 0.02607],
                        [-1.29226, -0.89846, 8.0, 0.02617],
                        [-1.49848, -0.85839, 8.0, 0.02626],
                        [-1.70505, -0.81724, 6.72891, 0.0313],
                        [-1.91184, -0.7754, 5.55174, 0.038],
                        [-2.11836, -0.73396, 4.62043, 0.04559],
                        [-2.32397, -0.69418, 3.97078, 0.05274],
                        [-2.5279, -0.65741, 3.46622, 0.05978],
                        [-2.7294, -0.62488, 3.01372, 0.06773],
                        [-2.92751, -0.59819, 2.65043, 0.07542],
                        [-3.12121, -0.57886, 2.32453, 0.08374],
                        [-3.30948, -0.5684, 2.03364, 0.09272],
                        [-3.49116, -0.56853, 1.78014, 0.10206],
                        [-3.66515, -0.58083, 1.64316, 0.10616],
                        [-3.83023, -0.60709, 1.64316, 0.10173],
                        [-3.98492, -0.64939, 1.64316, 0.09759],
                        [-4.12732, -0.7103, 1.64316, 0.09426],
                        [-4.25618, -0.79128, 1.81361, 0.08391],
                        [-4.37392, -0.88823, 1.98955, 0.07666],
                        [-4.482, -0.99858, 2.15293, 0.07175],
                        [-4.58124, -1.12073, 2.31599, 0.06795],
                        [-4.67215, -1.25348, 2.47519, 0.065],
                        [-4.75505, -1.3959, 2.65954, 0.06196],
                        [-4.83033, -1.54702, 2.80586, 0.06017],
                        [-4.89806, -1.70622, 2.96357, 0.05838],
                        [-4.95839, -1.87285, 3.13487, 0.05653],
                        [-5.01146, -2.04625, 3.32059, 0.05461],
                        [-5.05751, -2.22569, 3.52189, 0.0526],
                        [-5.09682, -2.41046, 3.74228, 0.05048],
                        [-5.12974, -2.59984, 3.99044, 0.04817],
                        [-5.15671, -2.79312, 4.28302, 0.04557],
                        [-5.17823, -2.98964, 4.64736, 0.04254],
                        [-5.19493, -3.18876, 5.12188, 0.03901],
                        [-5.20754, -3.38989, 5.74101, 0.0351],
                        [-5.21681, -3.59254, 6.1789, 0.03283],
                        [-5.22314, -3.79641, 5.89755, 0.03459],
                        [-5.22685, -4.00128, 4.1587, 0.04927],
                        [-5.22814, -4.20699, 3.36703, 0.0611],
                        [-5.2274, -4.37682, 2.80195, 0.06061],
                        [-5.22892, -4.54471, 2.40416, 0.06984],
                        [-5.23476, -4.70899, 2.10874, 0.07796],
                        [-5.24673, -4.86836, 1.84661, 0.08655],
                        [-5.26677, -5.02152, 1.61153, 0.09585],
                        [-5.29668, -5.16722, 1.42417, 0.10444],
                        [-5.33799, -5.3042, 1.24246, 0.11515],
                        [-5.39244, -5.4307, 1.2, 0.11477],
                        [-5.46213, -5.54426, 1.2, 0.11103],
                        [-5.54926, -5.64164, 1.2, 0.10889],
                        [-5.6575, -5.71665, 1.2, 0.10975],
                        [-5.78742, -5.76364, 1.48513, 0.09302],
                        [-5.93125, -5.78823, 1.62931, 0.08956],
                        [-6.08651, -5.79021, 1.77517, 0.08747],
                        [-6.25071, -5.76913, 1.93106, 0.08572],
                        [-6.42095, -5.72495, 2.08121, 0.08451],
                        [-6.59409, -5.65799, 2.21302, 0.08389],
                        [-6.76689, -5.56889, 2.35902, 0.08241],
                        [-6.93636, -5.45926, 2.48488, 0.08123],
                        [-7.10009, -5.33067, 2.59893, 0.08011],
                        [-7.2562, -5.18468, 2.34732, 0.09106],
                        [-7.40306, -5.02242, 2.34732, 0.09323],
                        [-7.53934, -4.845, 2.34732, 0.09531],
                        [-7.66207, -4.65143, 2.34732, 0.09764],
                        [-7.76462, -4.43833, 2.63065, 0.0899],
                        [-7.84869, -4.21013, 2.85448, 0.0852],
                        [-7.91526, -3.9691, 3.07747, 0.08125],
                        [-7.96524, -3.71701, 3.30398, 0.07779],
                        [-7.99943, -3.45528, 3.55091, 0.07433],
                        [-8.01875, -3.18528, 3.76353, 0.07192],
                        [-8.02378, -2.90811, 3.97412, 0.06976],
                        [-8.01506, -2.62485, 4.18239, 0.06776],
                        [-7.99314, -2.33663, 4.38208, 0.06596],
                        [-7.9586, -2.04463, 4.56561, 0.0644],
                        [-7.912, -1.7501, 4.72417, 0.06312],
                        [-7.85391, -1.45438, 4.84819, 0.06216],
                        [-7.78492, -1.15893, 4.92981, 0.06154],
                        [-7.70559, -0.8653, 4.90582, 0.062],
                        [-7.61652, -0.57508, 4.83243, 0.06282],
                        [-7.51834, -0.28982, 4.75845, 0.0634],
                        [-7.41177, -0.0109, 4.63329, 0.06444],
                        [-7.29742, 0.26039, 4.45259, 0.06612]]

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
