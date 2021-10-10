import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi,radians
from numpy.core.numeric import tensordot
import pymap3d as pm
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import CubicSpline


class gps_tracking:

    def __init__(self):
        self.min_index = 0
        self.cut_length = 50
        self.track_distance = []


    def pure_pursuit(self, current_position, current_index, total_track):
        min_ld = 5          # 최소 Look ahead distance
        max_ld = 20         # 최대 Look ahead distance
        L = 1.03            # Wheelbase (m)
        # cut_length = 50     # 현재 위치에서부터 얼마나 앞을 볼 것인지 (현재 50인 경우에는 50번째 index 경로까지 보겠다는 뜻)
        steering_gain = 2   # 조향각 gain 

        # 특정 시간, 특정 위치에 있는 차량의 순간을 가정해보자. 이때 각각의 total_track 에 대해 차량 위치와의 distance 를 계산하고,
        # 그 중에서 distance 가 가장 최소가 되는 지점을 찾고 그때의 index 를 선택 -> 차량과 가장 가까운 포인트를 지정할 수 있다
        #######GPS&LaneTracking#######
        # if curr_position > 0.6 or curr_position < -0.6:
        #     wheelcmd =
        # else:
        #     pure pursuit

        current_angle_index = 0
        for i in range(current_index, len(total_track[0])):
            dis = ((total_track[0][current_index] - total_track[0][i]) ** 2 + (total_track[1][current_index] - total_track[1][i]) ** 2) ** 0.5
            if dis >= 5:
                break
            current_angle_index = i
        
        # first_angle = atan2(total_track[0][5] - total_track[0][0], total_track[1][5] - total_track[1][0])
        first_angle = 0

        # imu = atan2((total_track[0][current_angle_index] - total_track[0][current_index]), (total_track[1][current_angle_index] - total_track[1][current_index]))
        imu = 0*pi/180
        offset_angle = 20 * pi / 180


        car_yaw = imu + offset_angle
        target_velocity = self.curve_velocity(total_track)


        
        # [current_latitude, current_longitude] = [37.293949, 126.9774586]
        # [current_latitude, current_longitude] = [37.294, 126.9775]
        # [current_latitude, current_longitude] = [37.2392374651, 126.773160155]
        
        # rospy.loginfo("%f", current_latitude)
        current_height = 0

        # reference_latitude = 37.23861340 # deg
        # reference_longitude = 126.77248216  # deg, 연습장 기준

        reference_latitude = 37.293607 # deg
        reference_longitude = 126.977482  # deg, 학교 기준


        reference_height = 0     # meters

        # current_gps2coordinate = pm.geodetic2enu(current_latitude, current_longitude, current_height, reference_latitude, reference_longitude, reference_height, ell=pm.utils.Ellipsoid('grs80'))
        # p_curr = np.array([current_gps2coordinate[0], current_gps2coordinate[1]])
        p_curr = current_position


        if self.min_index + self.cut_length < len(self.total_track[0]):
            self.track_distance = ((p_curr[0] - total_track[0][self.min_index:self.min_index + self.cut_length]) ** 2 + (p_curr[1] - total_track[1][self.min_index:self.min_index + self.cut_length]) ** 2) ** 0.5
        else:
            self.track_distance = ((p_curr[0] - total_track[0][self.min_index:-1]) ** 2 + (p_curr[1] - total_track[1][self.min_index:-1]) ** 2) ** 0.5
        self.track_distance = self.track_distance.tolist()
        self.min_index += self.track_distance.index(min(self.track_distance))

        # min_index_2, _ = find_peaks(-np.array(distance))

        # print(min_index_2)
        # local_min = []

        # for i in range(len(local_min_index)):
        #     local_min.append(out_vel_plan[local_min_index[i]])

        # plt.figure(3)
        # plt.plot(self.distance)


        if self.min_index > len(total_track[0]) - 10:
            print('끝')

        # 이제는 min_index 를 기준으로 해서 이 지점부터의 total_track 을 보면 된다

        x_summary = total_track[0][self.min_index: self.min_index + self.cut_length]
        y_summary = total_track[1][self.min_index: self.min_index + self.cut_length]
        
        x_local = cos(first_angle + car_yaw) * (total_track[0] - p_curr[0]) - sin(first_angle + car_yaw) * (total_track[1] - p_curr[1])
        y_local = sin(first_angle + car_yaw) * (total_track[0] - p_curr[0]) + cos(first_angle + car_yaw) * (total_track[1] - p_curr[1])


        # x_local = cos(car_yaw) * (x_summary - p_curr[0]) - sin(car_yaw) * (y_summary - p_curr[1])
        # y_local = sin(car_yaw) * (x_summary - p_curr[0]) + cos(car_yaw) * (y_summary - p_curr[1])
        # 이제는 위의 좌표 데이터들을 차량의 입장에서 보는 것으로 바꿔야한다 (전역 좌표계 -> 차량 좌표계로 시점 변환)


        lookahead_distance = target_velocity[self.min_index] * 5/6
        # lookahead_distance = 10
        if lookahead_distance < min_ld:
            lookahead_distance = min_ld
        elif lookahead_distance > max_ld:
            lookahead_distance = max_ld


        for i in range(self.min_index, len(total_track[0])):
            dis = ((total_track[0][self.min_index] - total_track[0][i]) ** 2 + (total_track[1][self.min_index] - total_track[1][i]) ** 2) ** 0.5
            if dis >= lookahead_distance:
                break
            lookahead_index = i

        print(self.min_index, lookahead_index)
        # for i in range(cut_length):
        #     dis = ((x_summary[0] - x_summary[i]) ** 2 + (y_summary[0] - y_summary[i]) ** 2) ** 0.5
        #     if dis >= lookahead_distance:
        #         break
        #     lookahead_index = i
        # Look ahead distance 거리에 위치한 목표 지점

        x_target = x_local[lookahead_index]
        y_target = y_local[lookahead_index]
        # 차량과 목표 지점 사이의 각도 alpha

        alpha = atan2(x_target, y_target)

        # 조향각 계산
        wheelcmd = atan2(2 * L * sin(alpha), lookahead_distance) * steering_gain * 180 / pi  # L은 축거, Ld는 Look ahead distance (따로 지정), gain은 조향 게인
        
        # plt.figure(1)
        # plt.axis((70,90,120,140))
        # plt.plot(total_track[0], total_track[1])
        # # plt.scatter(total_track[0][0], total_track[1][0])
        # plt.scatter(p_curr[0], p_curr[1], c='g')
        # plt.scatter(total_track[0][min_index], total_track[1][min_index], c='b')
        # plt.scatter(total_track[0][lookahead_index], total_track[1][lookahead_index], c='r')
        # plt.show()

        print('현재 속도 = ', target_velocity[self.min_index], 'm/s')
        print('look ahead distance = ', lookahead_distance, 'm')
        print('car yaw = ', car_yaw * 180/pi, 'degree')
        print('조향각 = ', wheelcmd, 'degree')
        print('alpha = ', alpha * 180/pi, 'degree')
        print('min_index = ', self.min_index)
        print('------------------------------------------------------')
        self.gps_monitor4pure_pursuit([x_local, y_local], lookahead_index)

        return wheelcmd



    def stanley(self, current_position, current_index, total_track):
        lateral_error_gain = 1
        velocity_plus_gain = 5
        stanley_distance = 1                # lateral error, heading error를 구할 때
        L = 1.03            # Wheelbase (m)
        cut_length = 50     # 현재 위치에서부터 얼마나 앞을 볼 것인지 (현재 50인 경우에는 50번째 index 경로까지 보겠다는 뜻)

        # 특정 시간, 특정 위치에 있는 차량의 순간을 가정해보자. 이때 각각의 total_track 에 대해 차량 위치와의 distance 를 계산하고,
        # 그 중에서 distance 가 가장 최소가 되는 지점을 찾고 그때의 index 를 선택 -> 차량과 가장 가까운 포인트를 지정할 수 있다
        #######GPS&LaneTracking#######
        # if curr_position > 0.6 or curr_position < -0.6:
        #     wheelcmd =
        # else:
        #     pure pursuit

        current_angle_index = 0
        for i in range(current_index, len(total_track[0])):
            dis = ((total_track[0][current_index] - total_track[0][i]) ** 2 + (total_track[1][current_index] - total_track[1][i]) ** 2) ** 0.5
            if dis >= 5:
                break
            current_angle_index = i
        
        # imu = atan2((total_track[0][current_angle_index] - total_track[0][current_index]), (total_track[1][current_angle_index] - total_track[1][current_index])) 
        imu = 0*pi/180

        car_yaw = imu
        target_velocity = np.array(self.curve_velocity(total_track))     
        
        # [current_latitude, current_longitude] = [37.293949, 126.9774586]
        # [current_latitude, current_longitude] = [37.294, 126.9775]
        # [current_latitude, current_longitude] = [37.2392374651, 126.773160155]
        
        # rospy.loginfo("%f", current_latitude)
        current_height = 0

        # reference_latitude = 37.23861340 # deg
        # reference_longitude = 126.77248216  # deg, 연습장 기준

        reference_latitude = 37.293607 # deg
        reference_longitude = 126.977482  # deg, 학교 기준

        reference_height = 0     # meters

        # current_gps2coordinate = pm.geodetic2enu(current_latitude, current_longitude, current_height, reference_latitude, reference_longitude, reference_height, ell=pm.utils.Ellipsoid('grs80'))
        # p_curr = np.array([current_gps2coordinate[0], current_gps2coordinate[1]])
        p_curr = current_position

        p_curr_front = [p_curr[0] + L * sin(car_yaw), p_curr[1] + L * cos(car_yaw)]


        # x_curr = current_position[0] + L * sin(car_yaw)
        # y_curr = current_position[1] + L * cos(car_yaw)
        # p_curr = [x_curr, y_curr]
        # print(p_curr)

        if self.min_index + self.cut_length < len(total_track[0]):
            self.track_distance = ((p_curr_front[0] - total_track[0][self.min_index:self.min_index + self.cut_length]) ** 2 + (p_curr_front[1] - total_track[1][self.min_index:self.min_index + self.cut_length]) ** 2) ** 0.5
        else:
            self.track_distance = ((p_curr_front[0] - total_track[0][self.min_index:-1]) ** 2 + (p_curr_front[1] - total_track[1][self.min_index:-1]) ** 2) ** 0.5
        self.track_distance = self.track_distance.tolist()
        self.min_index += self.track_distance.index(min(self.track_distance))


        if self.min_index > len(total_track[0]) - 10:
            print('끝')

        # 이제는 min_index 를 기준으로 해서 이 지점부터의 total_track 을 보면 된다

        # x_summary = total_track[0][min_index: min_index + cut_length]
        # y_summary = total_track[1][min_index: min_index + cut_length]

        x_local = cos(car_yaw) * (total_track[0] - p_curr_front[0]) - sin(car_yaw) * (total_track[1] - p_curr_front[1])
        y_local = sin(car_yaw) * (total_track[0] - p_curr_front[0]) + cos(car_yaw) * (total_track[1] - p_curr_front[1])

        stanley_index = 0
        for i in range(self.min_index, len(total_track[0])):
            dis = ((total_track[0][self.min_index] - total_track[0][i]) ** 2 + (total_track[1][self.min_index] - total_track[1][i]) ** 2) ** 0.5
            if dis >= stanley_distance:
                break
            stanley_index = i

        lateral_error = x_local[stanley_index]
        heading_error = atan2(x_local[stanley_index] - x_local[self.min_index], y_local[stanley_index] - y_local[self.min_index])
        
        print('heading_error = ', heading_error * 180/pi)
        print('lateral_error = ', lateral_error)
        # 조향각 계산
        wheelcmd = (heading_error + atan2(lateral_error_gain * lateral_error, (target_velocity[self.min_index] / 3.6 + velocity_plus_gain))) * 180 / pi 
        
        # plt.figure(1)
        # plt.axis((70,90,120,140))
        # plt.plot(total_track[0], total_track[1])
        # # plt.scatter(total_track[0][0], total_track[1][0])
        # plt.scatter(p_curr[0], p_curr[1], c='g')
        # plt.scatter(total_track[0][min_index], total_track[1][min_index], c='b')
        # plt.scatter(total_track[0][lookahead_index], total_track[1][lookahead_index], c='r')
        # plt.show()

        print('현재 속도 = ', target_velocity[self.min_index], 'km/h')
        # print('look ahead distance = ', lookahead_distance, 'm')
        print('car yaw = ', car_yaw * 180/pi, 'degree')
        print('조향각 = ', wheelcmd, 'degree')
        # print('alpha = ', alpha * 180/pi, 'degree')
        print('------------------------------------------------------')
        # self.gps_monitor4stanley([x_local, y_local], stanley_index)

        plt.scatter(x_local, y_local)
        plt.scatter(0,0, c = 'g')
        plt.show()

        return wheelcmd




    def curve_velocity(self, global_path):
        car_max_speed = 12 #km/h
        car_min_speed = 6
        car_stopline_speed = 8
        out_vel_plan = []
        road_friction = 0.7
        velocity_gain = 1
        interval = 70 #속도 극솟값 기준 앞뒤로 10m, 총 20m  

        dx_dt = np.gradient(global_path[0])
        dy_dt = np.gradient(global_path[1])
        dr_dt = np.array([dx_dt, dy_dt]).T

        x_dot = np.gradient(global_path[0])
        y_dot = np.gradient(global_path[1])
        x_doubledot = np.gradient(x_dot)
        y_doubledot = np.gradient(y_dot)

        ds_dt = (dx_dt ** 2 + dy_dt ** 2) ** 0.5

        curvature = abs(x_dot * y_doubledot - y_dot * x_doubledot) / (x_dot ** 2 + y_dot ** 2) ** 1.5
        radius = 1/curvature

        out_vel_plan = velocity_gain * (radius * 9.8 * road_friction) ** 0.5
        for i in range(len(out_vel_plan)):
            if out_vel_plan[i] > car_max_speed:
                out_vel_plan[i] = car_max_speed

        local_min_index, _ = find_peaks(-np.array(out_vel_plan), distance = interval * 2)
        local_min = []

        # for i in range(len(out_vel_plan)):
        #     if global_path[2][i] == 2:
        #         local_min_index.append()

        for i in range(len(local_min_index)):
            local_min.append(out_vel_plan[local_min_index[i]])

        for i in range(len(out_vel_plan)):
            if i not in local_min_index:
                out_vel_plan[i] = car_max_speed

        for i in local_min_index:
            if i > interval and i < len(out_vel_plan) - interval:
                for j in range(-int(interval/4) , int(interval/4)):
                    out_vel_plan[i - j] = out_vel_plan[i]

                for j in range(-interval, interval):
                    if (j in range(-interval, -int(interval/4)) or j in range(int(interval/4), interval)) and out_vel_plan[i] + (abs(j) - int(interval/4))/2 < car_max_speed:
                        out_vel_plan[i - j] = out_vel_plan[i] + (abs(j) - int(interval/4))/2

        for i in range(len(out_vel_plan)):
            if out_vel_plan[i] < car_min_speed:
                out_vel_plan[i] = car_min_speed

        for i in range(len(out_vel_plan)):
            if global_path[2][i] == 2:
                out_vel_plan[i] = car_stopline_speed

        # out_vel_plan[-1] = 0
        # for i in range(1, interval):
        #     if i - 1 < car_max_speed:
        #         out_vel_plan[len(out_vel_plan) - i] = i - 1

        out_vel_plan = out_vel_plan.tolist()
        del(out_vel_plan[0:5])
        for i in range(5):
            out_vel_plan.append(car_max_speed)
        
        plt.figure(3)
        plt.plot(global_path[0], global_path[1])
        plt.axis('scaled')

        plt.figure(4)
        plt.plot(out_vel_plan)
        plt.show()
        return out_vel_plan


    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi

        while angle < -pi:
            angle += 2 * pi

        return angle


    def pathreader(self):
        f = open("/home/changjun/catkin_ws/src/HEVEN-AutonomousCar-2021/path/GPSlogging/practice/Delivery1.csv", 'r')

        path = []
        for i in f.readlines():
            line = i.split(',')
            # path.append([float(line[0]), float(line[1])])
            path.append([float(line[1].rstrip()), float(line[0].rstrip()), int(line[2].rstrip())])  # For lon, lat
        f.close()
        path2 = np.array(path).T

        # lat0 = 37.293607 # deg
        # lon0 = 126.977482  # deg
        lat0 = 37.23861340 # deg
        lon0 = 126.77248216  # deg
        h0 = 0     # meters

        # The point of interest
        lat = path2[0]
        lon = path2[1]
        h = 0      # meters

        gps2coordinate = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0, ell=pm.utils.Ellipsoid('grs80'))
        track = [gps2coordinate[0], gps2coordinate[1], path2[2]]
        # track = path2

        total_track = [[track[0][0], track[1][0], track[2][0]]]
        track_index = 0
        while ((track[0][track_index] - track[0][-1]) ** 2 + (track[1][track_index] - track[1][-1]) ** 2) ** 0.5 >= 0.5:
            for i in range(track_index, len(track[0])):
                dis = ((track[0][track_index] - track[0][i]) ** 2 + (track[1][track_index] - track[1][i]) ** 2) ** 0.5
                if dis >= 0.5:
                    total_track.append([track[0][i], track[1][i], track[2][i]])
                    track_index = i
                    break
        
        # global_path = np.array(total_track).T
        global_path = np.array(track)
        print(global_path[0])
        
        total_distance = 0
        for i in range(len(global_path[0]) - 1):
            total_distance += ((global_path[0][i] - global_path[0][i + 1])  ** 2 + (global_path[1][i] - global_path[1][i + 1])  ** 2)
        

        # reference_angle = 28.3168 * pi/180
        reference_angle = atan2(global_path[0][5] - global_path[0][0], global_path[1][5] - global_path[1][0])
        # print(reference_angle * 180 / pi)
        global_path_rotation = [cos(reference_angle) * global_path[0] - sin(reference_angle) * global_path[1], sin(reference_angle) * global_path[0] + cos(reference_angle) * global_path[1]]

        return global_path, total_distance


    def gps_monitor4pure_pursuit(self, track, index):
        plt.ion()
        plt.scatter(track[0], track[1], s=1)
        plt.scatter(track[0][index], track[1][index], c='r')
        plt.scatter(0, 0, c='g')
        plt.plot([0, track[0][index]], [0, track[1][index]], c='k')
        # plt.xlim(-10, 10)
        # plt.ylim(-5, 15)
        plt.axis('scaled')
        plt.grid()
        plt.show()
        plt.pause(0.1)
        plt.clf()

    def gps_monitor4stanley(self, track, index):
        # plt.ion()
        plt.scatter(track[0], track[1], s = 1)
        plt.scatter(track[0][index], track[1][index], c='r')
        plt.plot(0,0, c = 'g', marker = 's', markersize = 5)
        # plt.plot(0,1.03, c = 'g', marker = 's', markersize = 5)
        # plt.plot([0, track[0][index]], [0, track[1][index]], c = 'k')
        plt.xlim(-10, 10)
        plt.ylim(-5, 15)
        plt.grid()
        plt.show()
        plt.pause(0.1)
        plt.clf()


Gps_tracking = gps_tracking()

total_track, total_distance = Gps_tracking.pathreader()

# plt.figure(4)
# plt.plot(total_track[0], total_track[1])
# plt.axis = 'scaled'
# plt.show()

index = 0
# wheelcmd = Gps_tracking.pure_pursuit([total_track[0][index]+20 , total_track[1][index]-20], index, total_track)
wheelcmd = Gps_tracking.stanley([total_track[0][index] - 0, total_track[1][index] + 0], index, total_track)


# for i in range(0, len(total_track[0])):
#     print(Gps_tracking.distance)
#     # wheelcmd = Gps_tracking.pure_pursuit([total_track[0][i*1], total_track[1][i*1]], i*1, total_track)
#     wheelcmd = Gps_tracking.stanley([total_track[0][i*1], total_track[1][i*1]], i*1, total_track)
