#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import os
import random
import shapely
from shapely.geometry import LineString
from assignment import Hungarian
from math import pi
from draw_lqr import draw_car

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../QuinticPolynomialsPlanner/")
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../CubicSpline/")

try:
    from quintic_polynomials_planner import QuinticPolynomial
    import cubic_spline_planner
except ImportError:
    raise

SIM_LOOP = 500

# Parameter
MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.2  # time tick [s]
MAX_T = 5.0  # max prediction time [m]
MIN_T = 4.0  # min prediction time [m]
TARGET_SPEED = 30.0 / 3.6  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2.0  # robot radius [m]
t_thre = 1.0 #碰撞检测时间阈值[s]

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))
        if len(fp.yaw) !=0:
            fp.yaw.append(fp.yaw[-1])
        if len(fp.ds) != 0:
            fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

#进行碰撞处理，有碰撞危险时编号后面的车等待
def AvoidCrash(x,y,v):
    t=[] #储存各轨迹到达各点的时间，最终形式t=[[time1,..],[time2,...],[time3,...]]
    ds=[]
    #得到ds
    for i in range(len(x)):
        #求出连续两个采样点间的直线距离并存储到ds
        dx=np.diff(x[i])
        dy=np.diff(y[i])
        ds.append(np.hypot(dx,dy))
    #得到t
    for i in range(len(x)):
        t_0=[ds[i][j]/v[i][j] for j in range(len(ds[i]))]
        t_i=[0]
        for j in range(1,len(t_0)+1):
            t_i.append(sum(t_0[0:j]))
        t.append(t_i)
    #print(t)
    line=[]
    ins_condition=[] #存储插入情况的列表，每个元素的形式为[j,ind_s,ind_e]
    for i in range(len(x)-1):
        for j in range(i+1,len(x)):
            #得到两条轨迹交叉点
            line.append(LineString(np.column_stack((x[i],y[i]))))
            line.append(LineString(np.column_stack((x[j],y[j]))))
            intersection = line[0].intersection(line[1])
            if isinstance(intersection,shapely.geometry.multipoint.MultiPoint):
                for ii in intersection:
                    a=ii.xy
                    print(a[0][0],a[1][0])
                    plt.plot(*ii.xy,'ro')
                    #计算轨迹上各点与交叉点距离，取最小距离的下标
                    d_i=[(x[i][k]-a[0][0])**2+(y[i][k]-a[1][0])**2 for k in range(len(x[i]))]
                    d_j=[(x[j][k]-a[0][0])**2+(y[j][k]-a[1][0])**2 for k in range(len(x[j]))]
                    index_i=d_i.index(min(d_i))
                    index_j=d_j.index(min(d_j))
                    #计算两条轨迹从起点到该点的时间差
                    t_delta=abs(t[i][index_i]-t[j][index_j])
                    #时间差与阈值比较,小于阈值对j相应的x,y,v作插入处理
                    if t_delta<=t_thre:
                        print("Dangerous!")
                        #找到交叉点阈值时间前t[j]的下标
                        t_jj=[xx for xx in t[j] if xx<(t[j][index_j]-t_thre)]
                        #print(t_jj)
                        ind_s=t[j].index(t_jj[-1]) #插入的起点下标
                        ind_e=index_j #插入的终点下标
                        ins_c=[j,ind_s,ind_e]
                        #print(ins_c)
                        ins_condition.append(ins_c)
            elif isinstance(intersection,shapely.geometry.point.Point):
                a=intersection.xy
                print(a[0][0],a[1][0])
                plt.plot(*intersection.xy,'ro')
                #计算轨迹上各点与交叉点距离，取最小距离的下标
                d_i=[(x[i][k]-a[0][0])**2+(y[i][k]-a[1][0])**2 for k in range(len(x[i]))]
                d_j=[(x[j][k]-a[0][0])**2+(y[j][k]-a[1][0])**2 for k in range(len(x[j]))]
                index_i=d_i.index(min(d_i))
                index_j=d_j.index(min(d_j))
                #计算两条轨迹从起点到该点的时间差
                t_delta=abs(t[i][index_i]-t[j][index_j])
                #时间差与阈值比较,小于阈值对j相应的x,y,v作插入处理
                if t_delta<=t_thre:
                    print("Dangerous!")
                    #找到交叉点阈值时间前t[j]的下标
                    t_jj=[xx for xx in t[j] if xx<(t[j][index_j]-t_thre)]
                    #print(t_jj)
                    ind_s=t[j].index(t_jj[-1]) #插入的起点下标
                    ind_e=index_j #插入的终点下标
                    ins_c=[j,ind_s,ind_e]
                    #print(ins_c)
                    ins_condition.append(ins_c)
            else:
                pass
                print('The lines are not intersection')
            line=[]
    #在相应的位置插入
    for i in range(len(ins_condition)):
        ind=ins_condition[i][0]
        ind_s=ins_condition[i][1]
        ind_e=ins_condition[i][2]
        for k in range(ind_s+1, ind_e+1):
            x[ind].insert(k,x[ind][ind_s])
            y[ind].insert(k,y[ind][ind_s])
            v[ind].insert(k,0)
    return x,y,v


#计算各车路径的函数，输入为起点，终点和障碍物位置
def calc_roadpoint(vp_x,vp_y,target_x,target_y,ob):
    v_path_x, v_path_y,v_speed = [], [],[] # 各车依次路径点的x，y坐标及速度
    car_num=len(vp_x)
    cost_matrix = np.zeros((car_num, car_num))  # 代价矩阵
    for i in range(car_num):
        for j in range(car_num):
            sq=(target_x[j]-vp_x[i])**2+(target_y[j]-vp_y[i])**2
            cost_matrix[i, j] = sq
    hungarian = Hungarian(cost_matrix, is_profit_matrix=False)
    hungarian.calculate()
    ind=hungarian.get_results()
    for i in range(len(ind)):
        ind_start, ind_end = ind[i][0], ind[i][1]
        wx = [vp_x[ind_start],target_x[ind_end]]
        wy = [vp_y[ind_start],target_y[ind_end]]
        tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
        v_x, v_y ,v_s= [], [],[]
        # area = 20.0  # animation area length [m]
        c_speed = 10.0 / 3.6  # current speed [m/s]
        c_d = 1.0  # current lateral position [m]
        c_d_d = 0.0  # current lateral speed [m/s]
        c_d_dd = 0.0  # current lateral acceleration [m/s]
        s0 = 0.0  # current course position
        for k in range(SIM_LOOP):
            path = frenet_optimal_planning(
            csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
            s0 = path.s[1]
            c_d = path.d[1]
            c_d_d = path.d_d[1]
            c_d_dd = path.d_dd[1]
            c_speed = path.s_d[1]

            v_x.append(path.x[1])
            v_y.append(path.y[1])
            v_s.append(c_speed)
            if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
                print("Goal")
                break

            # plt.plot(path.x[1:], path.y[1:], "-og")
            # plt.plot(path.x[1], path.y[1], "vc")
            # plt.title("speed[km/h]:" + str(c_speed * 3.6)[0:4])
            # plt.grid(True)
        v_path_x.append(v_x)
        v_path_y.append(v_y)
        v_speed.append(v_s)
    v_path_x,v_path_y,v_speed=AvoidCrash(v_path_x,v_path_y,v_speed)
    length=[]
    for i in range(len(ind)):
        length.append(len(v_path_x[i]))
    maxx=max(length)
    for i in range(len(ind)):
        v_path_x[i].extend([v_path_x[i][-1]] * (maxx - len(v_path_x[i])))
        v_path_y[i].extend([v_path_y[i][-1]] * (maxx - len(v_path_y[i])))
        v_speed[i].extend([0]*(maxx-len(v_speed[i])))
    return v_path_x,v_path_y,v_speed

def formation_capture(target_position,car_num,r):
    target_x,target_y=[],[]
    theta=2*pi/car_num
    for i in range(car_num):
        target_x.append(target_position[0]+r*math.cos(i*theta))
        target_y.append(target_position[1]+r*math.sin(i*theta))
    return target_x,target_y

def relative_position(i,N_l,L_s):
    P1=lambda x:2*x-1
    P2=lambda x:2*x-2
    com=lambda p,q: 1 if p>q else 0
    modd=lambda x,y:y if x%y==0 else x%y
    xi=(P2(math.ceil(i/N_l))+com(modd(i,N_l),math.ceil(N_l/2)))*L_s
    yi=P1(modd(i,N_l))-com(modd(i,N_l),math.ceil(N_l/2))*P1(math.ceil(N_l/2))
    return [xi,yi]

#绝对坐标:编号、车道数、车间距、1号车坐标、车道宽
def coordinate_position(i,N_l,L_s,f_p,L_w):
    xi,yi=relative_position(i,N_l,L_s)[0],relative_position(i,N_l,L_s)[1]
    XI=f_p[0]-xi
    YI=f_p[1]-(yi-1)*L_w
    return [XI,YI]

#直线队形的位置生成，输入为中心点坐标，直线与x轴夹角(范围-90-90)，车数，前后车间距
def formation_line(target_position,theta,car_num,l_s):
    target_x,target_y=[],[]
    theta=theta*pi/180
    if car_num%2==0:
        for i in range(car_num):
            target_x.append(target_position[0]+((car_num-1)/2-i)*l_s*math.cos(theta))
            target_y.append(target_position[1]+((car_num-1)/2-i)*l_s*math.sin(theta))
    else:
        for i in range(car_num):
            target_x.append(target_position[0] + (car_num // 2 - i) * l_s * math.cos(theta))
            target_y.append(target_position[1] + (car_num // 2 - i) * l_s * math.sin(theta))
    return target_x,target_y

#三角形队形的位置生成,正三角形三个顶点为A,B,C,位于输出列表的前三个位置，几何中心为O,输入为中心点坐标,OA与x轴正向夹角，车数,外接圆半径
def formation_triangle(target_position,theta,car_num,r):
    target_x,target_y=[],[]
    theta=theta*pi/180
    #生成A,B,C三点坐标
    for i in range(3):
        target_x.append(target_position[0]+r*math.cos(theta+i*2*pi/3))
        target_y.append(target_position[1]+r*math.sin(theta+i*2*pi/3))
    m,n=car_num//3-1,car_num%3 #m为三角形每条边上（不含顶点）车的数量，n为需要增加1辆车的边数（比如n=2，则在OA和OB边上各增加一辆车)
    side=[m]*3
    if n==1:
        side[0]=m+1
    elif n==2:
        side[0],side[1]=m+1,m+1
    k=0
    for i in range(2):
        for j in range(i+1,3):
                OA = [target_x[i] - target_position[0], target_y[i] - target_position[1]]
                OB = [target_x[j] - target_position[0], target_y[j] - target_position[1]]
                for l in range(side[k]):
                    target_x.append(target_position[0]+(side[k]-l)*OA[0]/(side[k]+1)+(l+1)*OB[0]/(side[k]+1))
                    target_y.append(target_position[1]+(side[k]-l)*OA[1]/(side[k]+1)+(l+1)*OB[1]/(side[k]+1))
                k=k+1
    return target_x,target_y

#对于路点计算其偏航角
def getYaw(v_x,v_y):
  m,n=len(v_x),len(v_x[0])
  yaw=[]
  for i in range(m):
    yaw_0=[0]
    for j in range(n-1):
      yaw_0.append(math.atan2(v_y[i][j+1]-v_y[i][j],v_x[i][j+1]-v_x[i][j]))
    yaw.append(yaw_0)
  return yaw  

def  getRoadPoint():
    intersection_point=[] #轨迹交叉的点
    v_path_x,v_path_y,v_path=[],[],[]
    car_num=3
    # x = np.linspace(0, 100, 1000)
    v_speed=[]
    N_l, L_w ,L_s,f_p= 5,3.5,3,[5,7]
    # for i in range(N_l + 1):
    #     y = [1 + L_w * (i - 1)] * 1000
    #     plt.plot(x, y, color='blue')
    # tp=[0,0] #编队围捕目标点位置
    vp_x=[coordinate_position(i+1,N_l,L_s,f_p,L_w)[0] for i in range(car_num)]
    vp_y=[coordinate_position(i+1,N_l,L_s,f_p,L_w)[1] for i in range(car_num)]
    #第一次变队形后的目标位置（三角队形）
    tp_1,theta1,r=[35,5],20,7
    target_x1,target_y1=formation_triangle(tp_1,theta1,car_num,r)
    #第二次变队形后的目标位置(直线）
    tp_2,theta2,l_s=[70,5],90,3
    target_x2,target_y2=formation_line(tp_2,theta2,car_num,l_s)
    # obstacle lists
    ob = np.array([
                   [15,3],
                   [15,5],
                   [15,7],
                   [50,1],
                   [50,3],
                   [50,5]
                   ])
    v_path_x1,v_path_y1,v_speed1=calc_roadpoint(vp_x,vp_y,target_x1,target_y1,ob)
    v_path_x2,v_path_y2,v_speed2=calc_roadpoint(target_x1,target_y1,target_x2,target_y2,ob)
    for i in range(car_num):
        v_path_x.append(v_path_x1[i]+v_path_x2[i])
        v_path_y.append(v_path_y1[i]+v_path_y2[i])
        v_speed.append(v_speed1[i]+v_speed2[i])
    return v_path_x,v_path_y,v_speed
  
def talker():
  pub1=rospy.Publisher('Car1',String,queue_size=10)
  pub2=rospy.Publisher('Car2',String,queue_size=10)
  pub3=rospy.Publisher('Car3',String,queue_size=10)
  rospy.init_node('frenet_interlace',anonymous=True)
  rate=rospy.Rate(60)
  v_path_x,v_path_y,v_speed=getRoadPoint()
  num,i=len(v_path_x[0]),0
  while not rospy.is_shutdown():
    if i==num:
      i=0
    car1_str="Position:(%f,%f),Speed: %f m/s" %(v_path_x[0][i],v_path_y[0][i],v_speed[0][i])
    car2_str="Position:(%f,%f),Speed: %f m/s" %(v_path_x[1][i],v_path_y[1][i],v_speed[1][i])
    car3_str="Position:(%f,%f),Speed: %f m/s" %(v_path_x[2][i],v_path_y[2][i],v_speed[2][i])
    rospy.loginfo(car1_str)
    rospy.loginfo(car2_str)
    rospy.loginfo(car3_str)
    pub1.publish(car1_str)
    pub2.publish(car2_str)
    pub3.publish(car3_str)
    i=i+1
    rate.sleep()

if __name__=='__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
