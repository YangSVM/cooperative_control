"""
formation collection.


返回numpy数组。(n,2) 每一行代表一个点，每行第一个值代表x,第二个值代表y

待开发新功能:
    通用化函数参数。
    车头朝向
"""

import matplotlib.pyplot as plt
import math
import numpy as np
from enum import Enum

pi = math.pi
show_animation = False
plt.ion()


class FormationType(Enum):
    Stagger =1
    Line=2
    Triangle=3
    DoubleLine=4
    Point=5
    Vertical=6



def gen_formation(center:np.ndarray, theta, n_car:int, d_car:int, formation_type: FormationType):
    '''
    Params:
        theta: 弧度制.队形朝向
    '''
    if formation_type == FormationType.Triangle:
        pos_formation = formation_triangle(center, theta, n_car, d_car)
    elif formation_type == FormationType.Line:
        pos_formation = formation_line(center, theta, n_car, d_car)
    elif formation_type == FormationType.Point:
        pos_formation = formation_point(center, n_car)
    elif formation_type == FormationType.Vertical:
        pos_formation = formation_line(center, theta-pi/2, n_car, d_car)
    else:
        assert False, print('invalid input formation_type')
    return pos_formation



# 交错编队。绝对坐标: 总车数 1号车坐标、车道数、车间距、车道宽
# TODO: 增加旋转变换。没有考虑车道与x轴成夹角的情况
def formation_stagger(center, n_car, N_l=3, L_s=3.5, L_w=3):
    target_x, target_y = [], []
    for i in range(n_car):
        xi,yi=relative_position(i+1,N_l,L_s)[0],relative_position(i+1,N_l,L_s)[1]
        XI=center[0]-xi
        YI=center[1]-(yi-1)*L_w
        target_x.append(XI)
        target_y.append(YI)

    res = np.array([target_x, target_y]).T
    return res


# 相对坐标
def relative_position(i,N_l,L_s):
    P1=lambda x:2*x-1
    P2=lambda x:2*x-2
    com=lambda p,q: 1 if p>q else 0
    modd=lambda x,y:y if x%y==0 else x%y
    xi=(P2(math.ceil(i/N_l))+com(modd(i,N_l),math.ceil(N_l/2)))*L_s
    yi=P1(modd(i,N_l))-com(modd(i,N_l),math.ceil(N_l/2))*P1(math.ceil(N_l/2))
    return [xi,yi]


def formation_capture(target_position,car_num,r):
    target_x,target_y=[],[]
    theta=2*pi/car_num
    for i in range(car_num):
        target_x.append(target_position[0]+r*math.cos(i*theta))
        target_y.append(target_position[1]+r*math.sin(i*theta))
    
    res = np.array([target_x, target_y]).T
    return res


def formation_line(center, theta, n_car, d_car):
    ''' 直线队形的位置生成，输入为中心点坐标，直线与x轴夹角(范围-pi~pi)，车数，前后车间距
    theta: 弧度制
    '''
    center = np.array(center)
    target_x, target_y=[],[]
    # theta=theta*pi/180
    theta = theta + pi/2
    if n_car%2==0:          # 偶数辆车
        for i in range(n_car):      # 从0到n_car-1逐渐增加距离，并保持center时，位置不变
            target_x.append(center[..., 0]+((n_car-1)/2-i)*d_car*math.cos(theta))
            target_y.append(center[..., 1]+((n_car-1)/2-i)*d_car*math.sin(theta))
    else:                               # 奇数辆车
        for i in range(n_car):      # 从0到n_car-1逐渐增加距离，并保持center时，位置不变
            target_x.append(center[...,0] + (n_car // 2 - i) * d_car * math.cos(theta))
            target_y.append(center[...,1] + (n_car // 2 - i) * d_car * math.sin(theta))
    res = np.array([target_x, target_y]).T
    return res



def formation_triangle(center,theta,n_car, d_car):
    '''三角队形。正三角形三个顶点为A,B,C,位于输出列表的前三个位置.三角队形中空。
    Params:
        center为几何中心,输入为中心点坐标,
        theta: OA与x轴正向夹角，弧度制
        n_car: 车数,
        d_car: 车辆间距
    Return:
        pos: np.array. [n_car, 2]
    '''
    center = np.array(center)
    # 计算每条边有多少辆车
    if n_car%3 == 0 :
        n_edge_car = n_car//3
    else:
        n_edge_car = n_car//3 + 1
    # 外接圆半径计算。原则：每条边上车辆距离应该是定值
    r = (d_car * n_edge_car)/math.sqrt(3) * math.sqrt(3)

    target_x,target_y=[],[]
    # theta=theta*pi/180
    min_edge = min(n_car, 3)        # 支持两辆车编队
    # 生成A,B,C三点坐标
    for i in range(min_edge):
        target_x.append(center[..., 0]+r*math.cos(theta+i*2*pi/3))
        target_y.append(center[..., 1]+r*math.sin(theta+i*2*pi/3))
    m,n=n_car//3-1,n_car%3 # m为三角形每条边上（不含顶点）车的数量，n为需要增加1辆车的边数（比如n=2，则在OA和OB边上各增加一辆车)
    side=[m]*3
    if n==1:
        side[0]=m+1
    elif n==2:
        side[0],side[1]=m+1,m+1
    k=0
    i_max = min(2, n_car-1)         # 支持两辆车的三角队形
    j_max = min(3, n_car)
    for i in range(i_max):
        for j in range(i+1,j_max):
                OA = [target_x[i] - center[..., 0], target_y[i] - center[..., 1]]
                OB = [target_x[j] - center[..., 0], target_y[j] - center[..., 1]]
                for l in range(side[k]):
                    target_x.append(center[..., 0]+(side[k]-l)*OA[0]/(side[k]+1)+(l+1)*OB[0]/(side[k]+1))
                    target_y.append(center[..., 1]+(side[k]-l)*OA[1]/(side[k]+1)+(l+1)*OB[1]/(side[k]+1))
                k=k+1
    res = np.array([target_x, target_y]).T
    return res


def test_stagger():
    car_num = 3
    f_p = [-5,15]    # 1号车位置
    vp_x, vp_y = formation_stagger(car_num, f_p)
    plt.plot(vp_x, vp_y, 'ro')


def extend_formation(x,y, theta):
    # 将队形朝某个方向前后延伸一段距离。 theta: degree

    n_car = len(x[0])
    d_dis = 0.2
    dis = np.arange(-2,3)*d_dis
    dx = dis*np.cos(np.deg2rad(theta))
    dy = dis*np.sin(np.deg2rad(theta))
    x_res = []
    y_res = []
    for xi,yi in zip(x,y):
        x_temp = np.zeros([5,n_car])
        y_temp = np.zeros([5,n_car])

        for i_car in range(n_car):
            x_temp[:,i_car] = xi[i_car] + dx
            y_temp[:,i_car] = yi[i_car] + dy

        x_res.extend([xj.tolist() for xj in x_temp])
        y_res.extend([yj.tolist() for yj in y_temp])
    return x_res,y_res


def formation_double_line(points_through, d_car, n_car):
    ''' 通过点，往后垂直延伸，按照d_car放置
    '''
    points_through = np.array(points_through)
    delta_pos = points_through[1, :] - points_through[0, :]
    theta_norm = math.atan2(delta_pos[1], delta_pos[0]) + np.pi/2
    d_norm = np.array([math.cos(theta_norm), math.sin(theta_norm)])

    pos_formation  = np.zeros([n_car, 2])

    for i_car in range(n_car):
        n_d = i_car//2      # 整除
        i_d = i_car%2       # 取余数
        pos_formation[i_car, :] = points_through[i_d, :] + d_car*n_d*d_norm
    
    return pos_formation

def formation_point(point, n_car):
    ''' 返回多个重复点的
    '''
    pos_formations = np.ones([n_car, 2])
    for i in range(n_car):
        pos_formations[i, :] = np.array(point)
    return pos_formations


def calc_pass_order(formations, theta_deg):
    '''给定角度。根据队形位置判定谁先通行。(队形位置越靠前的越先通行). theta 角度制
    params:
        formation: [n_car, 2]. 每辆车的位置
        theta: .队形朝向(车头朝向)
    returns:
        pass_order: 
    '''
    # 投影看目标位置
    theta = theta_deg/180*pi
    theta_vec = np.array([math.cos(theta), math.sin(theta)])
    dot_product = (formations * theta_vec).sum(axis=1)
    pass_order = np.argsort(dot_product)
    pass_order = pass_order[::-1]
    return pass_order



if __name__ == '__main__':
    # test_stagger()
    points_through = [[0,0],[1,5]]
    pos_formation = formation_double_line(points_through, 2, 5)
    plt.figure(1)
    plt.plot(pos_formation[:, 0], pos_formation[:, 1], 'ro')
    plt.axis("equal")
    plt.show()