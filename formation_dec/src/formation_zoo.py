"""
formation collection

"""

import matplotlib.pyplot as plt
import math
import numpy as np

pi = math.pi
show_animation = False
plt.ion()


# 交错编队。绝对坐标: 总车数 1号车坐标、车道数、车间距、车道宽
def formation_stagger(n_car,f_p, N_l=3, L_s=3.5, L_w=3):
    target_x, target_y = [], []
    for i in range(n_car):
        xi,yi=relative_position(i+1,N_l,L_s)[0],relative_position(i+1,N_l,L_s)[1]
        XI=f_p[0]-xi
        YI=f_p[1]-(yi-1)*L_w
        target_x.append(XI)
        target_y.append(YI)
    return target_x, target_y


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
    return target_x,target_y


# 直线队形的位置生成，输入为中心点坐标，直线与x轴夹角(范围-90-90)，车数，前后车间距
def formation_line(target_position,theta,car_num,l_s):
    target_x, target_y=[],[]
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


# 三角队形。正三角形三个顶点为A,B,C,位于输出列表的前三个位置，center为几何中心,输入为中心点坐标,OA与x轴正向夹角，车数,外接圆半径
# 三角队形中空。
def formation_triangle(center,theta,car_num,r):
    target_x,target_y=[],[]
    theta=theta*pi/180
    # 生成A,B,C三点坐标
    for i in range(3):
        target_x.append(center[0]+r*math.cos(theta+i*2*pi/3))
        target_y.append(center[1]+r*math.sin(theta+i*2*pi/3))
    m,n=car_num//3-1,car_num%3 # m为三角形每条边上（不含顶点）车的数量，n为需要增加1辆车的边数（比如n=2，则在OA和OB边上各增加一辆车)
    side=[m]*3
    if n==1:
        side[0]=m+1
    elif n==2:
        side[0],side[1]=m+1,m+1
    k=0
    for i in range(2):
        for j in range(i+1,3):
                OA = [target_x[i] - center[0], target_y[i] - center[1]]
                OB = [target_x[j] - center[0], target_y[j] - center[1]]
                for l in range(side[k]):
                    target_x.append(center[0]+(side[k]-l)*OA[0]/(side[k]+1)+(l+1)*OB[0]/(side[k]+1))
                    target_y.append(center[1]+(side[k]-l)*OA[1]/(side[k]+1)+(l+1)*OB[1]/(side[k]+1))
                k=k+1
    return target_x,target_y


def test_stagger():
    car_num = 3
    f_p = [-5,15]    # 1号车位置
    vp_x, vp_y = formation_stagger(car_num, f_p)
    plt.plot(vp_x, vp_y, 'ro')


# 将队形朝某个方向前后延伸一段距离。 theta: degree
def extend_formation(x,y, theta):
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



if __name__ == '__main__':
    test_stagger()
