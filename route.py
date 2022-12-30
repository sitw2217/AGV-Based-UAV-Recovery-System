import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve


def dist(a,b):
    #两点间距离
    return ((a[0]-b[0])**2+(a[1]-b[1])**2)**0.5

def AGV_O(r,AGV_pos,AGV_dir):
    #获取小车顺/逆时针圆周运动圆心，返回数组第一位为逆时针运动圆心
    O_left=AGV_pos+r*np.array([-AGV_dir[1],AGV_dir[0]])
    O_right=AGV_pos+r*np.array([AGV_dir[1],-AGV_dir[0]])
    return [O_left,O_right]

def cal_angle(v2,v1):
    # 计算从v2到v1沿顺时针方向的角度
    # 即计算输入参数前减后沿顺时针方向的角度
    a1 = np.arctan2(v1[1], v1[0]) * 180 / np.pi
    a2 = np.arctan2(v2[1], v2[0]) * 180 / np.pi
    a1_std = a1 if a1 > 0 else 360 + a1
    a2_std = a2 if a2 > 0 else 360 + a2


    b = a2_std - a1_std
    # print(f'angle={b if b > 0 else b + 360}')
    # print(f'计算角度的函数中，a1_std={a1_std},a2_std={a2_std}')
    # print(f'计算角度的函数中，a1={a1},a2={a2}')
    return b if b > 0 else b + 360




#应尝试用循环等方法来缩减长度，这样枚举太蠢了

#定义尺寸相关参数，单位cm
WIDTH_FIELD=335
LENGTH_FIELD=336.8
velocity=1 #小车行驶速度



def get_route(drone_pos,drone_dir,AGV_pos,AGV_dir,r_min):
    def cal_path_dist(AGV_dir,drone_dir,O_pos,R_pos,path):
    #O_pos[0]为左转圆心
    
        error=1e-2
        if path[0] == 0:
            OR=R_pos[0]-O_pos[0]
            e1=[-OR[1],OR[0]] #逆时针旋转90
            e2=[OR[1],-OR[0]] #顺时针
            e= e1 if abs(cal_angle(e1,OR) - 90)<error else e2
            theta1=cal_angle(e,AGV_dir)
            theta2=cal_angle(e,-drone_dir)
        elif path[0] == 1:
            OR=R_pos[1]-O_pos[0]
            e1=[-OR[1],OR[0]] #逆时针旋转90
            e2=[OR[1],-OR[0]] #顺时针
            e= e1 if abs(cal_angle(e1,OR) - 90)<error else e2
            theta1=cal_angle(e,AGV_dir)
            theta2=cal_angle(-drone_dir,e)
        elif path[0] == 2:
            OR=R_pos[0]-O_pos[1]
            e1=[-OR[1],OR[0]] #逆时针旋转90
            e2=[OR[1],-OR[0]] #顺时针
            e= e2 if abs(cal_angle(e2,OR) - 270)<error else e1
            theta1=cal_angle(AGV_dir,e)
            theta2=cal_angle(e,-drone_dir)
        elif path[0] == 3:
            OR=R_pos[1]-O_pos[1]
            e1=[-OR[1],OR[0]] #逆时针旋转90
            e2=[OR[1],-OR[0]] #顺时针
            e= e2 if abs(cal_angle(e2,OR) - 270)<error else e1
            theta1=cal_angle(AGV_dir,e)
            theta2=cal_angle(-drone_dir,e)
        path_dist = path[1] * theta1 / 180 + r_min * theta2 / 180
        return [path_dist,theta1,theta2,e]
    def f0(r):
    #小车逆时针（左拐），进入飞机左接驳圆
        return abs(dist(AGV_O(r,AGV_pos,AGV_dir)[0],R_left)-(r+r_min))
    def f1(r):
    #小车逆时针（左拐），进入飞机右接驳圆
        return abs(dist(AGV_O(r,AGV_pos,AGV_dir)[0],R_right)-(r-r_min))
    def f2(r):
    #小车顺时针（右拐），进入飞机左接驳圆
        return abs(dist(AGV_O(r,AGV_pos,AGV_dir)[1],R_left)-(r-r_min))
    def f3(r):
    #小车顺时针（右拐），进入飞机右接驳圆
        return abs(dist(AGV_O(r,AGV_pos,AGV_dir)[1],R_right)-(r+r_min))

    print('looping')
    #随机初始化无人机坐标与方位
   

    R_left=drone_pos+r_min*np.array([-drone_dir[1],drone_dir[0]])
    R_right=drone_pos+r_min*np.array([drone_dir[1],-drone_dir[0]])

    

    # O_left=AGV_pos+r_min*np.array([-AGV_dir[1],AGV_dir[0]])
    # O_right=AGV_pos+r_min*np.array([AGV_dir[1],-AGV_dir[0]])
    print(f'无人机的位置是{drone_pos}')
    print(f'无人机的方向向量是{drone_dir}')

    roots=[]
    sol_fsolve0 = fsolve(f0, r_min)
    if np.isclose(f0(sol_fsolve0),[0])[0]==True and sol_fsolve0<500*r_min and sol_fsolve0>0:
        roots.append([0,sol_fsolve0[0]])

    sol_fsolve1 = fsolve(f1, r_min)
    if np.isclose(f1(sol_fsolve1),[0])[0]==True and sol_fsolve1<500*r_min and sol_fsolve1>0:
        roots.append([1,sol_fsolve1[0]])

    sol_fsolve2 = fsolve(f2, r_min)
    if np.isclose(f2(sol_fsolve2),[0])[0]==True and sol_fsolve2<500*r_min and sol_fsolve2>0:
        roots.append([2,sol_fsolve2[0]])

    sol_fsolve3 = fsolve(f3, r_min)
    if np.isclose(f3(sol_fsolve3),[0])[0]==True and sol_fsolve3<500*r_min and sol_fsolve3>0:
        roots.append([3,sol_fsolve3[0]])

    print(roots)
    #选择一条路径最短的最有路径，在此直接取第一个根
    #可视化结果
    #自动判决

    # fig,ax=plt.subplots()

    # x = [[0, LENGTH_FIELD],[0,0],[0,LENGTH_FIELD],[LENGTH_FIELD,LENGTH_FIELD]] # [1,3]是线段1两端点的x坐标
    # y = [[WIDTH_FIELD,WIDTH_FIELD],[0,WIDTH_FIELD],[0,0],[0,WIDTH_FIELD]]  # [1,3]是线段1两端点的y坐标


    # for i in range(len(x)):
    #     ax.plot(x[i], y[i], color='g')

    dist_list=np.array([])
    for i in range(len(roots)):
        dist_list=np.append(dist_list,cal_path_dist(AGV_dir,drone_dir,AGV_O(roots[i][1],AGV_pos,AGV_dir),[R_left,R_right],roots[i])[0])
    # path_dist=np.min(dist_list)
    try:
        best_index=np.where(dist_list==np.min(dist_list))[0][0]
    except ValueError:
        print('方程没有求解出来，zero-size array')
        return 0
    [dir,r]=roots[best_index]
    #在调用一遍计算路径的函数，来输出角度，可能会造成额外的时间开支，待优化
    [theta1,theta2]=[cal_path_dist(AGV_dir,drone_dir,AGV_O(roots[best_index][1],AGV_pos,AGV_dir),[R_left,R_right],roots[best_index])[1],
                    cal_path_dist(AGV_dir,drone_dir,AGV_O(roots[best_index][1],AGV_pos,AGV_dir),[R_left,R_right],roots[best_index])[2]]
    t1=theta1/180*np.pi*r/velocity
    t2=theta2/180*np.pi*r_min/velocity
    #测试区

    print(f'len(roots)={len(roots)}')
    print(f'best_index={best_index}')
    print(f'dist_list={dist_list}')
    print(f"最短路径的半径和方向是{r},{dir}")
    print(f'圆心角为{[theta1,theta2]}')
    print(f'行驶时间为{[t1,t2]}')

    O_left = AGV_pos + r * np.array([-AGV_dir[1], AGV_dir[0]])
    O_right = AGV_pos + r * np.array([AGV_dir[1], -AGV_dir[0]])
   
    # xx = O_left[0]+ roots[0][1]* np.cos(aa)
    # yy = O_left[1] +roots[0][1]* np.sin(aa)
    # zz = R_left[0] + r_min * np.cos(aa)
    # ww = R_left[1] + r_min * np.sin(aa)
    # z2 = R_right[0] + r_min * np.cos(aa)
    # w2 = R_right[1] + r_min * np.sin(aa)
    # if dir == 0:
    #     xx = O_left[0] + r * np.cos(aa)
    #     yy = O_left[1] + r * np.sin(aa)

    # elif dir == 1:
    #     xx = O_left[0] + r * np.cos(aa)
    #     yy = O_left[1] + r * np.sin(aa)
    # elif dir == 2:
    #     xx = O_right[0] + r * np.cos(aa)
    #     yy = O_right[1] + r * np.sin(aa)
    # elif dir == 3:
    #     xx = O_right[0] + r * np.cos(aa)
    #     yy = O_right[1] + r * np.sin(aa)
    # plt.axis('scaled')
    # plt.arrow(drone_pos[0], drone_pos[1], drone_dir[0] * 60, drone_dir[1] * 60, width=3)
    # plt.arrow(AGV_pos[0], AGV_pos[1], AGV_dir[0] * 60, AGV_dir[1] * 60, width=3)
    # ax.plot(xx,yy)
    # ax.plot(zz,ww,'r')
    # ax.plot(z2,w2,'r')
    # plt.show()
    dist(drone_pos,R_right)
    dist(drone_pos,R_right)
    # plt.arrow(0,0, cal_path_dist(AGV_dir,drone_dir,AGV_O(roots[best_index][1],AGV_pos,AGV_dir),[R_left,R_right],roots[best_index])[3][0],
    #              cal_path_dist(AGV_dir,drone_dir,AGV_O(roots[best_index][1],AGV_pos,AGV_dir),[R_left,R_right],roots[best_index])[3][1], width=3)
    # plt.show()
    O_artan_left = np.arctan2(AGV_pos[1]-O_left[1],AGV_pos[0]-O_left[0])*180/3.1415
    O_artan_right = np.arctan2(AGV_pos[1]-O_right[1],AGV_pos[0]-O_right[0])*180/3.1415
    R_artan_left = np.arctan2(drone_pos[1]-R_left[1],drone_pos[0]-R_left[0])*180/3.1415
    R_artan_right = np.arctan2(drone_pos[1]-R_right[1],drone_pos[0]-R_right[0])*180/3.1415




    if dir == 0:
     
        return O_left , R_left , -O_artan_left, -O_artan_left - theta1, -R_artan_left, -R_artan_left - theta2,  r, 1,0,theta1

    elif dir == 1:
        
        return O_left , R_right ,-O_artan_left, -O_artan_left - theta1, -R_artan_right, -R_artan_right + theta2,  r,1,1,theta1

    elif dir == 2:
       
        return O_right , R_left , -O_artan_right, -O_artan_right + theta1, -R_artan_left, -R_artan_left - theta2,  r,0,0,theta1

    elif dir == 3:
       
        return O_right , R_right , -O_artan_right, -O_artan_right + theta1, -R_artan_right, -R_artan_right + theta2,  r,0,1,theta1


    
   
    # plt.arrow(0,0, cal_path_dist(AGV_dir,drone_dir,AGV_O(roots[best_index][1],AGV_pos,AGV_dir),[R_left,R_right],roots[best_index])[3][0],
    #                 cal_path_dist(AGV_dir,drone_dir,AGV_O(roots[best_index][1],AGV_pos,AGV_dir),[R_left,R_right],roots[best_index])[3][1], width=3)
    # plt.show()
#test

# drone_pos=np.array([np.random.rand(1)[0]*80+100,np.random.rand(1)[0]*80+200])
# theta=np.random.rand(1)[0]*np.pi*2
# drone_dir=np.array([np.cos(theta),np.sin(theta)])

# #从场端获取车头坐标与方位
# AGV_pos=[0.3*WIDTH_FIELD,30]
# AGV_dir=[0,1]
# print(get_route(drone_pos,drone_dir,AGV_pos,AGV_dir))