import math
import numpy as np
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

def which_way_to_go(reverse, step,init_pos,current_pos, AGV_dir, alpha, O_connect, O_tan, r_connect, r_tan, dir_tan, dir_connect, theta1):
    #alpha:小车转向角，单位是弧度(rad)
    #可调整步长
    h=step
    # step1:判断应该跟着哪个圆走：离哪个圆心近

    ###################
    #11.24采用角度+距离综合判断
    dist_connect = ((O_connect[0] - current_pos[0]) ** 2 + (O_connect[1] - current_pos[1]) ** 2) ** 0.5 - r_connect
    dist_tan = ((O_tan[0] - current_pos[0]) ** 2 + (O_tan[1] - current_pos[1]) ** 2) ** 0.5 - r_tan
    ###################
    if dir_tan == 0:
        v2=list(np.array(init_pos)-np.array(O_tan))
        v1=list(np.array(current_pos)-np.array(O_tan))
    else:
        v1=list(np.array(init_pos)-np.array(O_tan))
        v2=list(np.array(current_pos)-np.array(O_tan))
    theta_AGV=cal_angle(v2,v1)
    # dist_connect = ((O_connect[0] - current_pos[0]) ** 2 + (O_connect[1] - current_pos[1]) ** 2) ** 0.5 - r_connect
    # dist_tan = ((O_tan[0] - current_pos[0]) ** 2 + (O_tan[1] - current_pos[1]) ** 2) ** 0.5 - r_tan
    # if dist_tan < dist_connect:
    if reverse == False:
        #前进:
        if dist_tan > dist_connect:
            #距离判断为接驳圆，则实际绕行接驳圆
            # print('绕行接驳圆')
            circle = 1
            O_current = O_connect
            r_current = r_connect
            O_next = O_tan
            r_next = r_tan
            if reverse == False:
                dir_current = dir_connect
                dir_next = dir_tan
            else:
                dir_current = (dir_connect + 1) % 2
                dir_next = (dir_tan + 1) % 2
        else:
            #距离判断为切圆，则实际绕行路径以角度判断为准

            if theta_AGV<theta1:
                # print('绕行切圆')
                circle = 0
                O_current = O_tan
                r_current = r_tan
                O_next = O_connect
                r_next = r_connect
                if reverse == False:
                    dir_current = dir_tan
                    dir_next = dir_connect
                else:
                    dir_current = ( dir_tan + 1 ) % 2
                    #若切圆太小，或当前小车位置沿切圆绕行方向的剩余路程太短，则仍会判断出现折圆弧的情况
                    #暂时的处理方法是，按折圆弧处理
                    #理想的处理方法是，若步长超过当前小车到目的地的距离，则以目的地为理想下一步位置
                    dir_next = ( dir_connect + 1) % 2
            else:
                # print('绕行接驳圆')
                circle = 1
                O_current = O_connect
                r_current = r_connect
                O_next = O_tan
                r_next = r_tan
                if reverse == False:
                    dir_current = dir_connect
                    dir_next = dir_tan
                else:
                    dir_current = (dir_connect + 1) % 2
                    dir_next = (dir_tan + 1) % 2
    else:
        #后退
        if dist_tan < dist_connect:
            # 距离判断为切圆，则实际绕行切圆
            # print('绕行切圆')
            circle = 0
            O_current = O_tan
            r_current = r_tan
            O_next = O_connect
            r_next = r_connect
            if reverse == False:
                dir_current = dir_tan
                dir_next = dir_connect
            else:
                dir_current = (dir_tan + 1) % 2
                dir_next = (dir_connect + 1) % 2
        else:
            # 距离判断为接驳圆，则实际绕行路径以角度判断为准
            if theta_AGV < theta1:
                # print('绕行切圆')
                circle = 0
                O_current = O_tan
                r_current = r_tan
                O_next = O_connect
                r_next = r_connect
                if reverse == False:
                    dir_current = dir_tan
                    dir_next = dir_connect
                else:
                    dir_current = (dir_tan + 1) % 2
                    dir_next = (dir_connect + 1) % 2
            else:
                # print('绕行接驳圆')
                circle = 1
                O_current = O_connect
                r_current = r_connect
                O_next = O_tan
                r_next = r_tan
                if reverse == False:
                    dir_current = dir_connect
                    dir_next = dir_tan
                else:
                    dir_current = (dir_connect + 1) % 2
                    dir_next = (dir_tan + 1) % 2
    # step2:求解交点坐标
    # step2.1:求两点连线方程
    line_k = (current_pos[1] - O_current[1]) / (current_pos[0] - O_current[0])
    line_b = current_pos[1] - current_pos[0] * line_k

    # step2.2:连线与圆交点
    if current_pos[0] == O_current[0]:
        # 若圆心与小车x相同，连线x也相同
        if current_pos[1] >= O_current[1]:
            cross_pos = (current_pos[0], O_current[1] + r_current)
        else:
            #无所谓
            return current_pos[0], O_current[1] - r_current
    elif current_pos[0] > O_current[0]:
        # 若小车在圆心右边，连线x在圆心右侧
        cross_x = O_current[0] + ((r_current) ** 2 / (1 + (line_k) ** 2)) ** 0.5
        cross_y = line_k * cross_x + line_b
        cross_pos = (cross_x, cross_y)
    else:
        # 若小车在圆心左边，连线x在圆心左侧
        cross_x = O_current[0] - ((r_current) ** 2 / (1 + (line_k) ** 2)) ** 0.5
        cross_y = line_k * cross_x + line_b
        cross_pos = (cross_x, cross_y)

    #v3是【绕行圆圆心】到【圆心到AGV当前位置的射线与绕行圆的交点】的向量
    v3 = [cross_pos[0] - O_current[0], cross_pos[1] - O_current[1]]
    v4 = [1, 0]
    # print(f'a_now_angle={a_now_angle}')

    ########################################################
    #利用传入参数，重新计算一下两元切点坐标；之所以不在主程序里算，是因为改接口太麻烦
    if dir_tan == 0:
        #theta4为两圆切点对应切圆的极坐标角度值
        theta4 = cal_angle(init_pos-O_tan,v4)-theta1

    else:
        theta4 = cal_angle(init_pos - O_tan, v4) + theta1
    shift_pos_x = O_tan[0] + (r_tan * np.cos(theta4 * np.pi / 180))
    shift_pos_y = O_tan[1] + (r_tan * np.sin(theta4 * np.pi / 180))
    shift_pos=[shift_pos_x,shift_pos_y]
    ########################################################
    if dir_current == 0:
        #theta3为【当前位置对应圆上一点】到【两圆切点】的圆弧对应的圆心角
        theta3 = cal_angle(v3,shift_pos-O_current)
    else:
        theta3 = cal_angle(shift_pos - O_current,v3)
    #print(f'theta3={theta3}')
    if h > r_current*theta3/180*np.pi:
        #print('下一步理想路径为折圆弧')
        #默认这种情况只会出现在前进时绕行圆发生变换，或后退时绕行圆发生变换的时候
        h_remain=h-r_current*theta3/180*np.pi
        v5 = shift_pos - O_next
        a_now_angle = cal_angle(v5,v4)
        if dir_next == 0:
            # 折合到第二段绕行路径上的圆心角变化量
            a_nextstep_angle = a_now_angle - h_remain / r_next * 180 / np.pi

        else:
            a_nextstep_angle = a_now_angle + h_remain / r_next* 180 / np.pi
        a_nextstep_x = O_next[0] + (r_next * np.cos(a_nextstep_angle * np.pi / 180))
        a_nextstep_y = O_next[1] + (r_next * np.sin(a_nextstep_angle * np.pi / 180))
        a_nextstep_pos = [a_nextstep_x, a_nextstep_y]

    else: #h < ...
       # print('下一步理想路径为整圆弧')
        a_now_angle = cal_angle(v3, v4)
        if dir_current == 0:  # 顺时针
            a_nextstep_angle = a_now_angle - h/r_current*180/np.pi
        else:
            a_nextstep_angle = a_now_angle + h/r_current*180/np.pi
        # print(f'delta_h={abs(a_nextstep_angle-a_now_angle)/180*np.pi*r_current}')
        # print(f'a_next_angle={a_nextstep_angle}')
        a_nextstep_x = O_current[0] + (r_current * np.cos(a_nextstep_angle * np.pi /180))
        a_nextstep_y = O_current[1] + (r_current * np.sin(a_nextstep_angle * np.pi /180))
        a_nextstep_pos = [a_nextstep_x,a_nextstep_y]
    #step4:判断小车向左/前/右走一步后，哪个离a_nextstep_pos近
    #step4.1:根据小车当前指向，求出小车向左向右的单位矢量
    # print(f'a_nextstep_pos={a_nextstep_pos}')
    rot_mat_l = np.array([[np.cos(alpha),-np.sin(alpha)],[np.sin(alpha),np.cos(alpha)]])
    rot_mat_r = np.array([[np.cos(alpha), np.sin(alpha)], [-np.sin(alpha), np.cos(alpha)]])
    AGV_left = np.matmul(rot_mat_l,AGV_dir)
    AGV_right = np.matmul(rot_mat_r,AGV_dir)
    # print(f'shift_pos={shift_pos}')
    # print(f'a_nextstep_angle={a_nextstep_angle}')
    # print(f'a_now_angle={a_now_angle}')
    #step4.2:向左/前/右走一步后的位置
    AGV_leftforward_pos = (current_pos[0] + h * AGV_left[0], current_pos[1] + h * AGV_left[1])
    #print(AGV_leftforward_pos)

    AGV_rightforward_pos = (current_pos[0] + h * AGV_right[0], current_pos[1] + h * AGV_right[1])
    #print(AGV_rightforward_pos)

    AGV_forward_pos = (current_pos[0] + h * AGV_dir[0], current_pos[1] + h * AGV_dir[1])
    #print(AGV_forward_pos)
    #step4.3:判断哪个近 0直走，1左转，2右转
    dis_l = (AGV_leftforward_pos[0]-a_nextstep_pos[0])**2 + (AGV_leftforward_pos[1]-a_nextstep_pos[1])**2
    #print(dis_l)
    dis_r = (AGV_rightforward_pos[0] - a_nextstep_pos[0]) ** 2 + (AGV_rightforward_pos[1] - a_nextstep_pos[1]) ** 2
    #print(dis_r)
    dis_f = (AGV_forward_pos[0] - a_nextstep_pos[0]) ** 2 + (AGV_forward_pos[1] - a_nextstep_pos[1]) ** 2
    #print(dis_f)
    if dis_f<=dis_l:
        if dis_f<=dis_r:return 0
        else:return 2
    else:
        if dis_l<=dis_r:return 1
        else:return 2








