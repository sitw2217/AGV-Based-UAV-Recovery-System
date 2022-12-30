from __future__ import division
import socket
import time
import Adafruit_PCA9685
socket.setdefaulttimeout(5)

HOST = '0.0.0.0' #连接本地服务器，可通过ipconfig/all看IPV4的地址
PORT = 2217#设置端口号，自己设置即可
pwm = Adafruit_PCA9685.PCA9685()
# socket.AF_INET用于服务器与服务器之间的网络通信
# socket.SOCK_STREAM代表基于TCP的流式socket通信
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT)) #绑定端口，告诉别人，这个端口我使用了，其他人别用了
sock.listen(5)  #监听这个端口，可连接最多5个设备
# try:
#     connection, address = sock.accept()  # 接收客户端的连接请求
# except socket.timeout:
#     print('time_out')
channel1 = 8;
channel2 = 15;
PWM1=300;
PWM2=300;
def set_servo_angle(channel1,channel2, angle):#输入角度转换成12^精度的数值
    date=int(4096*((angle*11)+500)/20000)#进行四舍五入运算 date=int(4096*((angle*11)+500)/(20000)+0.5)
    #print("here");
    pwm.set_pwm(channel1, 0, date);
    pwm.set_pwm(channel2, 0, date);    
def Control(PWM1,PWM2,angle):
    pwm.set_pwm(6, 0, PWM2);
    pwm.set_pwm(2, 0, PWM1);
    set_servo_angle(channel1,channel2, angle);
def model_change():
    pwm.set_pwm(2, 0, 400);
    time.sleep(0.1);
    pwm.set_pwm(2, 0, 370);
    time.sleep(0.1);
    pwm.set_pwm(2, 0, 400);
    time.sleep(0.1);
    pwm.set_pwm(2, 0, 370);
while True:  #死循环，服务器端一直提供服务。
    try:
        connection, address = sock.accept()  # 接收客户端的连接请求
        print('looping')
        # connection.settimeout(2)  #设置时限
        buf = connection.recv(1024)#接收数据实例化
        if buf:  #接收成功
            #connection.send(b'welcome to server,cwh2217!')  #发送消息，b表示bytes类型
            print('Connection success!')
            print(buf.decode())
            temp = []
            paras = []  # 需要输出的参数数组
            l = list(buf.decode())
            for i in range(len(l)):
                if l[i] != ',':
                    temp.append(l[i])
                else:
                    paras.append(float(''.join(temp)))
                    temp = []
            paras.append(float(''.join(temp)))
            #参数传递
            PWM1_BEF=PWM1;
            PWM1=paras[0];
            PWM2=paras[1];
            angle=paras[2];
            #调用马达、舵机控制函数
            if(PWM1_BEF>400 and PWM1<370):
                model_change();
                
            Control(PWM1,PWM2,angle);
            print(paras)
        else:  #接收失败
            connection.send(b'Please go out!')
            print('connetion failed')
            break
    except socket.timeout:  #超时
        print('time out!')
        break

try:
    connection.close()  #关闭连接
    print('disconnected')
    print(f'参数列表为为{paras}')
    steer_paras=[]
    time_paras=[]
    for i in range(len(paras)):
        if i<(len(paras)+1)/2-1:
            steer_paras.append(paras[i])
        else:
            time_paras.append(paras[i])
    print(f'时间参数列表为{time_paras}\n转向参数列表为{steer_paras}')
    print(type(paras))
except NameError:
    print('no connection request')

