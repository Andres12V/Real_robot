# https://www.youtube.com/watch?v=V-BQfwlCDj4&ab_channel=Rub%C3%A9nLoredoRub%C3%A9nLoredo

import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
import numpy as np
from sympy.matrices import Matrix
from sympy import cos, symbols, sin, simplify, pi

GPIO.setmode(GPIO.BOARD)

L_pwm = 33
R_pwm = 31
L_En = 21
R_En = 19
Enc_A = 23
Enc_B = 29
GPIO.setup(L_pwm, GPIO.OUT)
GPIO.setup(R_pwm, GPIO.OUT)
GPIO.setup(L_En, GPIO.OUT)
GPIO.setup(R_En, GPIO.OUT)
GPIO.setup(Enc_A, GPIO.IN)
GPIO.setup(Enc_B, GPIO.IN)

# Segundo motor

GPIO.output(L_En,1)
GPIO.output(R_En,1)

iA = 0
iB = 0
my_time = list()

pos = list()
pos_e=list()
Pos_radf= list()
ticksA=list()
ticksB=list()
vel = list()
x2e_vec = list()

vel_R = list()
# vel_R.append(0.0)

ref = 3
ref_vec = list()
# ref_vec.append(ref)

e_vec =list()
# e_vec.append(ref)

u_vec =list()
# u_vec.append(0.0)



t = 0
# my_time.append(t)
## ts=0.4
kp=0.25
ki=4
int_e=0

# Kalman filter variables
R = 0.0056
Q = 0.1728

mein_x_hat_1 = []
mein_x_hat_2 = []
mein_x_hat_3 = []
mein_P_1 = []
mein_P_2 = []
mein_P_3 = []
filtered = []

def inputChng_M1A(channel):
    global iA
    if ref>=0:
        iA += 1
    if ref<0:
        iA = iA-1
def inputChng_M1B(channel):
    global iB
    if ref>=0:
        iB += 1
    if ref<0:
        iB = iB-1

p = GPIO.PWM(L_pwm, 1000)
q = GPIO.PWM(R_pwm, 1000)
# p.ChangeDutyCycle(16)
p.start(0)

GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=inputChng_M1A, bouncetime=1)
GPIO.add_event_detect(Enc_B, GPIO.RISING, callback=inputChng_M1B, bouncetime=1)
c=0
#u=30
# kc=10.6832
# k1p=17.9058
# k2p=47.2376
kc=0.0976
k1p=-0.7745
k2p=-17.3387

f=0.2865

#PPR=34.0*22.0
PPR=600.0
try:
    while 1:
        #print('Pulses', Pulses)
        start = time.time()
        Pulses=iA+iB
        ticksA.append(Pulses)
        Turns=Pulses/PPR
        Pos_rad=Turns*2*np.pi
        Pos_radf.append(Pos_rad)
        RPM=(179*2*60)/PPR
        #print(t)
        print('Turns', Turns)
        #print('Velocity', RPM)
        try:
            try:
                velocity=x2e
            except:
                velocity=0
            try:
                velocity_dx = ( (Pos_rad-Pos_radf[-2])/0.01 )
            except:
                velocity_dx=0

            # Ref vector:
            #ref+=0.001
            ## Error:
            e = ref - velocity
            c+=1
            # if c*0.05==4:
            #     u=u+13
            #     print('u',u)
            #     c=0
            try:
                xcf=xc+e
            except:
                xcf=e
            try:
                u=kc*xc-k1p*x2e-k2p*x3e
            except:
                u=0
            #if t<1:
                #u = 0
            #if t>1:
                #u = 30
            #if t>2:
                #u = 0
            #u = 12
            # Observador
            try:
                x1f=0.0534*x1e+0.0081*x2e+0.0031*x3e+0.0006*u+0.9466*Pos_rad
                x2f=-6.7435*x1e+0.6219*x2e+0.2561*x3e+0.1289*u+6.8486*Pos_rad
                x3f=7.7435*x1e-0.0373*x2e-0.0159*x3e+0.0282*u-7.7435*Pos_rad
            except:
                x1f=0.0006*u+0.9466*Pos_rad
                x2f=0.1289*u+6.8486*Pos_rad
                x3f=0.0282*u-7.7435*Pos_rad
            u_1=np.abs(u)
            if u_1>=50:
                u_1=50
            if u_1<=0:
                u_1=0
            x1e=x1f
            x2e=x2f
            x3e=x3f
            xc=xcf

            pos_e.append(x1e)
            print('x2f',x2f)
            p.ChangeDutyCycle(u_1)
            #print('---', u, int_e, e)

            # if t>=5:
            #     ref = 0.0
            # if t>=8:
            #     p.stop()
            #     p.ChangeDutyCycle(0)
            # if t>=10:
            #     ref = -1.5
            #     q.start(0)
            #     q.ChangeDutyCycle(u)
        except:
            pass

        # Kalman Filter #
        try:
            x_hat_ant_1 = mein_x_hat_1[-1]
            x_hat_ant_2 = mein_x_hat_2[-1]
            #x_hat_ant_3 = mein_x_hat_3[-1]
            p1_ant = mein_P_1[-1]
            p2_ant = mein_P_2[-1]
            #p3_ant = mein_P_3[-1]
        except:
            x_hat_ant_1 = 0
            x_hat_ant_2 = 0
            #x_hat_ant_3 = 0
            p1_ant = 0
            p2_ant = 0
            #p3_ant = 0

        x_hat_p1= 0.6219*x_hat_ant_1+0.2561*x_hat_ant_2+0.1289*u
        x_hat_p2= -0.0373*x_hat_ant_1-0.0159*x_hat_ant_2+0.0282*u
        #x_hat_p3=-0.0373*x_hat_ant_2-0.0159*x_hat_ant_3+0.0282*u

        p_p1= 0.4571*p1_ant-0.0274*p2_ant+Q
        p_p2=-0.0274*p1_ant+0.0016*p2_ant
        #p_p3=-0.0003*p1_ant-0.0274*p2_ant+0.0016*p3_ant

        K1=p_p1*(p_p1+R)**-1
        K2=0*(p_p2+R)**-1
        #K3=(p_p3+R)**-1;

        x_hat_1=x_hat_p1+K1*(velocity-x_hat_p1)
        x_hat_2=x_hat_p2+K2*(velocity)
        #x_hat_3=x_hat_p3+K3*(Pos_rad)

        p1=(1-K1)*p_p1
        p2=(1)*p_p2
        #p3=(1)*p_p3

        mein_x_hat_1.append(x_hat_1)
        mein_x_hat_2.append(x_hat_2)
        #mein_x_hat_3.append(x_hat_3)

        mein_P_1.append(p1)
        mein_P_2.append(p2)
        #mein_P_3.append(p3)

        filtered.append(x_hat_1)

        u_vec.append(u_1)
        #Q = np.cov(u_vec)

        vel.append(velocity)
        ref_vec.append(ref)
        my_time.append(t)
        pos.append(Pos_rad)
        #R = np.cov(pos)

        x2e_vec.append(velocity_dx)
        #R = np.cov(vel)
        #Q = np.cov(vel)
        #print('R eeeeeeeee',Q)
        # Pos in m
        pos_m=round(Pos_rad*0.035, 3)
        print('Pos_m', pos_m)

        L=0.33
        try:
            Dr=-(ticksA[-1]-ticksA[-2])*2*np.pi*0.0325/PPR
            Dl=(ticksB[-1]-ticksB[-2])*2*np.pi*0.0325/PPR
            print('Angle_Dl',((Dr-Dl)*180/np.pi)/L)
        except:
            pass
        # angle=(-Pos_rad2-Pos_rad)*0.0325/L
        # angle_Deg=angle*180/np.pi
        # print('Angle',angle_Deg)
        time.sleep(0.01)
        stop = time.time()
        print('----------', stop-start)
        t=t+float(stop-start)
        print(t)
        if t>5:
            break
except KeyboardInterrupt:
    pass
p.stop()
q.stop()
GPIO.remove_event_detect(Enc_A)
GPIO.remove_event_detect(Enc_B)
GPIO.cleanup()

if iA > iB:
    print('CW dir')
else:
    print('CCW dir')
print('PulsesA',iA)
print('PulsesB',iB)
with open('pos.txt', 'w') as temp_file:
    for item in Pos_radf:
        print >> temp_file, item
with open('time.txt', 'w') as temp_file:
    for item2 in my_time:
        print >> temp_file, item2
with open('vel.txt', 'w') as temp_file:
    for item3 in x2e_vec:
        print >> temp_file, item3
plt.figure(1)
plt.plot(my_time, ref_vec, color='red' , linewidth=3, linestyle='--',label='Desired Velocity')
plt.plot(my_time,vel, 'b',label='Motor velocity')
plt.plot(my_time,filtered, 'g',label='Filtered')
plt.plot(my_time,x2e_vec, 'y',label='Motor''s velocity 2')
plt.ylabel("Velocity (rad/s)")
plt.xlabel("Time (s)")
plt.legend()
plt.figure(2)
plt.plot(my_time, u_vec, 'g')
plt.figure(3)
plt.plot(my_time, Pos_radf, 'r')
plt.figure(4)
#plt.plot(my_time,x2e_vec, 'g')
plt.plot(my_time,filtered, 'r')
# plt.plot(my_time, pos_e, 'y')
# plt.plot(my_time, pos, 'r')
plt.show()
