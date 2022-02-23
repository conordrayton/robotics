from time import time
from math import sqrt, pi, sin, cos, atan2
from numpy import array, dot, zeros, matmul
from numpy.linalg import norm



from PID import PIDControllers

from myRobot import *

# Set up sampling period T_s and stopping time T_f
T_s = 0.01
T_f = 20.0



# Initialize PID controller
Kp = array([2.0, 2.0, 0.5]).T
Ki =  array([0.5, 0.5, 0.2]).T
Kd =  array([0.01, 0.01, 0.01]).T
pid = PIDControllers(Kp, Ki, Kd, T_s)



# Set up initial pose
x_0 =0.0
y_0 =0.0
theta_0 =pi/6

# Set up goal position

#changed the final y posistion kept bumping into walls
x_f = 1.0
y_f = 1.1

# Set up p_0 and p_f
p_0 = array([x_0, y_0, theta_0]).T
p_f = array([x_f,y_f,pi/2]).T

# Set up error tolerance
epsilon =0.001

# set up d_free and d_min
d_min = 0.08
d_free = 0.1

# Set up controller gains
k_rho = 1.0
k_beta = -1.0
k_alpha = (2.0/pi)*k_rho - (5.0/3.0)*k_beta + 0.5

# Initialize vector pr_dot to be used for inverse kinematics
pr_dot =array([0.0,0.0,0.0]).T

# Initialize vector d for storing the sensor measurements d_i 
d = array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

# Initialize a 6x3 matrix u_iR.  The vector u_i^R is assigned the ith column
# of this matrix.  See Eq. (4.6) in lab manual.
u_iR = zeros(shape=(3,6))
#dont use the u_iR matrix 

# Sensor frame location as per Table 4.1
R = 0.13
halfR = R/2.0
ok = sqrt(3.0)*R/2.0

# Sensor frame location as per Table 4.1
sensor_loc = array([
   (-R/2, -ok*R, -(2.0/3.0)*pi),
   (-ok*R, halfR ,(5.0/6.0)*pi),
   (0,R,(1.0/2.0)*pi ),
   (halfR,ok*R,(1.0/3.0)*pi ),
   (ok*R,halfR,(1.0/6.0)*pi),
   (ok*R,-halfR,-(1.0/6.0)*pi)
])



# open a file to store robot pose for plotting later
f = open('pose.csv', 'w')

# Initial robot and other variables
robot = myRobot(T_s)


robot.initialize(theta_0)

p = p_0
dp = p_f - p
rho = norm(dp[0:2])
goal_reached = False
elapsed_time = 0.0

# Set start time
start_time = time()

# Control loop
while ( (not goal_reached) and (elapsed_time < T_f) ):

    robot.get_readings_update()
    

    # save data to file for plotting
    f.write('%10.3e %10.3e % 10.3e %10.3e %10.3e\n' % (elapsed_time, p[0], p[1], p[2], rho))

    theta = p[2]
    
    #find all required values 
    wheel_angular_velocities=robot.angular_velocities
    wheel_angular_velocities_bar=robot.motor_limit(wheel_angular_velocities,5*pi)
    p_dot = robot.forward_kinematics(wheel_angular_velocities_bar,theta) 
    ydelta=dp[1]
    xdelta=dp[0]	
    alpha = atan2(ydelta,xdelta)-theta
    


      
    collision=False
    #flag to see if a collision is possible 
    # assume no collision initially
    dt=0.0
    # to summ all non collision d[i] values
    for i in range(0,6): 
        d[i] = robot.Vraw_to_distance(robot.ir_sensors_raw_values[i])
        if (d[i]<=d_min):
            #at risk of collision set collision flag to true 
            collision=True     
        elif (d[i]>d_free):
            #not at risk of collision
            dt+=d[i]
            #add to dt sum       
 
    sum=[0.0,0.0,0.0]
    uR=[0.0,0.0,0.0]
    #initialize variables for detection
          
    #code only executed if collision is possible
    if (collision):
        for i in range(0,6):
            if d[i] > d_free:
                delta_Si=array([d[i],0 ,1]).T
                x=sensor_loc[i]
                HRSi=robot.Hmatrix(x)
                #H-matrix in terms of robot frame
                delta_R=matmul(HRSi,delta_Si)
                #matmul is matrix multipliication
                
                #magnitude of deltaR
                length=norm(delta_R)
                sum[0]+=delta_R[0]
                sum[1]+=delta_R[1]
                sum[2]+=delta_R[2]
                    
                uR[0]+=(sum[0]/length)*(d[i]/dt)
                uR[1]+=(sum[1]/length)*(d[i]/dt)
                uR[2]+=(sum[2]/length)*(d[i]/dt)
                
                #find uR at every d[i] not at risk of collision
         
         
       
        U_0=matmul(robot.Hmatrix(p_0),uR)
        #H matrix from robot to base frame
        p_T=0.1*(U_0)
        #equation 4.9 temp goal        
        #find pbar alphabar 
        dif=p_T-p
        pbar=norm(dif[0:2])
        xd=dif[0]
        yd=dif[1]             
        alphabar=atan2(yd,xd)-theta
        beta=-(theta+alpha) 
        v = k_rho*pbar
        w = k_alpha*alphabar+k_beta*beta
            
    
    elif(not collision):
        #no collision find pr_dot variables normally 
        print("no collision")
        beta = -(theta + alpha)
        v=k_rho*rho
        w=k_alpha*alpha+k_beta*beta

        
        
   
    #find pr_dot in both cases 
    #however v and w will change if there is or isnt collision detection
    pr_dot[0] =v*cos(theta) 
    pr_dot[1] =v*sin(theta)
    pr_dot[2] = w   
        
    # Now use Inverse Kinematics to determine wheel ref velocities
    
    #unnecessarily added PID control. if error is low and no collision detected 
    #then robot can easily get to goal posistion
    # for example can get to goal posistions that are very close to obstacles like xf=0.4 yf=0.6
    
    if((rho<0.25) and (not collision)):
        pd_dot=pid(p_f,p)
        w=robot.inverse_kinematics(pd_dot,theta)
        robot.set_angular_velocities(w)
    

    else:
        wheel_ref_vel =robot.inverse_kinematics(pr_dot,theta) 
	    
	    # Apply motor limits
        wheel_ref_vel_lim = robot.motor_limit(wheel_ref_vel,5*pi)
	   

	    # Execute motion
        robot.set_angular_velocities(wheel_ref_vel_lim)

        
    # Odometry update
    p=p+(p_dot)*T_s

    # Replace calculated update for theta with measured value from IMU
    p[2]=robot.orientation 

    # Check to see if goal is reached
    dp = p_f - p
    #print(dp)
    rho = norm(dp[0:2])
    #print('hello')
    #print(rho)    
    goal_reached = ( rho <= epsilon)
    
    

    
    
   

    # time update
    elapsed_time = time() - start_time


print('ELPASED TIME = %s rho = %s' % (elapsed_time, rho))

# Either goal is reached or current_time > T_f
if goal_reached:
   print('Goal is reached, error norm is', rho)
else:
   print('Failed to reach goal, error norm is', rho)

robot.stop()
robot.close()

f.close()
