from math import sin, cos, exp, sqrt, pi
from numpy import array, dot

from Robot3WD import Robot

class myRobot(Robot):
    def __init__(self, sampling_period, wheel_radius=None, L=None):
        Robot.__init__(self, sampling_period, wheel_radius, L)


    def inverse_kinematics(self, p_dot, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        wheel_angular_velocities = array([ ((1/wheel_radius)*(p_dot[0]*sin(theta) - p_dot[1]*cos(theta) - L*p_dot[2])), 
        ((1/wheel_radius)*(p_dot[0]*cos((pi/6)+theta) + p_dot[1]*sin((pi/6)+theta) - L*p_dot[2])), 
        ((1/wheel_radius)*(-p_dot[0]*cos((pi/6)-theta) + p_dot[1]*sin((pi/6)-theta) - L*p_dot[2])) ]).T
        return wheel_angular_velocities

    def forward_kinematics(self, wheel_angular_velocities, theta):
        L=self._L
        wheel_radius=self._wheel_radius
        p_dot=array([(wheel_radius/3)*(wheel_angular_velocities[0]*2*sin(theta)+wheel_angular_velocities[1]*2*cos(theta+(pi/6))-wheel_angular_velocities[2]*2*sin(theta+(pi/3))),
        (wheel_radius/3)*(-wheel_angular_velocities[0]*2*cos(theta)+wheel_angular_velocities[1]*2*cos(theta+-pi/3)+wheel_angular_velocities[2]*2*cos(theta+(pi/3))),
        (wheel_radius/3)*(-wheel_angular_velocities[0]*1/L-wheel_angular_velocities[1]*1/L-wheel_angular_velocities[2]*1/L)]).T
        return p_dot
    


    def motor_limit(self, wheel_angular_velocities, limit_value):
        wheel_angular_velocities_bar=array([0.0,0.0,0.0])	
        j=0
        while j<3:
            for i in wheel_angular_velocities:
                if i<-limit_value:
                    wheel_angular_velocities_bar[j]=-limit_value
                    j=j+1
                if -limit_value <= i <=limit_value:
                    wheel_angular_velocities_bar[j]=i
                    j=j+1
                elif i>limit_value:
                    wheel_angular_velocities_bar[j]=limit_value
                    j=j+1
        return wheel_angular_velocities_bar

	
    def Hmatrix(self,q):
        x,y,theta=q.T #unpack the array 
        print(x,y,theta)
        H=array([
        (cos(theta),-sin(theta),x),
        (sin(theta),cos(theta),y),
        (0,0,1)])
        return H

    def Vraw_to_distance(self,vraw):
        #print(vraw)
        d=0.68057*exp(-0.05686*sqrt(vraw))
        return d


    def move_left(self, vx, theta):
        p_dot = array([-vx, 0.0, 0.0]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)
    
    def move_forward(self, vy, theta):
        p_dot = array([0.0, vy, 0.0]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)
    
    def move_backward(self, vy, theta):
        p_dot = array([0.0, -vy, 0.0]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)
    
    def move_right(self, vx, theta):
        p_dot = array([vx, 0.0, 0.0]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)
    
    def rotate_CCW(self, w, theta):
        p_dot = array([0.0, 0.0, w]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)
    
    def rotate_CW(self, w, theta):
        p_dot = array([0.0, 0.0, -w]).T
        PHI_dot = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(PHI_dot)
        
