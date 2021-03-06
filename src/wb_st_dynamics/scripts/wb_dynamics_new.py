#!/usr/bin/env python3

import rospy
from wb_msgs.msg import WbStates
from wb_msgs.msg import Command


import numpy as np

class WhirlybirdSim():

    def __init__(self):
        # initialize member variables
        self.state = np.zeros((6,1)) # [phi theta psi phid thetad psid]' = [q' qd']'
        self.command = np.zeros((2,1)) # [ul ur]'
        self.command_esc = np.zeros((2,1)) # command throttled to ESC rate with zero-order hold

        self.initialized = False

        # Constant disturbances on parameters. These are used to test the integrators.

        # get parameters
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')

        # Calcuulate disturbances
        self.mass_disturbance = 0   # Constant disturbance on the front mass
        self.esc_disturbance = 0    # Constant disturbance on the left motor
        self.disturbances = self.param['disturbances']
        self.disturbance_percent = self.param['disturbance_percent']
        if self.disturbances :
            self.mass_disturbance = (np.random.random()*2 - 1)*self.param['m1']*self.disturbance_percent
            self.esc_disturbance  = (np.random.random()*2 - 1)*self.disturbance_percent
            print("Mass Disturbance: ", self.mass_disturbance)
            print("ESC Disturbance: ", self.esc_disturbance)


        # publish/subscribe:
        self.command_sub = rospy.Subscriber('/wb_command', Command, self.command_callback)
        self.whirlybird_pub = rospy.Publisher('/wb_states', WbStates, queue_size = 1)

        # setup simulation timer
        esc_rate = self.param['esc_command_rate']
        rospy.Timer(rospy.Duration(1.0/esc_rate), self.esc_timer_callback)

        dynamics_rate = self.param['dynamics_rate']
        rospy.Timer(rospy.Duration(1.0/dynamics_rate), self.dynamics_timer_callback)

        # spin
        rospy.spin()

    def command_callback(self, msg):
        self.command[0] = msg.left_motor # lab handout parameters expect PWM in range [0,100]
        self.command[1] = msg.right_motor
         #TODO saturate commands

    def esc_timer_callback(self, event):
        self.command_esc = self.command

    def dynamics_timer_callback(self, event):
        if not self.initialized:
            self.initialized = True
            return

        # propagate dynamics
        self.propagate((event.current_real - event.last_real).to_sec())

        # sensors
        enc = self.encoders()
        acc, gyro = self.imu()

        # publish
        self.whirlybird_pub.publish(
            roll = enc[0], pitch = enc[1], yaw = enc[2],
            accel_x = acc[0], accel_y = acc[1], accel_z = acc[2],
            gyro_x = gyro[0], gyro_y = gyro[1], gyro_z = gyro[2])

    def propagate(self, dt):
        # RK4 integration
        k1 = self.dynamics(self.state, self.command_esc)
        k2 = self.dynamics(self.state + dt/2*k1, self.command_esc)
        k3 = self.dynamics(self.state + dt/2*k2, self.command_esc)
        k4 = self.dynamics(self.state + dt*k3, self.command_esc)
        self.state += dt/6 * (k1 + 2*k2 + 2*k3 + k4)

        # Implement hard stops on angles
        phi    = self.state[0]
        theta  = self.state[1]
        psi    = self.state[2]
        phid   = self.state[3]
        thetad = self.state[4]
        psid   = self.state[5]

        if phi > self.param['phi_max']:
            phi = self.param['phi_max']
            if phid > 0:
                phid *= -0.5  # bounce against limit
        elif phi < self.param['phi_min']:
            phi = self.param['phi_min']
            if phid < 0:
                phid *= -0.5 # bounce against limit

        if psi > self.param['psi_max']:
            psi = self.param['psi_max']
            if psid > 0:
                psid *= -0.5  # bounce against limit
        elif psi < self.param['psi_min']:
            psi = self.param['psi_min']
            if psid < 0:
                psid *= -0.5 # bounce against limit

        if theta > self.param['theta_max']:
            theta = self.param['theta_max']
            if thetad > 0:
                thetad *= -0.5  # bounce against limit
        elif theta < self.param['theta_min']:
            theta = self.param['theta_min']
            if thetad < 0:
                thetad *= -0.5 # bounce against limit

        # pack back up
        self.state[0] = phi
        self.state[1] = theta
        self.state[2] = psi
        self.state[3] = phid
        self.state[4] = thetad
        self.state[5] = psid

    def dynamics(self, state, command):
        # Get parameters of ros param server
        g  = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1'] + self.mass_disturbance
        m2 = self.param['m2']
        d  = self.param['d']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']

        phi = state.item(0)
        theta = state.item(1)
        psi = state.item(2)
        phid = state.item(3)
        thetad = state.item(4)
        psid = state.item(5)

        # adjust forces for gains
        fl = km * command[0] + self.esc_disturbance
        if fl < 0:
            fl = 0
        fr = km * command[1]

        xdot = np.zeros((6,1))

        # angle dynamics
        xdot[0:3] = state[3:6]

        # angle rate dynamics
        sphi   = np.sin(phi)
        cphi   = np.cos(phi)
        stheta = np.sin(theta)
        ctheta = np.cos(theta)
        spsi   = np.sin(psi)
        cpsi   = np.cos(psi)

        ################################################
        # Implement Dynamics for Accelerations Here    #

        M = np.zeros((3,3))
        M[0,0] = Jx
        M[0,1] = 0
        M[0,2] = -Jx*stheta
        M[1,0] = 0
        M[1,1] = m1*l1**2+m2*l2**2+Jy*cphi**2+Jz*sphi**2
        M[1,2] = (Jy-Jz)*sphi*cphi*ctheta
        M[2,0] = -Jx*stheta
        M[2,1] = (Jy-Jz)*sphi*cphi*ctheta
        M[2,2] = (m1*l1**2+m2*l2**2+Jy*sphi**2+Jz*cphi**2)*ctheta**2+Jx*stheta**2

        c = np.zeros((3,1))
        c[0,0] = -thetad**2*(Jz-Jy)*sphi*cphi+psid**2*(Jz-Jy)*sphi*cphi*ctheta**2-thetad*psid*ctheta*(Jx-(Jz-Jy)*(cphi**2-sphi**2))
        c[1,0] = psid**2*stheta*ctheta*(-Jx+m1*l1**2+m2*l2**2+Jy*sphi**2+Jz*cphi**2)-2*phid*thetad*(Jz-Jy)*sphi*cphi-phid*psid*ctheta*(-Jx+(Jz-Jy)*(cphi**2-sphi**2))
        c[2,0] = thetad**2*(Jz-Jy)*sphi*cphi*stheta-phid*thetad*ctheta*(Jx+(Jz-Jy)*(cphi**2-sphi**2))-2*phid*psid*(Jz-Jy)*ctheta**2*sphi*cphi+2*thetad*psid*stheta*ctheta*(Jx-m1*l1**2-m2*l2**2-Jy*sphi**2-Jz*cphi**2)

        P = np.zeros((3,1))
        P[0,0] = 0
        P[1,0] = (m1*l1-m2*l2)*g*ctheta
        P[2,0] = 0

        Q = np.zeros((3,1))
        Q[0,0] = d*(fl-fr)
        Q[1,0] = l1*(fl+fr)*cphi
        Q[2,0] = l1*(fl+fr)*ctheta*sphi+d*(fr-fl)*stheta

        b = Q-c-P

        xdot[3:6] = np.linalg.inv(M).dot(b)
        ################################################

        return xdot

    def encoders(self):
        return self.state[0:3]

    def imu(self):
        # imu data is not used in ECEn 483
        phi = self.state[0]
        theta = self.state[1]
        psi = self.state[2]

        # accelerometer
        accel = np.zeros((3,1)) # TODO implement

        # rate gyro
        B = np.zeros((3,3))
        B[0,0] = 1.0
        B[0,2] = -np.sin(theta)
        B[1,1] = np.cos(phi)
        B[1,2] = np.sin(phi)*np.cos(theta)
        B[2,1] = -np.sin(phi)
        B[2,2] = np.cos(phi)*np.cos(theta)

        gyro = B.dot(self.state[3:6]) #TODO add noise

        return (accel, gyro)

    def sat(x, max, min):
        if x > max:
            x = max
        elif x < min:
            x = min
        return x

if __name__ == '__main__':
    rospy.init_node('wb_dynamics')
    try:
        whirlybird = WhirlybirdSim()
    except:
        rospy.ROSInterruptException
    pass
