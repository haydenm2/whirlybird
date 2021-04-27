#!/usr/bin/env python3

# license removed for brevity


# This file is a basic structure to write a controller that
# communicates with ROS. It will be the students responsibility
# to tune the gains and fill in the missing information

# As an example this file contains PID gains, but other
# controllers use different types of gains so the class
# will need to be modified to accomodate those changes

import rospy
import time
import numpy as np
from wb_msgs.msg import Command
from wb_msgs.msg import WbStates
from wb_msgs.msg import WbReferenceStates


class Controller():

    def __init__(self):

        # get parameters
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')

        self.g = self.param['g']
        self.l1 = self.param['l1']
        self.l2 = self.param['l2']
        self.m1 = self.param['m1'] #0.8 #perturbed
        self.m2 = self.param['m2']
        self.d = self.param['d']
        self.h = self.param['h']
        self.r = self.param['r']
        self.Jx = self.param['Jx']
        self.Jy = self.param['Jy']
        self.Jz = self.param['Jz']
        self.km = self.param['km']
        self.Ts = 0.01
        self.tau = 0.05

        self.Fe = (self.m1*self.l1-self.m2*self.l2)*self.g/self.l1 #Note this is not the correct value for Fe, you will have to find that yourself

        # Roll Gains
        tr_phi = 0.1
        wn_phi = 2.2/tr_phi
        zeta_phi = 0.7
        self.P_phi_ = wn_phi**2*self.Jx #0.252756
        self.I_phi_ = 0.0
        self.D_phi_ = 2*zeta_phi*wn_phi*self.Jx #0.04825
        self.Int_phi = 0.0
        self.prev_phi = 0.0
        self.prev_e_phi = 0.0
        self.phidot = 0.0

        # Pitch Gains
        tr_theta = 0.1
        wn_theta = 2.2/tr_phi
        zeta_theta = 0.7
        b_theta = 1.152
        self.theta_r = 0.0
        #self.P_theta_ = 2.604
        self.P_theta_ = 67.222 #wn_theta**2/b_theta #
        self.I_theta_ = 40.0
        #self.D_theta_ = 3.4722
        self.D_theta_ = 15.1219 #2*zeta_theta*wn_theta/b_theta
        self.prev_theta = 0.0
        self.Int_theta = 0.0
        self.prev_e_theta = 0.0
        self.thetadot = 0.0

        # Yaw Gains
        tr_psi = 10*tr_phi
        wn_psi = 2.2/tr_psi
        zeta_psi = 0.7
        b_psi = self.l1*self.Fe/(self.m1*self.l1**2+self.m2*self.l2**2+self.Jz) #5.9933
        self.psi_r = 0.0
        self.P_psi_ = wn_psi**2/b_psi #8.97303
        self.I_psi_ = 0.0
        self.D_psi_ = 2*zeta_psi*wn_psi/b_psi #1.71303
        self.prev_psi = 0.0
        self.Int_psi = 0.0
        self.prev_e_psi = 0.0
        self.psidot = 0.0

        self.Klat = np.array([2.7434, 1.8371, 0.1592, 1.2859])
        self.krlat = 1.8371
        self.Klon = np.array([22.0331,5.3487]) #np.array([1.0164,0.2467]) #np.array([19.6279, 8.2337]) #np.array([19.6279, 1.2337]) #np.array([2.3951, 2.4112])
        self.krlon = 16.8102 #0.7755 #19.387 #2.1541

        self.Alatd = np.matrix([[1.0,0,0.01,0], [0.0003,1.0,0,0.01], [0, 0, 1.0, 0], [0.0599,0,0.0003,1.0]])
        self.Blatd = np.matrix([[0.0106], [0], [2.1277], [0.0002]])
        self.Clatd = np.matrix([[1.0,0,0,0], [0,1.0,0,0]])
        self.Alond = np.matrix([[1, 0.01], [0,1]])
        self.Blond = np.matrix([[0.0001], [0.0115]])
        self.Clond = np.matrix([1,0])

        self.Llatd = np.matrix([[0.8032,-0.1926], [0.2846,0.6975], [5.9567,7.6484], [-0.1102,10.2033]])
        self.Llond = np.matrix([[0.3088], [4.1428]]) #np.matrix([[0.3094], [4.2119]]) #np.matrix([[1.3460], [55.7447]])

        self.xlathat = np.matrix([[0],[0],[0],[0]])
        self.xlonhat = np.matrix([[0],[0]])
        self.F = self.Fe;
        self.T = 0;

        self.prev_time = rospy.Time.now()

        self.wb_state_sub_ = rospy.Subscriber('/wb_states', WbStates, self.whirlybirdCallback, queue_size=5)
        self.wb_r_state_sub_ = rospy.Subscriber('/wb_reference_states', WbReferenceStates, self.referenceStateCallback, queue_size=5)
        self.wb_command_pub_ = rospy.Publisher('/wb_command', Command, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()



    def referenceStateCallback(self, msg):
        self.theta_r = msg.theta_r
        self.psi_r = msg.psi_r


    def whirlybirdCallback(self, msg):

        phi = msg.roll
        theta = msg.pitch
        psi = msg.yaw
        # e_phi = self.phi_r-phi
        e_theta = self.theta_r-theta
        e_psi = self.psi_r-psi
        phidot = msg.gyro_x
        thetadot = msg.gyro_y
        psidot = msg.gyro_z

        self.Fe = (self.m1*self.l1-self.m2*self.l2)*self.g/self.l1*np.cos(theta)

        self.xlat = np.array([phi,psi,phidot,psidot])
        self.xlon = np.array([theta,thetadot])
        ylat = np.matrix([[phi],[psi]])
        ylon = np.matrix([theta])

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now
        # self.phidot = (2*self.tau-self.Ts)/(2*self.tau+self.Ts)*self.phidot-2/(2*self.tau+self.Ts)*(phi-self.prev_phi)
        # self.thetadot = (2*self.tau-self.Ts)/(2*self.tau+self.Ts)*self.thetadot-2/(2*self.tau+self.Ts)*(theta-self.prev_theta)
        # self.psidot = (2*self.tau-self.Ts)/(2*self.tau+self.Ts)*self.psidot-2/(2*self.tau+self.Ts)*(psi-self.prev_psi)
        # # self.Int_phi = self.Int_phi+self.Ts/2*(e_phi-self.prev_e_phi)
        # self.Int_theta = self.Int_theta+self.Ts/2*(e_theta+self.prev_e_theta)
        # # self.Int_psi = self.Int_psi+self.Ts/2*(e_psi+self.prev_e_psi)

        #################################################
        ######### Implement Your Controller Here ########



        #State space observer control
        self.xlathat = np.matmul(self.Alatd,self.xlathat)+np.dot(self.Blatd,self.T)+np.matmul(self.Llatd,(ylat-np.matmul(self.Clatd,self.xlathat)))
        self.xlonhat = np.matmul(self.Alond,self.xlonhat)+np.dot(self.Blond,self.F-self.Fe)+np.matmul(self.Llond,(ylon-np.matmul(self.Clond,self.xlonhat)))
        # self.xlonhat[0] = self.xlon[0] #test replacing observer with actual values
        # rospy.loginfo('%s', np.transpose(self.xlonhat)-self.xlon)
        self.F = self.Fe-np.dot(self.Klon,self.xlonhat)+np.dot(self.krlon,self.theta_r)
        self.T = -np.dot(self.Klat,self.xlathat)+np.dot(self.krlat,self.psi_r)
        #Full-state control
        # self.F = self.Fe-np.dot(self.Klon,xlon)+np.dot(self.krlon,self.theta_r)
        # self.T = -np.dot(self.Klat,xlat)+np.dot(self.krlat,self.psi_r)
        #PID CONTROL
        # self.F = self.Fe + self.P_theta_*e_theta+self.D_theta_*self.thetadot+self.I_theta_*self.Int_theta
        # self.phi_r = self.P_psi_*(self.psi_r-psi)+self.D_psi_*self.psidot
        # self.T = self.P_phi_*(self.phi_r-phi)+self.D_phi_*self.phidot
        #PD CONTROL
        # self.F = self.Fe + self.P_theta_*(self.theta_r-theta)-self.D_theta_*thetadot
        # self.phi_r = self.P_psi_*(self.psi_r-psi)-self.D_psi_*psidot
        # self.T = self.P_phi_*(self.phi_r-phi)-self.D_phi_*phidot

        left_force = self.F+self.T/self.d
        right_force = self.F-self.T/self.d

        #age data
        self.prev_phi = phi
        self.prev_theta = theta
        self.prev_psi = psi
        # self.prev_e_phi = e_phi
        self.prev_e_theta = e_theta
        self.prev_e_psi = e_psi

        #################################################

        # Scale Output and saturate
        l_out = left_force/self.km/2.0
        r_out = right_force/self.km/2.0
        l_unsat = l_out
        r_unsat = r_out

        if(l_out < 0):
            l_out = 0
        elif(l_out > 0.7):
            l_out = 0.7

        if(r_out < 0):
            r_out = 0
        elif(r_out > 0.7):
            r_out = 0.7

    	# integrator anti-windup
        if self.I_theta_ != 0:
            self.Int_theta += (self.Ts/self.I_theta_)*(l_out-l_unsat)

        # Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        self.wb_command_pub_.publish(command)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass
