# -*- coding: utf-8 -*-
"""
minimal template for BasicEPuck.ePuckVRep
for usage with ePuckS5V12.ttm

@author: hoch ralph
"""
import time
import numpy as np
import math
from BasicEPuck.ePuckVRep import EPuckVRep

def calculateValues(goal, act):
    
    delta_x = goal[0] - act[0]
    delta_y = goal[1] - act[1]
    theta = act[2]
    
    rho = math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
    alpha = -theta + math.atan2(delta_y, delta_x)
    beta = -theta - alpha
    
    return rho, alpha, beta

def calculateWheelspeed(constants, goal, act):
    
    r = 0.04/2
    l = 0.0541
    
    rho, alpha, beta = calculateValues(goal, act)
    vel = np.matmul(np.array([[constants['k_rho']/r, constants['k_alpha']*l/r, constants['k_beta']*l/r],
                               [constants['k_rho']/r, -constants['k_alpha']*l/r, -constants['k_beta']*l/r ]
                               ]), 
                    np.array([[rho],[alpha],[beta]]) )
    
    return vel

def calculateMotorValues(goal, act, constants, odometry, wheelparameter):
    """
    TODO: include parameters
    :return: (float,float)
        left and right motor velocity
    """
    # maximum velocity = ~2 Rad
    maxVel = 120 * 3.1415 / 180
    # TODO: calculate left and right motor velocity
    velRight = 0
    velLeft = 0
    
    if odometry:
        nextPoint = nextPointOdometry(act, wheelparameter)
        vel = calculateWheelspeed(constants, goal, act=nextPoint)
    else:
        vel = calculateWheelspeed(constants, goal, act=act)
    
    
    velLeft =  vel[0][0]
    velRight = vel[1][0]

    return velLeft, velRight


def nextPointOdometry(actPose, wheelparameter):
    
    wheelEncoderLeft.append(wheelparameter[0])
    wheelEncoderRight.append(wheelparameter[1])
    
    theta = 2
    r     = 0.04/2
    
    if step == 0:
        delta_sr = delta_sl = delta_theta = 0
        
    else:
        delta_sr     = r * abs(wheelEncoderRight[step] - wheelEncoderRight[step-1]) 
        delta_sl     = r * abs(wheelEncoderLeft[step] - wheelEncoderLeft[step-1])
        delta_theta  = (delta_sr - delta_sl) / (2*r)
    
    delta_s = (delta_sr + delta_sl) * .5
    
    diffPose = np.array([[delta_s*np.cos(actPose[theta]+delta_theta/2)],
                         [delta_s*np.sin(actPose[theta]+delta_theta/2)],
                         [delta_theta]])
      
    nextPoint = np.add(np.array([[actPose[0]],
                                 [actPose[1]],
                                 [actPose[2]]]), diffPose)
    
    return nextPoint


def main():
            
    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    robot.enableAllSensors()
    robot.enablePose()
    robot.setSensesAllTogether(False)  # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    noDetectionDistance = 0.05 * robot.getS()  # maximum distance that proximity sensors of ePuck may sense
    
    global wheelEncoderRight, wheelEncoderLeft, step
    
    step              =  0
    wheelEncoderRight = list()
    wheelEncoderLeft  = list()
    
    #startpos = [0.5, .5, 0.0]
    goal = [1.225, 0.0, 0.0]                # wieso x so genau?
    
    constants = {'k_rho'  :  0.05,          # p-Anteil ?
                 'k_beta' : +0.01,          # d-Anteil ?
                 'k_alpha': -0.1}           # i-Anteil ?
    
    # main sense-act cycle
    while robot.isConnected():
        # print( 'proximity: ', robot.getProximitySensorValues())
        # print( 'ground: ', robot.getGroundSensorValues())
        # print( 'acceleration: ', robot.getAccelerometerValues())
        print( 'wheel encoding: ', robot.getWheelEncoderValues())
        
        robot.fastSensingOverSignal()
        
        print('Pose: ',robot.getPose())

        # sense
        #distVector = robot.getProximitySensorValues()
        
        wheelEncodingValues = robot.getWheelEncoderValues()

        # plan
        leftMotor, rightMotor = calculateMotorValues(goal           = goal,
                                                     act            = robot.getPose(), 
                                                     wheelparameter = wheelEncodingValues, 
                                                     constants      = constants,
                                                     odometry       = True)

        # act
        robot.setMotorSpeeds(leftMotor, rightMotor)
        step = step + 1

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()
