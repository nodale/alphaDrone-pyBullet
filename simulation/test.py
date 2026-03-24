import sys
import math
sys.path.append("include")

from quick_pybullet import QuickBullet

import pybullet as p
import time, sys, select


def main():
    qB = QuickBullet(address='tcpin:localhost:4560', baudrate=921600)

    #this udpout might be a problem for the lockstep sitl
    qB.initSecondaryCom(address='udpout:localhost:14580', baudrate=57600) #'udpin:localhost:14540' 
    #qB.initTertiaryCom(address='udpout:localhost:14550', baudrate=57600) 

    qB.resetLogFiles()
    #qB.setupCamera()
    qB.freq = 250.0
    p.setTimeStep(1.0/qB.freq)

    #qB.pVel = 0.1
    #qB.genRandomCurve()

    reset_button = p.addUserDebugParameter("reset", 1, 0, 0)
    step_button = p.addUserDebugParameter("step", 0, 1, 0.5)

    #qB.go2FirstCurve()

    _time = int(0)
    while True:
        #_time = int(qB.timestamp * 1e6) & 0xFFFFFFFF
        _time += int((1.0 / qB.freq) * 1e6)  & 0xFFFFFFFF
        button_value = p.readUserDebugParameter(reset_button)
        step_value = p.readUserDebugParameter(step_button)

        qB.runSimpleSensorsSim(_time)
        qB.sendFakeOdometry(_time)

        time.sleep(1.0/qB.freq)

        #setpx = 0.6 * math.sin(0.2 * time.time())
        #setpy = 0.6 * math.sin(0.2 * time.time())
        setpx = 0.0
        setpy = 0.0
    
        #qB.sendPlanarVelocityTarget(_time, 0.3, 0.0, -2.5)
        qB.sendPositionTarget(_time, setpx, setpy, -0.5)
        #qB.traverseCurve()

        qB.getActuatorOutput()
        qB.actuateVehicle()
        #qB.actuateFakeVehicle()
        #qB.printCamera()
        p.stepSimulation()
    
        #qB.showState()
        if button_value == 1:
            #p.removeUserDebugItem(reset_button)
            reset_button = p.addUserDebugParameter("reset", 1, 0, 0)
            qB.reset()

        if step_value == 1:
            #p.removeUserDebugItem(step_button)
            #step_button = p.addUserDebugParameter("step", 1, 0, 0)
            p.stepSimulation()

        #if qB.llp.closest_u >= 0.9:
        #    qB.genRandomCurve()
        #    qB.llp.transition()

if __name__ == "__main__":
    main()
