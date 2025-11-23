import sys
sys.path.append("include")

from quick_pybullet import QuickBullet

import pybullet as p
import time, sys, select

def wait(qB):
    print("press Enter to continue")
    while True:
        qB.runSimpleSensorsSim()
        qB.sendFakeOdometry()

        time.sleep(1.0/qB.freq)

        _time = int(time.time() * 1e6) & 0xFFFFFFFF
        qB.sendPositionTarget(_time, 0.0, 0.0, -2.5)

        qB.getActuatorOutput()
        qB.actuateVehicle()
        p.stepSimulation()

        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   
            _line = sys.stdin.readline()
            if _line.strip() == "":
                print("break out of loop")
                break


def main():
    qB = QuickBullet(address='tcpin:localhost:4560', baudrate=57600)

    #this udpout might be a problem for the lockstep sitl
    qB.initSecondaryCom(address='udpout:localhost:14580', baudrate=57600) #'udpin:localhost:14540' 
    #qB.initTertiaryCom(address='udpout:localhost:14550', baudrate=57600) 

    qB.resetLogFiles()
    qB.freq = 250.0
    p.setTimeStep(1.0/qB.freq)

    qB.pVel = 0.1
    qB.genRandomCurve()

    reset_button = p.addUserDebugParameter("reset", 1, 0, 0)

    wait(qB)
    #qB.go2FirstCurve()

    while True:
        button_value = p.readUserDebugParameter(reset_button)

        qB.runSimpleSensorsSim()
        qB.sendFakeOdometry()

        time.sleep(1.0/qB.freq)

        _time = int(time.time() * 1e6) & 0xFFFFFFFF
        #qB.sendPlanarVelocityTarget(_time, 0.3, 0.0, -2.5)
        qB.traverseCurve()

        qB.getActuatorOutput()
        qB.actuateVehicle()
        p.stepSimulation()
    
        #qB.showState()
        if button_value == 1:
            reset_button = p.addUserDebugParameter("reset", 1, 0, 0)
            qB.reset()

        if qB.llp.closest_u >= 0.9:
            qB.genRandomCurve()
            qB.llp.transition()

if __name__ == "__main__":
    main()
