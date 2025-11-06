import sys
sys.path.append("include")

from quick_pybullet import QuickBullet

import pybullet as p
import time, sys, select

#def wait(qB):
#    print("press Enter to continue")
#    qB.arm()
#    while True:
#        qB.runSimpleSensorsSim()
#        qB.sendFakeOdometry()
#
#        _time = int(time.time() * 1e6) & 0xFFFFFFFF
#        qB.sendPositionTarget(_time, 0.0, 0.0, 30.0)
#
#        p.stepSimulation()
#        time.sleep(1/qB.freq)
#        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   
#            _line = sys.stdin.readline()
#            if _line.strip() == "":
#                print("break out of loop")
#                break


def main():
    qB = QuickBullet(address='tcpin:localhost:4560', baudrate=57600)
    qB.initSecondaryCom(address='udpin:localhost:14540', baudrate=57600) #'udpin:localhost:14540' 
    qB.initTertiaryCom(address='udpout:localhost:14540', baudrate=57600) 

    qB.resetLogFiles()
    qB.freq = 400.0
    p.setTimeStep(1.0/qB.freq)

    qB.pVel = 0.1

    reset_button = p.addUserDebugParameter("reset", 1, 0, 0)

#    wait(qB)
#    qB.takeoff(-1.0)

    while True:
        button_value = p.readUserDebugParameter(reset_button)

        qB.runSimpleSensorsSim()
        qB.sendFakeOdometry()

        _time = int(time.time() * 1e6) & 0xFFFFFFFF
        qB.sendPositionTarget(_time, 0.0, 0.0, 2.5)

        qB.getActuatorOutput()
        qB.actuateVehicle()
        p.stepSimulation()
    
        time.sleep(1.0/qB.freq)
        #qB.showState()
        if button_value == 1:
            reset_button = p.addUserDebugParameter("reset", 1, 0, 0)
            qB.reset()

if __name__ == "__main__":
    main()
