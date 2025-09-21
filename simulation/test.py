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
        p.stepSimulation()
        time.sleep(1/qB.freq)
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   
            _line = sys.stdin.readline()
            if _line.strip() == "":
                print("break out of loop")
                break

def main():
    qB = QuickBullet(address='tcpin:localhost:4560', baudrate=57600)
    qB.resetLogFiles()
    qB.freq = 1000

    qB.pVel = 0.1

    wait(qB)
    qB.setFlightmode('OFFBOARD')
    qB.takeoff(-1.0)

    while True:
        qB.runSimpleSensorsSim()
        qB.getActuatorOutput()
        qB.sendFakeOdometry()
        qB.actuateFakeVehicle()
        p.stepSimulation()
        time.sleep(1/qB.freq)

if __name__ == "__main__":
    main()
