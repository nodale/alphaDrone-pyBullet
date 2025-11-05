import sys
sys.path.append("include")

from quick_pybullet import QuickBullet

import pybullet as p
import time, sys, select

def wait(qB):
    print("press Enter to continue")
    qB.arm()
    while True:
        qB.runSimpleSensorsSim()
        qB.sendFakeOdometry()

        _time = int(time.time() * 1e6) & 0xFFFFFFFF
        qB.sendPositionTarget(_time, 0.0, 0.0, -3.0)
        qB.arm()

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
    qB.freq = 400

    qB.pVel = 0.1

    wait(qB)
    qB.takeoff(-1.0)

    while True:
        qB.runSimpleSensorsSim()
        qB.sendFakeOdometry()

        _time = int(time.time() * 1e6) & 0xFFFFFFFF
        qB.sendPositionTarget(_time, 0.0, 0.0, -3.0)

        qB.getActuatorOutput()
        qB.actuateVehicle()
        p.stepSimulation()
        time.sleep(1/qB.freq)

if __name__ == "__main__":
    main()
