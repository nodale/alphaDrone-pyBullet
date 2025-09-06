import sys
sys.path.append("include")

from quick_pybullet import QuickBullet

import pybullet as p
import time

def main():
    qB = QuickBullet(address='tcpin:localhost:4560', baudrate=57600)

    while True:
        qB.runSimpleSensorsSim()
        qB.sendFakeOdometry()
#        qB.actuateVehicle()
        p.stepSimulation()
        time.sleep(1/qB.freq)

if __name__ == "__main__":
    main()
