# alphaDrone-pyBullet

This repository gives a pipeline for integrating CAD model to PyBullet, as well as providing a base code template for running PX4-PyBullet Software-In-The-Loop simulation.
Everything has been tested on Ubuntu 24.04.3 LTS

# Installation
## Cloning repository
the first step is to clone this repository. simply do

    git clone git@github.com:nodale/alphaDrone-pyBullet.git

## Python environment
python virtual environment is preferred and requires libraries in the requirements.txt. One can simply recursively install all of them
    
    pip install -r requirements.txt

## PX4 setup
as for PX4, one needs the PX4-Autopilot repository (https://github.com/PX4/PX4-Autopilot)
the following build is required for Software-In-The-Loop simulation

    make px4_sitl_nolockstep none_iris

after building it, one can simply run PX4

    Your-PX4-repository-folder/build/px4_sitl_nolockstep/bin/
    ./px4


run it and open QGroundControl. Go to parameters and set the following
    
    COM_RC_IN_MODE  Stick Input Disabled
    EKF2_BARO_CTRL  Disabled
    EKF2_EV_CTRL    15
    EKF2_GPS_CTRL   3
    EKF2_IMU_CTRL   0
    EKF2_HGT_REF    Vision
    SYS_HAS_BARO    Disabled
    SYS_HAS_MAG     0

also check you SYS_AUTOSTART, it should be

    SYS_AUTOSTART   10016

# Usage
To run the simulation, got to your cloned PX4 Repository
    
    cd Your-PX4-repository-folder/build/px4_sitl_no_lockstep/bin/
    ./px4

This will run the PX4 software itself, QGroundControl should automatically pick this up. Then you can run your python script. Don't forget to activate your virtual environment.

    python examply.py

    
