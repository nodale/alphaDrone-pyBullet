# alphaDrone-pyBullet

This repository gives a pipeline for integrating CAD model to PyBullet, as well as providing a base code template for running PX4-PyBullet Software-In-The-Loop simulation.

# Installation
## Python environment
A python environment is preferably setup and requires libraries in the requirements.txt. One can simply recursively install all of them by doing the following
    
    pip install -r requirements.txt

## Cloning repository
as for the repository itself, simply do

    git clone git@github.com:nodale/alphaDrone-pyBullet.git

## PX4 setup
as for PX4, one needs the following build

    make px4_sitl_nolockstep none_iris

after building it, one can simply run PX4 here

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

# Usage
To run the simulation, got to your cloned PX4 Repository
    
    cd Your-PX4-repository-folder/build/px4_sitl_no_lockstep/bin/
    ./px4

This will run the PX4 software itself, QGroundControl should automatically pick this up. Then you can run your python script. Don't forget to activate your virtual environment.

    python examply.py

    
