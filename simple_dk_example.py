import nengo
import numpy as np

from dk_copter import Quadcopter

model = nengo.Network('Erle Copter')
with model:
    # North East Down Yaw
    # Input in Meters
    position_input = nengo.Node([0,0,-5])
    
    # Input in Degrees
    yaw_input = nengo.Node([0])

    quadcopter = nengo.Node(Quadcopter(), 
                            size_in=4,
                            size_out=3)

    nengo.Connection(position_input, quadcopter[:3])
    nengo.Connection(yaw_input, quadcopter[3])

