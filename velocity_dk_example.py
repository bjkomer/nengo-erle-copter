import nengo
import numpy as np

from dk_copter import Quadcopter

model = nengo.Network('Erle Copter')
with model:
    # North East Down Yaw
    # Input in meters/second?
    velocity_input = nengo.Node([0,0,0])
    
    # Input in Degrees
    #yaw_input = nengo.Node([0])

    quadcopter = nengo.Node(Quadcopter(simulation=True,
                                       control_style='velocity'), 
                            size_in=3,
                            size_out=3)

    nengo.Connection(velocity_input, quadcopter[:3])
    #nengo.Connection(yaw_input, quadcopter[3])
