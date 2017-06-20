import nengo
import numpy as np

from ros_copter import Quadcopter

model = nengo.Network('Erle Copter')
with model:
    position_input = nengo.Node([0,0,5])

    yaw_input = nengo.Node([0])

    quadcopter = nengo.Node(Quadcopter(disable_signals=True), 
                            size_in=4,
                            size_out=3)

    nengo.Connection(position_input, quadcopter[:3])
    nengo.Connection(yaw_input, quadcopter[3])

