import nengo
import numpy as np

from ros_copter import Quadcopter

model = nengo.Network('Erle Copter')
with model:
    velocity_input = nengo.Node([0,0,0,0])

    quadcopter = nengo.Node(Quadcopter(disable_signals=True,
                                       control_style='velocity',
                                      ), 
                            size_in=4,
                            size_out=3)

    nengo.Connection(velocity_input, quadcopter)
