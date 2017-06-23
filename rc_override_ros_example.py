import nengo
import numpy as np

from ros_copter import Quadcopter

model = nengo.Network('Erle Copter')
with model:
    # Inputs between -1 and 1
    control_input = nengo.Node([0,0,0])

    quadcopter = nengo.Node(Quadcopter(disable_signals=True,
                                       control_style='alt_hold',
                                       do_arming=True,
                                      ), 
                            size_in=3,
                            size_out=3)

    nengo.Connection(control_input, quadcopter)
