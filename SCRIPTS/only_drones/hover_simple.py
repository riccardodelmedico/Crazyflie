# import logging
# import time
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
# from own_module import crazyfun as crazy, \
#     script_variables as sc_v, script_setup as sc_s
import numpy as np
a=np.array([0.0,0.0,0.0])
print(a*3)
print(np.linalg.norm(a,2))
print(len(np.nonzero(a)))

# with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
#
#     datalog = crazy.datalog(scf)
#     datalog.start()
#
#     with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
#
#         logging.info("Take-off!")
#         time.sleep(1)
#
#     datalog.stop()
