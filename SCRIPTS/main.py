from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from Model_Identify import Model_Compensation
from DroneGuidance import *
from own_module import script_setup as sc_s, \
    script_variables as sc_v
from DroneManager import DroneManager
from Target import Target


def setpriority(pid=None, priority=1):
    """ Set The Priority of a Windows Process.  Priority is a value between 0-5 where
        2 is normal priority.  Default sets the priority of the current
        python process but can take any valid process ID. """

    import win32api, win32process, win32con

    priorityclasses = [win32process.IDLE_PRIORITY_CLASS,
                       win32process.BELOW_NORMAL_PRIORITY_CLASS,
                       win32process.NORMAL_PRIORITY_CLASS,
                       win32process.ABOVE_NORMAL_PRIORITY_CLASS,
                       win32process.HIGH_PRIORITY_CLASS,
                       win32process.REALTIME_PRIORITY_CLASS]
    if pid == None:
        pid = win32api.GetCurrentProcessId()
    handle = win32api.OpenProcess(win32con.PROCESS_ALL_ACCESS, True, pid)
    win32process.SetPriorityClass(handle, priorityclasses[priority])


#setpriority(priority=5)
print('Wand initial position initialized to zero')
in_p = np.zeros(3)
while len(np.argwhere(in_p == 0.0)) == 3:
    in_p = get_wand_position()
print("Wand position initialized with 'real' values")
#
# in_p = np.array([-1.5, 1.5, 0])
# in_v = np.array([0.3, -0.3, 0])
# in_a = 0.0
in_p = np.array([-1.5, 1.5, 0])
in_v = np.array([0.0, 0.0, 0])
in_a = 0.0
guidance_beta = np.array([0.3, 0.4])
target_beta = 0.4
yr_ff_beta = 0.3
delta = 0.02
vc = 0.6

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    comp = Model_Compensation()
    target = Target(initial_pos=in_p, initial_vel=in_v, #initial_acc_module=in_a,
                    dt=0.01, use_wand_target=True)

    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                         box=np.array([1.2, -1.5, 2.0, -1.5]))

    guidance = DroneGuidance(guidance_beta, target_beta, yr_ff_beta, target,
                             drone, guidance_velocity=vc, dt=delta, N=5)
    guidance.start(vc, 0.0)
    guidance.stop()
    print('Main Finished')

# test venuti egregiamente :
# crazyfun__20220112_151923