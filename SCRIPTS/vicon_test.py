import ctypes
import logging
import time
from own_module import script_variables as sc_v
from own_module import script_setup as sc_s
from vicon_dssdk import ViconDataStream
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


# setpriority(priority=5)
winmm = ctypes.WinDLL('winmm')
winmm.timeBeginPeriod(1)
#time.sleep(1)
# try:
#     sc_s.vicon.GetFrame()
# except ViconDataStream.DataStreamException as exc:
#     print("Error while getting a frame in the core!")
#
# sc_v.wand_pos = sc_s.vicon. \
#     GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]
# print(f'primo pos:{sc_v.wand_pos}')
# timecode = sc_s.vicon.GetFrameNumber()
# print(f'primo timecode:{timecode}')
# time.sleep(5)
# sc_v.wand_pos = sc_s.vicon. \
#     GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]
# print(f'secondo pos:{sc_v.wand_pos}')
#
# try:
#     sc_s.vicon.GetFrame()
# except ViconDataStream.DataStreamException as exc:
#     logging.error("Error while getting a frame in the core! "
#                   "--> %s", str(exc))
# sc_v.wand_pos = sc_s.vicon. \
#     GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]
# print(f'terzo pos:{sc_v.wand_pos}')
for i in range(5):
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))
    timecode = sc_s.vicon.GetFrameNumber()
    print(f'il framenumber {i}:{timecode} viene plottato al tempo di python {time.time()}')
    time.sleep(1)
