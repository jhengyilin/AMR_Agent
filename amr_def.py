import threading
import queue

# smart assist #################################################################
PRINT_EVERYTHING = True
REPLAY_AUDIO = True

INITIAL_LOCATION = "Base_station"


# control ######################################################################
class AMRControl:
    start_event = threading.Event()
    reset_event = threading.Event()
    start_listen_event = threading.Event()
    finish_listen_event = threading.Event()

    smart_assist_prompt = queue.Queue()
    smart_assist_location_name_setting = queue.Queue()
    smart_assist_pose_setting = queue.Queue()
    current_location_name = []

