import threading
import queue

# smart assist #################################################################
PRINT_EVEYTHING = False
REPLAY_AUDIO = False

INITIAL_LOCATION = "base_station"


# control ######################################################################
class AMRControl:
    start_event = threading.Event()
    reset_event = threading.Event()
    start_listen_event = threading.Event()
    finish_listen_event = threading.Event()

    smart_assist_prompt = queue.Queue()
    current_location_name = []

