from message_handler import MessageHandler
from smart_assistant import SmartAssistant
from amr_def import *
import threading

class AMR_Agent:
    def __init__(self):

        self.__amr_control = AMRControl()

        self.__message_handler = MessageHandler(self.__amr_control)
        self.__message_handler_thread = threading.Thread(
            target=self.__message_handler.run)

        self.__smart_assist = SmartAssistant(self.__amr_control)
        self.__smart_assist_thread = threading.Thread(
            target=self.__smart_assist.run)

    def run(self):
        self.__message_handler_thread.start()
        self.__smart_assist_thread.start()

        # self.__display.run()

        self.__message_handler_thread.join()

        self.__smart_assist_thread.join()

    __amr_control = None

    __message_handler = None
    __message_handler_thread = None

    __smart_assist = None
    __smart_assist_thread = None



if __name__ == "__main__":
    agent = AMR_Agent() 
    agent.run()
