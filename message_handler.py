import threading
import time
from amr_def import *
from utils.audio import *
import rclpy
from nav2_commander.nav2_commander import Nav2Commander

class MessageHandler:
    def __init__(self, amr_control: AMRControl):
        self.__amr_control = amr_control
        self.__stop_event = threading.Event()

        rclpy.init()

        # Create an instance of the Nav2Commander node
        self.nav2_commander_node = Nav2Commander()

        # Start the ROS spinning in a separate thread
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.nav2_commander_node,))
        self.spin_thread.start()

    def send_consecutive_goals(self, goals):
        # goals = [(1.27, 0.04, 1.0), (2.15, -1.36, 1.0), (-0.18, -1.49, 1.0), (0.0, 0.0, 1.0)]
        
        for goal in goals:
            with self.nav2_commander_node.goal_condition:
                self.nav2_commander_node.send_goal(*goal)

                # Wait until the goal is reached
                while self.nav2_commander_node.goal_active:
                    self.nav2_commander_node.goal_condition.wait()

    # def extract_poses_to_dict(self):
    #     poses = []
    #     while not self.__amr_control.smart_assist_pose_setting.empty():
    #         pose = self.__amr_control.smart_assist_pose_setting.get()
    #         poses.append(pose)

    #     # Format the poses into dictionary
    #     poses_dict = {"poses": poses}
    #     return poses_dict

    def run(self):

        while (not self.__stop_event.is_set()) and self.__amr_control.send_command_flag:
            # rclpy.spin(self.node)
            try:
                # Send goals in sequence
                
                print("message handler is commanding the robot")
                print("location name setting:")
                location_name_setting = self.__amr_control.smart_assist_location_name_setting.get()
                print(location_name_setting)
                print("pose setting:")
                pose_setting = self.__amr_control.smart_assist_pose_setting.get()
                print(pose_setting)
                
                goals = []
                for i in range(len(location_name_setting)):
                    print("Location name:", location_name_setting[i])
                    print("Pose x:", pose_setting[i]["x"])
                    print("Pose y:", pose_setting[i]["y"])
                    print("Pose z:", pose_setting[i]["z"])
                    print("Pose orientation x:", pose_setting[i]["orientation"]["x"])
                    print("Pose orientation y:", pose_setting[i]["orientation"]["y"])
                    print("Pose orientation z:", pose_setting[i]["orientation"]["z"])
                    print("Pose orientation w:", pose_setting[i]["orientation"]["w"])
                    print("data type: ", type(pose_setting))
                    goals.append((float(pose_setting[i]["x"]), float(pose_setting[i]["y"]), float(pose_setting[i]["z"]), float(pose_setting[i]["orientation"]["x"]), float(pose_setting[i]["orientation"]["y"]), float(pose_setting[i]["orientation"]["z"]), float(pose_setting[i]["orientation"]["w"])))

                print("goals:")
                print(goals)
                self.send_consecutive_goals(goals)
                if PLAY_TEXT2PSEECH_AUDIO:
                    text_to_speech("Mission Completed")

            finally:
                # Clean up and shutdown
                self.nav2_commander_node.destroy_node()
                rclpy.shutdown()
                self.spin_thread.join()


            

            # print("Pose x:", pose_setting[0]["x"])
            # print("Pose y:", pose_setting[0]["y"])
            # print("Pose z:", pose_setting[0]["z"])
            # print("Pose orientation x:", pose_setting[0]["orientation"]["x"])
            # print("Pose orientation y:", pose_setting[0]["orientation"]["y"])
            # print("Pose orientation z:", pose_setting[0]["orientation"]["z"])
            # print("Pose orientation w:", pose_setting[0]["orientation"]["w"])
            # print("data type: ", type(pose_setting))

            

            self.__amr_control.send_command_flag = False

        # poses_dict = self.extract_poses_to_dict()
        # print("Formatted Poses Dictionary:")
        # print(poses_dict)

        # try:
        #     # Send goals in sequence
        #     self.send_consecutive_goals(self.nav2_commander_node)
        # finally:
        #     # Clean up and shutdown
        #     self.nav2_commander_node.destroy_node()
        #     rclpy.shutdown()
        #     self.spin_thread.join()
    def stop(self):
        self.__stop_event.set()
        rclpy.shutdown()
        self.spin_thread.join()

    __amr_control = None
    



