from utils.audio import *
from utils.command import *
import json
from amr_def import *
import time

class SmartAssistant:

    def __init__(self, amr_control: AMRControl):
        self.__amr_control = amr_control

        self.__current_amr_state = "idle"
        self.__amr_control.current_location_name.append(INITIAL_LOCATION)
        self.__previous_response = ""
        self.__user_input = ""
        self.__user_new_input = ""

    def __ai_interact(self):
        self.__amr_control.smart_assist_prompt.put(
            "Recording\naudio input")
        # print("Recording audio input")
        audio_data = record_audio(duration=5)

        if REPLAY_AUDIO:
            play_audio(audio_data)

        self.__amr_control.smart_assist_prompt.put(
            "Processing\naudio input")
        self.__user_new_input = voice_to_text(audio_data)
        if PRINT_EVERYTHING:
            print(f"Recognized text: {self.__user_new_input}")

        prompt = """

        This is the instruction:

            You have been asked to control an Autonomous Mobile Robot (AMR) in a factory setting.

            The factory has a 400 x 400 square meter floor space. (pose: [0, 0, 0, 0, 0, 0, 1] ~ [400, 400, 0, 0, 0, 0, 1])

            Respond to this request sent to the factory AMR controller only in JSON format which will be interpreted by the application code to execute the actions:

            - "command": change the state of the AMR navigation
            
            You would have to give the final "command" based on the AMR's initial location ({0}) and the previous response.

            The AMR can navigate to the following locations:
            "Table_Leg_Warehouse" has pose [200, 100, 0, 0, 0, 0, 1]
            "Table_Top_Warehouse" has pose [300, 200, 0, 0, 0, 0, 1]
            "Assembly_Line" has pose [100, 300, 0, 0, 0, 0, 1]
            "car_1" has pose [400, 300, 0, 0, 0, 0, 1]
            "Base_Station" has pose [0, 0, 0, 0, 0, 0, 1]

            Details about the response JSON:
            
            {{
                "command":{{
                    "navigate": "True"
                }},

                "target_locations":[
                    {{"location": "location_1", "pose": [x_1, y_1, z_1, x_orient_1, y_orient_1, z_orient_1, w_orient_1]}},
                    {{"location": "location_2", "pose": [x_2, y_2, z_2, x_orient_2, y_orient_2, z_orient_2, w_orient_2]}},
                    .....
                    {{"location": "location_n", "pose": [x_n, y_n, z_n, x_orient_n, y_orient_n, z_orient_n, w_orient_n]}},
                ],
                
                "comment": {{
                    "text": "You will need to add some comment here about the path planning and navigation"
                }}
                
            }}

            The "target_locations" property should be the generated desired navigation path:
                The content of the property should contain the pose [x, y, z, orientation] coordinates of the locations to traverse.
                The AMR ultimate goal is to help user to move parts from one location to another and to the assembly line.
                Ensure gentle and smooth navigation between locations to handle delicate parts.
                
            The "comment" property should provide commentary on the path planning and navigation, adding any relevant information about modifications if needed.

            Only output the response in JSON format! Do not add additional response text.
            Generate the "target_locations" based on the provided Python script for controlling the AMR using ROS 2.

        This is the request:

            The factory AMR controller needs to navigate to various locations within the factory floor.

            previous reponse: {1}

            User's request: {2}
            
            User's additonal request: {3}
            
            Modify the response JSON based on the previous response and the user's additional request, if any. 

        """.format(self.__amr_control.current_location_name[-1], self.__previous_response,
                   self.__user_input, self.__user_new_input)


        self.__amr_control.smart_assist_prompt.put(
            "Generating\nsmart output")
        response = generate_command(prompt, self.__user_new_input)
        self.__previous_response = response
        if PRINT_EVERYTHING:
            print(f"Generated command: {response}")

        start_index = response.find("{")
        end_index = response.rfind("}")
        json_response = response[start_index:end_index + 1]
        if PRINT_EVERYTHING:
            print(f'Generated json:\n{json_response}')

        response_data = json.loads(json_response)
        self.__location_names = []
        self.__poses = []

        # Extract data from the 'target_locations' property in the JSON response
        for location in response_data["target_locations"]:
            self.__location_names.append(location["location"])
            pose = location["pose"]
            self.__poses.append({
                "x": pose[0],
                "y": pose[1],
                "z": pose[2],
                "orientation": {
                    "x": pose[3],
                    "y": pose[4],
                    "z": pose[5],
                    "w": pose[6]
                }
            })

        # self.__amr_control.smart_assist_location_name_setting.put(location_names)
        # self.__amr_control.smart_assist_pose_setting.put(poses)

        if PRINT_EVERYTHING:
            print(f'Generated dict:\n{response_data}')

        self.__amr_control.smart_assist_prompt.put("Audio response")
        comment_text = response_data["comment"]["text"]
        text_to_speech(comment_text)

    def run(self):
        while True:
            # while not self.__amr_control.start_listen_event.is_set() and not self.__amr_control.reset_event.is_set():
            #     time.sleep(0.1)

             

            if self.__amr_control.reset_event.is_set():
                return

            print("start")

            self.__ai_interact()

            print("finish")

            print()

        
            # Check if the user wants to quit the loop
            print("Current target locations:")
            print(self.__location_names)
            print("Current target poses:")
            print(self.__poses)
            user_input = input("Press 'q' and Enter to quit the loop, Press 'c' to confirm the locations, or just Enter to continue adding request: ")
            if user_input.lower() == 'q':
                print("Quitting the loop...")
                break
            elif user_input.lower() == 'c':
                self.__amr_control.smart_assist_location_name_setting.put(self.__location_names)
                self.__amr_control.smart_assist_pose_setting.put(self.__poses)
                break
            elif user_input == '':
                self.__location_names = []
                self.__poses = []
                continue

            self.__amr_control.start_listen_event.clear()

            self.__amr_control.smart_assist_prompt.put(
                "Press button to\nstart listen")
            self.__amr_control.finish_listen_event.set()

    __amr_control = None

    __previous_response = None
    __user_input = None
    __user_new_input = None

    __location_names = None
    __poses = None


if __name__ == "__main__":
    smart_assitant = SmartAssistant()
    smart_assitant.run()
