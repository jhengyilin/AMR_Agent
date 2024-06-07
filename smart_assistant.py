from utils.audio import *
from utils.command import *
import json
from amr_def import *

class SmartAssistant:

    def __init__(self, amr_control: AMRControl):
        self.__amr_control = amr_control

        self.__current_amr_state = "idle"
        self.__amr_control.current_location_name.append(INITIAL_LOCATION)
        self.__previous_response = ""
        self.__user_input = ""
        self.__user_new_input = ""

        self.__stop_event = threading.Event()

    def __ai_interact(self):
        self.__amr_control.smart_assist_prompt.put(
            "Recording\naudio input")
        # print("Recording audio input")
        audio_data = record_audio(duration=RECORD_DURATION)

        if REPLAY_AUDIO:
            play_audio(audio_data)

        self.__amr_control.smart_assist_prompt.put(
            "Processing\naudio input")
        self.__user_new_input = voice_to_text(audio_data)
        if PRINT_EVERYTHING:
            print(f"Recognized text: {self.__user_new_input}")

        # self.__user_new_input = input("Enter your new request: ")

        prompt = """

        This is the instruction:

            You have been asked to control an Autonomous Mobile Robot (AMR) in a factory setting.

            The factory has a 5 x 5 meter floor space. (pose: [0, 0, 0, 0, 0, 0, 1] ~ [5, 5, 0, 0, 0, 0, 1])

            Respond to this request sent to the factory AMR controller only in JSON format which will be interpreted by the application code to execute the actions:

            - "command": change the state of the AMR navigation

            The AMR can navigate to the following locations:
            "Table_Top_Warehouse" has pose [3.86, -0.50, 1.0, 0.0, 0.0, 0.0, 1.0]
            "Table_Leg_Warehouse" has pose [3.81, -1.08, 1.0, 0.0, 0.0, 0.0, 1.0]
            "Chair_Leg_Warehouse" has pose [1.90, -1.02, 1.0, 0.0, 0.0, -0.74, 0.67]
            "Chair_Back_Warehouse" has pose [0.79, -1.09, 1.0, 0.0, 0.0, -0.74, 0.67]
            "Chair_Seat_Warehouse" has pose [1.39, -1.11, 1.0, 0.0, 0.0, -0.61, 0.79]
            "Screw_Warehouse" has pose [3.91, -0.07, 1.0, 0.0, 0.0, 0.0, 1.0]
            "Assembly_Line" has pose [0.02, -1.22, 1.0, 0.0, 0.0, 0.0, -0.62, 0.79]
            "Base_Station" has pose [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

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
                With direct mentioning, you can assume the machine only have the mentioned item on top and does not have any other item.
                Every product will need screw to assemble, make sure to include the part. 
                Generally, the AMR is empty and will need to gather item along the way and ultimately to the assembly line, then finally to the base station.
                Ensure gentle and smooth navigation between locations to handle delicate parts.
                
            The "comment" property should provide commentary on the path planning and navigation, adding any relevant information about modifications if needed.
                You can reponse in the user's input language at the comment.
            Only output the response in JSON format! Do not add additional response text.
            Generate the "target_locations" based on the provided Python script for controlling the AMR using ROS 2.

            This is the example response:

                {{
                    "command": {{
                        "navigate": true
                    }},

                    "target_locations": [
                        {{"location": "Table_Leg_Warehouse", "pose": [200, 100, 0, 0, 0, 0, 1]}},
                        {{"location": "Table_Top_Warehouse", "pose": [300, 200, 0, 0, 0, 0, 1]}},
                        {{"location": "Assembly_Line", "pose": [100, 300, 0, 0, 0, 0, 1]}}
                    ],

                    "comment": {{
                        "text": "The AMR will visit the table leg warehouse, pick up the table top and finally proceed to the assembly line for final assembly."
                    }}
                }}


        This is the request:

            The factory AMR controller needs to navigate to various locations within the factory floor.

            previous reponse: {1}

            User's request: {2}
            
            Modify the response JSON based on the previous response and the user's additional request, if any. 

        """.format(self.__amr_control.current_location_name[-1], self.__previous_response,
                   self.__user_input)
        if PRINT_EVERYTHING:
            print(f'Generated prompt:\n{prompt}')

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
        if PRINT_COMMENT:
            print(f"Comment: {comment_text}")
        if PLAY_TEXT2PSEECH_AUDIO:
            text_to_speech(comment_text)

    def run(self):
        while not self.__stop_event.is_set():
            print("start")

            user_input = input("Press Any key to start the smart assistant: ")

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
                return
            elif user_input.lower() == 'c':
                self.__amr_control.smart_assist_location_name_setting.put(self.__location_names)
                self.__amr_control.smart_assist_pose_setting.put(self.__poses)
                self.__amr_control.send_command_flag = True
                return
            elif user_input == '':
                self.__location_names = []
                self.__poses = []
                continue

            self.__amr_control.start_listen_event.clear()

            self.__amr_control.smart_assist_prompt.put(
                "Press button to\nstart listen")
            self.__amr_control.finish_listen_event.set()
        
    def stop(self):
        self.__stop_event.set()

    __amr_control = None

    __previous_response = None
    __user_input = None
    __user_new_input = None

    __location_names = None
    __poses = None


if __name__ == "__main__":
    smart_assitant = SmartAssistant()
    smart_assitant.run()
