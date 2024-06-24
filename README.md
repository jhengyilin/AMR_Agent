# AMR Agent

This repo contains the code for the AMR Agent project.

## Overview

The AMR Agent project consists of several Python scripts that work together to control an Autonomous Mobile Robot (AMR). Here's a brief description of each script:

- `agent.py`: This is the master Python script that coordinates the overall behavior of the AMR.

- `smart_asistant.py`: This script is responsible for accessing the LLM api to retrieve planned poses and orientations for the AMR.

- `message_handler.py`: This script handles the communication with the ROS2 (Robot Operating System) and controls the AMR.

- `nav2_commander/nav2_commander.py` is a ROS2 node that message_handler will communicate with using nav2 action client feature.(you will need to build this package by putting nav2_commander folder into ros2 src folder)

- `utils/audio.py`: This is a helper script that provides audio-related functionality.

- `utils/command.py`: This is another helper script to get command.

## Usage

To use the AMR Agent, follow these steps:

1. Create a new conda environment by running `conda create --name amr_agent python=3.9.18`.

2. Activate the conda environment by running `conda activate amr_agent`.

3. Install the required dependencies by running the following commands:
    ```bash
    sudo apt install ffmpeg
    pip install -r requirements.txt

    ----------navigate to ROS2 workspace----------
    
    colcon build --packages-select nav2_commander
    ```
4. Run the `application_loop.sh` or `agent.py` script to start the AMR Agent.

## Future Work

Implement localhost llm using Ollama or LM-Studio to have better  

1. Data Privacy

2. Local Usage

