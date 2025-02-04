# DeepSeek-Helper
Deploy DeepSeek model on local machine and use the pre-trained model for application.

## Installation
1. Install ollama
    ```
    pip3 install ollama
    ```
2. Download models
    ```
    ollama run deepseek-r1:1.5b
    ```

## 1. Fast synthetic scen generator
### Description
Generate simulation scenario for autonomous driving ADK evaluation with the help of DeepSeek model. The input is a description in natural language and the output is a structured scenario definition, including ego behavior, actor behavior, and parameter range definition. This functionality can be served as a quick start of a sythetic scenario construction.

### Run
    ```
    python3 fast_synthetic_scene_generator.py --model deepseek-r1:1.5b --yaml ./yaml_files/cut-in_passenger-vehicle_decel.scn.yaml
    ```

## 2. Code reviewer for git difference file
### Description
Provide code review feedback for pull requests and save time for human pr reviewer by filtering out unimportant code update.

### Run
    ```
    python3 pr_reviewer.py --model deepseek-r1:1.5b --diff ./diff_files/common_utils_lane_map.diff
    ```

## 3. Voice assistant
### Description
Provide a user-interface for generating ADK commands from natural language.

### Run
- Install dependencies
    ```
    pip3 install SpeechRecognition
    pip3 install PyAudio
    ```
- Run ROS2 node (arguments are optional)
    ```
    python3 voice_assistant.py --model deepseek-r1:1.5b --msg_file ./json_files/ego_command.json 
    ```

## 4. Traffic regulation text inteerpreter
### Description
Provide a user-interface for generating ADK commands from natural language.

### Run
- Run ROS2 node (arguments are optional)
    ```
    python3 regulation_interpreter.py --model deepseek-r1:1.5b --cfg_file ./json_files/traffic_signs.json
    ```
- Publishing Test Messages to "/traffic_sign_text"
    ros2 topic pub /traffic_sign_text std_msgs/msg/String "{data: 'Speed Limit 70 mph'}"


## Other ollama python example
    https://github.com/ollama/ollama-python/tree/main
