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

## Fast synthetic scen generator
### Description
Generate simulation scenario for autonomous driving ADK evaluation with the help of DeepSeek model. The input is a description in natural language and the output is a structured scenario definition, including ego behavior, actor behavior, and parameter range definition. This functionality can be served as a quick start of a sythetic scenario construction.

### Run
    ```
    python3 fast_synthetic_scene_generator.py --model deepseek-r1:1.5b --yaml ./yaml_files/cut-in_passenger-vehicle_decel.scn.yaml
    ```

## Code reviewer for git difference file
### Description
Provide code review feedback for pull requests and save time for human code reviewer by filtering out unimportant code update.

### Run
    ```
    python ai_code_reviewer.py --diff ./diff_files/common_utils_lane_map.diff --model deepseek-r1:1.5b
    ```

## Other ollama python example
    https://github.com/ollama/ollama-python/tree/main
