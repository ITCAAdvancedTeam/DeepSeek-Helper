# DeepSeek-Helper
Deploy the DeepSeek model on a local machine and utilize the pre-trained model for various applications.

## Deployment on a Local Machine
1. **Install Ollama:**
    ```sh
    pip3 install ollama
    ```

2. **Download Models:**
    ```sh
    ollama run deepseek-r1:1.5b
    ```
    Select models from [Ollama DeepSeek Models](https://ollama.com/library/deepseek-r1:1.5b)

---

## 1. Fast Synthetic Scenario Generator
### Description
Generate simulation scenarios for autonomous driving **ADK evaluation** using the DeepSeek model.  
- **Input:** A natural language description.  
- **Output:** A structured scenario definition, including:
  - Ego behavior  
  - Actor behavior  
  - Parameter range definition  

This functionality serves as a **quick-start tool** for synthetic scenario construction.

### Run
```sh
python3 fast_synthetic_scene_generator.py --model deepseek-r1:1.5b --yaml ./yaml_files/cut-in_passenger-vehicle_decel.scn.yaml
```

## 2. Code Reviewer for Git Difference File
### Description
Automates **code review feedback** for pull requests, reducing the workload of human reviewers by filtering out minor or unimportant code updates.  
- **Input:** A Git difference (`.diff`) file.  
- **Output:** A structured review highlighting relevant code changes that may require further attention.  

This tool streamlines the **pull request (PR) review process**, making it more efficient and focused.

### Run
```sh
python3 pr_reviewer.py --model deepseek-r1:1.5b --diff ./diff_files/common_utils_lane_map.diff
```

## 3. Voice Assistant
### Description
The **Voice Assistant** provides a **speech-to-command interface** for generating **ADK commands** from natural language.  
- **Input:** Spoken commands from the user.  
- **Output:** Structured ADK commands formatted as JSON messages.  

This module allows for intuitive **voice-driven command generation**, making it easier to control autonomous driving functions.

### Run
- **Install dependencies:**
    ```sh
    pip3 install SpeechRecognition PyAudio
    ```
- **Run the ROS2 node (arguments are optional):**
    ```sh
    python3 voice_assistant.py --model deepseek-r1:1.5b --msg_file ./json_files/ego_command.json
    ```

## 4. Traffic Regulation Text Interpreter
### Description
The **Traffic Regulation Text Interpreter** processes **natural language traffic regulations** and converts them into **structured ADK commands**.  
- **Input:** Text descriptions of traffic rules (e.g., "Speed Limit 70 mph").  
- **Output:** ADK-compatible commands that allow the system to comply with traffic regulations.  

This tool ensures that **autonomous driving systems** correctly interpret and adhere to real-world traffic rules. It is essential for **integrating regulatory compliance** into ADK-based autonomous systems.

### Run
- **Run the ROS2 node (arguments are optional):**
    ```sh
    python3 regulation_interpreter.py --model deepseek-r1:1.5b --cfg_file ./json_files/traffic_signs.json
    ```
- **Publish test messages to `/traffic_sign_text`:**
    ```sh
    ros2 topic pub /traffic_sign_text std_msgs/msg/String "{data: 'Speed Limit 70 mph'}"
    ```
- **Publish image messages to `/traffic_sign_image`:**
    ```sh
    python3 traffic_sign_image_publisher.py --img_file ./traffic_sign_images/speed_limit_70.jpg
    ```

## Other Ollama Python Examples
Here are some examples of calling Ollama models using the Ollama Python library:  
[Ollama Python GitHub Repository](https://github.com/ollama/ollama-python/tree/main)

## Future Work
Fine-tune an expert model based on a pretrained distilled model.  
The training pipeline can refer to: [Hugging Face Open-R1](https://github.com/huggingface/open-r1)

### Hardware Requirements for Fine-Tuning DeepSeek Models

| Model          | VRAM Requirement (FP16) | VRAM Requirement (4-bit) | Recommended GPUs                  | System RAM | Storage Requirement |
|--------------|------------------------|------------------------|----------------------------------|------------|---------------------|
| **DeepSeek 14B**  | ~30 GB                 | ~8 GB                  | RTX 4090, RTX 6000 Ada, A40, A100 | 64 GB+      | 50 GB – 500 GB      |
| **DeepSeek 671B** | ~1.2 TB               | ~400 GB                | H100, H200, Multi-GPU Cluster    | 512 GB+     | Several TB+         |

### Additional Considerations
- **Quantization:** Using **4-bit quantization** significantly reduces VRAM requirements.
- **Optimization Techniques:** Mixed precision training and gradient checkpointing help manage memory usage.
- **Distributed Training:** **DeepSeek 671B** requires **multi-node GPU clusters** for fine-tuning.

For the most up-to-date requirements, refer to the official DeepSeek documentation.
