import argparse
import asyncio
import json
import os
import re
from ollama import AsyncClient

async def load_scenario_policies(json_file_path):
    """Loads scenario policies from the specified JSON file."""
    if not os.path.exists(json_file_path):
        raise FileNotFoundError(f"❌ JSON file not found: {json_file_path}")

    with open(json_file_path, "r", encoding="utf-8") as file:
        return json.load(file)

async def extract_key_from_response(response_text, scenarios_json):
    """Extracts the valid scenario key by iterating from the end of the response."""
    
    # Split response into tokens while keeping spacing intact
    words = response_text.split()

    # Iterate backwards to find a valid scenario key
    for i in range(len(words) - 1, -1, -1):
        word = words[i].strip().lower()

        # If the word is a valid key in scenarios_json, return it
        if word in scenarios_json:
            print(f"✅ Matched Key Found: {word}")
            return word

    # If no valid key is found, print an error and return None
    print("❌ No valid scenario key found in the response.")
    return None

async def get_scenario_key(client, model_name, user_input, scenarios_json):
    """First chat: Uses DeepSeek to classify the input scenario and return only the key."""

    # Construct policy descriptions for better classification
    policy_descriptions = "\n".join(
        [f"- {key}: {value.get('description', 'No description available')}" for key, value in scenarios_json.items()]
    )

    prompt = (
        "Given the following autonomous driving scenario description:\n\n"
        f"\"{user_input}\"\n\n"
        "Identify the most appropriate scenario type from the following options based on their descriptions:\n"
        f"{policy_descriptions}\n\n"
        "Return ONLY the matched scenario name from the list at the very end of your response. The name should be an exact match to one of the options listed above. Do NOT add extra words, punctuation, formatting, or explanations."
    )

    messages = [{"role": "user", "content": prompt}]
    key_response = ""

    print("\n🤖 Thinking (Matching Scenario)...", end="", flush=True)

    async for part in (await client.chat(model=model_name, messages=messages, stream=True)):
        content = part['message']['content']
        key_response += content  # Append response text
        print(content, end='', flush=True)  # Stream response live

    print("\n", "-" * 80)  # Separator for clarity

    # Extract the correct key from response
    matched_scenario = await extract_key_from_response(key_response, scenarios_json)

    return matched_scenario

async def get_scenario_details(client, model_name, user_input, matched_scenario, scenarios_json):
    """Second chat: Uses DeepSeek to determine details for the matched scenario while following a strict JSON format."""
    
    if matched_scenario not in scenarios_json:
        return None  # No valid match

    # Retrieve the original structure from policies.json
    scenario_template = scenarios_json[matched_scenario]

    # Ensure DeepSeek follows this exact structure
    prompt = (
        f"Now that we have determined the scenario type as: \"{matched_scenario}\"\n\n"
        f"Based on the given description: \"{user_input}\"\n\n"
        "Fill in the missing values in the JSON structure below based on your understanding of the scenario.\n"
        "Modify only the values while keeping the structure identical:\n\n"
        f"{json.dumps(scenario_template, indent=4)}\n\n"
        "**IMPORTANT:**\n"
        "- Return **only** a JSON object **without any explanations, comments, or extra text**.\n"
        "- Do not add new fields or modify existing field names.\n"
        "- Ensure values are consistent with their expected data types (floats, booleans, etc.).\n"
        "- Keep the response clean and strictly in JSON format."
    )

    messages = [{"role": "user", "content": prompt}]
    details_response = ""

    print("\n🤖 Thinking (Filling Scenario Details)...", end="", flush=True)

    async for part in (await client.chat(model=model_name, messages=messages, stream=True)):
        content = part['message']['content']
        details_response += content  # Append response text
        print(content, end='', flush=True)  # Stream response live

    print("\n", "-" * 80)  # Separator for clarity

    # Extract the JSON object using regex
    match = re.search(r"\{.*\}", details_response, re.DOTALL)
    
    if match:
        json_str = match.group(0)  # Extracted JSON part

        try:
            details_data = json.loads(json_str)

            # Ensure proper data types
            if "geometry" in details_data:
                details_data["geometry"] = {k: float(v) for k, v in details_data["geometry"].items()}
            
            if "traffic_rule" in details_data:
                details_data["traffic_rule"] = {k: bool(v) for k, v in details_data["traffic_rule"].items()}

            if "traffic_density" in details_data:
                details_data["traffic_density"] = max(0.0, min(1.0, float(details_data["traffic_density"])))  # Clamp between 0.0 and 1.0
            
            return details_data  # ✅ Successfully parsed JSON

        except json.JSONDecodeError:
            print("\n❌ JSON decoding failed. The response format was incorrect.")
            return None
    else:
        print("\n❌ No valid JSON object found in the response.")
        return None

async def chat(model_name, client, scenarios_json):
    """Interactive terminal chat session."""
    print(f"🚀 Scenario Matching Chat ({model_name}) - Type 'exit' to quit")

    while True:
        user_input = input("\nEnter a scenario description: ")

        if user_input.lower() == "exit":
            print("👋 Exiting chat. Goodbye!")
            break
        
        # Step 1: Get the matched scenario key
        matched_scenario = await get_scenario_key(client, model_name, user_input, scenarios_json)

        print("\n✅ Matched Scenario:", matched_scenario.capitalize())

        if not matched_scenario:
            print("\n❌ No matching scenario found. Try a different description.")
            continue

        # Step 2: Get detailed scenario information
        scenario_details = await get_scenario_details(client, model_name, user_input, matched_scenario, scenarios_json)

        if scenario_details:
            print("\n📋 Scenario Details:")
            print(json.dumps(scenario_details, indent=4))
        else:
            print("\n⚠️ Failed to retrieve scenario details.")

async def main():
    parser = argparse.ArgumentParser(description="Autonomous Driving Scenario Matcher using Ollama")
    parser.add_argument("--model", type=str, default="deepseek-r1:1.5b", help="Specify the DeepSeek model to use (e.g., deepseek-r1:1.5b)")
    parser.add_argument("--cfg", type=str, default="./json_files/policies.json", help="Path to the JSON policy file")

    args = parser.parse_args()
    client = AsyncClient()

    # Load the policy JSON file dynamically
    scenarios_json = await load_scenario_policies(args.cfg)

    await chat(args.model, client, scenarios_json)

if __name__ == "__main__":
    asyncio.run(main())  # ✅ Single event loop execution
