import argparse
import asyncio
import json
import os
from ollama import AsyncClient

async def load_scenario_policies(json_file_path):
    """Loads scenario policies from the specified JSON file."""
    if not os.path.exists(json_file_path):
        raise FileNotFoundError(f"❌ JSON file not found: {json_file_path}")

    with open(json_file_path, "r", encoding="utf-8") as file:
        return json.load(file)

async def get_scenario_match(client, model_name, user_input, scenarios_json):
    """Uses the DeepSeek model to classify the input scenario description and streams its thought process."""

    # Construct policy descriptions for better classification
    policy_descriptions = "\n".join(
        [f"- {key}: {value.get('description', 'No description available')}" for key, value in scenarios_json.items()]
    )

    prompt = (
        "Given the following autonomous driving scenario description:\n\n"
        f"\"{user_input}\"\n\n"
        "Identify the most appropriate scenario type from the following options based on their descriptions:\n"
        f"{policy_descriptions}\n\n"
        "Return ONLY the matched scenario name from the list."
    )

    messages = [{"role": "user", "content": prompt}]
    key_response = []

    print("\n🤖 Thinking...", end="", flush=True)

    async for part in (await client.chat(model=model_name, messages=messages, stream=True)):
        content = part['message']['content'].strip()
        key_response.append(content)  # Append each part separately
        print(content, end=" ", flush=True)  # Print with space to prevent word merging

    print("\n")  # Print newline after streaming

    # Join response parts properly
    full_response = " ".join(key_response).strip('"\'')

    # Validate the matched scenario
    if full_response.lower() in scenarios_json:
        return full_response.lower()
    else:
        return None

async def chat(model_name, client, scenarios_json):
    """Interactive terminal chat session."""
    print(f"🚀 Scenario Matching Chat ({model_name}) - Type 'exit' to quit")

    while True:
        user_input = input("\nEnter a scenario description: ")

        if user_input.lower() == "exit":
            print("👋 Exiting chat. Goodbye!")
            break
        
        matched_scenario = await get_scenario_match(client, model_name, user_input, scenarios_json)

        if matched_scenario:
            print("\n✅ Matched Scenario:", matched_scenario.capitalize())
            print(json.dumps(scenarios_json[matched_scenario], indent=4))
        else:
            print("\n❌ No matching scenario found. Try a different description.")

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
