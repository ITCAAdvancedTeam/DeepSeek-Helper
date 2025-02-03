import argparse
import asyncio
import yaml
from ollama import AsyncClient

async def load_yaml(model_name, client, messages, yaml_file):
    """Loads YAML content and sends it to DeepSeek for memory storage, printing the AI’s response."""
    try:
        with open(yaml_file, "r", encoding="utf-8") as file:
            yaml_content = file.read()
    except FileNotFoundError:
        print(f"⚠️ Error: YAML file '{yaml_file}' not found.")
        return

    yaml_input = f"Here is an example of a scenario definition Yaml file {yaml_file} for autonomous driving simulation.\n Please understand the relation between description and the rest of the Yaml content \n\n{yaml_content}"
    
    messages.append({"role": "user", "content": yaml_input})
    print("\n📥 Sending YAML file to DeepSeek...")

    response_text = ""
    print("\nDeepSeek: ", end='', flush=True)

    async for part in await client.chat(model=model_name, messages=messages, stream=True):
        print(part['message']['content'], end='', flush=True)
        response_text += part['message']['content']

    print("\n✅ YAML file processed!\n")
    messages.append({"role": "assistant", "content": response_text})

async def chat(model_name, client, messages):
    """Interactive terminal chat session."""
    print(f"🚀 DeepSeek Terminal Chat ({model_name}) - Type 'exit' to quit")

    while True:
        user_input = input("\nScenario description: ")

        if user_input.lower() == "exit":
            print("👋 Exiting chat. Goodbye!")
            break
        
        requirement_input = "Please construct a yaml response based on the following scenario description.\n"
        messages.append({"role": "user", "content": requirement_input + user_input})

        print("DeepSeek: ", end='', flush=True)

        response_text = ""
        async for part in await client.chat(model=model_name, messages=messages, stream=True):
            print(part['message']['content'], end='', flush=True)
            response_text += part['message']['content']

        print("\n", "-" * 50)  

        messages.append({"role": "assistant", "content": response_text})

async def main():
    parser = argparse.ArgumentParser(description="DeepSeek Terminal Chatbot with YAML Initialization")
    parser.add_argument("--model", type=str, default="deepseek-r1:1.5b", help="Specify the DeepSeek model to use (e.g., deepseek-r1:1.5b)")
    parser.add_argument("--yaml", type=str, required=True, help="Path to the YAML file to initialize the AI context")

    args = parser.parse_args()
    client = AsyncClient()
    messages = []

    await load_yaml(args.model, client, messages, args.yaml)
    await chat(args.model, client, messages)

if __name__ == "__main__":
    asyncio.run(main())  # ✅ Single event loop execution
