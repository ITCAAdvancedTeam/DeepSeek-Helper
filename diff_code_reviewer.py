import argparse
import asyncio
from ollama import AsyncClient
import re

async def load_diff(model_name, client, messages, diff_file):
    """Loads a .diff file, extracts code changes, and requests AI review for each change."""
    try:
        with open(diff_file, "r", encoding="utf-8") as file:
            diff_content = file.readlines()
    except FileNotFoundError:
        print(f"⚠️ Error: .diff file '{diff_file}' not found.")
        return

    print("\n📥 Sending .diff file to DeepSeek for review...")

    change_blocks = []
    current_block = []
    file_changes = {}
    current_file = None

    for line in diff_content:
        if line.startswith("diff --git"):
            # New file change detected
            if current_block:
                file_changes[current_file] = file_changes.get(current_file, []) + [current_block]
            current_block = []
            match = re.search(r"a/(.+?)\s+b/(.+)", line)
            if match:
                current_file = match.group(2)
        elif line.startswith("@@"):
            # New code change detected within the same file
            if current_block:
                file_changes[current_file] = file_changes.get(current_file, []) + [current_block]
            current_block = []
        current_block.append(line.strip())

    if current_block and current_file:
        file_changes[current_file] = file_changes.get(current_file, []) + [current_block]

    # Process each change separately
    for file_path, changes in file_changes.items():
        print(f"\n📂 Reviewing file: {file_path}")
        for idx, block in enumerate(changes, start=1):
            change_text = "\n".join(block)
            review_prompt = (
                f"### Review {idx} in {file_path}:\n\n"
                f"The following is a code change from a diff file. Provide constructive feedback on improvements, correctness, and optimizations.\n\n"
                f"```diff\n{change_text}\n```"
            )

            messages.append({"role": "user", "content": review_prompt})
            print(f"\n🔍 Reviewing change {idx} in {file_path}...")

            response_text = ""
            print("\nDeepSeek: ", end='', flush=True)

            async for part in (await client.chat(model=model_name, messages=messages, stream=True)):
                print(part['message']['content'], end='', flush=True)
                response_text += part['message']['content']

            print("\n", "-" * 80)

            messages.append({"role": "assistant", "content": response_text})


async def main():
    parser = argparse.ArgumentParser(description="AI Code Reviewer with DeepSeek")
    parser.add_argument("--model", type=str, default="deepseek-r1:1.5b", help="Specify the DeepSeek model (default: deepseek-r1:1.5b)")
    parser.add_argument("--diff", type=str, required=True, help="Path to the .diff file for review")

    args = parser.parse_args()
    client = AsyncClient()
    messages = []

    await load_diff(args.model, client, messages, args.diff)

if __name__ == "__main__":
    asyncio.run(main())  # ✅ Single event loop execution
