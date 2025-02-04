import argparse
import asyncio
from ollama import AsyncClient
import re
import os

async def load_diff(model_name, client, messages, diff_file, output_file):
    """Loads a .diff file, extracts code changes, requests AI review for each change, and saves the responses."""
    try:
        with open(diff_file, "r", encoding="utf-8") as file:
            diff_content = file.readlines()
    except FileNotFoundError:
        print(f"⚠️ Error: .diff file '{diff_file}' not found.")
        return

    print("\n📥 Sending .diff file to DeepSeek for review...")

    file_changes = {}
    current_file = None
    current_block = []

    for line in diff_content:
        if line.startswith("diff --git"):
            if current_block and current_file:
                file_changes.setdefault(current_file, []).append(current_block)
            current_block = []
            match = re.search(r"a/(.+?)\s+b/(.+)", line)
            if match:
                current_file = match.group(2)
        elif line.startswith("@@"):
            if current_block and current_file:
                file_changes.setdefault(current_file, []).append(current_block)
            current_block = []
        current_block.append(line.strip())

    if current_block and current_file:
        file_changes.setdefault(current_file, []).append(current_block)

    # Open output file for writing responses
    with open(output_file, "w", encoding="utf-8") as out_file:
        out_file.write(f"AI Review Results for {diff_file}\n")
        out_file.write("=" * 80 + "\n")

    # Process each file and its changes
    for file_path, changes in file_changes.items():
        print(f"\n📂 Reviewing file: {file_path}")
        for idx, block in enumerate(changes, start=1):
            change_text = "\n".join(block)
            review_prompt = (
                f"### Review {idx} in {file_path}:\n\n"
                f"The following is a code change from a diff file. Note that the diff file is from git diff log. Provide the understanding of the change.\n\n"
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

            # Save to file
            with open(output_file, "a", encoding="utf-8") as out_file:
                out_file.write(f"\n📂 File: {file_path}\n")
                out_file.write(f"🔍 Review {idx}:\n")
                out_file.write("-" * 40 + "\n")
                out_file.write(f"{response_text}\n")
                out_file.write("-" * 80 + "\n")

            messages.append({"role": "assistant", "content": response_text})

    print(f"\n✅ Review completed. Results saved to: {output_file}")

async def main():
    parser = argparse.ArgumentParser(description="AI Code Reviewer with DeepSeek")
    parser.add_argument("--model", type=str, default="deepseek-r1:1.5b", help="Specify the DeepSeek model (default: deepseek-r1:1.5b)")
    parser.add_argument("--diff", type=str, required=True, help="Path to the .diff file for review")
    parser.add_argument("--output", type=str, default="review_results.txt", help="Path to save the AI review results")

    args = parser.parse_args()
    client = AsyncClient()
    messages = []

    await load_diff(args.model, client, messages, args.diff, args.output)

if __name__ == "__main__":
    asyncio.run(main())  # ✅ Single event loop execution
