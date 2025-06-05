#!/usr/bin/env python3
import os
import json
import pandas as pd

import rclpy
from rclpy.node import Node

from dotenv import load_dotenv
from openai import OpenAI

# -----------------------------------------------------------------------------
# This ROS2 node implements a chat interface (via OpenAI's Chat API) that
# continuously lets the user add, rename, or remove objects from a CSV file.
# The CSV is expected to have at least a column named "object". Each time the
# user issues a free-form instruction, the node asks GPT-4 to decide whether
# to call one of three functions (remove_objects, rename_object, add_object).
# After applying that function to the CSV, GPT-4 acknowledges, and the chat
# remains open for further edits. ROS2 is used solely for parameter passing
# (e.g., csv_path) and node startup/shutdown; all terminal I/O is done with
# print()/input().
# -----------------------------------------------------------------------------

# Load OpenAI API key from .env (or environment)
load_dotenv()
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")

if not OPENAI_API_KEY:
    print("Error: OPENAI_API_KEY not found in environment or .env.")
    exit(1)

# Instantiate the new OpenAI client
client = OpenAI(api_key=OPENAI_API_KEY)

# Definitions of the OpenAI functions for function‐calling
FUNCTIONS = [
    {
        "name": "remove_objects",
        "description": "Remove one or more objects from the CSV.",
        "parameters": {
            "type": "object",
            "properties": {
                "objects": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "List of object names to remove"
                }
            },
            "required": ["objects"]
        }
    },
    {
        "name": "rename_object",
        "description": "Rename a single object in the CSV.",
        "parameters": {
            "type": "object",
            "properties": {
                "old_name": {
                    "type": "string",
                    "description": "The existing object name to rename"
                },
                "new_name": {
                    "type": "string",
                    "description": "The new name for that object"
                }
            },
            "required": ["old_name", "new_name"]
        }
    },
    {
        "name": "add_object",
        "description": "Add a new object to the CSV. Other columns will be blank or default.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {
                    "type": "string",
                    "description": "The name of the new object to add"
                }
            },
            "required": ["object_name"]
        }
    }
]

# System prompt instructing GPT-4 how to behave
SYSTEM_PROMPT = """
You are an assistant that edits a CSV file of detected objects. The CSV has a column named "object", 
and each row corresponds to one detected object. The user will issue free-form instructions like 
"remove all vehicles", "rename bird to sparrow", or "add umbrella", or complex ones like "take 
away everything that has to do with flying". 

You have three functions available to modify the CSV:
  1. remove_objects(objects: List[str]) — removes all rows whose "object" column exactly matches 
     any string in objects.
  2. rename_object(old_name: str, new_name: str) — renames every row whose "object" column equals 
     old_name to new_name.
  3. add_object(object_name: str) — appends a new row to the CSV with the given object name. Other 
     columns should be blank.

Your task:
- When the user gives an instruction, decide if you can carry it out unambiguously with the functions 
  above.
  - If yes, call the appropriate function with exact object names.
  - If no (for example, user says "remove everything that flies" and you're not certain whether 
    "suitcase" is a flying object), ask a follow-up question (e.g., "Should I also remove 'suitcase'?") 
    so that you can be sure.
- Do NOT output any text except:
  - A follow-up question (if you need clarification), or
  - A function call in JSON form that matches one of the defined functions.
- After the function call, wait for the code to execute it and send you back a function result. Then 
  produce a final acknowledgement like "Removed X objects." or "Renamed 'bird' to 'sparrow'." followed by 
  "You can ask me to make more changes at any time."
"""

class CsvChatManagerNode(Node):
    def __init__(self):
        super().__init__('object_review_llm')

        # Declare a parameter 'csv_path'; default is a placeholder
        self.declare_parameter(
            'csv_path',
            '/home/shawn/Documents/spot/robot_suite_v2/detected_objects.csv'
        )
        self.csv_path = self.get_parameter('csv_path').value

        # Verify API key again within ROS 2 context
        if not OPENAI_API_KEY:
            self.get_logger().error("OPENAI_API_KEY not set in environment/.env. Shutting down.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"CSV Chat Manager initialized with CSV: {self.csv_path}")

        # Start the interactive chat loop (this blocks until user types 'exit' or 'quit')
        self.run_chat_loop()

    # Read the current CSV into a DataFrame
    def read_csv(self):
        try:
            df = pd.read_csv(self.csv_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV at {self.csv_path}: {e}")
            df = pd.DataFrame(columns=['object'])
        return df

    # Overwrite the CSV with the given DataFrame
    def write_csv(self, df: pd.DataFrame):
        try:
            df.to_csv(self.csv_path, index=False)
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV at {self.csv_path}: {e}")

    # Helper to call OpenAI's new ChatCompletion API
    def call_openai(self, messages, functions=None, function_call="auto"):
        call_kwargs = {
            "model": "gpt-4-turbo",
            "messages": messages
        }
        if functions is not None:
            call_kwargs["functions"] = functions
            call_kwargs["function_call"] = function_call

        # Use the new client API: client.chat.completions.create(…)
        response = client.chat.completions.create(**call_kwargs)
        # Return the single 'message' object from the first choice
        return response.choices[0].message

    # Execute one of the defined functions on the CSV, returning a result dict
    def execute_function_call(self, func_name, arguments):
        df = self.read_csv()
        result = {}

        if func_name == "remove_objects":
            names_to_remove = arguments.get("objects", [])
            before = len(df)
            df = df[~df["object"].isin(names_to_remove)].copy()
            after = len(df)
            removed_count = before - after
            self.write_csv(df)
            result = {"status": "success", "removed_count": removed_count}

        elif func_name == "rename_object":
            old = arguments.get("old_name", "")
            new = arguments.get("new_name", "")
            match_mask = df["object"] == old
            renamed_count = int(match_mask.sum())
            df.loc[match_mask, "object"] = new
            self.write_csv(df)
            result = {"status": "success", "renamed_count": renamed_count}

        elif func_name == "add_object":
            new_name = arguments.get("object_name", "")
            # Keep existing columns; fill them with blank for the new row
            columns = df.columns.tolist()
            new_row = {col: "" for col in columns}
            new_row["object"] = new_name
            df = pd.concat([df, pd.DataFrame([new_row])], ignore_index=True)
            self.write_csv(df)
            result = {"status": "success", "added": new_name}

        else:
            result = {"status": "error", "message": f"Unknown function: {func_name}"}

        return result

    # Start an interactive chat loop. This will block until the user types 'exit' or 'quit'.
    def run_chat_loop(self):
        print("=== CSV Chat Manager (ROS2 Node) ===")
        print("Type 'exit' or 'quit' to end the session.\n")

        # Initialize conversation history
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]

        # Let the assistant ask the opening question
        assistant_msg = self.call_openai(
            messages,
            functions=FUNCTIONS,
            function_call="none"
        )
        # Access content via attribute
        if assistant_msg.content:
            print("Assistant:", assistant_msg.content.strip())
            messages.append({"role": "assistant", "content": assistant_msg.content})

        # Main loop
        while rclpy.ok():
            try:
                user_input = input("You: ").strip()
            except EOFError:
                # Terminal closed unexpectedly
                break

            if user_input.lower() in ["exit", "quit"]:
                print("Exiting chat. Goodbye!")
                break

            # Append the user's message
            messages.append({"role": "user", "content": user_input})

            # Let the model decide if it wants to call a function
            assistant_msg = self.call_openai(
                messages,
                functions=FUNCTIONS,
                function_call="auto"
            )

            # If the assistant returns plain content (no function_call), it's a follow-up
            if assistant_msg.content and assistant_msg.function_call is None:
                print("Assistant:", assistant_msg.content.strip())
                messages.append({"role": "assistant", "content": assistant_msg.content})
                continue

            # If the assistant wants to call a function:
            if assistant_msg.function_call:
                func_name = assistant_msg.function_call.name
                raw_args = assistant_msg.function_call.arguments or "{}"

                try:
                    arguments = json.loads(raw_args)
                except json.JSONDecodeError:
                    # Malformed JSON: ask the assistant to clarify
                    print("Assistant attempted a function call but provided invalid JSON.")
                    print("Raw arguments:", raw_args)
                    messages.append({"role": "assistant", "content": "<malformed function_call>"})
                    continue

                # Execute the function on the CSV
                func_result = self.execute_function_call(func_name, arguments)
                # Append the assistant's function_call message to history
                messages.append({
                    "role": "assistant",
                    "content": None,
                    "function_call": {
                        "name": func_name,
                        "arguments": raw_args
                    }
                })

                # Create a "function" role message with the result
                func_result_msg = {
                    "role": "function",
                    "name": func_name,
                    "content": json.dumps(func_result)
                }
                messages.append(func_result_msg)

                # Let the model produce a final acknowledgement
                final_ack = self.call_openai(messages)
                if final_ack.content:
                    print("Assistant:", final_ack.content.strip())
                    messages.append({"role": "assistant", "content": final_ack.content})

                # Continue waiting for the next user input
                continue

            # If something unexpected occurs:
            print("Assistant (unexpected response):", assistant_msg)
            messages.append({"role": "assistant", "content": "<unexpected response>"})

        # After exiting the loop, shut down the node
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CsvChatManagerNode()
    # The chat loop runs inside the node's __init__, blocking until exit.
    try:
        node.destroy_node()
    except Exception:
        pass


if __name__ == "__main__":
    main()
