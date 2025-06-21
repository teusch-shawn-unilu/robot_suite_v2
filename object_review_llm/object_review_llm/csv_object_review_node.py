import os
import json
import pandas as pd

import rclpy
from rclpy.node import Node

from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")

if not OPENAI_API_KEY:
    print("Error: OPENAI_API_KEY not found in environment or .env.")
    exit(1)

client = OpenAI(api_key=OPENAI_API_KEY)

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

        self.declare_parameter(
            'csv_path',
            '/home/shawn/Documents/spot/robot_suite_v2/detected_objects.csv'
        )
        self.csv_path = self.get_parameter('csv_path').value

        if not OPENAI_API_KEY:
            self.get_logger().error("OPENAI_API_KEY not set in environment/.env. Shutting down.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"CSV Chat Manager initialized with CSV: {self.csv_path}")

        self.run_chat_loop()

    def read_csv(self):
        try:
            df = pd.read_csv(self.csv_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV at {self.csv_path}: {e}")
            df = pd.DataFrame(columns=['object'])
        return df

    def write_csv(self, df: pd.DataFrame):
        try:
            df.to_csv(self.csv_path, index=False)
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV at {self.csv_path}: {e}")

    def call_openai(self, messages, functions=None, function_call="auto"):
        call_kwargs = {
            "model": "gpt-4-turbo",
            "messages": messages
        }
        if functions is not None:
            call_kwargs["functions"] = functions
            call_kwargs["function_call"] = function_call

        response = client.chat.completions.create(**call_kwargs)
        return response.choices[0].message

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
            columns = df.columns.tolist()
            new_row = {col: "" for col in columns}
            new_row["object"] = new_name
            df = pd.concat([df, pd.DataFrame([new_row])], ignore_index=True)
            self.write_csv(df)
            result = {"status": "success", "added": new_name}

        else:
            result = {"status": "error", "message": f"Unknown function: {func_name}"}

        return result

    def run_chat_loop(self):
        print("=== CSV Chat Manager (ROS2 Node) ===")
        print("Type 'exit' or 'quit' to end the session.\n")

        messages = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]

        assistant_msg = self.call_openai(
            messages,
            functions=FUNCTIONS,
            function_call="none"
        )

        if assistant_msg.content:
            print("Assistant:", assistant_msg.content.strip())
            messages.append({"role": "assistant", "content": assistant_msg.content})

        while rclpy.ok():
            try:
                user_input = input("You: ").strip()
            except EOFError:
                break

            if user_input.lower() in ["exit", "quit"]:
                print("Exiting chat. Goodbye!")
                break

            messages.append({"role": "user", "content": user_input})

            assistant_msg = self.call_openai(
                messages,
                functions=FUNCTIONS,
                function_call="auto"
            )

            if assistant_msg.content and assistant_msg.function_call is None:
                print("Assistant:", assistant_msg.content.strip())
                messages.append({"role": "assistant", "content": assistant_msg.content})
                continue

            if assistant_msg.function_call:
                func_name = assistant_msg.function_call.name
                raw_args = assistant_msg.function_call.arguments or "{}"

                try:
                    arguments = json.loads(raw_args)
                except json.JSONDecodeError:
                    print("Assistant attempted a function call but provided invalid JSON.")
                    print("Raw arguments:", raw_args)
                    messages.append({"role": "assistant", "content": "<malformed function_call>"})
                    continue

                func_result = self.execute_function_call(func_name, arguments)

                messages.append({
                    "role": "assistant",
                    "content": None,
                    "function_call": {
                        "name": func_name,
                        "arguments": raw_args
                    }
                })

                func_result_msg = {
                    "role": "function",
                    "name": func_name,
                    "content": json.dumps(func_result)
                }
                messages.append(func_result_msg)

                final_ack = self.call_openai(messages)
                if final_ack.content:
                    print("Assistant:", final_ack.content.strip())
                    messages.append({"role": "assistant", "content": final_ack.content})

                continue

            print("Assistant (unexpected response):", assistant_msg)
            messages.append({"role": "assistant", "content": "<unexpected response>"})

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CsvChatManagerNode()
    
    try:
        node.destroy_node()
    except Exception:
        pass

if __name__ == "__main__":
    main()
