from openai import OpenAI
import os 
from dotenv import load_dotenv

load_dotenv()

GPT_API_MODEL = "gpt-4o"  # Change this to your preferred GPT model

os.getenv("OPENAI_API_KEY")

gpt_client = OpenAI()

def generate_command(prompt, user_input):

    # #print messages
    # print("role: system", "content:", prompt)
    # print("role: user", "content:", user_input)

    completions = gpt_client.chat.completions.create(
        model=GPT_API_MODEL,
        messages=[
            {"role": "system", "content": prompt},
            {"role": "user", "content": user_input},
        ],
        response_format={ "type": "json_object" },
        max_tokens=1024,
        n=1,
        stop=None,
        temperature=0.5,
    )
    
    return completions.choices[0].message.content