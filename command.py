import openai

GPT_API_MODEL = "text-davinci-003"  # Change this to your preferred GPT model

def generate_command(prompt):
    completions = openai.Completion.create(
        engine=GPT_API_MODEL,
        prompt=prompt,
        max_tokens=1024,
        n=1,
        stop=None,
        temperature=0.5,
    )
    return completions.choices[0].text.strip()