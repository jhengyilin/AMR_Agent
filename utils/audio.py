import numpy as np
# install ffmpeg for pydub
from pydub import AudioSegment
import tempfile
import sounddevice as sd
import soundfile as sf
from openai import OpenAI
from gtts import gTTS
from io import BytesIO
import os 
from dotenv import load_dotenv
import time
from pygame import mixer
import pygame
import elevenlabs

mixer.init()

load_dotenv()

elevenlabs.set_api_key(os.getenv("ELEVENLABS_API_KEY"))

def read_audio_from_file_obj(file_obj):
    audio_bytes = BytesIO(file_obj.read())
    audio_segment = AudioSegment.from_file(audio_bytes, format="mp3")
    audio_array = np.array(audio_segment.get_array_of_samples())
    return audio_array

# duration: recording duration in seconds
# sample_rate: sample rate in Hz
def record_audio(duration=5, sample_rate=16000):
    print("Recording audio...")
    recording = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='int16')
    sd.wait()
    return np.squeeze(recording)

def voice_to_text(audio_data):
    # Save audio data to a temporary file
    audio_segment = AudioSegment(
        data=audio_data.tobytes(),
        sample_width=2,
        frame_rate=16000,
        channels=1,
    )

    client = OpenAI()

    with tempfile.NamedTemporaryFile(suffix=".mp3") as temp_audio_file:
        audio_segment.export(temp_audio_file.name, format="mp3")
        
        with open(temp_audio_file.name, 'rb') as file_obj:
            # Transcribe audio using OpenAI's Whisper API
            response = client.audio.transcriptions.create(
                model="whisper-1", 
                file=file_obj,
            )

    return response.text

# def text_to_speech(text):
    
#     tts = gTTS(text=text, lang='en', tld = 'us', slow=False)

#     with tempfile.NamedTemporaryFile(suffix=".mp3") as temp_audio_file:
#         tts.write_to_fp(temp_audio_file)
#         temp_audio_file.seek(0)

#         # Load audio data from file and convert it to a numpy array
#         audio_data = read_audio_from_file_obj(temp_audio_file)

#     play_audio(audio_data)


def play_audio(audio_data, sample_rate=16000):
    print("Playing audio...")
    sd.play(audio_data, sample_rate)
    sd.wait()

def text_to_speech(text):
    """
    Converts text to speech using an external API and plays it.
    """
    current_time = time.time()
    print("Generating speech...")
    
    audio = elevenlabs.generate(
        text=text,  # Directly use the text passed to the function
        voice="UIpYR5ztFVFz4W8qqDfi",  # Example: Joanne's voice
        model="eleven_multilingual_v2"
    )
    elevenlabs.save(audio, "./audio.wav")
    
    audio_time = time.time() - current_time
    print(f"Finished generating audio in {audio_time:.2f} seconds.")
    print("Playing audio response...")
    sound = mixer.Sound(str("./audio.wav"))
    sound.play()
    pygame.time.wait(int(sound.get_length() * 1000))
    print("Audio playback completed.")