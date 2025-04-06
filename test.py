import time
import speech_recognition as sr
import torch
import threading
import cv2
import sounddevice as sd
from scipy.io.wavfile import write
import numpy as np
import regex as re
import keyboard as keys
import librosa

from transformers.utils import logging
logging.set_verbosity(0) 
from transformers import pipeline, AutoModelForCausalLM, AutoProcessor


'''
use device_index=0 (Microsoft Sound Mapper Input Microphone)
use device_index=1 (AMD Audio Dev Input)
'''

# some globals
mutex_lock = threading.Lock()
collected_frames = []


def voice_chat(recognizer, microphone):
    with microphone as source:
        print("Speak Fool....")
        audio = recognizer.listen(source)
        time.sleep(5)

    print("Recognizing...")
    time.sleep(1)
    text = recognizer.recognize_google(audio)
    return text

def record_audio(duration=5, sample_rate=16000, save_as_wav=False):
    print("Recording in 2 seconds... Speak after the beep.")
    time.sleep(1)
    print("Beep!")

    audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='float32')
    sd.wait()
    if audio is None or len(audio) == 0:
        raise RuntimeError("No audio was recorded.")
    
    if save_as_wav:
        write("my_audio.wav", sample_rate, audio)
    
    return np.squeeze(audio)

def save_frames_to_video(frames, output_path="temp_video.mp4", fps=24):
    height, width, _ = frames[0].shape
    out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
    for frame in frames:
        out.write(frame)
    out.release()
    return output_path

def start_video_feed():
    vid_cap = cv2.VideoCapture(0)

    while vid_cap.isOpened():
        ret, frame = vid_cap.read()
        frame = cv2.flip(frame, 1)
        with mutex_lock:
            collected_frames.append(frame)

        if not ret:
            break

        cv2.imshow("cam_feed", frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            
            break

    cv2.destroyAllWindows()
    vid_cap.release()

# recognizer = sr.Recognizer()
# mic = sr.Microphone(device_index=1)

# print("press s to enable voice chat")
# flag = input()
# if flag=='s':
#     print("Opening Voice Chat")
#     time.sleep(1)
#     audio = record_audio(save_as_wav=True)
#     print(audio)
# print(text)
# matches = re.findall('(?i)(?:x|y)\s*(?:equals|equals to|=)\s*(\d+)', text)
# print(matches)
# print(audio)

model_name = "DAMO-NLP-SG/VideoLLaMA3-2B"

model = AutoModelForCausalLM.from_pretrained(
    model_name,
    trust_remote_code=True,
    device_map="auto",
    torch_dtype=torch.bfloat16, # to use half the required RAM => 4*2 = 8GB
    attn_implementation="eager",
)
processor = AutoProcessor.from_pretrained(model_name, trust_remote_code=True)

video_path = 'temp_video.mp4'
question = "Describe this video in detail."

conversation = [
    {"role": "system", "content": "You are a bitch ass nigga"},
    {
        "role": "user",
        "content": [
            {"type": "video", "video": {"video_path": video_path, "fps": 1, "max_frames": 128}},
            {"type": "text", "text": question},
        ]
    },
]


inputs = processor(conversation=conversation, return_tensors="pt")
# adding inputs to gpu/cuda
inputs = {k: v.cuda() if isinstance(v, torch.Tensor) else v for k, v in inputs.items()}

if 'pixel_values' in inputs.keys():
    inputs["pixel_values"] = inputs["pixel_values"].to(torch.bfloat16) # for faster inference

output_ids = model.generate(**inputs, max_new_tokens=128)
response = processor.batch_decode(output_ids, skip_special_tokens=True)
print(response)


