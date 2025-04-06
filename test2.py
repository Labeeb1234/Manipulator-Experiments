import time
import speech_recognition as sr
import torch
import threading
import cv2
from PIL import Image
import sounddevice as sd
from scipy.io.wavfile import write
import numpy as np
import regex as re
import keyboard as keys

from transformers.utils import logging
logging.set_verbosity(0) 
from transformers import pipeline, AutoModelForZeroShotObjectDetection, AutoProcessor


'''
use device_index=0 (Microsoft Sound Mapper Input Microphone)
use device_index=1 (AMD Audio Dev Input)
'''

# some globals
mutex_lock = threading.Lock()
boxes = []


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

        if not ret:
            break

        with mutex_lock:
                cv2.imwrite('my_image.jpg', frame)
                # Draw boxes from object detection
                for box in boxes:
                    frame = cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 3)
        
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

def object_detection():
    model_name = "IDEA-Research/grounding-dino-base"
    device = "cuda" if torch.cuda.is_available() else "cpu"

    processor = AutoProcessor.from_pretrained(model_name)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(model_name).to(device)

    while True:
        time.sleep(2)
        with mutex_lock:
            image_path = 'my_image.jpg'
            try:
                image = Image.open(image_path)
            except:
                continue  # if image is not ready or corrupted

        text = 'a bottle.' # VERY important: text queries need to be lowercased + end with a dot
        inputs = processor(images=image, text=text, return_tensors="pt").to(device)

        with torch.no_grad():
            outputs = model(**inputs)

        results = processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold=0.4,
            text_threshold=0.3,
            target_sizes=[image.size[::-1]]
        )[0]

        print(results)
        new_boxes = []
        for box, label, score in zip(results["boxes"], results["labels"], results["scores"]):
            box = box.to("cpu").numpy().astype(int) # pixels topLeft and bottomRight)
            new_boxes.append(box)
        
        with mutex_lock:
            boxes.clear()
            boxes.extend(new_boxes)


cam_thread = threading.Thread(target=start_video_feed)
detect_thread = threading.Thread(target=object_detection)

cam_thread.start()
detect_thread.start()

cam_thread.join()
detect_thread.join()

