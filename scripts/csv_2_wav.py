from aip import AipSpeech
import os
import csv
import wave 
import io 
from pydub import AudioSegment

def mp3_to_wav(data, wav_path):
    aud = io.BytesIO(data)
    sound = AudioSegment.from_file(aud, format="mp3")
    raw_data = sound._data 

    nframes = len(raw_data)
    f = wave.open(wav_path, 'wb')
    f.setnchannels(1)
    f.setsampwidth(2)
    f.setframerate(16000)
    f.setnframes(nframes)
    f.writeframes(raw_data)
    f.close()

APP_ID = '24846657'
API_KEY = 'pgnBOTAtFMCkxXHGseNhBcbc'
SECRET_KEY = 'Ya9UGYdGCm1eRi8wGhGgGiodNACGh9gE'

client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

csv_file = "../data/wav_translation.csv"
wav_dst_folder = "../data/wav/"

try:
    os.makedirs(wav_dst_folder)
except:
    pass

with open(csv_file, 'r') as f:
    reader = csv.reader(f, delimiter=",")
    for row in reader:
        wav_file_name = row[0]
        translation = row[1]
        # per 0-4, 0为女声，1为男声，3为情感合成-度逍遥，4为情感合成-度丫丫
        result  = client.synthesis(translation, 'zh', 1, {
            'vol': 5, 'per': 4, "spd": 4,
        })

        # 识别正确返回语音二进制 错误则返回dict 参照下面错误码
        if not isinstance(result, dict):
            mp3_to_wav(result, os.path.join(wav_dst_folder, wav_file_name))
            print("synthetizing ", wav_file_name)
        else:
            print("unable to synthetize ", wav_file_name)
            print(result)