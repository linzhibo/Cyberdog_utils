#!/usr/bin/env python3

from interaction_msgs.srv import CameraService

import rclpy
from rclpy.node import Node 

from aip import AipSpeech
from pydub import AudioSegment
import wave 
import io 

# if quota exceeded, use your own baidu account
APP_ID = '26004398'
API_KEY = 'xlpwaTsK42K5v4qQO8GZuDZR'
SECRET_KEY = 'oqZuIchTzf9BHrOT98S55wbRCMPlmFDc'
client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)
wav_path = "/opt/ros2/cyberdog/data/wav/777.wav"

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

class TextToSoundService(Node):

    def __init__(self):
        super().__init__("ttss_service")
        self.srv = self.create_service(CameraService, 'text_to_sound', self.text_to_sound_callback)
        self.last_msg = ""

    def text_to_sound_callback(self, request, response):
        self.get_logger().info("Incoming request\n %s" %(request.args))
        if(self.last_msg == request.args):
            response.result = 1
            self.get_logger().info("Same as previous msg:\n %s" %(self.last_msg))
            return response
        else:
            self.last_msg = request.args
        if (self.text_to_wav(request.args)):
            response.result = 1
        else:
            response.result = 0
        return response

    def text_to_wav(self, text):
        result  = client.synthesis(text, 'zh', 1, 
        {'vol': 5, 'per': 4, "spd": 4,})
        
        if not isinstance(result, dict):
            mp3_to_wav(result, wav_path)
            print("synthetizing ", wav_path)
            return True
        else:
            print("unable to synthetize ", wav_path)
            print(result)
            return False

def main(args=None):
    rclpy.init(args=args)
    ttss_service = TextToSoundService()
    rclpy.spin(ttss_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()