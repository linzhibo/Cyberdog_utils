from aip import AipSpeech
import os
import csv

APP_ID = '24845553'
API_KEY = 'EGTNEeQt0WLoyI6FmjatHTGn'
SECRET_KEY = 'lp9pUsjKZOEEguBDEd9NYITMSPd7G1Uz'

client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

# 读取文件
def get_file_content(filePath):
    with open(filePath, 'rb') as fp:
        return fp.read()

def main():
    wav_folder = "~/Documents/mystuff/wav_to_modify/"

    data = []
    for audio_file in sorted(os.listdir(wav_folder)):
        audio_file_path = os.path.join(wav_folder, audio_file)
        
        try:
            # res = client.asr(get_file_content(audio_file_path), 'wav', 16000, {
            #     'dev_pid': 1537,
            # })
            data.append([audio_file, res['result'][0]])
        except:
            continue

    with open("../data/wav_translation.csv", 'w', encoding='UTF8') as f:
        writer = csv.writer(f)
        writer.writerows(data)

def test():
    charging_statement = "正在充电.电量百分之"
    not_charging_statement = "电量百分之"
    data = []
    for i in range(201,300):
        data.append([str(i)+".wav", charging_statement+str(i-200)])
    
    for i in range(301,400):
        data.append([str(i)+".wav", not_charging_statement+str(i-300)])


    with open("../data/wav_translation_2.csv", 'w', encoding='UTF8') as f:
        writer = csv.writer(f)
        writer.writerows(data)




test()
# print(res['result'])