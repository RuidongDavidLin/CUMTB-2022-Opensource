import wave
from pyaudio import PyAudio
def playsound(txt, format = '.wav'):
    '''
    eg. playsound('voice/餐厅')，
    输入所需播放音频的地址与文件名，格式默认为.wav
    '''
    wf = wave.open(txt + '.wav', 'rb')
    p = PyAudio()
    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()), channels=wf.getnchannels(),rate=wf.getframerate(), output=True)
    data = wf.readframes(512)
    while data != b'':
        stream.write(data)
        data = wf.readframes(512)




