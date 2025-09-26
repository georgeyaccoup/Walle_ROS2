# voice_listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import sounddevice as sd
import numpy as np
import queue
import tempfile
import scipy.io.wavfile as wav

OPENAI_API_KEY = "<YOUR_API_KEY>"

class VoiceListener(Node):
    def __init__(self):
        super().__init__('voice_listener')
        self.pub = self.create_publisher(String, '/voice_commands', 10)
        self.get_logger().info("Voice Listener Started")
        self.q = queue.Queue()

        self.stream = sd.InputStream(callback=self.audio_callback, channels=1, samplerate=16000)
        self.stream.start()

    def audio_callback(self, indata, frames, time, status):
        self.q.put(indata.copy())

    def run(self):
        while rclpy.ok():
            if not self.q.empty():
                audio_data = self.q.get()
                # Save temporary WAV
                tmpfile = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
                wav.write(tmpfile.name, 16000, audio_data)
                # Send to OpenAI
                with open(tmpfile.name, "rb") as f:
                    transcript = openai.audio.transcriptions.create(file=f, model="whisper-1")
                text = transcript['text']
                msg = String()
                msg.data = text
                self.pub.publish(msg)
                self.get_logger().info(f"Published: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceListener()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
