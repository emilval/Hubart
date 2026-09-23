import queue
import collections
import numpy as np
import sounddevice as sd
import webrtcvad
from faster_whisper import WhisperModel

class AdaptiveAudioListener:
    def __init__(self):
        self.sample_rate = 16000
        self.frame_duration_ms = 30  # WebRTC VAD strictly requires 10, 20, or 30ms frames
        self.frame_size = int(self.sample_rate * self.frame_duration_ms / 1000)
        
        # Queue for raw audio bytes
        self.audio_queue = queue.Queue()
        
        self.model = WhisperModel("base", device="cpu", compute_type="int8")
        self.vad = webrtcvad.Vad(3)  # Aggressiveness mode 0-3 (3 aggressively filters out non-speech noise)

    def audio_callback(self, indata, frames, time, status):
        """Runs in the background, pushing raw audio bytes to the queue."""
        if status:
            print(status, flush=True)
        self.audio_queue.put(bytes(indata))

    def listen_for_audio(self, silence_timeout=1.0, pre_speech_padding=0.3):
        # Clear any old data from the queue
        while not self.audio_queue.empty():
            self.audio_queue.get_nowait()

        # Convert times into number of 30ms frames
        silence_frames_threshold = int(silence_timeout * 1000 / self.frame_duration_ms)
        padding_frames_threshold = int(pre_speech_padding * 1000 / self.frame_duration_ms)
        
        # A rolling buffer keeps a tiny bit of pre-speech audio so the first syllable isn't chopped off
        ring_buffer = collections.deque(maxlen=padding_frames_threshold)
        
        is_speaking = False
        silence_counter = 0
        audio_buffer = []

        # Use RawInputStream to get int16 bytes directly (required by WebRTC VAD)
        with sd.RawInputStream(samplerate=self.sample_rate, channels=1, dtype='int16', 
                               blocksize=self.frame_size, callback=self.audio_callback):
            print("Listening for speech...")
            try:
                while True:
                    frame = self.audio_queue.get()
                    is_speech = self.vad.is_speech(frame, self.sample_rate)

                    if not is_speaking:
                        # Keep a rolling history of the ambient audio
                        ring_buffer.append(frame)
                        
                        if is_speech:
                            is_speaking = True
                            # Move the pre-speech buffer into the main recording buffer
                            audio_buffer.extend(ring_buffer)
                            ring_buffer.clear()
                            print("Speech detected. Recording...", end="", flush=True)
                            
                    else:
                        audio_buffer.append(frame)
                        
                        if is_speech:
                            silence_counter = 0  # Reset silence counter if they keep talking
                        else:
                            silence_counter += 1

                        # If silence duration exceeds the threshold, the sentence is done
                        if silence_counter >= silence_frames_threshold:
                            print("\nSilence detected. Transcribing...")
                            
                            # Convert 16-bit PCM byte chunks to the normalized float32 format Whisper expects
                            raw_audio = b''.join(audio_buffer)
                            audio_np = np.frombuffer(raw_audio, dtype=np.int16).astype(np.float32) / 32768.0
                            
                            segments, _ = self.model.transcribe(audio_np, beam_size=5, language="en")
                            text = " ".join([segment.text.strip() for segment in segments])
                            
                            # Reset the tracking variables for the next sentence
                            is_speaking = False
                            silence_counter = 0
                            audio_buffer = []
                            
                            if text.strip():
                                return text.strip()
                            else:
                                print("No text recognized. Listening again...")

            except KeyboardInterrupt:
                print("\nMicrophone stream stopped.")

if __name__ == '__main__':
    listener = AdaptiveAudioListener()
    # It will wait for speech, and process the audio once 1.0 second of silence is detected
    result = listener.listen_for_audio(silence_timeout=1.0)
    print(f"Final Transcription: {result}")