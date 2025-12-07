---
id: chapter-5-lesson-1
title: "Chapter 5 – Lesson 1: Whisper and Speech-to-Text Pipeline"
---

# Chapter 5 – Lesson 1: Whisper and Speech-to-Text Pipeline

## Introduction to Speech-to-Text in Physical AI

Speech-to-text technology is a critical component of natural human-robot interaction in Physical AI systems. For humanoid robots, the ability to understand spoken commands enables intuitive and natural communication with humans. This lesson explores the Whisper model and how to implement a complete speech-to-text pipeline for humanoid robotics applications.

## Understanding the Whisper Model

### Overview of Whisper
Whisper is an automatic speech recognition (ASR) system developed by OpenAI that is highly capable of transcribing human speech into text, even in challenging conditions such as:
- Noisy environments
- Various accents and speaking styles
- Different languages (multilingual support)
- Overlapping speech

### Whisper Architecture
Whisper is built on the Transformer architecture and includes:
- **Encoder**: Processes audio input
- **Decoder**: Generates text output
- **Multilingual capability**: Can handle multiple languages
- **Robust training**: Trained on diverse, noisy datasets

### Whisper Models
Different Whisper models offer trade-offs between accuracy and speed:
- **tiny**: Fastest, least accurate
- **base**: Good balance
- **small**: Better accuracy
- **medium**: High accuracy
- **large**: Highest accuracy, slowest

## Implementing Whisper in ROS 2

### Installation and Setup
```bash
# Install Whisper for Python
pip install openai-whisper

# Alternative: Install with specific PyTorch version for GPU support
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install openai-whisper
```

### Basic Whisper Node Implementation
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import whisper
import numpy as np
import torch
import io
import wave

class WhisperSTTNode(Node):
    def __init__(self):
        super().__init__('whisper_stt_node')

        # Load Whisper model
        self.model_size = self.declare_parameter('model_size', 'base').value
        self.model = whisper.load_model(self.model_size)

        # Set device (GPU if available)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = self.model.to(self.device)

        # Publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )

        self.transcription_pub = self.create_publisher(
            String,
            '/speech_transcription',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/stt_status',
            10
        )

        # Configuration parameters
        self.language = self.declare_parameter('language', 'en').value
        self.enable_vad = self.declare_parameter('enable_vad', True).value

        self.get_logger().info(f'Whisper STT node initialized with {self.model_size} model')

    def audio_callback(self, msg):
        """Process incoming audio data"""
        try:
            # Convert audio data to numpy array
            audio_array = self.convert_audio_msg_to_array(msg)

            # Transcribe audio
            result = self.transcribe_audio(audio_array)

            if result and result['text'].strip():
                # Publish transcription
                transcription_msg = String()
                transcription_msg.data = result['text'].strip()
                self.transcription_pub.publish(transcription_msg)

                self.get_logger().info(f'Transcribed: {result["text"]}')

                # Publish confidence score if available
                if 'avg_logprob' in result:
                    confidence_msg = String()
                    confidence_msg.data = f"confidence:{result['avg_logprob']:.3f}"
                    self.status_pub.publish(confidence_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def convert_audio_msg_to_array(self, audio_msg):
        """Convert ROS AudioData message to numpy array"""
        # AudioData contains raw audio bytes
        # Assuming 16-bit signed integers (common format)
        audio_bytes = audio_msg.data
        audio_array = np.frombuffer(audio_bytes, dtype=np.int16)

        # Normalize to [-1, 1] range
        audio_array = audio_array.astype(np.float32) / 32768.0

        return audio_array

    def transcribe_audio(self, audio_array):
        """Transcribe audio using Whisper"""
        # Convert to tensor and move to device
        audio_tensor = torch.from_numpy(audio_array).to(self.device)

        # Transcribe with specific options
        options = {
            'language': self.language,
            'task': 'transcribe',
            'beam_size': 5,
            'best_of': 5,
            'temperature': (0.0, 0.2, 0.4, 0.6, 0.8, 1.0),
        }

        result = self.model.transcribe(
            audio_tensor,
            **options
        )

        return result
```

## Advanced Whisper Configuration

### Real-time Processing Pipeline
For humanoid robots, real-time processing is crucial:

```python
import threading
import queue
from collections import deque
import time

class RealTimeWhisperNode(Node):
    def __init__(self):
        super().__init__('realtime_whisper_node')

        # Load model
        self.model = whisper.load_model('small').to(self.device)

        # Audio buffer for continuous processing
        self.audio_buffer = deque(maxlen=16000 * 10)  # 10 seconds at 16kHz
        self.buffer_lock = threading.Lock()

        # Processing queue
        self.process_queue = queue.Queue()
        self.result_queue = queue.Queue()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_audio_thread)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Publishers
        self.transcription_pub = self.create_publisher(String, '/speech_transcription', 10)
        self.interim_pub = self.create_publisher(String, '/interim_transcription', 10)

        # Timer for periodic processing
        self.process_timer = self.create_timer(2.0, self.process_buffer)  # Process every 2 seconds

    def audio_callback(self, msg):
        """Add audio data to buffer"""
        audio_array = self.convert_audio_msg_to_array(msg)

        with self.buffer_lock:
            for sample in audio_array:
                self.audio_buffer.append(sample)

    def process_buffer(self):
        """Process buffered audio for transcription"""
        with self.buffer_lock:
            if len(self.audio_buffer) > 16000:  # At least 1 second of audio
                # Extract audio chunk (last 5 seconds)
                audio_chunk = list(self.audio_buffer)[-16000*5:]  # Last 5 seconds
                audio_array = np.array(audio_chunk, dtype=np.float32)

                # Add to processing queue
                self.process_queue.put(audio_array)

    def process_audio_thread(self):
        """Background thread for audio processing"""
        while rclpy.ok():
            try:
                # Get audio from queue
                audio_array = self.process_queue.get(timeout=1.0)

                # Transcribe audio
                result = self.model.transcribe(
                    torch.from_numpy(audio_array).to(self.device),
                    language=self.language,
                    task='transcribe',
                    beam_size=3
                )

                if result and result['text'].strip():
                    # Publish result
                    transcription_msg = String()
                    transcription_msg.data = result['text'].strip()
                    self.transcription_pub.publish(transcription_msg)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Processing error: {e}')
```

## Voice Activity Detection (VAD)

### Integrating VAD with Whisper
Voice Activity Detection helps identify when speech is occurring:

```python
import webrtcvad
import collections

class WhisperWithVADNode(Node):
    def __init__(self):
        super().__init__('whisper_vad_node')

        # Initialize Whisper
        self.model = whisper.load_model('base').to(self.device)

        # Initialize WebRTC VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Aggressiveness mode (0-3)

        # Audio parameters
        self.sample_rate = 16000
        self.frame_duration = 30  # ms
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000)

        # Speech detection state
        self.is_speaking = False
        self.speech_buffer = []
        self.silence_threshold = 20  # frames of silence to stop
        self.silence_counter = 0

        # Publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )

        self.transcription_pub = self.create_publisher(
            String,
            '/speech_transcription',
            10
        )

    def audio_callback(self, msg):
        """Process audio with VAD"""
        # Convert audio to 16kHz, 16-bit format
        audio_array = self.convert_audio_msg_to_array(msg)
        audio_int16 = (audio_array * 32767).astype(np.int16)

        # Process in frames for VAD
        for i in range(0, len(audio_int16), self.frame_size):
            frame = audio_int16[i:i+self.frame_size]

            # Pad frame if necessary
            if len(frame) < self.frame_size:
                frame = np.pad(frame, (0, self.frame_size - len(frame)))

            # Check for voice activity
            voice_active = self.vad.is_speech(
                frame.tobytes(),
                self.sample_rate
            )

            if voice_active:
                # Add to speech buffer
                self.speech_buffer.extend(frame)
                self.silence_counter = 0
                self.is_speaking = True
            else:
                if self.is_speaking:
                    self.silence_counter += 1

                    # If enough silence, process the speech
                    if self.silence_counter >= self.silence_threshold:
                        self.process_speech_buffer()
                        self.is_speaking = False
                        self.silence_counter = 0

    def process_speech_buffer(self):
        """Process collected speech buffer"""
        if len(self.speech_buffer) > 0:
            # Convert back to float32
            speech_audio = np.array(self.speech_buffer, dtype=np.float32) / 32768.0

            # Transcribe the speech
            result = self.model.transcribe(
                torch.from_numpy(speech_audio).to(self.device),
                language=self.language,
                task='transcribe'
            )

            if result and result['text'].strip():
                transcription_msg = String()
                transcription_msg.data = result['text'].strip()
                self.transcription_pub.publish(transcription_msg)

                self.get_logger().info(f'Speech detected: {result["text"]}')

            # Clear buffer
            self.speech_buffer = []
```

## Audio Input Configuration

### Setting up Audio Input for Humanoid Robots
```python
import pyaudio
import wave

class AudioInputNode(Node):
    def __init__(self):
        super().__init__('audio_input_node')

        # Audio configuration
        self.rate = self.declare_parameter('sample_rate', 16000).value
        self.chunk = self.declare_parameter('chunk_size', 1024).value
        self.channels = self.declare_parameter('channels', 1).value

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Audio publisher
        self.audio_pub = self.create_publisher(AudioData, '/audio_input', 10)

        # Start audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk,
            stream_callback=self.audio_callback
        )

        self.get_logger().info('Audio input node initialized')

    def audio_callback(self, in_data, frame_count, time_info, status):
        """Audio stream callback"""
        # Convert audio data to ROS message
        audio_msg = AudioData()
        audio_msg.data = in_data

        # Publish audio data
        self.audio_pub.publish(audio_msg)

        return (None, pyaudio.paContinue)

    def destroy_node(self):
        """Clean up audio resources"""
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()
```

## Quality and Performance Optimization

### Whisper Configuration for Robotics
```python
class OptimizedWhisperNode(Node):
    def __init__(self):
        super().__init__('optimized_whisper_node')

        # Model selection based on hardware
        if torch.cuda.is_available() and torch.cuda.get_device_name().startswith('NVIDIA'):
            self.model_size = 'small'  # Use larger model on GPU
        else:
            self.model_size = 'base'   # Use smaller model on CPU

        self.model = whisper.load_model(self.model_size)

        # Optimization parameters
        self.temperature = 0.0  # For deterministic output
        self.compression_ratio_threshold = 2.4
        self.logprob_threshold = -1.0
        self.no_speech_threshold = 0.6

        # Publishers
        self.transcription_pub = self.create_publisher(String, '/speech_transcription', 10)
        self.confidence_pub = self.create_publisher(String, '/transcription_confidence', 10)

    def transcribe_with_options(self, audio_tensor):
        """Transcribe with quality options"""
        options = whisper.DecodingOptions(
            language=self.language,
            task='transcribe',
            temperature=self.temperature,
            compression_ratio_threshold=self.compression_ratio_threshold,
            logprob_threshold=self.logprob_threshold,
            no_speech_threshold=self.no_speech_threshold,
            beam_size=5
        )

        result = whisper.decode(self.model, audio_tensor, options)
        return result

    def validate_transcription(self, text, confidence):
        """Validate transcription quality"""
        # Check for common issues
        if len(text.strip()) < 2:
            return False, "Too short"

        if confidence < -0.5:
            return False, "Low confidence"

        # Check for common error patterns
        error_patterns = ["you know", "um", "uh", "like"]
        if any(pattern in text.lower() for pattern in error_patterns):
            # These might be filler words or transcription errors
            pass

        return True, "Valid"
```

## Integration with Robot Control

### Speech Command Processing
```python
class SpeechCommandProcessor(Node):
    def __init__(self):
        super().__init__('speech_command_processor')

        # Subscribe to transcriptions
        self.transcription_sub = self.create_subscription(
            String,
            '/speech_transcription',
            self.transcription_callback,
            10
        )

        # Publisher for robot commands
        self.command_pub = self.create_publisher(String, '/robot_command', 10)

        # Command mapping
        self.command_mapping = {
            'move forward': 'move_forward',
            'move backward': 'move_backward',
            'turn left': 'turn_left',
            'turn right': 'turn_right',
            'stop': 'stop',
            'wave': 'wave',
            'dance': 'dance',
            'hello': 'greet',
            'help': 'request_help'
        }

    def transcription_callback(self, msg):
        """Process speech transcription for robot commands"""
        text = msg.data.lower().strip()

        # Match against known commands
        matched_command = None
        for speech_phrase, robot_command in self.command_mapping.items():
            if speech_phrase in text:
                matched_command = robot_command
                break

        if matched_command:
            # Publish robot command
            command_msg = String()
            command_msg.data = matched_command
            self.command_pub.publish(command_msg)

            self.get_logger().info(f'Command recognized: {matched_command}')
        else:
            self.get_logger().info(f'Unrecognized speech: {text}')
```

## Error Handling and Robustness

### Handling Common Issues
```python
class RobustWhisperNode(Node):
    def __init__(self):
        super().__init__('robust_whisper_node')

        # Initialize with error handling
        try:
            self.model = whisper.load_model('base')
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            self.model = None

        # Audio processing with error handling
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.safe_audio_callback,
            10
        )

    def safe_audio_callback(self, msg):
        """Audio callback with comprehensive error handling"""
        try:
            if self.model is None:
                return

            audio_array = self.convert_audio_msg_to_array(msg)

            # Validate audio data
            if len(audio_array) == 0:
                return

            # Check for audio level (avoid processing silence)
            audio_level = np.mean(np.abs(audio_array))
            if audio_level < 0.001:  # Very quiet, probably just noise
                return

            result = self.model.transcribe(
                torch.from_numpy(audio_array).to(self.device),
                language=self.language
            )

            if result and result['text'].strip():
                transcription_msg = String()
                transcription_msg.data = result['text'].strip()
                self.transcription_pub.publish(transcription_msg)

        except torch.cuda.OutOfMemoryError:
            self.get_logger().error('GPU out of memory, reducing model size')
            # Handle OOM error by switching to CPU or smaller model
            self.model = whisper.load_model('tiny').cpu()
        except Exception as e:
            self.get_logger().error(f'Error in audio processing: {e}')
```

## Summary

Whisper provides a powerful speech-to-text capability for humanoid robots, enabling natural human-robot interaction. The implementation requires careful consideration of real-time processing, voice activity detection, and integration with robot control systems. Proper configuration and optimization ensure reliable performance in robotic applications. The next lesson will explore how to use LLMs for planning based on speech input.