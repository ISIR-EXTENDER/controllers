#!/usr/bin/env python3

import torch  # Import PyTorch, required by Silero VAD and sentence-transformers tensors
import sounddevice as sd  # Import sounddevice to read audio from the microphone
import numpy as np  # Import NumPy to manipulate audio arrays
import json  # Import JSON to parse Vosk recognition results
import time  # Import time to store timestamps for voice confidence decay
import math  # Import math to compute exponential decay
import re  # Import regex to split recognized text into words

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from silero_vad import load_silero_vad  # Import Silero VAD loader
from vosk import Model, KaldiRecognizer  # Import Vosk model and recognizer
from sentence_transformers import SentenceTransformer, util  # Import sentence similarity model and cosine similarity utility


# =========================
# AUDIO / VAD CONFIG
# =========================

SAMPLE_RATE = 16000  # Use 16000 Hz audio sampling rate
CHUNK_SIZE = 512  # Read 512 samples per audio chunk, which is about 32 ms at 16 kHz
VAD_THRESHOLD = 0.5  # If Silero speech probability is above this value, we consider it speech
END_SILENCE_SECONDS = 0.8  # End the speech segment after 0.5 seconds of silence
MAX_SEGMENT_SECONDS = 4.0  # Maximum duration of one speech segment
MIN_SEGMENT_SECONDS = 0.25  # Ignore very short segments, usually noise


# =========================
# STT CONFIG
# =========================

VOSK_MODEL_PATH = "/home/woubraim/model-fr"  # Path to the French Vosk model


# =========================
# VOICE BOOST CONFIG
# =========================

TAU_VOICE = 4.0  # Time constant for exponential decay of voice confidence

last_voice_goal = None  # Store the last goal selected by voice
last_voice_confidence = 0.0  # Store the confidence of the last voice command
last_voice_time = 0.0  # Store the time when the last voice command was detected


# =========================
# LOAD MODELS
# =========================

vad_model = load_silero_vad()  # Load Silero VAD model for speech/silence detection

stt_model = Model(VOSK_MODEL_PATH)  # Load Vosk model for speech-to-text

similarity_model = SentenceTransformer("paraphrase-multilingual-MiniLM-L12-v2")  # Load multilingual sentence similarity model


# =========================
# EXAMPLE SENTENCES
# =========================

SINGLE_GOAL_EXAMPLES = [  # Examples where the user clearly mentions one goal
    ("le un", "single_goal"),  # Simple goal 1 phrase
    ("le deux", "single_goal"),  # Simple goal 2 phrase
    ("le trois", "single_goal"),  # Simple goal 3 phrase
    ("le quatre", "single_goal"),  # Simple goal 4 phrase
    ("le cinq", "single_goal"),  # Simple goal 5 phrase
    ("le six", "single_goal"),  # Simple goal 6 phrase

    ("tag un", "single_goal"),  # Goal 1 with tag keyword
    ("tag deux", "single_goal"),  # Goal 2 with tag keyword
    ("tag trois", "single_goal"),  # Goal 3 with tag keyword
    ("tag quatre", "single_goal"),  # Goal 4 with tag keyword
    ("tag cinq", "single_goal"),  # Goal 5 with tag keyword
    ("tag six", "single_goal"),  # Goal 6 with tag keyword

    ("april tag un", "single_goal"),  # Goal 1 with AprilTag keyword
    ("april tag deux", "single_goal"),  # Goal 2 with AprilTag keyword
    ("april tag trois", "single_goal"),  # Goal 3 with AprilTag keyword
    ("april tag quatre", "single_goal"),  # Goal 4 with AprilTag keyword
    ("april tag cinq", "single_goal"),  # Goal 5 with AprilTag keyword
    ("april tag six", "single_goal"),  # Goal 6 with AprilTag keyword

    ("va vers le un", "single_goal"),  # Navigation command to goal 1
    ("va vers le deux", "single_goal"),  # Navigation command to goal 2
    ("va vers le trois", "single_goal"),  # Navigation command to goal 3
    ("va vers le quatre", "single_goal"),  # Navigation command to goal 4
    ("va vers le cinq", "single_goal"),  # Navigation command to goal 5
    ("va vers le six", "single_goal"),  # Navigation command to goal 6

    ("je veux le un", "single_goal"),  # User wants goal 1
    ("je veux le deux", "single_goal"),  # User wants goal 2
    ("je veux le trois", "single_goal"),  # User wants goal 3
    ("je veux le quatre", "single_goal"),  # User wants goal 4
    ("je veux le cinq", "single_goal"),  # User wants goal 5
    ("je veux le six", "single_goal"),  # User wants goal 6

    ("prends le premier", "single_goal"),  # Ordinal form for goal 1
    ("prends le deuxième", "single_goal"),  # Ordinal form for goal 2
    ("prends le troisième", "single_goal"),  # Ordinal form for goal 3
    ("prends le quatrième", "single_goal"),  # Ordinal form for goal 4
    ("prends le cinquième", "single_goal"),  # Ordinal form for goal 5
    ("prends le sixième", "single_goal"),  # Ordinal form for goal 6

    ("objectif un", "single_goal"),  # Goal 1 with objective keyword
    ("objectif deux", "single_goal"),  # Goal 2 with objective keyword
    ("objectif trois", "single_goal"),  # Goal 3 with objective keyword
    ("objectif quatre", "single_goal"),  # Goal 4 with objective keyword
    ("objectif cinq", "single_goal"),  # Goal 5 with objective keyword
    ("objectif six", "single_goal"),  # Goal 6 with objective keyword

    ("numéro un", "single_goal"),  # Goal 1 with number keyword
    ("numéro deux", "single_goal"),  # Goal 2 with number keyword
    ("numéro trois", "single_goal"),  # Goal 3 with number keyword
    ("numéro quatre", "single_goal"),  # Goal 4 with number keyword
    ("numéro cinq", "single_goal"),  # Goal 5 with number keyword
    ("numéro six", "single_goal"),  # Goal 6 with number keyword
]

CORRECTION_SINGLE_GOAL_EXAMPLES = [  # Examples where the user corrects the previous command and gives one goal
    ("non le un", "correction_single_goal"),  # Correction to goal 1
    ("non le deux", "correction_single_goal"),  # Correction to goal 2
    ("non le trois", "correction_single_goal"),  # Correction to goal 3
    ("non le quatre", "correction_single_goal"),  # Correction to goal 4
    ("non le cinq", "correction_single_goal"),  # Correction to goal 5
    ("non le six", "correction_single_goal"),  # Correction to goal 6

    ("non plutôt le un", "correction_single_goal"),  # Correction to goal 1 with "rather"
    ("non plutôt le deux", "correction_single_goal"),  # Correction to goal 2 with "rather"
    ("non plutôt le trois", "correction_single_goal"),  # Correction to goal 3 with "rather"
    ("non plutôt le quatre", "correction_single_goal"),  # Correction to goal 4 with "rather"
    ("non plutôt le cinq", "correction_single_goal"),  # Correction to goal 5 with "rather"
    ("non plutôt le six", "correction_single_goal"),  # Correction to goal 6 with "rather"

    ("plutôt le un", "correction_single_goal"),  # Correction to goal 1
    ("plutôt le deux", "correction_single_goal"),  # Correction to goal 2
    ("plutôt le trois", "correction_single_goal"),  # Correction to goal 3
    ("plutôt le quatre", "correction_single_goal"),  # Correction to goal 4
    ("plutôt le cinq", "correction_single_goal"),  # Correction to goal 5
    ("plutôt le six", "correction_single_goal"),  # Correction to goal 6

    ("je corrige le un", "correction_single_goal"),  # Explicit correction to goal 1
    ("je corrige le deux", "correction_single_goal"),  # Explicit correction to goal 2
    ("je corrige le trois", "correction_single_goal"),  # Explicit correction to goal 3
    ("je corrige le quatre", "correction_single_goal"),  # Explicit correction to goal 4
    ("je corrige le cinq", "correction_single_goal"),  # Explicit correction to goal 5
    ("je corrige le six", "correction_single_goal"),  # Explicit correction to goal 6

    ("en fait le un", "correction_single_goal"),  # User changes to goal 1
    ("en fait le deux", "correction_single_goal"),  # User changes to goal 2
    ("en fait le trois", "correction_single_goal"),  # User changes to goal 3
    ("en fait le quatre", "correction_single_goal"),  # User changes to goal 4
    ("en fait le cinq", "correction_single_goal"),  # User changes to goal 5
    ("en fait le six", "correction_single_goal"),  # User changes to goal 6

    ("je voulais dire le un", "correction_single_goal"),  # User meant goal 1
    ("je voulais dire le deux", "correction_single_goal"),  # User meant goal 2
    ("je voulais dire le trois", "correction_single_goal"),  # User meant goal 3
    ("je voulais dire le quatre", "correction_single_goal"),  # User meant goal 4
    ("je voulais dire le cinq", "correction_single_goal"),  # User meant goal 5
    ("je voulais dire le six", "correction_single_goal"),  # User meant goal 6
]

MULTI_GOAL_AND_EXAMPLES = [  # Examples where multiple goals are mentioned without choosing one
    ("le un et le deux", "multi_goal_and"),  # Goals 1 and 2 together
    ("le deux et le trois", "multi_goal_and"),  # Goals 2 and 3 together
    ("le trois et le quatre", "multi_goal_and"),  # Goals 3 and 4 together
    ("le quatre et le cinq", "multi_goal_and"),  # Goals 4 and 5 together
    ("le cinq et le six", "multi_goal_and"),  # Goals 5 and 6 together

    ("le un avec le deux", "multi_goal_and"),  # Goals 1 and 2 together
    ("le deux avec le trois", "multi_goal_and"),  # Goals 2 and 3 together
    ("le trois avec le quatre", "multi_goal_and"),  # Goals 3 and 4 together
    ("le quatre avec le cinq", "multi_goal_and"),  # Goals 4 and 5 together
    ("le cinq avec le six", "multi_goal_and"),  # Goals 5 and 6 together

    ("le un puis le deux", "multi_goal_and"),  # Goal sequence without unique target
    ("le deux puis le trois", "multi_goal_and"),  # Goal sequence without unique target
    ("le trois puis le quatre", "multi_goal_and"),  # Goal sequence without unique target
    ("le quatre puis le cinq", "multi_goal_and"),  # Goal sequence without unique target
    ("le cinq puis le six", "multi_goal_and"),  # Goal sequence without unique target

    ("je veux le un et le deux", "multi_goal_and"),  # User asks for two goals
    ("je veux le deux et le trois", "multi_goal_and"),  # User asks for two goals
    ("je veux le trois et le quatre", "multi_goal_and"),  # User asks for two goals
    ("je veux le quatre et le cinq", "multi_goal_and"),  # User asks for two goals
    ("je veux le cinq et le six", "multi_goal_and"),  # User asks for two goals
]

CORRECTION_LAST_GOAL_EXAMPLES = [  # Examples where the last mentioned goal should be selected
    ("pas le un mais le deux", "correction_last"),  # Reject goal 1 and select goal 2
    ("pas le deux mais le trois", "correction_last"),  # Reject goal 2 and select goal 3
    ("pas le trois mais le quatre", "correction_last"),  # Reject goal 3 and select goal 4
    ("pas le quatre mais le cinq", "correction_last"),  # Reject goal 4 and select goal 5
    ("pas le cinq mais le six", "correction_last"),  # Reject goal 5 and select goal 6

    ("non pas le un le deux", "correction_last"),  # Reject goal 1 and select goal 2
    ("non pas le deux le trois", "correction_last"),  # Reject goal 2 and select goal 3
    ("non pas le trois le quatre", "correction_last"),  # Reject goal 3 and select goal 4
    ("non pas le quatre le cinq", "correction_last"),  # Reject goal 4 and select goal 5
    ("non pas le cinq le six", "correction_last"),  # Reject goal 5 and select goal 6

    ("le un non le deux", "correction_last"),  # Correct from goal 1 to goal 2
    ("le deux non le trois", "correction_last"),  # Correct from goal 2 to goal 3
    ("le trois non le quatre", "correction_last"),  # Correct from goal 3 to goal 4
    ("le quatre non le cinq", "correction_last"),  # Correct from goal 4 to goal 5
    ("le cinq non le six", "correction_last"),  # Correct from goal 5 to goal 6

    ("le un plutôt le deux", "correction_last"),  # Correct from goal 1 to goal 2
    ("le deux plutôt le trois", "correction_last"),  # Correct from goal 2 to goal 3
    ("le trois plutôt le quatre", "correction_last"),  # Correct from goal 3 to goal 4
    ("le quatre plutôt le cinq", "correction_last"),  # Correct from goal 4 to goal 5
    ("le cinq plutôt le six", "correction_last"),  # Correct from goal 5 to goal 6

    ("je me suis trompé le deux", "correction_last"),  # User says they made a mistake and selects goal 2
    ("je me suis trompé le trois", "correction_last"),  # User says they made a mistake and selects goal 3
    ("je me suis trompé le quatre", "correction_last"),  # User says they made a mistake and selects goal 4
    ("je me suis trompé le cinq", "correction_last"),  # User says they made a mistake and selects goal 5
    ("je me suis trompé le six", "correction_last"),  # User says they made a mistake and selects goal 6
]

CORRECTION_FIRST_GOAL_EXAMPLES = [  # Examples where the first mentioned goal should be selected
    ("plutôt le un que le deux", "correction_first"),  # Prefer goal 1 over goal 2
    ("plutôt le deux que le trois", "correction_first"),  # Prefer goal 2 over goal 3
    ("plutôt le trois que le quatre", "correction_first"),  # Prefer goal 3 over goal 4
    ("plutôt le quatre que le cinq", "correction_first"),  # Prefer goal 4 over goal 5
    ("plutôt le cinq que le six", "correction_first"),  # Prefer goal 5 over goal 6

    ("le un pas le deux", "correction_first"),  # Select goal 1 and reject goal 2
    ("le deux pas le trois", "correction_first"),  # Select goal 2 and reject goal 3
    ("le trois pas le quatre", "correction_first"),  # Select goal 3 and reject goal 4
    ("le quatre pas le cinq", "correction_first"),  # Select goal 4 and reject goal 5
    ("le cinq pas le six", "correction_first"),  # Select goal 5 and reject goal 6

    ("j'ai dit le un pas le deux", "correction_first"),  # User insists on goal 1, not goal 2
    ("j'ai dit le deux pas le trois", "correction_first"),  # User insists on goal 2, not goal 3
    ("j'ai dit le trois pas le quatre", "correction_first"),  # User insists on goal 3, not goal 4
    ("j'ai dit le quatre pas le cinq", "correction_first"),  # User insists on goal 4, not goal 5
    ("j'ai dit le cinq pas le six", "correction_first"),  # User insists on goal 5, not goal 6
]

AMBIGUOUS_MULTI_GOAL_EXAMPLES = [  # Examples where multiple goals are mentioned ambiguously
    ("le un ou le deux", "ambiguous"),  # Ambiguous between goal 1 and goal 2
    ("le deux ou le trois", "ambiguous"),  # Ambiguous between goal 2 and goal 3
    ("le trois ou le quatre", "ambiguous"),  # Ambiguous between goal 3 and goal 4
    ("le quatre ou le cinq", "ambiguous"),  # Ambiguous between goal 4 and goal 5
    ("le cinq ou le six", "ambiguous"),  # Ambiguous between goal 5 and goal 6

    ("soit le un soit le deux", "ambiguous"),  # Ambiguous either/or phrase
    ("soit le deux soit le trois", "ambiguous"),  # Ambiguous either/or phrase
    ("soit le trois soit le quatre", "ambiguous"),  # Ambiguous either/or phrase
    ("soit le quatre soit le cinq", "ambiguous"),  # Ambiguous either/or phrase
    ("soit le cinq soit le six", "ambiguous"),  # Ambiguous either/or phrase

    ("peut-être le un ou le deux", "ambiguous"),  # Uncertain phrase
    ("peut-être le deux ou le trois", "ambiguous"),  # Uncertain phrase
    ("peut-être le trois ou le quatre", "ambiguous"),  # Uncertain phrase
    ("peut-être le quatre ou le cinq", "ambiguous"),  # Uncertain phrase
    ("peut-être le cinq ou le six", "ambiguous"),  # Uncertain phrase

    ("entre le un et le deux", "ambiguous"),  # Ambiguous between two goals
    ("entre le deux et le trois", "ambiguous"),  # Ambiguous between two goals
    ("entre le trois et le quatre", "ambiguous"),  # Ambiguous between two goals
    ("entre le quatre et le cinq", "ambiguous"),  # Ambiguous between two goals
    ("entre le cinq et le six", "ambiguous"),  # Ambiguous between two goals
]

STOP_EXAMPLES = [  # Examples for stop commands
    ("stop", "stop"),  # Direct stop command
    ("arrête", "stop"),  # Stop command in French
    ("arrete", "stop"),  # Stop command without accent
    ("arrête toi", "stop"),  # Stop yourself
    ("arrête-toi", "stop"),  # Stop yourself with hyphen
    ("stoppe", "stop"),  # Alternative stop command
    ("pause", "stop"),  # Pause command
    ("attends", "stop"),  # Wait command
    ("ne bouge plus", "stop"),  # Do not move anymore
    ("immobile", "stop"),  # Stay still
    ("halte", "stop"),  # Halt
    ("reste là", "stop"),  # Stay there
    ("stop robot", "stop"),  # Stop robot
    ("arrête le robot", "stop"),  # Stop the robot
]

CANCEL_EXAMPLES = [  # Examples for cancel commands
    ("annule", "cancel"),  # Cancel command
    ("annule ça", "cancel"),  # Cancel that
    ("annule la commande", "cancel"),  # Cancel the command
    ("laisse tomber", "cancel"),  # Forget it
    ("oublie", "cancel"),  # Forget it
    ("oublie ça", "cancel"),  # Forget that
    ("ignore", "cancel"),  # Ignore
    ("ignore la commande", "cancel"),  # Ignore the command
    ("cancel", "cancel"),  # English cancel
    ("non laisse tomber", "cancel"),  # No, forget it
    ("c'est bon laisse tomber", "cancel"),  # It's okay, forget it
    ("ne fais rien", "cancel"),  # Do nothing
]

ALL_EXAMPLES = (  # Combine all examples for fallback similarity
    SINGLE_GOAL_EXAMPLES  # Add single goal examples
    + CORRECTION_SINGLE_GOAL_EXAMPLES  # Add single-goal correction examples
    + MULTI_GOAL_AND_EXAMPLES  # Add multiple-goal examples
    + CORRECTION_LAST_GOAL_EXAMPLES  # Add correction-last examples
    + CORRECTION_FIRST_GOAL_EXAMPLES  # Add correction-first examples
    + AMBIGUOUS_MULTI_GOAL_EXAMPLES  # Add ambiguous examples
    + STOP_EXAMPLES  # Add stop examples
    + CANCEL_EXAMPLES  # Add cancel examples
)


# =========================
# PRECOMPUTE EMBEDDINGS
# =========================

single_goal_sentences = [example[0] for example in SINGLE_GOAL_EXAMPLES]  # Extract single-goal example texts
correction_single_goal_sentences = [example[0] for example in CORRECTION_SINGLE_GOAL_EXAMPLES]  # Extract correction single-goal texts
multi_goal_and_sentences = [example[0] for example in MULTI_GOAL_AND_EXAMPLES]  # Extract multi-goal "and" texts
correction_last_sentences = [example[0] for example in CORRECTION_LAST_GOAL_EXAMPLES]  # Extract correction-last texts
correction_first_sentences = [example[0] for example in CORRECTION_FIRST_GOAL_EXAMPLES]  # Extract correction-first texts
ambiguous_sentences = [example[0] for example in AMBIGUOUS_MULTI_GOAL_EXAMPLES]  # Extract ambiguous texts
stop_sentences = [example[0] for example in STOP_EXAMPLES]  # Extract stop texts
cancel_sentences = [example[0] for example in CANCEL_EXAMPLES]  # Extract cancel texts
all_sentences = [example[0] for example in ALL_EXAMPLES]  # Extract all fallback texts

single_goal_embeddings = similarity_model.encode(single_goal_sentences, convert_to_tensor=True)  # Precompute single-goal embeddings
correction_single_goal_embeddings = similarity_model.encode(correction_single_goal_sentences, convert_to_tensor=True)  # Precompute correction single-goal embeddings
multi_goal_and_embeddings = similarity_model.encode(multi_goal_and_sentences, convert_to_tensor=True)  # Precompute multi-goal embeddings
correction_last_embeddings = similarity_model.encode(correction_last_sentences, convert_to_tensor=True)  # Precompute correction-last embeddings
correction_first_embeddings = similarity_model.encode(correction_first_sentences, convert_to_tensor=True)  # Precompute correction-first embeddings
ambiguous_embeddings = similarity_model.encode(ambiguous_sentences, convert_to_tensor=True)  # Precompute ambiguous embeddings
stop_embeddings = similarity_model.encode(stop_sentences, convert_to_tensor=True)  # Precompute stop embeddings
cancel_embeddings = similarity_model.encode(cancel_sentences, convert_to_tensor=True)  # Precompute cancel embeddings
all_embeddings = similarity_model.encode(all_sentences, convert_to_tensor=True)  # Precompute fallback embeddings


print("System ready. Talk now. Ctrl+C to stop.")  # Print startup message

rclpy.init()

voice_node = rclpy.create_node("voice_command_node")
voice_pub = voice_node.create_publisher(String, "/voice_command", 10)

voice_node.get_logger().info("Voice command node started. Publishing on /voice_command")


# =========================
# AUDIO UTILS
# =========================

def float32_to_int16_bytes(audio_float32):  # Define a function to convert float32 audio to int16 bytes
    audio_int16 = np.clip(audio_float32, -1.0, 1.0)  # Clip audio values to the valid range [-1, 1]
    audio_int16 = (audio_int16 * 32767).astype(np.int16)  # Convert float audio to 16-bit PCM integer audio
    return audio_int16.tobytes()  # Return raw bytes for Vosk


# =========================
# SPEECH TO TEXT
# =========================

def transcribe_segment(audio_chunks):  # Define a function that transcribes one completed speech segment
    audio = np.concatenate(audio_chunks)  # Concatenate all 32 ms chunks into one complete audio array
    audio_bytes = float32_to_int16_bytes(audio)  # Convert the audio array to int16 bytes for Vosk

    recognizer = KaldiRecognizer(stt_model, SAMPLE_RATE)  # Create a Vosk recognizer for this segment
    recognizer.SetWords(True)  # Ask Vosk to return word-level confidence scores

    recognizer.AcceptWaveform(audio_bytes)  # Send the complete audio segment to Vosk

    result = json.loads(recognizer.Result())  # Convert Vosk JSON string result into a Python dictionary

    text = result.get("text", "")  # Extract recognized text, or empty string if missing

    words = result.get("result", [])  # Extract word-level results, or empty list if missing

    if words:  # Check whether Vosk returned word-level information
        confidences = [word.get("conf", 0.0) for word in words]  # Extract confidence score of each recognized word
        asr_confidence = sum(confidences) / len(confidences)  # Compute the average word confidence
    else:  # Handle the case where no words were recognized
        asr_confidence = 0.0  # Set confidence to zero when no words exist

    return text, asr_confidence  # Return recognized text and ASR confidence


# =========================
# INTENT PARSER UTILS
# =========================

def extract_goals(text):  # Define a function to extract goal IDs from text
    text_lower = text.lower().strip()  # Normalize text to lowercase and remove external spaces

    number_map = {  # Define mapping from French number words to goal IDs
        "1": 1,  # Map digit 1 to goal 1
        "un": 1,  # Map "un" to goal 1
        "une": 1,  # Map "une" to goal 1
        "premier": 1,  # Map "premier" to goal 1
        "première": 1,  # Map "première" to goal 1

        "2": 2,  # Map digit 2 to goal 2
        "deux": 2,  # Map "deux" to goal 2
        "deuxième": 2,  # Map "deuxième" to goal 2
        "second": 2,  # Map "second" to goal 2
        "seconde": 2,  # Map "seconde" to goal 2

        "3": 3,  # Map digit 3 to goal 3
        "trois": 3,  # Map "trois" to goal 3
        "troisième": 3,  # Map "troisième" to goal 3

        "4": 4,  # Map digit 4 to goal 4
        "quatre": 4,  # Map "quatre" to goal 4
        "quatrième": 4,  # Map "quatrième" to goal 4

        "5": 5,  # Map digit 5 to goal 5
        "cinq": 5,  # Map "cinq" to goal 5
        "cinquième": 5,  # Map "cinquième" to goal 5

        "6": 6,  # Map digit 6 to goal 6
        "six": 6,  # Map "six" to goal 6
        "sixième": 6,  # Map "sixième" to goal 6
    }  # End of number map

    tokens = re.findall(r"\w+", text_lower)  # Split text into word-like tokens

    found_goals = []  # Create an empty list to store detected goals

    for token in tokens:  # Iterate over each token
        if token in number_map:  # Check whether the token is a known number
            found_goals.append(number_map[token])  # Add the corresponding goal ID

    return found_goals  # Return all detected goals in order


def classify_with_similarity(text, examples, embeddings):  # Define a generic similarity classifier
    text_lower = text.lower().strip()  # Normalize the input text

    user_embedding = similarity_model.encode(text_lower, convert_to_tensor=True)  # Convert user text into an embedding

    similarities = util.cos_sim(user_embedding, embeddings)[0]  # Compute cosine similarity with all example embeddings

    best_index = int(torch.argmax(similarities))  # Get the index of the best matching example

    best_score = float(similarities[best_index])  # Convert best similarity score to a Python float

    best_sentence = examples[best_index][0]  # Get the best matching example sentence

    best_label = examples[best_index][1]  # Get the label associated with the best example

    return best_label, best_score, best_sentence  # Return label, similarity score, and matched sentence


# =========================
# INTENT PARSER
# =========================

def parse_intent(text):  # Define the main intent parser
    text_lower = text.lower().strip()  # Normalize recognized text

    if not text_lower:  # Check if the recognized text is empty
        return {  # Return a no-intent result
            "intent": "none",  # No clear intent
            "goal_id": None,  # No goal selected
            "confidence": 0.0,  # Zero confidence
            "reason": "No text was recognized."  # Explanation
        }  # End return

    stop_words = ["stop", "arrête", "arrete", "attends", "pause", "halte"]  # Define direct stop words

    for word in stop_words:  # Iterate over stop words
        if word in text_lower:  # Check if a stop word appears in the text
            return {  # Return stop intent immediately
                "intent": "stop",  # Final intent is stop
                "goal_id": None,  # Stop does not target a goal
                "confidence": 0.98,  # High confidence for direct stop word
                "reason": "The user requested an immediate stop."  # Explanation
            }  # End return

    cancel_words = ["annule", "laisse tomber", "cancel", "oublie"]  # Define direct cancel words

    for word in cancel_words:  # Iterate over cancel words
        if word in text_lower:  # Check if a cancel word appears in the text
            return {  # Return cancel intent immediately
                "intent": "cancel",  # Final intent is cancel
                "goal_id": None,  # Cancel does not target a goal
                "confidence": 0.9,  # High confidence for direct cancel word
                "reason": "The user cancelled the previous command."  # Explanation
            }  # End return

    found_goals = extract_goals(text_lower)  # Extract goal IDs from the recognized text

    if not found_goals:  # Check if no goal was found
        return {  # Return none intent
            "intent": "none",  # No clear goal intent
            "goal_id": None,  # No goal selected
            "confidence": 0.2,  # Low confidence
            "reason": "No explicit goal was detected."  # Explanation
        }  # End return

    if len(found_goals) == 1:  # Handle the case where exactly one goal is mentioned
        goal_id = found_goals[0]  # Select the only detected goal

        single_label, single_score, single_sentence = classify_with_similarity(  # Compare with simple single-goal examples
            text_lower,  # User text
            SINGLE_GOAL_EXAMPLES,  # Single-goal examples
            single_goal_embeddings  # Single-goal embeddings
        )  # End similarity call

        correction_label, correction_score, correction_sentence = classify_with_similarity(  # Compare with correction single-goal examples
            text_lower,  # User text
            CORRECTION_SINGLE_GOAL_EXAMPLES,  # Correction examples
            correction_single_goal_embeddings  # Correction embeddings
        )  # End similarity call

        if correction_score > single_score and correction_score >= 0.55:  # Check whether correction pattern is stronger than simple goal pattern
            return {  # Return a boost_goal result with correction context
                "intent": "boost_goal",  # Final intent remains boost_goal
                "goal_id": goal_id,  # Selected goal is the only detected goal
                "confidence": min(0.88, correction_score),  # Use similarity score capped at 0.88
                "reason": f"Single goal with correction pattern similar to: '{correction_sentence}'."  # Explanation
            }  # End return

        return {  # Return a normal single-goal result
            "intent": "boost_goal",  # Final intent is boost_goal
            "goal_id": goal_id,  # Selected goal is the only detected goal
            "confidence": min(0.9, max(0.75, single_score)),  # Keep confidence between 0.75 and 0.9
            "reason": f"Single clear goal detected: {goal_id}."  # Explanation
        }  # End return

    labels_scores = []  # Create a list to store similarity results for multi-goal patterns

    label, score, sentence = classify_with_similarity(text_lower, MULTI_GOAL_AND_EXAMPLES, multi_goal_and_embeddings)  # Classify as multi-goal "and"
    labels_scores.append((label, score, sentence))  # Store this classification result

    label, score, sentence = classify_with_similarity(text_lower, CORRECTION_LAST_GOAL_EXAMPLES, correction_last_embeddings)  # Classify as correction to last goal
    labels_scores.append((label, score, sentence))  # Store this classification result

    label, score, sentence = classify_with_similarity(text_lower, CORRECTION_FIRST_GOAL_EXAMPLES, correction_first_embeddings)  # Classify as correction to first goal
    labels_scores.append((label, score, sentence))  # Store this classification result

    label, score, sentence = classify_with_similarity(text_lower, AMBIGUOUS_MULTI_GOAL_EXAMPLES, ambiguous_embeddings)  # Classify as ambiguous multi-goal
    labels_scores.append((label, score, sentence))  # Store this classification result

    best_label, best_score, best_sentence = max(labels_scores, key=lambda x: x[1])  # Select the pattern with the highest similarity score

    if best_label == "correction_last" and best_score >= 0.55:  # Check if the phrase means "take the last mentioned goal"
        goal_id = found_goals[-1]  # Select the last detected goal

        return {  # Return corrected boost goal
            "intent": "boost_goal",  # Final intent is boost_goal
            "goal_id": goal_id,  # Selected goal is last goal
            "confidence": min(0.9, best_score),  # Use similarity score capped at 0.9
            "reason": f"Multiple goals detected; correction_last pattern similar to: '{best_sentence}'. Selected last goal {goal_id}."  # Explanation
        }  # End return

    if best_label == "correction_first" and best_score >= 0.55:  # Check if the phrase means "keep the first mentioned goal"
        goal_id = found_goals[0]  # Select the first detected goal

        return {  # Return corrected boost goal
            "intent": "boost_goal",  # Final intent is boost_goal
            "goal_id": goal_id,  # Selected goal is first goal
            "confidence": min(0.85, best_score),  # Use similarity score capped at 0.85
            "reason": f"Multiple goals detected; correction_first pattern similar to: '{best_sentence}'. Selected first goal {goal_id}."  # Explanation
        }  # End return

    if best_label == "ambiguous" and best_score >= 0.55:  # Check if the phrase is ambiguous
        return {  # Return none because no unique goal should be selected
            "intent": "none",  # No safe intent
            "goal_id": None,  # No goal selected
            "confidence": 0.4,  # Low confidence
            "reason": f"Multiple goals were mentioned ambiguously; similar to: '{best_sentence}'."  # Explanation
        }  # End return

    if best_label == "multi_goal_and" and best_score >= 0.55:  # Check if the phrase mentions multiple goals together
        return {  # Return none because the robot expects one unique goal
            "intent": "none",  # No single safe goal intent
            "goal_id": None,  # No goal selected
            "confidence": 0.45,  # Low confidence
            "reason": f"Multiple goals were mentioned without a unique target; similar to: '{best_sentence}'."  # Explanation
        }  # End return

    goal_id = found_goals[-1]  # Fallback strategy selects the last mentioned goal

    return {  # Return fallback result
        "intent": "boost_goal",  # Final intent is boost_goal
        "goal_id": goal_id,  # Selected goal is last goal
        "confidence": 0.6,  # Low confidence because pattern was unclear
        "reason": f"Multiple goals detected but pattern unclear. Fallback selected last goal {goal_id} with low confidence."  # Explanation
    }  # End return


def parse_intent_with_similarity(text):  # Define wrapper parser with fallback similarity
    result = parse_intent(text)  # First use the main parser

    if result["intent"] != "none":  # If the main parser found a clear intent
        return result  # Return it directly

    text_lower = text.lower().strip()  # Normalize text

    if not text_lower:  # Check if text is empty
        return result  # Return original none result

    label, score, sentence = classify_with_similarity(text_lower, ALL_EXAMPLES, all_embeddings)  # Classify with all examples as fallback

    if score < 0.60:  # If fallback similarity is too weak
        return result  # Return original none result

    if label == "stop":  # Check if fallback matched a stop example
        return {  # Return stop intent
            "intent": "stop",  # Final intent is stop
            "goal_id": None,  # Stop has no goal
            "confidence": score,  # Use similarity score as confidence
            "reason": f"Similarity fallback matched stop example: '{sentence}'."  # Explanation
        }  # End return

    if label == "cancel":  # Check if fallback matched a cancel example
        return {  # Return cancel intent
            "intent": "cancel",  # Final intent is cancel
            "goal_id": None,  # Cancel has no goal
            "confidence": score,  # Use similarity score as confidence
            "reason": f"Similarity fallback matched cancel example: '{sentence}'."  # Explanation
        }  # End return

    return result  # Do not invent a goal if no explicit goal was detected


# =========================
# TEMPORARY VOICE CONFIDENCE
# =========================

def compute_voice_confidence(goal_id, now):  # Define a function to compute current voice confidence for a goal
    global last_voice_goal  # Use global last_voice_goal
    global last_voice_confidence  # Use global last_voice_confidence
    global last_voice_time  # Use global last_voice_time

    if last_voice_goal != goal_id:  # Check if the requested goal is not the last voice goal
        return 0.0  # Return zero confidence for other goals

    dt = now - last_voice_time  # Compute elapsed time since the last voice command

    return last_voice_confidence * math.exp(-dt / TAU_VOICE)  # Return exponentially decayed confidence


def update_voice_boost(intent_result):  # Define a function to update the voice boost state
    global last_voice_goal  # Use global last_voice_goal
    global last_voice_confidence  # Use global last_voice_confidence
    global last_voice_time  # Use global last_voice_time

    if intent_result["intent"] == "boost_goal" and intent_result["goal_id"] is not None:  # Update only for valid boost_goal intents
        last_voice_goal = intent_result["goal_id"]  # Store selected goal
        last_voice_confidence = intent_result["confidence"]  # Store command confidence
        last_voice_time = time.time()  # Store current time


# =========================
# MAIN LOOP
# =========================

recording = False  # Track whether the system is currently collecting a speech segment
current_segment = []  # Store audio chunks for the current speech segment

silence_duration = 0.0  # Store how long silence has lasted after speech started
segment_duration = 0.0  # Store total duration of the current segment

chunk_duration = CHUNK_SIZE / SAMPLE_RATE  # Compute duration of one audio chunk in seconds

with sd.InputStream(  # Open microphone input stream
    samplerate=SAMPLE_RATE,  # Set microphone sample rate
    channels=1,  # Use mono audio
    dtype="float32",  # Read audio as float32 values between about -1 and 1
    blocksize=CHUNK_SIZE,  # Read one chunk of 512 samples at a time
) as stream:  # Store opened microphone stream in variable stream

    while True:  # Run forever until Ctrl+C
        audio_chunk, overflowed = stream.read(CHUNK_SIZE)  # Read one audio chunk from microphone

        if overflowed:  # Check if audio data was lost
            print("Audio overflow")  # Print warning

        audio_chunk = audio_chunk[:, 0]  # Convert shape from (512, 1) to (512,)

        audio_tensor = torch.from_numpy(audio_chunk)  # Convert NumPy audio chunk to PyTorch tensor

        speech_prob = vad_model(audio_tensor, SAMPLE_RATE).item()  # Compute speech probability with Silero VAD

        is_speech = speech_prob > VAD_THRESHOLD  # Convert probability into True or False decision

        if is_speech:  # If current chunk contains speech
            if not recording:  # If this is the beginning of a new speech segment
                print("\nStart speech")  # Print start message
                recording = True  # Mark that we are recording a segment
                current_segment = []  # Reset current audio segment
                silence_duration = 0.0  # Reset silence duration
                segment_duration = 0.0  # Reset segment duration

            current_segment.append(audio_chunk.copy())  # Add current audio chunk to the segment
            silence_duration = 0.0  # Reset silence duration because speech is active
            segment_duration += chunk_duration  # Increase segment duration by one chunk

        else:  # If current chunk is silence or non-speech
            if recording:  # Only care about silence if we were already recording speech
                current_segment.append(audio_chunk.copy())  # Add silence chunk to the segment ending
                silence_duration += chunk_duration  # Increase silence duration
                segment_duration += chunk_duration  # Increase segment duration

                end_of_speech = silence_duration >= END_SILENCE_SECONDS  # Check if enough silence means speech ended
                too_long = segment_duration >= MAX_SEGMENT_SECONDS  # Check if segment is too long

                if end_of_speech or too_long:  # Finalize segment if speech ended or max duration reached
                    print("End speech")  # Print end message

                    if segment_duration >= MIN_SEGMENT_SECONDS:  # Ignore very short noise segments
                        text, asr_confidence = transcribe_segment(current_segment)  # Transcribe the speech segment with Vosk

                        print("STT text:", text)  # Print recognized text
                        print("ASR confidence:", asr_confidence)  # Print speech-to-text confidence

                        intent_result = parse_intent_with_similarity(text)  # Parse the recognized text into an intent

                        print("Intent result:")  # Print title
                        print(json.dumps(intent_result, indent=2, ensure_ascii=False))  # Print intent JSON nicely

                        voice_msg_data = {
                            "intent": intent_result["intent"],
                            "goal_id": intent_result["goal_id"],
                            "confidence": intent_result["confidence"],
                            "reason": intent_result["reason"],
                            "text": text,
                            "asr_confidence": asr_confidence,
                            "time": time.time()
                        }

                        voice_msg = String()
                        voice_msg.data = json.dumps(voice_msg_data, ensure_ascii=False)

                        voice_pub.publish(voice_msg)

                        voice_node.get_logger().info(f"Published voice command: {voice_msg.data}")

                        update_voice_boost(intent_result)  # Update temporary voice boost if intent is boost_goal

                        now = time.time()  # Get current time

                        for goal_id in range(1, 7):  # Check goals 1 to 6
                            c_voice = compute_voice_confidence(goal_id, now)  # Compute decayed voice confidence for this goal
                            if c_voice > 0.01:  # Print only meaningful confidence values
                                print(f"c_voice_{goal_id} = {c_voice:.3f}")  # Print current voice confidence

                    recording = False  # Mark that we are no longer recording
                    current_segment = []  # Clear current segment
                    silence_duration = 0.0  # Reset silence duration
                    segment_duration = 0.0  # Reset segment duration
