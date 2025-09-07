#!/usr/bin/env python3
import wave
import sys

def merge_wav(wav_path1, wav_path2, out_path):
    # Open first input
    with wave.open(wav_path1, 'rb') as wav1:
        params1 = wav1.getparams()
        frames1 = wav1.readframes(wav1.getnframes())

    # Open second input
    with wave.open(wav_path2, 'rb') as wav2:
        params2 = wav2.getparams()
        frames2 = wav2.readframes(wav2.getnframes())

    # Ensure both files have exactly the same audio format
    if params1 != params2:
        print("Error: Input files have different parameters:")
        print(" File1:", params1)
        print(" File2:", params2)
        sys.exit(1)

    # Write combined data
    with wave.open(out_path, 'wb') as out:
        out.setparams(params1)
        out.writeframes(frames1 + frames2)
    print(f"Merged '{wav_path1}' + '{wav_path2}' → '{out_path}'")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python merge_wav.py input1.wav input2.wav output.wav")
        sys.exit(1)
    merge_wav(sys.argv[1], sys.argv[2], sys.argv[3])
