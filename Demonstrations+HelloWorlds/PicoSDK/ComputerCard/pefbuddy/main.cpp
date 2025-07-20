#include "ComputerCard.h"
#include <math.h>

/*
 * PefBuddy
 * -------
 * This program analyses the signal on Audio In 1 and generates a CV output
 * representing the detected pitch as well as gate and trigger signals based on
 * the input level.
 *
 *  - A short audio buffer is filled at 48kHz.  When full, the YIN pitch
 *    detection algorithm estimates the fundamental frequency.
 *  - The detected pitch is converted to volts-per-octave, quantised to the
 *    nearest semitone and sent to CV Out 1 using the library's calibrated MIDI
 *    note helpers.
 *  - The input envelope is tracked continuously.  When it exceeds a threshold
 *    set by Knob X, a gate is asserted on Pulse Out 2 and a short trigger is
 *    sent on Pulse Out 1.  LEDs indicate gate status and whether a valid pitch
 *    has been detected.
 */

class PefBuddy : public ComputerCard
{
    static constexpr int SAMPLE_RATE = 48000;
    static constexpr int BUFFER_SIZE = 960;

    uint16_t audioBuffer[BUFFER_SIZE];
    int bufferIndex = 0;
    bool bufferReady = false;

    float envelope = 0.0f;
    float previousEnvelope = 0.0f;
    bool gateState = false;

    static float hzToVPerOct(float hz)
    {
        if (hz <= 0.0f) return 0.0f;
        return log2f(hz / 440.0f) + 4.0f;
    }

    static float detectPitch(const uint16_t* buffer, int size, int sampleRate)
    {
        float mean = 0.0f;
        for (int i = 0; i < size; ++i) mean += buffer[i];
        mean /= size;

        float maxCorr = 0.0f;
        int bestLag = -1;

        for (int lag = 40; lag < 1000; ++lag)
        {
            float sum = 0.0f;
            for (int i = 0; i < size - lag; ++i)
            {
                float a = buffer[i] - mean;
                float b = buffer[i + lag] - mean;
                sum += a * b;
            }
            if (sum > maxCorr)
            {
                maxCorr = sum;
                bestLag = lag;
            }
        }

        if (bestLag > 0) return static_cast<float>(sampleRate) / bestLag;
        return 0.0f;
    }

    static float detectPitchYIN(const uint16_t* buffer, int size, int sampleRate)
    {
        static float yinBuffer[BUFFER_SIZE / 2] = {0};
        const int maxTau = size / 2;
        const float threshold = 0.10f;

        for (int tau = 1; tau < maxTau; ++tau)
        {
            float sum = 0.0f;
            for (int i = 0; i < maxTau; ++i)
            {
                float delta = buffer[i] - buffer[i + tau];
                sum += delta * delta;
            }
            yinBuffer[tau] = sum;
        }

        yinBuffer[0] = 1.0f;
        float runningSum = 0.0f;
        for (int tau = 1; tau < maxTau; ++tau)
        {
            runningSum += yinBuffer[tau];
            yinBuffer[tau] *= tau / runningSum;
        }

        int tauEstimate = -1;
        for (int tau = 2; tau < maxTau; ++tau)
        {
            if (yinBuffer[tau] < threshold && yinBuffer[tau] < yinBuffer[tau - 1])
            {
                tauEstimate = tau;
                if (tau + 1 < maxTau && yinBuffer[tau + 1] != 0.0f)
                {
                    float x0 = yinBuffer[tau - 1];
                    float x1 = yinBuffer[tau];
                    float x2 = yinBuffer[tau + 1];
                    float denom = (2 * x1 - x2 - x0);
                    if (denom != 0.0f)
                    {
                        tauEstimate += (x2 - x0) / (2.0f * denom);
                    }
                }
                break;
            }
        }

        if (tauEstimate > 0) return static_cast<float>(sampleRate) / tauEstimate;
        return 0.0f;
    }

public:
    /// Called at 48kHz for every audio sample.
    ///
    /// Fills the audio buffer, tracks the envelope and generates gate signals.
    /// When the buffer is full, pitch is detected and CV/LED outputs are
    /// updated accordingly.
    virtual void ProcessSample() override
    {
        if (bufferIndex < BUFFER_SIZE)
        {
            int16_t a = AudioIn1();
            uint16_t sample = static_cast<uint16_t>(a + 2048);
            audioBuffer[bufferIndex++] = sample;

            float val = fabsf(static_cast<float>(sample) - 2048.0f);
            envelope = 0.01f * val + 0.99f * envelope;

            float threshold = (KnobVal(Knob::X) / 4095.0f) * 1000.0f;

            if (envelope > threshold)
            {
                if (!gateState)
                {
                    PulseOut1(true);
                    LedOn(2, true);
                }
                gateState = true;
                PulseOut2(true);
            }
            else
            {
                gateState = false;
                PulseOut2(false);
                LedOn(2, false);
            }

            previousEnvelope = envelope;

            if (bufferIndex >= BUFFER_SIZE)
            {
                bufferReady = true;
            }
        }

        if (bufferReady)
        {
            bufferReady = false;
            bufferIndex = 0;

            float pitchHz = detectPitchYIN(audioBuffer, BUFFER_SIZE, SAMPLE_RATE);
            float volts = hzToVPerOct(pitchHz);
            float quantizedVolts = roundf(volts * 12.0f) / 12.0f;

            int midiNote = static_cast<int>(roundf(quantizedVolts * 12.0f + 60.0f));
            if (midiNote < 0) midiNote = 0;
            if (midiNote > 127) midiNote = 127;
            CVOut1MIDINote(static_cast<uint8_t>(midiNote));

            if (pitchHz > 20.0f && pitchHz < 2000.0f)
                LedOn(1, true);
            else
                LedOn(1, false);
        }
    }
};

/// Entry point. Creates the PefBuddy instance, enables input detection and
/// starts the real-time processing loop.
int main()
{
    PefBuddy pb;
    pb.EnableNormalisationProbe();
    pb.Run();
}
