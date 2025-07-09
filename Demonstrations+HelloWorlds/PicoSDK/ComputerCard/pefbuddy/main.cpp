#include "computercard.h"
#include <math.h>

ComputerCard card;

const int SAMPLE_RATE = 48000;
const int BUFFER_SIZE = 960;

uint16_t audioBuffer[BUFFER_SIZE];
volatile int bufferIndex = 0;
volatile bool bufferReady = false;
float envelope = 0.0f;
float previousEnvelope = 0.0f;

bool gateState = false;

// Sample audio into buffer
bool sampleAudio(struct repeating_timer *t) {
    if (bufferIndex < BUFFER_SIZE) {
        float a = card.audioIn1();  // -1.0 to +1.0
        uint16_t sample = (uint16_t)((a + 1.0f) * 2048.0f);  // 0–4096
        audioBuffer[bufferIndex++] = sample;

        float val = fabsf((float)sample - 2048.0f);
        envelope = 0.01f * val + 0.99f * envelope;

        // Envelope threshold controlled by X knob (scaled to 0–1000)
        float threshold = card.x() * 1000.0f;

        // GATE LOGIC
        if (envelope > threshold) {
            if (!gateState) {
                // Rising edge — trigger pulse
                card.pulseOut();
                card.led2(HIGH);  // flash LED2 on trigger
            }
            gateState = true;
            card.gateOut(HIGH);
        } else {
            gateState = false;
            card.gateOut(LOW);
            card.led2(LOW);
        }

        previousEnvelope = envelope;

        if (bufferIndex >= BUFFER_SIZE) {
            bufferReady = true;
        }
    }
    return true;
}

// Pitch detection
float detectPitch(const uint16_t *buffer, int size, int sampleRate) {
    float mean = 0.0f;
    for (int i = 0; i < size; i++) mean += buffer[i];
    mean /= size;

    float norm[size];
    for (int i = 0; i < size; i++) norm[i] = buffer[i] - mean;

    float maxCorr = 0.0f;
    int bestLag = -1;

    for (int lag = 40; lag < 1000; lag++) {
        float sum = 0.0f;
        for (int i = 0; i < size - lag; i++)
            sum += norm[i] * norm[i + lag];

        if (sum > maxCorr) {
            maxCorr = sum;
            bestLag = lag;
        }
    }

    if (bestLag > 0) return (float)sampleRate / bestLag;
    return 0.0f;
}

float hzToVPerOct(float hz) {
    if (hz <= 0.0f) return 0.0f;
    return log2f(hz / 440.0f) + 4.0f;
}

float detectPitchYIN(const uint16_t* buffer, int size, int sampleRate) {
    static float yinBuffer[BUFFER_SIZE / 2] = {0};

    const int maxTau = size / 2;
    const float threshold = 0.10f;

    // 1. Difference function
    for (int tau = 1; tau < maxTau; tau++) {
        float sum = 0;
        for (int i = 0; i < maxTau; i++) {
            float delta = buffer[i] - buffer[i + tau];
            sum += delta * delta;
        }
        yinBuffer[tau] = sum;
    }

    // 2. Cumulative mean normalized difference
    yinBuffer[0] = 1.0f;
    float runningSum = 0;
    for (int tau = 1; tau < maxTau; tau++) {
        runningSum += yinBuffer[tau];
        yinBuffer[tau] *= tau / runningSum;
    }

    // 3. Find first dip below threshold
    int tauEstimate = -1;
    for (int tau = 2; tau < maxTau; tau++) {
        if (yinBuffer[tau] < threshold &&
            yinBuffer[tau] < yinBuffer[tau - 1]) {
            tauEstimate = tau;

            // Optional: parabolic interpolation for precision
            if (tau + 1 < maxTau && yinBuffer[tau + 1] != 0.0f) {
                float x0 = yinBuffer[tau - 1];
                float x1 = yinBuffer[tau];
                float x2 = yinBuffer[tau + 1];
                float denom = (2 * x1 - x2 - x0);
                if (denom != 0.0f) {
                    tauEstimate += (x2 - x0) / (2.0f * denom);
                }
            }
            break;
        }
    }

    if (tauEstimate > 0) {
        return sampleRate / tauEstimate;
    }

    return 0.0f;  // No pitch found
}


void setup() {
    card.begin();
    add_repeating_timer_us(-1000000 / SAMPLE_RATE, sampleAudio, NULL, NULL);
}

void loop() {
    if (bufferReady) {
        noInterrupts();
        bufferReady = false;
        bufferIndex = 0;
        interrupts();
        //
        float pitchHz = detectPitchYIN(audioBuffer, BUFFER_SIZE, SAMPLE_RATE); // yin pitch detection
        
        //float pitchHz = detectPitch(audioBuffer, BUFFER_SIZE, SAMPLE_RATE);  //standard pitch detection

        float volts = hzToVPerOct(pitchHz);
        float quantizedVolts = roundf(volts * 12.0f) / 12.0f;

        card.cvOut(quantizedVolts);

        if (pitchHz > 20.0f && pitchHz < 2000.0f) {
            card.led1(HIGH);
        } else {
            card.led1(LOW);
        }
    }
}
