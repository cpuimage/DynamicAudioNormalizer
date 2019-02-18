#ifndef DynamicAudioNormalizer_H_
#define DynamicAudioNormalizer_H_

#include <stdint.h>
#include <deque>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <queue>
#include <algorithm>


#define MY_DELETE(X) do \
{ \
    if((X)) \
    { \
        delete (X); \
        (X) = NULL; \
    } \
} \
while(0)

#define MY_DELETE_ARRAY(X) do \
{ \
    if((X)) \
    { \
        delete [] (X); \
        (X) = NULL; \
    } \
} \
while(0)

#define MY_FREE(X) do \
{ \
    if((X)) \
    { \
        free(X); \
        (X) = NULL; \
    } \
} \
while(0)


class GaussianFilter {
public:
    GaussianFilter(const uint32_t &filterSize, const float &sigma);

    virtual ~GaussianFilter();

    float apply(const std::deque<float> &values);

private:
    const uint32_t m_filterSize;
    float *m_weights;

    GaussianFilter &operator=(const GaussianFilter &) { throw 666; }
};

class FrameData {
public:
    FrameData(const uint32_t &channels, const uint32_t &frameLength);

    ~FrameData();

    inline float *data(const uint32_t &channel) {
        assert(channel < m_channels);
        return m_data[channel];
    }

    inline const float *data(const uint32_t &channel) const {
        assert(channel < m_channels);
        return m_data[channel];
    }

    inline const uint32_t &channels() { return m_channels; }

    inline const uint32_t &frameLength() { return m_frameLength; }

    inline void write(const float *const *const src, const uint32_t &srcOffset, const uint32_t &destOffset,
                      const uint32_t &length) {
        assert(length + destOffset <= m_frameLength);
        for (uint32_t c = 0; c < m_channels; c++) {
            memcpy(&m_data[c][destOffset], &src[c][srcOffset], length * sizeof(float));
        }
    }

    inline void write(const FrameData *const src, const uint32_t &srcOffset, const uint32_t &destOffset,
                      const uint32_t &length) {
        assert(length + destOffset <= m_frameLength);
        for (uint32_t c = 0; c < m_channels; c++) {
            memcpy(&m_data[c][destOffset], &src->data(c)[srcOffset], length * sizeof(float));
        }
    }

    inline void
    read(float *const *const dest, const uint32_t &destOffset, const uint32_t &srcOffset, const uint32_t &length) {
        assert(length + srcOffset <= m_frameLength);
        for (uint32_t c = 0; c < m_channels; c++) {
            memcpy(&dest[c][destOffset], &m_data[c][srcOffset], length * sizeof(float));
        }
    }

    inline void
    read(FrameData *const dest, const uint32_t &destOffset, const uint32_t &srcOffset, const uint32_t &length) {
        assert(length + srcOffset <= m_frameLength);
        for (uint32_t c = 0; c < m_channels; c++) {
            memcpy(&dest->data(c)[destOffset], &m_data[c][srcOffset], length * sizeof(float));
        }
    }

    void clear();

private:
    FrameData(const FrameData &) : m_channels(0), m_frameLength(0) { throw "unsupported"; }

    FrameData &operator=(const FrameData &) { throw "unsupported"; }

    const uint32_t m_channels;
    const uint32_t m_frameLength;

    float **m_data;
};

class FrameFIFO {
public:
    FrameFIFO(const uint32_t &channels, const uint32_t &frameLength);

    ~FrameFIFO();

    inline uint32_t samplesLeftPut() { return m_leftPut; }

    inline uint32_t samplesLeftGet() { return m_leftGet; }

    inline void putSamples(const float *const *const src, const uint32_t &srcOffset, const uint32_t &length) {
        assert(length <= samplesLeftPut());
        m_data->write(src, srcOffset, m_posPut, length);
        m_posPut += length;
        m_leftPut -= length;
        m_leftGet += length;
    }

    inline void putSamples(const FrameData *const src, const uint32_t &srcOffset, const uint32_t &length) {
        assert(length <= samplesLeftPut());
        m_data->write(src, srcOffset, m_posPut, length);
        m_posPut += length;
        m_leftPut -= length;
        m_leftGet += length;
    }

    inline void getSamples(float *const *const dest, const uint32_t &destOffset, const uint32_t &length) {
        assert(length <= samplesLeftGet());
        m_data->read(dest, destOffset, m_posGet, length);
        m_posGet += length;
        m_leftGet -= length;
    }

    inline void getSamples(FrameData *const dest, const uint32_t &destOffset, const uint32_t &length) {
        assert(length <= samplesLeftGet());
        m_data->read(dest, destOffset, m_posGet, length);
        m_posGet += length;
        m_leftGet -= length;
    }

    inline FrameData *data() {
        return m_data;
    }

    void reset(const bool &bForceClear = true);

private:
    FrameData *m_data;

    uint32_t m_posPut;
    uint32_t m_posGet;
    uint32_t m_leftPut;
    uint32_t m_leftGet;
};

class FrameBuffer {
public:
    FrameBuffer(const uint32_t &channels, const uint32_t &frameLength, const uint32_t &frameCount);

    ~FrameBuffer();

    bool putFrame(FrameFIFO *const src);

    bool getFrame(FrameFIFO *const dest);

    inline const uint32_t &channels() { return m_channels; }

    inline const uint32_t &frameLength() { return m_frameLength; }

    inline const uint32_t &frameCount() { return m_frameCount; }

    inline const uint32_t &framesFree() { return m_framesFree; }

    inline const uint32_t &framesUsed() { return m_framesUsed; }

    void reset();

private:
    FrameBuffer(const FrameBuffer &) : m_channels(0), m_frameLength(0), m_frameCount(0) { throw "unsupported"; }

    FrameBuffer &operator=(const FrameBuffer &) { throw "unsupported"; }

    const uint32_t m_channels;
    const uint32_t m_frameLength;
    const uint32_t m_frameCount;

    uint32_t m_framesFree;
    uint32_t m_framesUsed;

    uint32_t m_posPut;
    uint32_t m_posGet;

    FrameData **m_frames;
};


/*Opaque Data Class*/
class MDynamicAudioNormalizer_PrivateData;

/*Dynamic Normalizer Class*/
class MDynamicAudioNormalizer {
public:
    /*Constructor & Destructor*/
    MDynamicAudioNormalizer(const uint32_t channels, const uint32_t sampleRate, const uint32_t frameLenMsec = 500,
                            const uint32_t filterSize = 31, const float peakValue = 0.95f,
                            const float maxAmplification = 10.0f, const float targetRms = 0.0f,
                            const float compressFactor = 0.0f, const bool channelsCoupled = true,
                            const bool enableDCCorrection = false, const bool altBoundaryMode = false);

    virtual ~MDynamicAudioNormalizer();

    /*Public API*/
    bool initialize();

    bool process(const float *const *const samplesIn, float *const *const samplesOut, const int64_t inputSize,
                 int64_t &outputSize);

    bool processInplace(float *const *const samplesInOut, const int64_t inputSize, int64_t &outputSize);

    bool flushBuffer(float *const *const samplesOut, const int64_t bufferSize, int64_t &outputSize);

    bool reset();

    bool getConfiguration(uint32_t &channels, uint32_t &sampleRate, uint32_t &frameLen, uint32_t &filterSize);

    bool getInternalDelay(int64_t &delayInSamples);


private:
    MDynamicAudioNormalizer(const MDynamicAudioNormalizer &) : p(NULL) { throw "unsupported"; }

    MDynamicAudioNormalizer &operator=(const MDynamicAudioNormalizer &) { throw "unsupported"; }

    MDynamicAudioNormalizer_PrivateData *const p;
};


template<typename T>
static inline T LIMIT(const T &min, const T &val, const T &max) {
    return std::min(max, std::max(min, val));
}

template<typename T>
static inline void UPDATE_MAX(T &max, const T &val) {
    if (val > max) { max = val; }
}

static inline uint32_t FRAME_SIZE(const uint32_t &sampleRate, const uint32_t &frameLenMsec) {
    const uint32_t frameSize = static_cast<uint32_t>(round(float(sampleRate) * (float(frameLenMsec) / 1000.0f)));
    return frameSize + (frameSize % 2);
}

static inline float UPDATE_VALUE(const float &NEW, const float &OLD, const float &aggressiveness) {
    assert((aggressiveness >= 0.0) && (aggressiveness <= 1.0));
    return (aggressiveness * NEW) + ((1.0f - aggressiveness) * OLD);
}

static inline float
FADE(const float &prev, const float &next, const uint32_t &pos, const float *const fadeFactors[2]) {
    return (fadeFactors[0][pos] * prev) + (fadeFactors[1][pos] * next);
}

static inline float BOUND(const float &threshold, const float &val) {
    const float SQRT_PI = 0.8862269254527580136490837416705725913987747280611935f; //sqrt(PI) / 2.0
    return erf(SQRT_PI * (val / threshold)) * threshold;
}

static inline float POW2(const float &value) {
    return value * value;
}

#define BOOLIFY(X) ((X) ? "YES" : "NO")
#define POW2(X) ((X)*(X))

#define CLEAR_QUEUE(X) do \
{ \
    while(!(X).empty()) (X).pop(); \
} \
while(0)

///////////////////////////////////////////////////////////////////////////////
// MDynamicAudioNormalizer_PrivateData
///////////////////////////////////////////////////////////////////////////////

class MDynamicAudioNormalizer_PrivateData {
public:
    MDynamicAudioNormalizer_PrivateData(const uint32_t channels, const uint32_t sampleRate, const uint32_t frameLenMsec,
                                        const uint32_t filterSize, const float peakValue,
                                        const float maxAmplification, const float targetRms,
                                        const float compressThresh, const bool channelsCoupled,
                                        const bool enableDCCorrection, const bool altBoundaryMode);

    ~MDynamicAudioNormalizer_PrivateData();

    bool initialize();

    bool process(const float *const *const samplesIn, float *const *const samplesOut, const int64_t inputSize,
                 int64_t &outputSize, const bool &bFlush);

    bool flushBuffer(float *const *const samplesOut, const int64_t bufferSize, int64_t &outputSize);

    bool reset();

    bool getConfiguration(uint32_t &channels, uint32_t &sampleRate, uint32_t &frameLen, uint32_t &filterSize);

    bool getInternalDelay(int64_t &delayInSamples);

private:
    const uint32_t m_channels;
    const uint32_t m_sampleRate;
    const uint32_t m_frameLen;
    const uint32_t m_filterSize;
    const uint32_t m_prefillLen;
    const uint32_t m_delay;

    const float m_peakValue;
    const float m_maxAmplification;
    const float m_targetRms;
    const float m_compressFactor;

    const bool m_channelsCoupled;
    const bool m_enableDCCorrection;
    const bool m_altBoundaryMode;

    bool m_initialized;
    bool m_flushBuffer;

    FrameFIFO *m_buffSrc;
    FrameFIFO *m_buffOut;

    int64_t m_delayedSamples;

    uint64_t m_sampleCounterTotal;
    uint64_t m_sampleCounterClips;

    FrameBuffer *m_frameBuffer;

    std::deque<float> *m_gainHistory_original;
    std::deque<float> *m_gainHistory_minimum;
    std::deque<float> *m_gainHistory_smoothed;

    std::queue<float> *m_loggingData_original;
    std::queue<float> *m_loggingData_minimum;
    std::queue<float> *m_loggingData_smoothed;
    GaussianFilter *m_gaussianFilter;
    float *m_prevAmplificationFactor;
    float *m_dcCorrectionValue;
    float *m_compressThreshold;

    float *m_fadeFactors[2];

protected:
    void analyzeFrame(FrameData *const frame);

    void amplifyFrame(FrameData *const frame);

    float getMaxLocalGain(FrameData *const frame, const uint32_t channel = UINT32_MAX);

    float findPeakMagnitude(FrameData *const frame, const uint32_t channel = UINT32_MAX);

    float computeFrameRMS(const FrameData *const frame, const uint32_t channel = UINT32_MAX);

    float computeFrameStdDev(const FrameData *const frame, const uint32_t channel = UINT32_MAX);

    void updateGainHistory(const uint32_t &channel, const float &currentGainFactor);

    void perfromDCCorrection(FrameData *const frame, const bool &isFirstFrame);

    void perfromCompression(FrameData *const frame, const bool &isFirstFrame);

    void printParameters();

    static void precalculateFadeFactors(float *const fadeFactors[2], const uint32_t frameLen);

    static float setupCompressThresh(const float &dThreshold);
};

#endif  // WORLD_COMMON_H_