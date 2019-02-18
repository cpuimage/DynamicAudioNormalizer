
// DynamicAudioNormalizer input.wav output.wav
// input.wav  : Input file
// output.wav : Output file

#define DR_MP3_IMPLEMENTATION

#include "dr_mp3.h"

#define DR_WAV_IMPLEMENTATION


#include "dr_wav.h"

#include "DynamicAudioNormalizer.h"

#include <stdint.h>
#include <deque>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <queue>
#include <algorithm>

void DisplayInformation(int fs, int input_length) {
    printf("File information\n");
    printf("Length %d [sample]\n", input_length);
    printf("Length %f [sec]\n", static_cast<float>(input_length) / fs);
}

///////////////////////////////////////////////////////////////////////////////
// Constructor & Destructor
///////////////////////////////////////////////////////////////////////////////

GaussianFilter::GaussianFilter(const uint32_t &filterSize, const float &sigma)
        :
        m_filterSize(filterSize) {
    if ((filterSize < 1) || ((filterSize % 2) != 1)) {
        fprintf(stderr, "Filter size must be a positive and odd value!");
    }

    //Allocate weights
    m_weights = new float[filterSize];
    float totalWeight = 0.0;

    //Pre-computer constants
    const uint32_t offset = m_filterSize / 2;
    const float c1 = 1.0f / (sigma * sqrtf(2.0f *
                                           3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679f));
    const float c2 = 2.0f * powf(sigma, 2.0f);

    //Compute weights
    for (uint32_t i = 0; i < m_filterSize; i++) {
        const int32_t x = int32_t(i) - int32_t(offset);
        m_weights[i] = c1 * expf(-(powf(x, 2.0f) / c2));
        totalWeight += m_weights[i];
    }

    //Adjust weights
    const float adjust = 1.0f / totalWeight;
    for (uint32_t i = 0; i < m_filterSize; i++) {
        m_weights[i] *= adjust;
    }
}

GaussianFilter::~GaussianFilter() {
    MY_DELETE_ARRAY(m_weights);
}

///////////////////////////////////////////////////////////////////////////////
// Apply Filter
///////////////////////////////////////////////////////////////////////////////

float GaussianFilter::apply(const std::deque<float> &values) {
    if (values.size() != m_filterSize) {
        fprintf(stderr, "Input data has the wrong size!\n");
    }

    uint32_t w = 0;
    float result = 0.0;

    //Apply Gaussian filter
    for (std::deque<float>::const_iterator iter = values.begin(); iter != values.end(); iter++) {
        result += (*iter) * m_weights[w++];
    }

    return result;
}



///////////////////////////////////////////////////////////////////////////////
// Frame Data
///////////////////////////////////////////////////////////////////////////////

FrameData::FrameData(const uint32_t &channels, const uint32_t &frameLength)
        :
        m_channels(channels),
        m_frameLength(frameLength) {
    m_data = new float *[m_channels];

    for (uint32_t c = 0; c < m_channels; c++) {
        m_data[c] = new float[m_frameLength];
    }

    clear();
}

FrameData::~FrameData() {
    for (uint32_t c = 0; c < m_channels; c++) {
        MY_DELETE_ARRAY(m_data[c]);
    }

    MY_DELETE_ARRAY(m_data);
}

void FrameData::clear() {
    for (uint32_t c = 0; c < m_channels; c++) {
        memset(m_data[c], 0, m_frameLength * sizeof(float));
    }
}

///////////////////////////////////////////////////////////////////////////////
// Frame FIFO
///////////////////////////////////////////////////////////////////////////////

FrameFIFO::FrameFIFO(const uint32_t &channels, const uint32_t &frameLength) {
    m_data = new FrameData(channels, frameLength);
    reset(false);
}

FrameFIFO::~FrameFIFO() {
    MY_DELETE(m_data);
}

void FrameFIFO::reset(const bool &bForceClear) {
    if (bForceClear) m_data->clear();
    m_posPut = m_posGet = m_leftGet = 0;
    m_leftPut = m_data->frameLength();
}

///////////////////////////////////////////////////////////////////////////////
// Constructor & Destructor
///////////////////////////////////////////////////////////////////////////////

FrameBuffer::FrameBuffer(const uint32_t &channels, const uint32_t &frameLength,
                         const uint32_t &frameCount)
        :
        m_channels(channels),
        m_frameLength(frameLength),
        m_frameCount(frameCount) {
    m_framesFree = m_frameCount;
    m_framesUsed = 0;
    m_posPut = m_posGet = 0;

    m_frames = new FrameData *[m_frameCount];

    for (uint32_t i = 0; i < m_frameCount; i++) {
        m_frames[i] = new FrameData(m_channels, m_frameLength);
    }
}

FrameBuffer::~FrameBuffer() {
    for (uint32_t i = 0; i < m_frameCount; i++) {
        MY_DELETE(m_frames[i]);
    }

    MY_DELETE_ARRAY(m_frames);
}

///////////////////////////////////////////////////////////////////////////////
// Reset
///////////////////////////////////////////////////////////////////////////////

void FrameBuffer::reset() {
    m_framesFree = m_frameCount;
    m_framesUsed = 0;
    m_posPut = m_posGet = 0;

    for (uint32_t i = 0; i < m_frameCount; i++) {
        m_frames[i]->clear();
    }
}

///////////////////////////////////////////////////////////////////////////////
// Put / Get Frame
///////////////////////////////////////////////////////////////////////////////

bool FrameBuffer::putFrame(FrameFIFO *const src) {
    if ((m_framesFree < 1) && (src->samplesLeftGet() < m_frameLength)) {
        return false;
    }

    src->getSamples(m_frames[m_posPut], 0, m_frameLength);
    m_posPut = ((m_posPut + 1) % m_frameCount);

    m_framesUsed++;
    m_framesFree--;

    return true;
}

bool FrameBuffer::getFrame(FrameFIFO *const dest) {
    if ((m_framesUsed < 1) && (dest->samplesLeftPut() < m_frameLength)) {
        return false;
    }

    dest->putSamples(m_frames[m_posGet], 0, m_frameLength);
    m_posGet = ((m_posGet + 1) % m_frameCount);

    m_framesUsed--;
    m_framesFree++;

    return true;
}


///////////////////////////////////////////////////////////////////////////////
// Constructor & Destructor
///////////////////////////////////////////////////////////////////////////////

MDynamicAudioNormalizer::MDynamicAudioNormalizer(const uint32_t channels, const uint32_t sampleRate,
                                                 const uint32_t frameLenMsec, const uint32_t filterSize,
                                                 const float peakValue, const float maxAmplification,
                                                 const float targetRms, const float compressThresh,
                                                 const bool channelsCoupled, const bool enableDCCorrection,
                                                 const bool altBoundaryMode)
        :
        p(new MDynamicAudioNormalizer_PrivateData(channels, sampleRate, frameLenMsec, filterSize, peakValue,
                                                  maxAmplification, targetRms, compressThresh, channelsCoupled,
                                                  enableDCCorrection, altBoundaryMode)) {
    /*nothing to do here*/
}

MDynamicAudioNormalizer_PrivateData::MDynamicAudioNormalizer_PrivateData(const uint32_t channels,
                                                                         const uint32_t sampleRate,
                                                                         const uint32_t frameLenMsec,
                                                                         const uint32_t filterSize,
                                                                         const float peakValue,
                                                                         const float maxAmplification,
                                                                         const float targetRms,
                                                                         const float compressFactor,
                                                                         const bool channelsCoupled,
                                                                         const bool enableDCCorrection,
                                                                         const bool altBoundaryMode)
        :
        m_channels(channels),
        m_sampleRate(sampleRate),
        m_frameLen(FRAME_SIZE(sampleRate, frameLenMsec)),
        m_filterSize(LIMIT(3u, filterSize, 301u)),
        m_prefillLen(m_filterSize / 2u),
        m_delay(m_frameLen * m_filterSize),
        m_peakValue(LIMIT(0.01f, peakValue, 1.0f)),
        m_maxAmplification(LIMIT(1.0f, maxAmplification, 100.0f)),
        m_targetRms(LIMIT(0.0f, targetRms, 1.0f)),
        m_compressFactor(compressFactor ? LIMIT(1.0f, compressFactor, 30.0f) : 0.0f),
        m_channelsCoupled(channelsCoupled),
        m_enableDCCorrection(enableDCCorrection),
        m_altBoundaryMode(altBoundaryMode) {
    printf("channels: %u\n", channels);
    printf("sampleRate: %u\n", sampleRate);
    printf("frameLenMsec: %u\n", frameLenMsec);
    printf("filterSize: %u\n", filterSize);
    printf("peakValue: %.4f\n", peakValue);
    printf("maxAmplification: %.4f\n", maxAmplification);
    printf("targetRms: %.4f\n", targetRms);
    printf("compressFactor: %.4f\n", compressFactor);
    printf("channelsCoupled: %s\n", channelsCoupled ? "YES" : "NO");
    printf("enableDCCorrection: %s\n", enableDCCorrection ? "YES" : "NO");
    printf("altBoundaryMode: %s\n", altBoundaryMode ? "YES" : "NO");

    m_initialized = false;
    m_flushBuffer = false;

    m_buffSrc = NULL;
    m_buffOut = NULL;

    m_delayedSamples = 0;
    m_sampleCounterClips = m_sampleCounterTotal = 0;

    m_frameBuffer = NULL;
    m_gaussianFilter = NULL;
    m_gainHistory_original = NULL;
    m_gainHistory_minimum = NULL;
    m_gainHistory_smoothed = NULL;

    m_loggingData_original = NULL;
    m_loggingData_minimum = NULL;
    m_loggingData_smoothed = NULL;

    m_prevAmplificationFactor = NULL;
    m_dcCorrectionValue = NULL;
    m_compressThreshold = NULL;

    m_fadeFactors[0] = m_fadeFactors[1] = NULL;
}

MDynamicAudioNormalizer::~MDynamicAudioNormalizer() {
    delete p;
}

MDynamicAudioNormalizer_PrivateData::~MDynamicAudioNormalizer_PrivateData() {

    MY_DELETE(m_buffSrc);
    MY_DELETE(m_buffOut);

    MY_DELETE(m_frameBuffer);
    MY_DELETE(m_gaussianFilter);
    MY_DELETE_ARRAY(m_gainHistory_original);
    MY_DELETE_ARRAY(m_gainHistory_minimum);
    MY_DELETE_ARRAY(m_gainHistory_smoothed);

    MY_DELETE_ARRAY(m_loggingData_original);
    MY_DELETE_ARRAY(m_loggingData_minimum);
    MY_DELETE_ARRAY(m_loggingData_smoothed);

    MY_DELETE_ARRAY(m_prevAmplificationFactor);
    MY_DELETE_ARRAY(m_dcCorrectionValue);
    MY_DELETE_ARRAY(m_compressThreshold);

    MY_DELETE_ARRAY(m_fadeFactors[0]);
    MY_DELETE_ARRAY(m_fadeFactors[1]);
}

///////////////////////////////////////////////////////////////////////////////
// Public API
///////////////////////////////////////////////////////////////////////////////

bool MDynamicAudioNormalizer::initialize() {
    try {
        return p->initialize();
    }
    catch (std::exception &e) {
        return false;
    }
}

bool MDynamicAudioNormalizer_PrivateData::initialize() {
    if (m_initialized) {
        fprintf(stderr, "Already initialized -> ignoring!");
        return true;
    }

    if ((m_channels < 1) || (m_channels > 8)) {
        fprintf(stderr, "Invalid or unsupported channel count. Should be in the %d to %d range!", 1, 8);
        return false;
    }
    if ((m_sampleRate < 11025) || (m_channels > 192000)) {
        fprintf(stderr, "Invalid or unsupported sampling rate. Should be in the %d to %d range!", 11025, 192000);
        return false;
    }
    if ((m_frameLen < 32) || (m_frameLen > 2097152)) {
        fprintf(stderr, "Invalid or unsupported frame size. Should be in the %d to %d range!", 32, 2097152);
        return false;
    }

    m_buffSrc = new FrameFIFO(m_channels, m_frameLen);
    m_buffOut = new FrameFIFO(m_channels, m_frameLen);

    m_frameBuffer = new FrameBuffer(m_channels, m_frameLen, m_filterSize + 1);

    m_gainHistory_original = new std::deque<float>[m_channels];
    m_gainHistory_minimum = new std::deque<float>[m_channels];
    m_gainHistory_smoothed = new std::deque<float>[m_channels];

    m_loggingData_original = new std::queue<float>[m_channels];
    m_loggingData_minimum = new std::queue<float>[m_channels];
    m_loggingData_smoothed = new std::queue<float>[m_channels];

    const float sigma = (((float(m_filterSize) / 2.0f) - 1.0f) / 3.0f) + (1.0f / 3.0f);

    m_gaussianFilter = new GaussianFilter(m_filterSize, sigma);

    m_dcCorrectionValue = new float[m_channels];
    m_prevAmplificationFactor = new float[m_channels];
    m_compressThreshold = new float[m_channels];

    m_fadeFactors[0] = new float[m_frameLen];
    m_fadeFactors[1] = new float[m_frameLen];

    precalculateFadeFactors(m_fadeFactors, m_frameLen);

    m_initialized = true;
    reset();
    return true;
}

bool MDynamicAudioNormalizer::reset() {
    try {
        return p->reset();
    }
    catch (std::exception &e) {
        return false;
    }
}

bool MDynamicAudioNormalizer_PrivateData::reset() {
    //Check audio normalizer status
    if (!m_initialized) {
        fprintf(stderr, "Not initialized yet. Must call initialize() first!");
        return false;
    }

    m_delayedSamples = 0;
    m_flushBuffer = false;

    m_buffSrc->reset();
    m_buffOut->reset();

    m_frameBuffer->reset();

    for (uint32_t c = 0; c < m_channels; c++) {
        m_gainHistory_original[c].clear();
        m_gainHistory_minimum[c].clear();
        m_gainHistory_smoothed[c].clear();

        CLEAR_QUEUE(m_loggingData_original[c]);
        CLEAR_QUEUE(m_loggingData_minimum[c]);
        CLEAR_QUEUE(m_loggingData_smoothed[c]);

        m_dcCorrectionValue[c] = 0.0;
        m_prevAmplificationFactor[c] = 1.0;
        m_compressThreshold[c] = 0.0;
    }

    return true;
}

bool MDynamicAudioNormalizer::getConfiguration(uint32_t &channels, uint32_t &sampleRate, uint32_t &frameLen,
                                               uint32_t &filterSize) {
    try {
        return p->getConfiguration(channels, sampleRate, frameLen, filterSize);
    }
    catch (std::exception &e) {
        return false;
    }
}

bool MDynamicAudioNormalizer::getInternalDelay(int64_t &delayInSamples) {
    try {
        return p->getInternalDelay(delayInSamples);
    }
    catch (std::exception &e) {
        return false;
    }
}

bool MDynamicAudioNormalizer_PrivateData::getConfiguration(uint32_t &channels, uint32_t &sampleRate, uint32_t &frameLen,
                                                           uint32_t &filterSize) {
    //Check audio normalizer status
    if (!m_initialized) {
        fprintf(stderr, "Not initialized yet. Must call initialize() first!");
        return false;
    }

    channels = m_channels;
    sampleRate = m_sampleRate;
    frameLen = m_frameLen;
    filterSize = m_filterSize;

    return true;
}

bool MDynamicAudioNormalizer_PrivateData::getInternalDelay(int64_t &delayInSamples) {
    //Check audio normalizer status
    if (!m_initialized) {
        fprintf(stderr, "Not initialized yet. Must call initialize() first!");
        return false;
    }

    delayInSamples = m_delay; //m_frameLen * m_filterSize;
    return true;
}

bool MDynamicAudioNormalizer::process(const float *const *const samplesIn, float *const *const samplesOut,
                                      const int64_t inputSize, int64_t &outputSize) {
    try {
        return p->process(samplesIn, samplesOut, inputSize, outputSize, false);
    }
    catch (std::exception &e) {
        return false;
    }
}

bool MDynamicAudioNormalizer::processInplace(float *const *const samplesInOut, const int64_t inputSize,
                                             int64_t &outputSize) {
    try {
        return p->process(samplesInOut, samplesInOut, inputSize, outputSize, false);
    }
    catch (std::exception &e) {
        return false;
    }
}

bool MDynamicAudioNormalizer_PrivateData::process(const float *const *const samplesIn, float *const *const samplesOut,
                                                  const int64_t inputSize, int64_t &outputSize, const bool &bFlush) {
    outputSize = 0;

    //Check audio normalizer status
    if (!m_initialized) {
        fprintf(stderr, "Not initialized yet. Must call initialize() first!");
        return false;
    }
    if (m_flushBuffer && (!bFlush)) {
        fprintf(stderr, "Must not call processInplace() after flushBuffer(). Call reset() first!");
        return false;
    }

    bool bStop = false;

    uint32_t inputPos = 0;
    uint32_t inputSamplesLeft = static_cast<uint32_t>(LIMIT(int64_t(0), inputSize, int64_t(UINT32_MAX)));

    uint32_t outputPos = 0;
    uint32_t outputBufferLeft = 0;

    while (!bStop) {
        bStop = true;

        //Read as many input samples as possible
        while ((inputSamplesLeft > 0) && (m_buffSrc->samplesLeftPut() > 0)) {
            bStop = false;

            const uint32_t copyLen = std::min(inputSamplesLeft, m_buffSrc->samplesLeftPut());
            m_buffSrc->putSamples(samplesIn, inputPos, copyLen);

            inputPos += copyLen;
            inputSamplesLeft -= copyLen;
            outputBufferLeft += copyLen;

            if (!bFlush) {
                m_delayedSamples += copyLen;
            }
        }

        //Analyze next input frame, if we have enough input
        if (m_buffSrc->samplesLeftGet() >= m_frameLen) {
            bStop = false;
            analyzeFrame(m_buffSrc->data());

            if (!m_frameBuffer->putFrame(m_buffSrc)) {
                fprintf(stderr, "Failed to append current input frame to internal buffer!");
                return false;
            }

            m_buffSrc->reset();
        }

        //Amplify next output frame, if we have enough output
        if ((m_buffOut->samplesLeftPut() >= m_frameLen) && (m_frameBuffer->framesUsed() > 0) &&
            (!m_gainHistory_smoothed[0].empty())) {
            bStop = false;

            if (!m_frameBuffer->getFrame(m_buffOut)) {
                fprintf(stderr, "Failed to retrieve next output frame from internal buffer!");
                return false;
            }

            amplifyFrame(m_buffOut->data());
        }

        //Write as many output samples as possible
        while ((outputBufferLeft > 0) && (m_buffOut->samplesLeftGet() > 0) &&
               (bFlush || (m_delayedSamples > m_delay))) {
            bStop = false;

            const uint32_t pending = bFlush ? UINT32_MAX : uint32_t(m_delayedSamples - m_delay);
            const uint32_t copyLen = std::min(std::min(outputBufferLeft, m_buffOut->samplesLeftGet()), pending);
            m_buffOut->getSamples(samplesOut, outputPos, copyLen);

            outputPos += copyLen;
            outputBufferLeft -= copyLen;
            m_delayedSamples -= copyLen;

            if ((m_buffOut->samplesLeftGet() < 1) && (m_buffOut->samplesLeftPut() < 1)) {
                m_buffOut->reset();
            }
        }
    }

    outputSize = int64_t(outputPos);

    if (inputSamplesLeft > 0) {
        fprintf(stderr, "No all input samples could be processed -> discarding pending input!");
        return false;
    }

    return true;
}

bool
MDynamicAudioNormalizer::flushBuffer(float *const *const samplesOut, const int64_t bufferSize, int64_t &outputSize) {
    try {
        return p->flushBuffer(samplesOut, bufferSize, outputSize);
    }
    catch (std::exception &e) {
        return false;
    }
}

bool MDynamicAudioNormalizer_PrivateData::flushBuffer(float *const *const samplesOut, const int64_t bufferSize,
                                                      int64_t &outputSize) {
    outputSize = 0;

    //Check audio normalizer status
    if (!m_initialized) {
        fprintf(stderr, "Not initialized yet. Must call initialize() first!");
        return false;
    }

    m_flushBuffer = true;
    const uint32_t pendingSamples = static_cast<uint32_t>(LIMIT(int64_t(0), std::min(m_delayedSamples, bufferSize),
                                                                int64_t(UINT32_MAX)));

    if (pendingSamples < 1) {
        return true; /*no pending samples left*/
    }

    bool success = false;
    do {
        for (uint32_t c = 0; c < m_channels; c++) {
            for (uint32_t i = 0; i < pendingSamples; i++) {
                samplesOut[c][i] = m_altBoundaryMode ? FLT_EPSILON : ((m_targetRms > FLT_EPSILON) ? std::min(
                        m_peakValue, m_targetRms) : m_peakValue);
                if (m_enableDCCorrection) {
                    samplesOut[c][i] *= ((i % 2) == 1) ? (-1) : 1;
                    samplesOut[c][i] += m_dcCorrectionValue[c];
                }
            }
        }

        success = process(samplesOut, samplesOut, pendingSamples, outputSize, true);
    } while (success && (outputSize <= 0));

    return success;
}


///////////////////////////////////////////////////////////////////////////////
// Procesing Functions
///////////////////////////////////////////////////////////////////////////////

void MDynamicAudioNormalizer_PrivateData::analyzeFrame(FrameData *const frame) {
    //Perform DC Correction (optional)
    if (m_enableDCCorrection) {
        perfromDCCorrection(frame, m_gainHistory_original[0].empty());
    }

    //Perform compression (optional)
    if (m_compressFactor > DBL_EPSILON) {
        perfromCompression(frame, m_gainHistory_original[0].empty());
    }

    //Find the frame's peak sample value
    if (m_channelsCoupled) {
        const float currentGainFactor = getMaxLocalGain(frame);
        for (uint32_t c = 0; c < m_channels; c++) {
            updateGainHistory(c, currentGainFactor);
        }
    } else {
        for (uint32_t c = 0; c < m_channels; c++) {
            updateGainHistory(c, getMaxLocalGain(frame, c));
        }
    }
}

void MDynamicAudioNormalizer_PrivateData::amplifyFrame(FrameData *const frame) {
    for (uint32_t c = 0; c < m_channels; c++) {
        if (m_gainHistory_smoothed[c].empty()) {
            fprintf(stderr, "There are no information available for the current frame!");
            break;
        }

        float *const dataPtr = frame->data(c);
        const float currAmplificationFactor = m_gainHistory_smoothed[c].front();
        m_gainHistory_smoothed[c].pop_front();

        for (uint32_t i = 0; i < m_frameLen; i++) {
            const float amplificationFactor = FADE(m_prevAmplificationFactor[c], currAmplificationFactor, i,
                                                   m_fadeFactors);
            dataPtr[i] *= amplificationFactor;

            if (fabs(dataPtr[i]) > m_peakValue) {
                m_sampleCounterClips++;
                dataPtr[i] = copysign(m_peakValue, dataPtr[i]); /*fix rare clipping*/
            }
        }

        m_prevAmplificationFactor[c] = currAmplificationFactor;
        m_sampleCounterTotal += m_frameLen;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

float
MDynamicAudioNormalizer_PrivateData::getMaxLocalGain(FrameData *const frame, const uint32_t channel) {
    const float maximumGain = m_peakValue / findPeakMagnitude(frame, channel);
    const float rmsGain = (m_targetRms > FLT_EPSILON) ? (m_targetRms / computeFrameRMS(frame, channel)) : FLT_MAX;
    return BOUND(m_maxAmplification, std::min(maximumGain, rmsGain));
}

float
MDynamicAudioNormalizer_PrivateData::findPeakMagnitude(FrameData *const frame, const uint32_t channel) {
    float dMax = FLT_EPSILON;

    if (channel == UINT32_MAX) {
        for (uint32_t c = 0; c < m_channels; c++) {
            float *const dataPtr = frame->data(c);
            for (uint32_t i = 0; i < m_frameLen; i++) {
                UPDATE_MAX(dMax, fabs(dataPtr[i]));
            }
        }
    } else {
        float *const dataPtr = frame->data(channel);
        for (uint32_t i = 0; i < m_frameLen; i++) {
            UPDATE_MAX(dMax, fabs(dataPtr[i]));
        }
    }

    return dMax;
}

float MDynamicAudioNormalizer_PrivateData::computeFrameRMS(const FrameData *const frame,
                                                           const uint32_t channel) {
    float rmsValue = 0.0;

    if (channel == UINT32_MAX) {
        for (uint32_t c = 0; c < m_channels; c++) {
            const float *dataPtr = frame->data(c);
            for (uint32_t i = 0; i < m_frameLen; i++) {
                rmsValue += POW2(dataPtr[i]);
            }
        }
        rmsValue /= float(m_frameLen * m_channels);
    } else {
        const float *dataPtr = frame->data(channel);
        for (uint32_t i = 0; i < m_frameLen; i++) {
            rmsValue += POW2(dataPtr[i]);
        }
        rmsValue /= float(m_frameLen);
    }

    return std::max(sqrtf(rmsValue), FLT_EPSILON);
}

float MDynamicAudioNormalizer_PrivateData::computeFrameStdDev(const FrameData *const frame,
                                                              const uint32_t channel) {
    float variance = 0.0;

    if (channel == UINT32_MAX) {
        for (uint32_t c = 0; c < m_channels; c++) {
            const float *dataPtr = frame->data(c);
            for (uint32_t i = 0; i < m_frameLen; i++) {
                variance += POW2(dataPtr[i]);    //Assume that MEAN is *zero*
            }
        }
        variance /= float((m_channels * m_frameLen) - 1);
    } else {
        const float *dataPtr = frame->data(channel);
        for (uint32_t i = 0; i < m_frameLen; i++) {
            variance += POW2(dataPtr[i]);    //Assume that MEAN is *zero*
        }
        variance /= float(m_frameLen - 1);
    }


    return std::max(sqrtf(variance), FLT_EPSILON);
}

void MDynamicAudioNormalizer_PrivateData::updateGainHistory(const uint32_t &channel, const float &currentGainFactor) {
    //Pre-fill the gain history
    if (m_gainHistory_original[channel].empty() || m_gainHistory_minimum[channel].empty()) {
        const float initial_value = m_altBoundaryMode ? currentGainFactor : 1.0f;
        m_prevAmplificationFactor[channel] = initial_value;
        while (m_gainHistory_original[channel].size() < m_prefillLen) {
            m_gainHistory_original[channel].push_back(initial_value);
        }
    }

    //Insert current gain factor
    m_gainHistory_original[channel].push_back(currentGainFactor);
    m_loggingData_original[channel].push(currentGainFactor);

    //Apply the minimum filter
    while (m_gainHistory_original[channel].size() >= m_filterSize) {
        assert(m_gainHistory_original[channel].size() == m_filterSize);
        if (m_gainHistory_minimum[channel].empty()) {
            float initial_value = m_altBoundaryMode ? m_gainHistory_original[channel].front() : 1.0f;
            std::deque<float>::const_iterator input = m_gainHistory_original[channel].begin() + m_prefillLen;
            while (m_gainHistory_minimum[channel].size() < m_prefillLen) {
                initial_value = std::min(initial_value, *(++input));
                m_gainHistory_minimum[channel].push_back(initial_value);
            }
        }
        const float minimum = *std::min_element(m_gainHistory_original[channel].begin(),
                                                m_gainHistory_original[channel].end());
        m_gainHistory_original[channel].pop_front();
        m_gainHistory_minimum[channel].push_back(minimum);
        m_loggingData_minimum[channel].push(minimum);
    }

    //Apply the Gaussian filter
    while (m_gainHistory_minimum[channel].size() >= m_filterSize) {
        assert(m_gainHistory_minimum[channel].size() == m_filterSize);
        const double smoothed = m_gaussianFilter->apply(m_gainHistory_minimum[channel]);
        m_gainHistory_minimum[channel].pop_front();
        m_gainHistory_smoothed[channel].push_back(smoothed);
        m_loggingData_smoothed[channel].push(smoothed);
    }
}

void MDynamicAudioNormalizer_PrivateData::perfromDCCorrection(FrameData *const frame,
                                                              const bool &isFirstFrame) {
    const float diff = 1.0f / float(m_frameLen);

    for (uint32_t c = 0; c < m_channels; c++) {
        float *const dataPtr = frame->data(c);
        float currentAverageValue = 0.0f;

        for (uint32_t i = 0; i < m_frameLen; i++) {
            currentAverageValue += (dataPtr[i] * diff);
        }

        const float prevValue = isFirstFrame ? currentAverageValue : m_dcCorrectionValue[c];
        m_dcCorrectionValue[c] = isFirstFrame ? currentAverageValue : UPDATE_VALUE(currentAverageValue,
                                                                                   m_dcCorrectionValue[c], 0.1);

        for (uint32_t i = 0; i < m_frameLen; i++) {
            dataPtr[i] -= FADE(prevValue, m_dcCorrectionValue[c], i, m_fadeFactors);
        }
    }
}

void MDynamicAudioNormalizer_PrivateData::perfromCompression(FrameData *const frame,
                                                             const bool &isFirstFrame) {
    if (m_channelsCoupled) {
        const float standardDeviation = computeFrameStdDev(frame);
        const float currentThreshold = std::min(1.0f, m_compressFactor * standardDeviation);

        const float prevValue = isFirstFrame ? currentThreshold : m_compressThreshold[0];
        m_compressThreshold[0] = isFirstFrame ? currentThreshold : UPDATE_VALUE(currentThreshold,
                                                                                m_compressThreshold[0], (1.0f / 3.0f));

        const float prevActualThresh = setupCompressThresh(prevValue);
        const float currActualThresh = setupCompressThresh(m_compressThreshold[0]);

        for (uint32_t c = 0; c < m_channels; c++) {
            float *const dataPtr = frame->data(c);
            for (uint32_t i = 0; i < m_frameLen; i++) {
                const float localThresh = FADE(prevActualThresh, currActualThresh, i, m_fadeFactors);
                dataPtr[i] = copysign(BOUND(localThresh, fabs(dataPtr[i])), dataPtr[i]);
            }
        }
    } else {
        for (uint32_t c = 0; c < m_channels; c++) {
            const float standardDeviation = computeFrameStdDev(frame, c);
            const float currentThreshold = setupCompressThresh(std::min(1.0f, m_compressFactor * standardDeviation));

            const float prevValue = isFirstFrame ? currentThreshold : m_compressThreshold[c];
            m_compressThreshold[c] = isFirstFrame ? currentThreshold : UPDATE_VALUE(currentThreshold,
                                                                                    m_compressThreshold[c],
                                                                                    (1.0f / 3.0f));

            const float prevActualThresh = setupCompressThresh(prevValue);
            const float currActualThresh = setupCompressThresh(m_compressThreshold[c]);

            float *const dataPtr = frame->data(c);
            for (uint32_t i = 0; i < m_frameLen; i++) {
                const float localThresh = FADE(prevActualThresh, currActualThresh, i, m_fadeFactors);
                dataPtr[i] = copysign(BOUND(localThresh, fabs(dataPtr[i])), dataPtr[i]);
            }
        }
    }
}


void MDynamicAudioNormalizer_PrivateData::printParameters() {
    printf("------- DynamicAudioNormalizer -------\n");
    printf("m_channels           : %u\n", m_channels);
    printf("m_sampleRate         : %u\n", m_sampleRate);
    printf("m_frameLen           : %u\n", m_frameLen);
    printf("m_filterSize         : %u\n", m_filterSize);
    printf("m_peakValue          : %.4f\n", m_peakValue);
    printf("m_maxAmplification   : %.4f\n", m_maxAmplification);
    printf("m_targetRms          : %.4f\n", m_targetRms);
    printf("m_compressFactor     : %.4f\n", m_compressFactor);
    printf("m_channelsCoupled    : %s\n", BOOLIFY(m_channelsCoupled));
    printf("m_enableDCCorrection : %s\n", BOOLIFY(m_enableDCCorrection));
    printf("m_altBoundaryMode    : %s\n", BOOLIFY(m_altBoundaryMode));
    printf("------- DynamicAudioNormalizer -------\n");
}

///////////////////////////////////////////////////////////////////////////////
// Static Utility Functions
///////////////////////////////////////////////////////////////////////////////

void
MDynamicAudioNormalizer_PrivateData::precalculateFadeFactors(float *const fadeFactors[2], const uint32_t frameLen) {
    assert((frameLen > 0) && ((frameLen % 2) == 0));
    const float dStepSize = 1.0f / float(frameLen);

    for (uint32_t pos = 0; pos < frameLen; pos++) {
        fadeFactors[0][pos] = 1.0f - (dStepSize * float(pos + 1U));
        fadeFactors[1][pos] = 1.0f - fadeFactors[0][pos];
    }
}

float MDynamicAudioNormalizer_PrivateData::setupCompressThresh(const float &dThreshold) {
    if ((dThreshold > DBL_EPSILON) && (dThreshold < (1.0 - DBL_EPSILON))) {
        float dCurrentThreshold = dThreshold;
        float dStepSize = 1.0;
        while (dStepSize > DBL_EPSILON) {
            while ((dCurrentThreshold + dStepSize > dCurrentThreshold) &&
                   (BOUND(dCurrentThreshold + dStepSize, 1.0) <= dThreshold)) {
                dCurrentThreshold += dStepSize;
            }
            dStepSize /= 2.0;
        }
        return dCurrentThreshold;
    } else {
        return dThreshold;
    }
}


static int
processingLoop(MDynamicAudioNormalizer *normalizer, const float *source, float *output, float **buffer,
               const uint32_t channels, const int64_t length, size_t frameSize) {
    int64_t remaining = length / channels;
    bool error = false;
    for (;;) {
        int64_t samplesRead = 0;
        if (remaining >= frameSize) {
            samplesRead = frameSize;
            for (int c = 0; c < channels; c++) {
                for (int k = 0; k < frameSize; k++)
                    buffer[c][k] = source[k * channels + c];
            }
            source += frameSize * channels;
        }

        if (samplesRead > 0) {
            if (length != INT64_MAX) {
                remaining -= samplesRead;
            }

            int64_t outputSize;
            if (!normalizer->processInplace(buffer, samplesRead, outputSize)) {
                error = true;
                break; /*internal error*/
            }

            if (outputSize > 0) {
                for (int c = 0; c < channels; c++) {
                    for (int k = 0; k < outputSize; k++)
                        output[k * channels + c] = buffer[c][k];
                }

                output += outputSize * channels;
            }
        }

        if (samplesRead < int64_t(frameSize)) {
            break; /*end of file*/
        }
    }
    //Flush all the delayed samples
    while (!error) {
        int64_t outputSize;
        if (!normalizer->flushBuffer(buffer, int64_t(frameSize), outputSize)) {
            error = true;
            break; /*internal error*/
        }

        if (outputSize > 0) {
            for (int c = 0; c < channels; c++) {
                for (int k = 0; k < outputSize; k++)
                    output[k * channels + c] = buffer[c][k];
            }
            output += outputSize * channels;

        } else {
            break; /*no more pending samples*/
        }
    }
    //Check remaining samples
    if (!error) {
        const int64_t samples_delta = (length != INT64_MAX) ? abs(remaining) : (-1);
        if (samples_delta > 0) {
            const float delta_fract = float(samples_delta) / float(length);
            if (!(error = (delta_fract >= 0.25))) {
            }
        }
    } else {
//Error checking
        fprintf(stderr, "I/O error encountered -> stopping!\n");
    }
    return error ? EXIT_FAILURE : EXIT_SUCCESS;
}


void wavWrite_f32(const char *filename, float *buffer, int sampleRate, uint32_t totalSampleCount, uint32_t channels) {
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_IEEE_FLOAT;
    format.channels = channels;
    format.sampleRate = (drwav_uint32) sampleRate;
    format.bitsPerSample = 32;

    drwav *pWav = drwav_open_file_write(filename, &format);
    if (pWav) {
        drwav_uint64 samplesWritten = drwav_write(pWav, totalSampleCount, buffer);
        drwav_uninit(pWav);
        if (samplesWritten != totalSampleCount) {
            fprintf(stderr, "write file [%s] error.\n", filename);
            exit(1);
        }
    }
}

float *wavRead_f32(const char *filename, uint32_t *sampleRate, uint64_t *sampleCount, uint32_t *channels) {
    drwav_uint64 totalSampleCount = 0;
    float *input = drwav_open_file_and_read_pcm_frames_f32(filename, channels, sampleRate, &totalSampleCount);
    if (input == NULL) {
        drmp3_config pConfig;
        input = drmp3_open_file_and_read_f32(filename, &pConfig, &totalSampleCount);
        if (input != NULL) {
            *channels = pConfig.outputChannels;
            *sampleRate = pConfig.outputSampleRate;
        }
    }
    if (input == NULL) {
        fprintf(stderr, "read file [%s] error.\n", filename);
        exit(1);
    }
    *sampleCount = totalSampleCount * (*channels);

    return input;
}

void splitpath(const char *path, char *drv, char *dir, char *name, char *ext) {
    const char *end;
    const char *p;
    const char *s;
    if (path[0] && path[1] == ':') {
        if (drv) {
            *drv++ = *path++;
            *drv++ = *path++;
            *drv = '\0';
        }
    } else if (drv)
        *drv = '\0';
    for (end = path; *end && *end != ':';)
        end++;
    for (p = end; p > path && *--p != '\\' && *p != '/';)
        if (*p == '.') {
            end = p;
            break;
        }
    if (ext)
        for (s = end; (*ext = *s++);)
            ext++;
    for (p = end; p > path;)
        if (*--p == '\\' || *p == '/') {
            p++;
            break;
        }
    if (name) {
        for (s = p; s < end;)
            *name++ = *s++;
        *name = '\0';
    }
    if (dir) {
        for (s = path; s < p;)
            *dir++ = *s++;
        *dir = '\0';
    }
}

int main(int argc, char *argv[]) {

    if (argc != 2 && argc != 3) {
        printf("usage :\n");
        printf(" DynamicAudioNormalizer input.wav\n");
        printf(" DynamicAudioNormalizer input.mp3\n");
        printf(" DynamicAudioNormalizer input.wav output.wav\n");
        printf(" DynamicAudioNormalizer input.mp3 output.wav\n");
        return -2;
    }
    static const size_t frameSize = 1024;

    const char *inFile = argv[1];
    const char *outFile = argv[2];
    uint32_t sampleRate = 0;
    uint64_t sampleCount = 0;
    uint32_t channels = 0;
    float *input = wavRead_f32(inFile, &sampleRate, &sampleCount, &channels);
    if (input == NULL) {
        printf("error: The file is not .wav format.\n");
        return -1;
    }
    float *output = new float[sampleCount * channels];

    DisplayInformation(sampleRate, sampleCount);

    //default Parameters
    int frameLenMsec = 500;
    int filterSize = 31;
    bool enableDCCorrection = false;
    bool altBoundaryMode = false;
    bool channelsCoupled = true;
    float peakValue = 0.95;
    float maxAmplification = 10.00;
    float targetRms = 0.00;
    float compressFactor = 0.00;

    //Create the normalizer instance
    MDynamicAudioNormalizer *normalizer = new MDynamicAudioNormalizer(
            channels,
            sampleRate,
            frameLenMsec,
            filterSize,
            peakValue,
            maxAmplification,
            targetRms,
            compressFactor,
            channelsCoupled,
            enableDCCorrection,
            altBoundaryMode
    );

    //Initialze normalizer
    if (!normalizer->initialize()) {

        MY_DELETE(normalizer);
        return EXIT_FAILURE;
    }
    //Allocate buffers
    float **buffer = new float *[channels];
    for (uint32_t channel = 0; channel < channels; channel++) {
        buffer[channel] = new float[frameSize];
    }
    //Run normalizer now!
    processingLoop(normalizer, input, output, buffer, channels, sampleCount, frameSize);

    //Destroy the normalizer
    MY_DELETE(normalizer);

    //Memory clean-up
    for (uint32_t channel = 0; channel < channels; channel++) {
        MY_DELETE_ARRAY(buffer[channel]);
    }
    MY_DELETE_ARRAY(buffer);
    char drive[3];
    char dir[256];
    char fname[256];
    char ext[256];
    char out_file[1024];
    splitpath(inFile, drive, dir, fname, ext);
    sprintf(out_file, "%s%s%s_out.wav", drive, dir, fname);
    if (outFile == NULL)
        wavWrite_f32(out_file, output, sampleRate, sampleCount, channels);
    else
        wavWrite_f32(outFile, output, sampleRate, sampleCount, channels);

    delete[] output;
    free(input);
    printf("complete.\n");
    return 0;
}
