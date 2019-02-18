% ![](img/dyauno/DynAudNorm.png)  
Dynamic Audio Normalizer
% by LoRd_MuldeR &lt;<mulder2@gmx>&gt; | <http://muldersoft.com/>

# Introduction #

**&#9733; *Free/libre software · This software is provided 100% free of charge · No adware or spyware!* &#9733;**

**Dynamic Audio Normalizer** is a library for *advanced* [audio normalization](http://en.wikipedia.org/wiki/Audio_normalization) purposes. It applies a certain amount of gain to the input audio in order to bring its peak magnitude to a target level (e.g. 0 dBFS). However, in contrast to more "simple" normalization algorithms, the Dynamic Audio Normalizer *dynamically* re-adjusts the gain factor to the input audio. This allows for applying extra gain to the "quiet" sections of the audio while avoiding distortions or clipping the "loud" sections. In other words: The Dynamic Audio Normalizer will "even out" the volume of quiet and loud sections, in the sense that the volume of each section is brought to the same target level. Note, however, that the Dynamic Audio Normalizer achieves this goal *without* applying "dynamic range compressing". It will retain 100% of the dynamic range *within* each "local" region of the audio file.

The *Dynamic Audio Normalizer* is available as a small standalone [command-line](http://en.wikipedia.org/wiki/Command-line_interface) utility and also as an effect in the [SoX](http://sox.sourceforge.net/) audio processor as well as in the [FFmpeg](https://ffmpeg.org/) audio/video converter. Furthermore, it can be integrated into your favourite DAW (digital audio workstation), as a [VST](http://de.wikipedia.org/wiki/Virtual_Studio_Technology) plug-in, or into your favourite media player, as a [Winamp](http://www.winamp.com/) plug-in. Last but not least, the "core" library can be integrated into custom applications easily, thanks to a straightforward [API](http://en.wikipedia.org/wiki/Application_programming_interface) (application programming interface). The "native" API is written in *C++*, but language [bindings](http://en.wikipedia.org/wiki/Language_binding) for *C99*, *Microsoft.NET*, *Java*, *Python* and *Pascal* are provided.


# How It Works #

A "standard" (non-dynamic) audio normalization algorithm applies the same *constant* amount of gain to *all* samples in the file. Consequently, the gain factor must be chosen in a way that won't cause clipping (distortion), even for the input sample that has the highest magnitude. So if `S_max` denotes the highest magnitude sample in the *whole* input audio and ``Peak`` is the desired peak magnitude, then the gain factor will be chosen as `G=Peak/abs(S_max)`. This works fine, as long as the volume of the input audio is constant, more or less. If, however, the volume of the input audio varies significantly over time – as is the case with many "real world" recordings – the standard normalization algorithm will *not* give satisfying result. That's because the "loud" parts can *not* be amplified any further (without distortions) and thus the "quiet" parts will remain quiet too.

**Dynamic Audio Normalizer** solves this problem by processing the input audio in small chunks, referred to as *frames*. A frame typically has a length 500 milliseconds, but the frame size can be adjusted as needed. The Dynamic Audio Normalizer then finds the highest magnitude sample *within* each frame, individually and independently. Next, it computes the maximum possible gain factor (without distortions) for each individual frame. So, if ``S_max[n]`` denotes the highest magnitude sample within the *n*-th frame, then the maximum possible gain factor for the *n*-th frame will be `G[n]=Peak/abs(S_max[n])`. Unfortunately, simply amplifying each frame with its own "local" maximum gain factor `G[n]` would **not** produce a satisfying overall result either. That's because the maximum gain factors can vary *strongly* and *unsteadily* between neighboring frames! Therefore, applying the maximum possible gain to each frame *without* taking neighboring frames into account would result in a strong *dynamic range compression* – which not only has a tendency to destroy the "vividness" of the audio but could also result in the "pumping" effect, i.e fast changes of the gain factor that become clearly noticeable to the listener.


## Dynamic Normalization Algorithm ##

The Dynamic Audio Normalizer tries to avoid these issues by applying an advanced *dynamic* normalization algorithm. Essentially, when processing a particular frame, it also takes into account a certain *neighborhood* around the current frame, i.e. the frames *preceding* and *succeeding* the current frame will be considered as well. However, while information about "past" frames can simply be stored as long as they are needed, information about "future" frames are *not* normally available. A simple approach to solve this issue would be using a *2-Pass* algorithm, but this would require processing the entire audio file *twice* – which also makes stream processing impossible. The Dynamic Audio Normalizer uses a different approach instead: It employs a tall "look ahead" buffer. This means that that all audio frames will progress trough an internal [*FIFO*](http://en.wikipedia.org/wiki/FIFO) (first in, first out) buffer. The size of this buffer is chosen sufficiently large, so that a frame's *complete* neighborhood, including the *subsequent* frames, will already be present in the buffer when *that* particular frame is being processed. The "look ahead" buffer eliminates the need for 2-Pass processing and thus gives an improved performance. It also makes stream processing possible.

With information about the frame's neighborhood available, a [*Gaussian* smoothing kernel](http://en.wikipedia.org/wiki/Gaussian_blur) can be applied on those gain factors. Put simply, this smoothing filter "mixes" the gain factor of the ``n``-th frames with those of its *preceding* frames (`n-1`, `n-2`, &hellip;) as well as with its *subsequent* frames (`n+1`, `n+2`, …) – where "nearby" frames have a stronger influence (weight), while "distant" frames have a declining influence. This way, abrupt changes of the gain factor are avoided and, instead, we get *smooth transitions* of the gain factor over time. Furthermore, since the filter also takes into account *future* frames, Dynamic Audio Normalizer avoids applying strong gain to "quiet" frames located shortly before "loud" frames. In other words, Dynamic Audio Normalizer adjusts the gain factor *early* and thus nicely prevents distortions or abrupt gain changes.

One more challenge to consider is that applying the Gaussian smoothing kernel alone can *not* solve all problems. That's because the smoothing kernel will *not* only smoothen *increasing* gain factors but also *declining* ones! If, for example, a very "loud" frame follows immediately after a sequence of "quiet" frames, the smoothing causes the gain factor to decrease early but slowly. As a result, the *filtered* gain factor of the "loud" frame could actually turn out to be *higher* than its (local) maximum gain factor – which results in distortion, if not taken care of! For this reason, the Dynamic Audio Normalizer *additionally* applies a "minimum" filter, i.e. a filter that replaces each gain factor with the *smallest* value within the neighborhood. This is done *before* the Gaussian smoothing kernel in order to ensure that all gain transitions will remain smooth.

The following example shows the results from a "real world" audio recording that has been processed by the Dynamic Audio Normalizer algorithm. The chart shows the maximum local gain factors for each individual frame (blue) as well as the minimum filtered gain factors (green) and the *final* smoothened gain factors (orange). Note the very "smooth" progression of the final gain factors. At the same time, the maximum local gain factors are approached as closely as possible. Last but not least note that the smoothened gain factors *never* exceed the maximum local gain factor, which prevents distortions.

![Progression of the gain factors for each audio frame](img/dyauno/Chart.png)


## Intra-Frame Normalization ##

So far it has been discussed how the "optimal" gain factor for each frame is determined. However, since each frame contains a large number of samples – at a typical sampling rate of 44,100 Hz and a standard frame size of 500 milliseconds we have 22,050 samples per frame – it is also required to infer the gain factor for each individual sample *within* the frame. The most simple approach, of course, would be applying the *same* gain factor to *all* samples in a certain frame. But this would also lead to abrupt changes of the gain factor at each frame boundary, while the gain factor remains completely constant within the frames. A better approach, as implemented in the Dynamic Audio Normalizer, is *interpolating* the per-sample gain factors. In particular, the Dynamic Audio Normalizer applies a straight-forward *linear interpolation*, which is used to compute the gain factors for the samples of the `n`-th frame from the gain factors `G'[n-1]`, `G'[n]` and `G'[n+1]` – where `G'[k]` denotes the gain factor of the `k`-th frame. The following graph shows how the per-sample gain factors (orange) are interpolated from the gain factors of the *preceding* (`G'[n-1]`, green), *current* (`G'[n]`, blue) and *subsequent* (`G'[n+1]`, purple) frame.

![Linear interpolation of the per-sample gain factors](img/dyauno/Interpolation.png)


## Real World Results (Example) ##

Finally, the following waveform view illustrates how the volume of a "real world" audio recording has been harmonized by the Dynamic Audio Normalizer. The upper graph shows the unprocessed original recording while the lower graph shows the output as created by the Dynamic Audio Normalizer. As can be seen, the significant volume variation between the "loud" and the "quiet" parts that existed in the original recording has been rectified – to a great extent – while the dynamics within each section of the input have been retained. Also, there is absolutely **no** clipping or distortion in the "loud" sections.

![Waveform *before* (top) and *after* (bottom) processing with the Dynamic Audio Normalizer](img/dyauno/Waveform.png)



# Download & Installation #

Dynamic Audio Normalizer can be downloaded from one of the following *official* mirror sites:
* https://github.com/lordmulder/DynamicAudioNormalizer/releases/latest
* https://bitbucket.org/muldersoft/dynamic-audio-normalizer/downloads
* https://sourceforge.net/projects/muldersoft/files/Dynamic%20Audio%20Normalizer/
* https://www.mediafire.com/folder/flrb14nitnh8i/Dynamic_Audio_Normalizer
* https://www.assembla.com/spaces/dynamicaudionormalizer/documents

**Note:** Windows binaries are provided in the compressed *ZIP* format. Simply use [7-Zip](http://www.7-zip.org/) or a similar tool to unzip *all* files to new/empty directory. If in doubt, Windows users should download the "static" version. That's it!

 
# Command-Line Usage #

The Dynamic Audio Normalizer *standalone* program can be invoked via [command-line interface](http://en.wikipedia.org/wiki/Command-line_interface) (CLI), which can be done either *manually* from the [command prompt](http://en.wikipedia.org/wiki/Command_Prompt) or *automatically* (scripted), e.g. by using a [batch](http://en.wikipedia.org/wiki/Batch_file) file.


## Basic Command-Line Syntax ##

The basic Dynamic Audio Normalizer command-line syntax is as follows:  
```
DynamicAudioNormalizer -i <;input_file> -o <output_file>
```
 

## Command-Line Options

The following Dynamic Audio Normalizer command-line options are available:  

* **Input/Output:**  
```
-i --input <file>        Input audio file [required] 
-o --output <file>       Output audio file [required] 
```
   
# Configuration #

This chapter describes the configuration options that can be used to tweak the behavior of the Dynamic Audio Normalizer.

While the default parameter of the Dynamic Audio Normalizer have been chosen to give satisfying results for a wide range of audio sources, it can be advantageous to adapt the parameters to the individual audio file as well as to your personal preferences. The *default* settings have been chosen rather "conservative", so that the dynamics of the audio are preserved well. People who desire a more "aggressive" effect should try using a *smaller* [*filter size*](#gaussian-filter-window-size), or enable [*input compression*](#input-compression).


## Gaussian Filter Window Size ##

Probably the most important parameter of the Dynamic Audio Normalizer is the "window size" of the Gaussian smoothing filter. It can be controlled with the **``--gauss-size``** option. The filter's window size is specified in *frames*, centered around the current frame. For the sake of simplicity, this must be an *odd* number. Consequently, the default value of **31** takes into account the current frame, as well as the *15* preceding frames and the *15* subsequent frames. Using a *larger* Gaussian window size results in a *stronger* smoothing effect and thus in *less* gain variation, i.e. slower gain adaptation. Conversely, using a *smaller* Gaussian window size results in a *weaker* smoothing effect and thus in *more* gain variation, i.e. faster gain adaptation. In other words, the more you *increase* this value, the more the Dynamic Audio Normalizer will behave like a "traditional" (non-dynamic) normalization filter. On the contrary, the more you *decrease* this value, the more "aggressively" the Dynamic Audio Normalizer will behave, i.e. more like a dynamic range *compressor*. The following graph illustrates the effect of different Gaussian window sizes – *11* (orange), *31* (green), and *61* (purple) frames – on the progression of the final filtered gain factor.

![The effect of different "window sizes" of the Gaussian smoothing filter](img/dyauno/FilterSize.png)


## Target Peak Magnitude ##

The target peak magnitude specifies the highest permissible magnitude level for the *normalized* audio file. It is controlled by the **``--peak``** option. Since the Dynamic Audio Normalizer represents audio samples as floating point values in the *-1.0* to *1.0* range – regardless of the input and output audio format – this value must be in the *0.0* to *1.0* range. Consequently, the value *1.0* is equal to [0 dBFS](http://en.wikipedia.org/wiki/DBFS), i.e. the maximum possible digital signal level (± 32767 in a 16-Bit file). The Dynamic Audio Normalizer will try to approach the target peak magnitude as closely as possible, but at the same time it also makes sure that the normalized signal will *never* exceed the peak magnitude. A frame's maximum local gain factor is imposed directly by the target peak magnitude. The default value is **0.95** and thus leaves a [headroom](http://en.wikipedia.org/wiki/Headroom_%28audio_signal_processing%29) of *5%*. It is **not** recommended to go *above* this value!


## Channel Coupling ##

By default, the Dynamic Audio Normalizer will amplify all channels by the same amount. This means the *same* gain factor will be applied to *all* channels, i.e. the maximum possible gain factor is determined by the "loudest" channel. In particular, the highest magnitude sample for the `n`-th frame is defined as `S_max[n]=Max(s_max[n][1],s_max[n][2],…,s_max[n][C])`, where `s_max[n][k]` denotes the highest magnitude sample in the `k`-th channel and `C` is the channel count. The gain factor for *all* channels is then derived from `S_max[n]`. This is referred to as *channel coupling* and for most audio files it gives the desired result. Therefore, channel coupling is *enabled* by default. However, in some recordings, it may happen that the volume of the different channels is *uneven*, e.g. one channel may be "quieter" than the other one(s). In this case, the **`--no-coupling`** option can be used to *disable* the channel coupling. This way, the gain factor will be determined *independently* for each channel `k`, depending only on the individual channel's highest magnitude sample `s_max[n][k]`. This allows for harmonizing the volume of the different channels. The following wave view illustrates the effect of channel coupling: It shows an input file with *uneven* channel volumes (left), the same file after normalization with channel coupling *enabled* (center) and again after normalization with channel coupling *disabled* (right).

![The effect of *channel coupling*](img/dyauno/Coupling.png)


## DC Bias Correction##

An audio signal (in the time domain) is a sequence of sample values. In the Dynamic Audio Normalizer these sample values are represented in the *-1.0* to *1.0* range, regardless of the original input format. Normally, the audio signal, or "waveform", should be centered around the *zero point*. That means if we calculate the *mean* value of all samples in a file, or in a single frame, then the result should be *0.0* or at least very close to that value. If, however, there is a significant deviation of the mean value from *0.0*, in either positive or negative direction, this is referred to as a [*DC bias*](http://en.wikipedia.org/wiki/DC_bias) or *DC offset*. Since a DC bias is clearly undesirable, the Dynamic Audio Normalizer provides optional *DC bias correction*, which can be enabled using the **``--correct-dc``** switch. With DC bias correction *enabled*, the Dynamic Audio Normalizer will determine the mean value, or "DC correction" offset, of each input frame and *subtract* that value from all of the frame's sample values – which ensures those samples are centered around *0.0* again. Also, in order to avoid "gaps" at the frame boundaries, the DC correction offset values will be interpolated smoothly between neighbouring frames. The following wave view illustrates the effect of DC bias correction: It shows an input file with positive DC bias (left), the same file after normalization with DC bias correction *disabled* (center) and again after normalization with DC bias correction *enabled* (right).

![The effect of *DC Bias Correction*](img/dyauno/DCCorrection.png)


## Maximum Gain Factor ##

The Dynamic Audio Normalizer determines the maximum possible (local) gain factor for each input frame, i.e. the maximum gain factor that does *not* result in clipping or distortion. The maximum gain factor is determined by the frame's highest magnitude sample. However, the Dynamic Audio Normalizer *additionally* bounds the frame's maximum gain factor by a predetermined (global) *maximum gain factor*. This is done in order to avoid "extreme" gain factors in silent, or almost silent, frames. By default, the *maximum gain factor* is **10.0**, but it can be adjusted using the **``--max-gain``** switch. For most input files the default value should be sufficient, and it usually is **not** recommended to increase this value above the default. Though, for input files with a rather low overall volume level, it may be necessary to allow even higher gain factors. Be aware, however, that the Dynamic Audio Normalizer does *not* simply apply a "hard" threshold (i.e. it does *not* simply cut off values above the threshold). Instead, a "sigmoid" thresholding function will be applied on the final gain factors, as depicted in the following chart. This way, the gain factors will smoothly approach the threshold value, but they will *never* exceed that value.

![The Gain Factor Threshold-Function](img/dyauno/Threshold.png)


## Target RMS Value ##

By default, the Dynamic Audio Normalizer performs "peak" normalization. This means that the maximum local gain factor for each frame is defined (only) by the frame's highest magnitude sample (the "loudest" peak). This way, the samples can be amplified as much as possible *without* exceeding the maximum signal level, i.e. *without* clipping. Optionally, however, the Dynamic Audio Normalizer can also take into account the frame's [*root mean square*](http://en.wikipedia.org/wiki/Root_mean_square), or, in short, **RMS**. In electrical engineering, the RMS is commonly used to determine the *power* of a time-varying signal. It is therefore considered that the RMS is a better approximation of the "perceived loudness" than just looking at the signal's peak magnitude. Consequently, by adjusting all frames to a *constant* RMS value, a *uniform* "perceived loudness" can be established. With the Dynamic Audio Normalizer, RMS processing can be enabled using the **``--target-rms``** switch. This specifies the desired RMS value, in the *0.0* to *1.0* range. There is **no** default value, because RMS processing is *disabled* by default. If a target RMS value has been specified, a frame's local gain factor is defined as the factor that would result in exactly *that* RMS value. Note, however, that the maximum local gain factor is still restricted by the frame's highest magnitude sample, in order to prevent clipping. The following chart shows the same file normalized *without* (green) and *with* (orange) RMS processing enabled.

![Root Mean Square (RMS) processing example](img/dyauno/RMS.png)


## Frame Length ##

The Dynamic Audio Normalizer processes the input audio in small chunks, referred to as *frames*. This is required, because a *peak magnitude* has no meaning for just a single sample value. Instead, we need to determine the peak magnitude for a contiguous sequence of sample values. While a "standard" normalizer would simply use the peak magnitude of the *complete* file, the Dynamic Audio Normalizer determines the peak magnitude *individually* for each frame. The length of a frame is specified in milliseconds. By default, the Dynamic Audio Normalizer uses a frame length of **500** milliseconds, which has been found to give good results with most files, but it can be adjusted using the **``--frame-len``** switch. Note that the exact frame length, in number of samples, will be determined automatically, based on the sampling rate of the individual input audio file.


## Boundary Mode ##

As explained before, the Dynamic Audio Normalizer takes into account a certain neighbourhood around each frame. This includes the *preceding* frames as well as the *subsequent* frames. However, for the "boundary" frames, located at the very beginning and at the very end of the audio file, **not** all neighbouring frames are available. In particular, for the *first* few frames in the audio file, the preceding frames are *not* known. And, similarly, for the *last* few frames in the audio file, the subsequent frames are *not* known. Thus, the question arises which gain factors should be assumed for the *missing* frames in the "boundary" region. The Dynamic Audio Normalizer implements two modes to deal with this situation. The *default* boundary mode assumes a gain factor of exactly *1.0* for the missing frames, resulting in a smooth "fade in" and "fade out" at the beginning and at the end of the file, respectively. The *alternative* boundary mode can be enabled by using the **``--alt-boundary``** switch. The latter mode assumes that the missing frames at the *beginning* of the file have the same gain factor as the very *first* available frame. It furthermore assumes that the missing frames at the *end* of the file have same gain factor as the very *last* frame. The following chart illustrates the difference between the *default* (green) and the *alternative* (red) boundary mode. Note hat, for simplicity's sake, a file containing *constant* volume white noise was used as input here.

![Default boundary mode vs. alternative boundary mode](img/dyauno/Boundary.png)


## Input Compression ##

By default, the Dynamic Audio Normalizer does **not** apply "traditional" compression. This means that signal peaks will **not** be pruned and thus the *full* dynamic range will be retained within each local neighbourhood. However, in some cases it may be desirable to *combine* the Dynamic Audio Normalizer's normalization algorithm with a more "traditional" compression. For this purpose, the Dynamic Audio Normalizer provides an *optional* compression (thresholding) function. It is disabled by default and can be enabled by using the **``--compress``** switch. If (and only if) the compression feature is enabled, all input frames will be processed by a [*soft knee*](https://mdn.mozillademos.org/files/5113/WebAudioKnee.png) thresholding function *prior to* the actual normalization process. Put simply, the thresholding function is going to prune all samples whose magnitude exceeds a certain threshold value. However, the Dynamic Audio Normalizer does *not* simply apply a fixed threshold value. Instead, the threshold value will be adjusted for each individual frame. More specifically, the threshold for a specific input frame is defined as ``T = μ + (c · σ)``, where **μ** is the *mean* of all sample magnitudes in the current frame, **σ** is the *standard deviation* of those sample magnitudes and **c** is the parameter controlled by the user. Note that, assuming the samples magnitudes are following (roughly) a [normal distribution](http://content.answcdn.com/main/content/img/barrons/accounting/New/images/normaldistribution2.jpg), about 68% of the sample values will be within the **μ ± σ** range, about 95% of the sample values will be within the **μ ± 2σ** range and more than 99% of the sample values will be within the **μ ± 3σ** range. Consequently, a parameter of **c = 2** will prune about 5% of the frame's highest magnitude samples, a parameter of **c = 3** will prune about 1% of the frame's highest magnitude samples, and so on. In general, *smaller* parameters result in *stronger* compression, and vice versa. Values below *3.0* are **not** recommended, because audible distortion may appear! The following waveform view illustrates the effect of the input compression feature: It shows the same input file *before* (upper view) and *after* (lower view) the thresholding function has been applied. Please note that, for the sake of clarity, the actual normalization process has been *disabled* in the following chart. Under normal circumstances, the normalization process is applied immediately after the thresholding function.

![The effect of the input compression (thresholding) function](img/dyauno/Compression-1.png)
 
# API Documentation #

This chapter describes the **MDynamicAudioNormalizer** application programming interface (API). The API specified in this document allows software developer to call the *Dynamic Audio Normalizer* library from their own programs.

 

## Quick Start Guide ##

The following listing summarizes the steps that an application needs to follow when using the API:

1. Create a new *MDynamicAudioNormalizer* instance. This allocates required resources.
2. Call the ``initialize()`` method, *once*, in order to initialize the MDynamicAudioNormalizer instance.
3. Call the ``processInplace()`` method, *in a loop*, until all input samples have been processed.  
    - **Note:** At the beginning, this function returns *less* output samples than input samples have been passed. Samples are guaranteed to be returned in FIFO order, but there is a certain "delay"; call `getInternalDelay()` for details.
4. Call the ``flushBuffer()`` method, *in a loop*, until all the pending "delayed" output samples have been flushed.
5. Destroy the *MDynamicAudioNormalizer* instance. This will free up all allocated resources.

*Note:* The Dynamic Audio Normalizer library processes audio samples, but it does **not** provide any audio I/O capabilities. Reading or writing the audio samples from or to an audio file is up to the application. A library like [libsndfile](http://www.mega-nerd.com/libsndfile/) is helpful!


## Function Reference ##

### MDynamicAudioNormalizer::MDynamicAudioNormalizer() ### {-}
```
MDynamicAudioNormalizer(
	const uint32_t channels,
	const uint32_t sampleRate,
	const uint32_t frameLenMsec,
	const uint32_t filterSize,
	const double peakValue,
	const double maxAmplification,
	const double targetRms,
	const bool channelsCoupled,
	const bool enableDCCorrection,
	const bool altBoundaryMode
);
```

Constructor. Creates a new *MDynamicAudioNormalizer* instance and sets up the normalization parameters.

**Parameters:**
* *channels*: The number of channels in the input/output audio stream (e.g. **2** for Stereo).
* *sampleRate*: The sampling rate of the input/output audio stream, in Hertz (e.g. **44100** for "CD Quality").
* *frameLenMsec*: The [frame length](#frame-length), in milliseconds. A typical value is **500** milliseconds.
* *filterSize*: The ["window size"](#gaussian-filter-window-size) of the Gaussian filter, in frames. Must be an *odd* number. (default: **31**).
* *peakValue*: Specifies the [peak magnitude](#target-peak-magnitude) for normalized audio, in the **0.0** to **1.0** range (default: **0.95**).
* *maxAmplification*: Specifies the [maximum gain factor](#maximum-gain-factor). Must be greater than **1.0** (default: **10.0**).
* *targetRms*: Specifies the [target RMS](#target-rms-value) value. Must be in the **0.0** to **1.0** range, **0.0** means *disabled* (default: **0.0**).
* *channelsCoupled*: Set to **true** in order to enable [channel coupling](#channel-coupling), or to **false** otherwise (default: **true**).
* *enableDCCorrection*: Set to **true** in order to enable [DC correction](#dc-bias-correction), or to **false** otherwise (default: **false**).
* *altBoundaryMode*: Set to **true** in order to enable the alternative [boundary mode](#boundary-mode) (default: **false**).

### MDynamicAudioNormalizer::~MDynamicAudioNormalizer() ### {-}
```
virtual ~MDynamicAudioNormalizer(void);
```

Destructor. Destroys the *MDynamicAudioNormalizer* instance and releases all memory that it occupied.


### MDynamicAudioNormalizer::initialize() ### {-}
```
bool initialize(void);
```

Initializes the MDynamicAudioNormalizer instance. Validates the parameters and allocates the required memory buffers.

This function *must* be called once for each new MDynamicAudioNormalizer instance. It *must* be called *before the `processInplace()` function can be called for the first time. Do **not** call `processInplace()`, if this function has failed!

**Return value:**
* Returns `true` if everything was successful or `false` if something went wrong.


### MDynamicAudioNormalizer::process() ### {-}
```
bool process(
	const double **samplesIn,
	double **samplesOut,
	int64_t inputSize,
	int64_t &outputSize
);
```

This is the "main" processing function. It shall be called *in a loop* until all input audio samples have been processed.

The function works "out-of-place": It *reads* the original input samples from the specified `samplesIn` buffer and then *writes* the normalized output samples, if any, back into the `samplesOut` buffer. The content of `samplesIn` will be preserved.

***Note:*** A call to this function reads *all* provided input samples from the buffer, but the number of output samples that are written back to the buffer may actually be *smaller* than the number of input samples that have been read! The pending samples are buffered internally and will be returned in a subsequent call. In other words, samples are returned with a certain "delay". This means that the *i*-th output sample does **not** necessarily correspond to the *i*-th input sample! Still, the samples are returned in strict [FIFO](https://en.wikipedia.org/wiki/FIFO_(computing_and_electronics)) order. The exact delay can be determined by calling the `getInternalDelay()` function. At the end of the process, when all input samples have been read, to application shall call `flushBuffer()` in order to *flush* all pending samples.

**Parameters:**
* *samplesIn*: The input buffer. This buffer contains the original input samples to be read. It will be treated as a *read-only* buffer. The *i*-th input sample for the *c*-th channel is assumed to be stored at `samplesIn[c][i]`, as a double-precision floating point number. All indices are *zero*-based. All sample values live in the **-1.0** to **+1.0** range.
* *samplesOut*: The output buffer. The processed output samples will be written back to this buffer. Its initial contents will therefore be *overwritten*. The *i*-th output sample for the *c*-th channel will be stored at `samplesOut[c][i]`. The size of the buffer must be sufficient! All indices are *zero*-based. All sample values live in the **-1.0** to **+1.0** range.
* *inputSize*: The number of original *input* samples that are available in the `samplesIn` buffer, per channel. This also specifies the *maximum* number of output samples that can be written back to the `samplesOut` buffer.
* *outputSize*: Receives the number of *output* samples that have actually been written back to the `samplesOut` buffer, per channel. Please note that this value can be *smaller* than the `inputSize` value. It can even be *zero*!

**Return value:**
* Returns `true` if everything was successful or `false` if something went wrong.


### MDynamicAudioNormalizer::processInplace() ### {-}
```
bool processInplace(
	double **samplesInOut,
	int64_t inputSize,
	int64_t &outputSize
);
```

This is the "main" processing function. It shall be called *in a loop* until all input audio samples have been processed.

The function works "in-place": It *reads* the original input samples from the specified buffer and then *writes* the normalized output samples, if any, back into the *same* buffer. So, the content of `samplesInOut` will **not** be preserved!

***Note:*** A call to this function reads *all* provided input samples from the buffer, but the number of output samples that are written back to the buffer may actually be *smaller* than the number of input samples that have been read! The pending samples are buffered internally and will be returned in a subsequent call. In other words, samples are returned with a certain "delay". This means that the *i*-th output sample does **not** necessarily correspond to the *i*-th input sample! Still, the samples are returned in strict [FIFO](https://en.wikipedia.org/wiki/FIFO_(computing_and_electronics)) order. The exact delay can be determined by calling the `getInternalDelay()` function. At the end of the process, when all input samples have been read, to application shall call `flushBuffer()` in order to *flush* all pending samples.

**Parameters:**
* *samplesInOut*: The input/output buffer. This buffer initially contains the original input samples to be read. Also, the processed output samples will be written back to this buffer. The *i*-th input sample for the *c*-th channel is assumed to be stored at `samplesInOut[c][i]`, as a double-precision floating point number. The output samples, if any, will be stored at the corresponding locations, thus *overwriting* the input data. Consequently, the *i*-th output sample for the *c*-th channel will be stored at `samplesInOut[c][i]`. All indices are *zero*-based. All sample values live in the **-1.0** to **+1.0** range.
* *inputSize*: The number of original *input* samples that are available in the `samplesInOut` buffer, per channel. This also specifies the *maximum* number of output samples that can be written back to the buffer.
* *outputSize*: Receives the number of *output* samples that have actually been written back to the `samplesInOut` buffer, per channel. Please note that this value can be *smaller* than the `inputSize` value. It can even be *zero*!

**Return value:**
* Returns `true` if everything was successful or `false` if something went wrong.


### MDynamicAudioNormalizer::flushBuffer() ### {-}
```
bool flushBuffer(
	double **samplesOut,
	const int64_t bufferSize,
	int64_t &outputSize
);
```

This function shall be called at the end of the process, after all input samples have been processed via `processInplace()` function, in order to flush the *pending* samples from the internal buffer. It writes the next pending output samples into the output buffer, in FIFO order, if and only if there are still any pending output samples left in the internal buffer. Once this function has been called, you must call `reset()` before the `processInplace()` function may be called again! If this function returns fewer output samples than the specified output buffer size, then this indicates that the internal buffer is empty now.

**Parameters:**
* *samplesOut*: The output buffer that will receive the normalized output samples. The *i*-th output sample for the *c*-th channel will be stored at `samplesOut[c][i]`. All indices are *zero*-based. All sample values live in the **-1.0** to **+1.0** range.
* *bufferSize*: Specifies the *maximum* number of output samples to be stored in the output buffer.
* *outputSize*: Receives the number of *output* samples that have actually been written to the `samplesOut` buffer. Please note that this value can be *smaller* than `bufferSize` size, if no more pending samples are left. It can even be *zero*!

**Return value:**
* Returns ``true`` if everything was successfull or ``false`` if something went wrong.


### MDynamicAudioNormalizer::reset() ### {-}
```
bool reset(void);
```

Resets the internal state of the *MDynamicAudioNormalizer* instance. It normally is **not** required or recommended to call this function! The only exception here is: If you really want to process *multiple* independent audio files with the *same* normalizer instance, then you *must* call the `reset()` function *after* all samples of the **current** audio file have been processed (and all of its pending samples have been flushed), but *before* processing the first sample of the **next** audio file.

**Return value:**
* Returns ``true`` if everything was successfull or ``false`` if something went wrong.


### MDynamicAudioNormalizer::getConfiguration() ### {-}
```
bool getConfiguration(
	uint32_t &channels,
	uint32_t &sampleRate,
	uint32_t &frameLen,
	uint32_t &filterSize
);
```

This is a convenience function to retrieve the current configuration of an existing *MDynamicAudioNormalizer* instance.

**Parameters:**
* *channels*: Receives the number of channels that was set in the constructor.
* *sampleRate*: Receives the sampling rate, in Hertz, that was set in the constructor.
* *frameLen*: Receives the current frame length, in samples (**not** milliseconds).
* *filterSize*: Receives the Gaussian filter size, in frames, that was set in the constructor.

**Return value:**
* Returns ``true`` if everything was successfull or ``false`` if something went wrong.


### MDynamicAudioNormalizer::getInternalDelay() ### {-}
```
bool getInternalDelay(
	int64_t &delayInSamples,
);
```

This function can be used to determine the internal "delay" of the *MDynamicAudioNormalizer* instance. This is the (maximum) number of samples, per channel, that will be buffered internally. The `processInplace()` function will **not** return any output samples, until (at least) *this* number of input samples have been read. This also specifies the (maximum) number of samples, per channel, that need to be flushed from the internal buffer, at the end of the process, using the `flushBuffer()` function.

**Parameters:**
* *delayInSamples*: Receives the size of the internal buffer, in samples (per channel).

**Return value:**
* Returns ``true`` if everything was successful or ``false`` if something went wrong.


# Source Code #

The source code of the Dynamic Audio Normalizer is available from one of the official [**Git**](http://git-scm.com/) repository mirrors:
* ``https://github.com/lordmulder/DynamicAudioNormalizer.git`` &nbsp; ([Browse](https://github.com/lordmulder/DynamicAudioNormalizer))
* ``https://bitbucket.org/muldersoft/dynamic-audio-normalizer.git`` &nbsp; ([Browse](https://bitbucket.org/muldersoft/dynamic-audio-normalizer/overview))
* ``https://gitlab.com/dynamic-audio-normalizer/dynamic-audio-normalizer.git`` &nbsp; ([Browse](https://gitlab.com/dynamic-audio-normalizer/dynamic-audio-normalizer/tree/master))
* ``https://git.assembla.com/dynamicaudionormalizer.git`` &nbsp; ([Browse](https://www.assembla.com/code/dynamicaudionormalizer/git/nodes))
* ``https://muldersoft.codebasehq.com/dynamicaudionormalizer/dynamicaudionormalizer.git`` &nbsp; ([Browse](https://muldersoft.codebasehq.com/changelog/dynamicaudionormalizer/dynamicaudionormalizer))
* ``https://repo.or.cz/DynamicAudioNormalizer.git`` &nbsp; ([Browse](http://repo.or.cz/w/DynamicAudioNormalizer.git))

# Changelog #

## Version 2.11 (2019-01-03) ## {-}
* Core library: Fixed a potential crash due to dereferencing a possibly invalidated iterator
* Core library: Use C++11 `std::mutex`, if supported → removes the dependency PThread library
* CLI front-end: Added support for decoding Opus input files via *libopusfile* library
* CLI front-end: Added new CLI option `--output-bps` to specify the desired *output* bit-depth
* Winamp plug-in: Some fixes and improvements; removed old workarounds
* Windows binaries: Updated the included libsndfile version to 1.0.28 (2017-04-02)
* Windows binaries: Updated build environment to Visual Studio 2017.9 (MSVC 14.16)

## Version 2.10 (2017-04-14) ## {-}
* Core library: Added `process()` function, i.e. an "out-of-place" version of `processInplace()`
* Implemented [Python](https://www.python.org/) API → Dynamic Audio Normalizer can be used in, e .g., *Python*-based applications
* CLI front-end: Added new CLI option `-t` to explicitly specify the desired *output* format
* CLI front-end: Added new CLI option `-d` to explicitly specify the desired *input* library
* CLI front-end: Added support for decoding input files via *libmpg123* library
* CLI front-end: Implemented automatic/heuristic selection of the suitable *input* library
* CLI front-end: Properly handle input files that provide more (or less) samples than what was projected
* Windows binaries: Updated the included *libsndfile* version to 1.0.27 (2016-06-19)
* Windows binaries: Updated build environment to Visual Studio 2015 (MSVC 14.0)

## Version 2.09 (2016-08-01) ## {-}
* Core library: Improved pre-filling code in order to avoid possible clipping at the very beginning

## Version 2.08 (2015-01-20) ## {-}
* CLI front-end: Very short files (shorter than Gaussian window size) are now handled properly
* Core library: Fixed case when ``flushBuffer()`` is called *before* internal buffer is filled entirely
* Core library: Workaround for the [*FMA3 bug*](https://connect.microsoft.com/VisualStudio/feedback/details/987093/x64-log-function-uses-vpsrlq-avx-instruction-without-regard-to-operating-system-so-it-crashes-on-vista-x64) in the Microsoft Visual C++ 2013 runtime libraries
* Makefile: Various improvements

## Version 2.07 (2014-11-01) ## {-}
* Implemented [.NET](http://en.wikipedia.org/wiki/.NET_Framework) API → Dynamic Audio Normalizer can be used in, e .g., *C#*-based applications
* Implemented [JNI](http://en.wikipedia.org/wiki/Java_Native_Interface) API → Dynamic Audio Normalizer can be used in *Java*-based  applications
* Implemented [Pascal](http://en.wikipedia.org/wiki/Object_Pascal) API → Dynamic Audio Normalizer can be used in *Pascal*-based applications
* Core library: Added new ``getConfiguration()`` API to retrieve the *active* configuration params
* Core library: Fixed a bug that caused the gain factors to *not* progress as "smoothly" as intended

## Version 2.06 (2014-09-22) ## {-}
* Implemented [Winamp](http://www.winamp.com/) wrapper → Dynamic Audio Normalizer can now be used as Winamp plug-in
* VST wrapper: Fixed potential audio corruptions due to the occasional insertion of "silent" samples
* VST wrapper: Fixed a potential "double free" crash in the VST wrapper code
* Core library: Fixed `reset()` API to actually work as expected (some state was *not* cleared before)
* Core library: Make sure the number of delayed samples remains *constant* throughout the process

## Version 2.05 (2014-09-10) ## {-}
* Significant overhaul of the *compression* (thresholding) function
* Implemented [VST](http://en.wikipedia.org/wiki/Virtual_Studio_Technology) wrapper → Dynamic Audio Normalizer can now be integrated in any VST host
* Added *64-Bit* library and VST plug-in binaries to the Windows release packages
* No longer use ``__declspec(thread)``, because it can crash libraries on Windows XP ([details](http://www.nynaeve.net/?p=187))

## Version 2.04 (2014-08-25) ## {-}
* Added an optional input *compression* (thresholding) function
* Implemented [SoX](http://sox.sourceforge.net/) wrapper → Dynamic Audio Normalizer can now be used as a SoX effect
* Improved internal handling of "raw" PCM data

## Version 2.03 (2014-08-11) ## {-}
* Implemented an *optional* RMS-based normalization mode
* Added support for "raw" (headerless) audio data
* Added pipeline support, i.e. reading from *stdin* or writing to *stdout*
* Enabled FLAC/Vorbis support in the *static* Win32 binaries
* Various fixes and minor improvements

## Version 2.02 (2014-08-03) ## {-}
* Update license → core library is now released under LGPL v2.1
* Enabled FLAC *output* in the command-line program
* Show legend in the log viewer program
* Some minor documentation and build file updates
* There are **no** code changes in the core library in this release

## Version 2.01 (2014-08-01) ## {-}
* Added graphical log viewer application to the distribution package
* Improved the threshold function for the handling of the maximum gain factor limit
* Added a new mode for handling the "boundary" frames (disabled by default)
* Much improved the format of the log file

## Version 2.00 (2014-07-26) ## {-}
* Implemented a large lookahead buffer, which eliminates the need of 2-Pass processing
* Dynamic Audio Normalizer now works with a *single* processing pass → results in up to 2× speed-up!
* Removed the ``setPass()`` API, because it is *not* applicable any more
* Added new ``flushBuffer()`` API, which provides a cleaner method of flushing the pending frames
* Added new ``reset()`` API, which can be used to reset the internal state of the normalizer instance
* Added new ``setLogFunction`` API, which can be used to set up a custom logging callback function
* There should be **no** changes of the normalized audio output in this release whatsoever

## Version 1.03 (2014-07-09) ## {-}
* Added *static* library configuration to Visual Studio solution
* Various compatibility fixes for Linux/GCC
* Added Makefiles for Linux/GCC, tested under Ubuntu 14.04 LTS
* There are **no** functional changes in this release

## Version 1.02 (2014-07-06) ## {-}
* First public release of the Dynamic Audio Normalizer.



# Frequently Asked Questions (FAQ)

## Q: How does DynAudNorm differ from dynamic range compression? ## {-}

A traditional [*audio compressor*](http://en.wikipedia.org/wiki/Dynamic_range_compression) will prune all samples whose magnitude is above a certain threshold. In particular, the portion of the sample's magnitude that is above the pre-defined threshold will be reduced by a certain *ratio*, typically *2:1* or *4:1*. In other words, the signal *peaks* will be *flattened*, while all samples below the threshold are passed through unmodified. This leaves a certain "headroom", i.e. after flattening the signal peaks the maximum magnitude remaining in the *compressed* file will be lower than in the original. For example, if we apply *2:1* reduction to all samples above a threshold of *80%*, then the maximum remaining magnitude will be at *90%*, leaving a headroom of *10%*. After the compression has been applied, the resulting sample values will (usually) be amplified again, by a certain *fixed* gain factor. This factor will be chosen as high as possible *without* exceeding the maximum allowable signal level, similar to a traditional *normalizer*. Clearly, the compression allows for a much stronger amplification of the signal than what would be possible otherwise. However, due to the *flattening* of the signal peaks, the *dynamic range*, i.e. the ratio between the largest and smallest sample value, will be *reduced* significantly – which has a strong tendency to destroy the "vividness" of the audio signal! The excessive use of dynamic range compression in many recent productions is also known as the ["loudness war"](https://www.youtube.com/watch?v=3Gmex_4hreQ).

The following waveform view shows an audio signal prior to dynamic range compression (left), after the compression step (center) and after the subsequent amplification step (right). It can be seen that the original audio had a *large* dynamic range, with each drumbeat causing a significant peak. It can also be seen how those peeks have been *eliminated* for the most part after the compression. This makes the drum sound much *less* catchy! Last but not least, it can be seen that the final amplified audio now appears much "louder" than the original, but the dynamics are mostly gone…

![Example of dynamic range compression](img/dyauno/Compression-2.png)

In contrast, the Dynamic Audio Normalizer also implements dynamic range compression *of some sort*, but it does **not** prune signal peaks above a *fixed* threshold. Actually it does **not** prune any signal peaks at all! Furthermore, it does **not** amplify the samples by a *fixed* gain factor. Instead, an "optimal" gain factor will be chosen for each *frame*. And, since a frame's gain factor is bounded by the highest magnitude sample within that frame, **100%** of the dynamic range will be preserved *within* each frame! The Dynamic Audio Normalizer only performs a "dynamic range compression" in the sense that the gain factors are *dynamically* adjusted over time, allowing "quieter" frames to get a stronger amplification than "louder" frames. This means that the volume differences between the "quiet" and the "loud" parts of an audio recording will be *harmonized* – but still the *full* dynamic range is being preserved within each of these parts. Finally, the Gaussian filter applied by the Dynamic Audio Normalizer ensures that any changes of the gain factor between neighbouring frames will be smooth and seamlessly, avoiding noticeable "jumps" of the audio volume.


## Q: Why does DynAudNorm *not* seem to change my audio at all? ## {-}

If you think that the Dynamic Audio Normalizer is *not* working properly, because it (seemingly) did *not* change your audio at all, you are almost certainly having the *wrong* expectations and the Dynamic Audio Normalizer actually ***is*** working just as it is supposed to work! Please keep in mind, that the Dynamic Audio Normalizer does *not* use compression, by default, which means that all local "peaks" are perfectly preserved. Also, the Dynamic Audio Normalizer *never* amplifies your audio more than what would bring the "loudest" local peak up to the threshold, i.e. *no* peak is ever allowed to exceed the threshold. Furthermore &ndash; and that is the important part &ndash; the Dynamic Audio Normalizer does **not** simply amplify each *frame* to its local maximum! Instead, it employs a Gaussian filter in order to "smooth" the amplification factors between neighboring frames, which ensures natural volume transitions and avoids noticeable "jumps" of the volume. But this also means that, due to the Gaussian smoothing filter, a particular *frame* may (and often will!) be amplified **less** than what its *local* maximum would allow.

Consequently, if the Dynamic Audio Normalizer did *not* change your audio in a noticeable way, it probably means that your audio simply could *not* be amplified any further &ndash; with the current Dynamic Audio Normalizer settings. This is **not** a "problem", it is just how the Dynamic Audio Normalizer is designed to work! Also keep in mind that the *default* settings of the Dynamic Audio Normalizer have been chosen rather "conservative", in order to make sure that the filter will *not* hurt the dynamics of the audio. Still, if you think that the Dynamic Audio Normalizer should be acting more "aggressively", you can try tweaking the settings. There are two settings that you may wish to adjust: Try a *smaller* [**filter size**](#gaussian-filter-window-size), or throw in some [**input compression**](#input-compression). The *smaller* the filter size, the *faster* (more aggressively) the filter is going to react. At the same time, *compression* may help to reduce "outstanding" peaks that prevent higher amplification, but too much compression may easily hurt audio quality!


## Q: How to *not* harmonize the "quiet" and "loud" parts? ## {-}

If you do *not* want the "quiet" and the "loud" parts of your audio to be harmonized, then the Dynamic Audio Normalizer simply may *not* be the right tool for what you are trying to achieve. It was purposely designed to work like that. Nonetheless, by using a larger [filter size](#gaussian-filter-window-size), the Dynamic Audio Normalizer may be configured to act more similar to a "traditional" normalization filter.


## Q: Why do I get audio reader warnings about more/less samples? ## {-}

This warning indicates that the audio reader delivered more/less audio samples than what was initially projected. For most file formats, the projected number of audio samples should be *accurate*, in which case the warning may actually indicate an I/O error. When processing MPEG audio files, however, the projected number of audio samples is only a *rough estimate*. That's because MPEG audio files, like `.mp2` or `.mp3` files, do **not** use a proper file format; they are just a "loose" collection of MPEG frames. Consequently, there is **no** way to accurately determine the number of samples in an MPEG audio file, except for decoding the *entire* file – which would be way too slow. That's why MPEG audio decoders, like ***libmpg123***, will try to *estimate* the total number of samples from the first couple of MPEG frames. Of course, this is **not** always accurate.

Consequently, when working with MPEG audio files, the warning about more/less audio samples than what was initially projected being read from the input file is most likely **not** a reason for concern; the warning can safely be *ignored* in that case. But, if the warning occurs with *other* types of audio files, please take care, since there probably was an I/O error!

*Note:* The Dynamic Audio Normalizer considers a discrepancy of the audio sample count of more than *25%* as critical error.


## Q: Why does the program crash with GURU MEDITATION error? ## {-}

This error message indicates that the program has encountered a serious problem. On possible reason is that your processor does **not** support the [SSE2](http://en.wikipedia.org/wiki/SSE2) instruction set. That's because the official Dynamic Audio Normalizer binaries have been compiled with SSE and SSE2 code enabled – like pretty much any compiler does *by default* nowadays. So without SSE2 support, the program cannot run, obviosuly. This can be fixed either by upgrading your system to a less antiquated processor, or by recompiling Dynamic Audio Normalizer from the sources with SSE2 code generation *disabled*. Note that SSE2 is supported by the Pentium 4 and Athon 64 processors as well as **all** later processors. Also *every* 64-Bit supports SSE2, because [x86-64](http://en.wikipedia.org/wiki/X86-64) has adopted SSE2 as "core" instructions. That means that *every processor from the last decade* almost certainly supports SSE2.

If your processor *does* support SSE2, but you still get the above error message, you probably have found a bug. In this case it is highly recommended to create a *debug build* and use a [debugger](http://en.wikipedia.org/wiki/Debugger) in order to track down the cause of the problem.


# License Terms #

This chapter lists the licenses that apply to the individual components of the Dynamic Audio Normalizer software.


## Dynamic Audio Normalizer Library ##

The Dynamic Audio Normalizer **library** (DynamicAudioNormalizerAPI) is released under the  
***GNU Lesser General Public License, Version 2.1***.

	Dynamic Audio Normalizer - Audio Processing Library
	Copyright (c) 2014-2019 LoRd_MuldeR <mulder2@gmx.de>. Some rights reserved.
	
	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.
	
	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

<http://www.gnu.org/licenses/lgpl-2.1.html>


## Dynamic Audio Normalizer CLI ##

The Dynamic Audio Normalizer **command-line program** (DynamicAudioNormalizerCLI) is released under the  
***GNU General Public License, Version 2***.

	Dynamic Audio Normalizer - Audio Processing Utility
	Copyright (c) 2014-2019 LoRd_MuldeR <mulder2@gmx.de>. Some rights reserved.
	
	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	
	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.


<http://www.gnu.org/licenses/gpl-2.0.html>


## Dynamic Audio Normalizer GUI ##

The Dynamic Audio Normalizer **log viewer program** (DynamicAudioNormalizerGUI) is released under the  
***GNU General Public License, Version 3***.

	Dynamic Audio Normalizer - Audio Processing Utility
	Copyright (c) 2014-2019 LoRd_MuldeR <mulder2@gmx.de>. Some rights reserved.
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

<http://www.gnu.org/licenses/gpl-3.0.html>


## Dynamic Audio Normalizer Plug-In Wrapper ##

The Dynamic Audio Normalizer **plug-in** wrappers for *SoX*, *VST* and *Winamp* are released under the  
***MIT/X11 License***.

	Dynamic Audio Normalizer - Audio Processing Utility
	Copyright (c) 2014-2019 LoRd_MuldeR <mulder2@gmx.de>. Some rights reserved.
	
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	
	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.
	
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.

<http://opensource.org/licenses/MIT>



# Acknowledgement #

The Dynamic Audio Normalizer **CLI program** (DynamicAudioNormalizerCLI) incorporates the following *third-party* software:

* [**libsndfile**](http://www.mega-nerd.com/libsndfile/)  
  C library for reading and writing files containing sampled sound through one standard library interface  
  Copyright (C) 1999-2016 Erik de Castro Lopo
  *Applicable license:* GNU Lesser General Public License, Version 2.1
  
* [**libmpg123**]()  
  C library for decoding of MPEG 1.0/2.0/2.5 layer I/II/III audio streams to interleaved PCM  
  Copyright (C) 1995-2016 by Michael Hipp, Thomas Orgis, Patrick Dehne, Jonathan Yong and others.  
  *Applicable license:* GNU Lesser General Public License, Version 2.1

The Dynamic Audio Normalizer **log viewer** (DynamicAudioNormalizerGUI) incorporates the following *third-party* software:

* [**Qt Framework**](http://qt-project.org/)  
  Cross-platform application and UI framework for developers using C++ or QML, a CSS & JavaScript like language  
  Copyright (C) 2014 Digia Plc and/or its subsidiary(-ies)  
  *Applicable license:* GNU Lesser General Public License, Version 2.1
  
* [**QCustomPlot**](http://www.qcustomplot.com/)  
  Qt C++ widget for plotting and data visualization that focuses on making good looking, publication quality 2D plots  
  Copyright (C) 2011-2014 Emanuel Eichhammer  
  *Applicable license:* GNU General Public License, Version 3

The Dynamic Audio Normalizer **VST wrapper** (DynamicAudioNormalizerVST) incorporates the following *third-party* software:

* [**Spooky Hash V2**](http://burtleburtle.net/bob/hash/spooky.html)  
  Public domain noncryptographic hash function producing well-distributed 128-bit hash values  
  Created by Bob Jenkins, 2012.  
  *Applicable license:* None / Public Domain

The Dynamic Audio Normalizer can operate as a **plug-in** (effect) using the following *third-party* technologies:

  * [**Sound eXchange**](http://sox.sourceforge.net/)  
  Cross-platform command line utility that can convert various formats of computer audio files in to other formats  
  Copyright (C) 1998-2009 Chris Bagwell and SoX contributors

  * [**Virtual Studio Technology (VST 2.x)**](http://en.wikipedia.org/wiki/Virtual_Studio_Technology)  
  Software interface that integrates software audio synthesizer and effect Plug-ins with audio editors and recording systems.  
  Copyright (C) 2006 Steinberg Media Technologies. All Rights Reserved.  
  <small>**VST PlugIn Interface Technology by Steinberg Media Technologies GmbH. VST is a trademark of Steinberg Media Technologies GmbH.**</small>

  * [**Winamp**](http://www.winamp.com/)  
  Popular media player for Windows, Android, and OS X that supports extensibility with plug-ins and skins.  
  Copyright (C) 1997-2013 Nullsoft, Inc. All Rights Reserved.  


&nbsp;  
&nbsp;  

e.o.f.
