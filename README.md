# OSCdynamicTest
working with TD1.55 @ platformio using latest dynamic-updates branch from h4yn0nnym0u5e/cores

<br>
AsyncAudioInputSPDIF3<br>
b*,NULL // void begin();<br>
getA*,NULL // double getAttenuation() const;<br>
getB*,NULL // double getBufferedTime() const;<br>
getH*,NULL // int32_t getHalfFilterLength() const;<br>
getI*,NULL // double getInputFrequency() const;<br>
getT*,NULL // double getTargetLantency() const;<br>
i*,NULL // static bool isLocked();<br>
<br>
AudioAmplifier<br>
g*,f // void gain(float n)<br>
<br>
AudioAnalyzeEvent<br>
getC*,NULL // uint32_t getCount(void) {return count;}<br>
<br>
AudioAnalyzeFFT1024<br>
ava*,NULL // bool available() {<br>
ave*,i // void averageTogether(uint8_t n) {<br>
r*,ii // float read(unsigned int binFirst, unsigned int binLast) {<br>
r*,i // float read(unsigned int binNumber) {<br>
<br>
AudioAnalyzeFFT256<br>
ava*,NULL // bool available() {<br>
ave*,i // void averageTogether(uint8_t n) {<br>
r*,ii // float read(unsigned int binFirst, unsigned int binLast) {<br>
r*,i // float read(unsigned int binNumber) {<br>
<br>
AudioAnalyzeNoteFrequency<br>
a*,NULL // bool available( void );<br>
b*,f // void begin( float threshold );<br>
p*,NULL // float probability( void );<br>
r*,NULL // float read( void );<br>
t*,f // void threshold( float p );<br>
<br>
AudioAnalyzePeak<br>
a*,NULL // bool available(void) {<br>
readP*,NULL // float readPeakToPeak(void) {<br>
read,NULL // float read(void) {<br>
<br>
AudioAnalyzePrint<br>
d*,i // void delay(uint32_t num) { delay_length = num; }<br>
l*,i // void length(uint32_t num) { print_length = num; }<br>
t*,NULL // void trigger(void);<br>
<br>
AudioAnalyzeRMS<br>
a*,NULL // bool available(void) {<br>
r*,NULL // float read(void);<br>
<br>
AudioAnalyzeToneDetect<br>
a*,NULL // bool available(void) {<br>
f*,fi // void frequency(float freq, uint16_t cycles=10) {<br>
r*,NULL // float read(void);<br>
s*,iii // void set_params(int32_t coef, uint16_t cycles, uint16_t len);<br>
t*,f // void threshold(float level) {<br>
<br>
AudioControlAK4558<br>
disableI*,NULL // bool disableIn(void);	//powers down ADC<br>
disableO*,NULL // bool disableOut(void);	//powers down DAC<br>
disable,NULL // bool disable(void) { return (disableIn()&&disableOut()); }	//powers down ADC/DAC<br>
enableI*,NULL // bool enableIn(void);	//powers up ADC<br>
enableO*,NULL // bool enableOut(void);	//powers up DAC<br>
enable,NULL // bool enable(void);		//enables the CODEC, does not power up ADC nor DAC (use enableIn() and enableOut() for selective power up)<br>
inputL*,f // bool inputLevel(float n) { return false; }	//not supported by AK4558<br>
inputS*,i // bool inputSelect(int n) { return false; }	//sets inputs to mono left, mono right, stereo (default stereo), not yet implemented<br>
volumeL*,f // bool volumeLeft(float n);	//sets LOUT volume to n (range 0.0 - 1.0)<br>
volumeR*,f // bool volumeRight(float n);	//sets ROUT volume to n (range 0.0 - 1.0)<br>
volume,f // bool volume(float n);	//sets LOUT/ROUT volume to n (range 0.0 - 1.0)<br>
<br>
AudioControlCS42448<br>
d*,NULL // bool disable(void) {<br>
e*,NULL // bool enable(void);<br>
inputL*,f // bool inputLevel(float level) {<br>
inputL*,if // bool inputLevel(int channel, float level) {<br>
inputS*,i // bool inputSelect(int n) {<br>
s*,i // void setAddress(uint8_t addr) {<br>
v*,f // bool volume(float level) {<br>
v*,if // bool volume(int channel, float level) {<br>
<br>
AudioControlCS4272<br>
da*,ff // bool dacVolume(float left, float right);<br>
da*,f // bool dacVolume(float n) { return volumeInteger(n * 127 + 0.499f); }<br>
disableD*,NULL // bool disableDither(void);<br>
disable,NULL // bool disable(void) { return false; }<br>
enableD*,NULL // bool enableDither(void);<br>
enable,NULL // bool enable(void);<br>
inputL*,f // bool inputLevel(float n) { return false; }<br>
inputS*,i // bool inputSelect(int n) { return false; }<br>
muteI*,NULL // bool muteInput(void);<br>
muteO*,NULL // bool muteOutput(void);<br>
unmuteI*,NULL // bool unmuteInput(void);<br>
unmuteO*,NULL // bool unmuteOutput(void);<br>
v*,ff // bool volume(float left, float right);<br>
v*,f // bool volume(float n) { return volumeInteger(n * 127 + 0.499f); }<br>
<br>
AudioControlSGTL5000<br>
adcHighPassFilterD*,NULL // unsigned short adcHighPassFilterDisable(void);<br>
adcHighPassFilterE*,NULL // unsigned short adcHighPassFilterEnable(void);<br>
adcHighPassFilterF*,NULL // unsigned short adcHighPassFilterFreeze(void);<br>
audioPo*,NULL // unsigned short audioPostProcessorEnable(void);<br>
audioPre*,NULL // unsigned short audioPreProcessorEnable(void);<br>
audioPro*,NULL // unsigned short audioProcessorDisable(void);<br>
autoVolumeC*,iiifff // unsigned short autoVolumeControl(uint8_t maxGain, uint8_t lbiResponse, uint8_t hardLimit, float threshold, float attack, float decay);<br>
autoVolumeD*,NULL // unsigned short autoVolumeDisable(void);<br>
autoVolumeE*,NULL // unsigned short autoVolumeEnable(void);<br>
dacVolumeRampD*,NULL // bool dacVolumeRampDisable();<br>
dacVolumeRampL*,NULL // bool dacVolumeRampLinear();<br>
dacVolumeRamp,NULL // bool dacVolumeRamp();<br>
dacVolume,ff // unsigned short dacVolume(float left, float right);<br>
dacVolume,f // unsigned short dacVolume(float n);<br>
di*,NULL // bool disable(void) { return false; }<br>
ena*,NULL // bool enable(void);//For Teensy LC the SGTL acts as master, for all other Teensys as slave.<br>
enhanceBassD*,NULL // unsigned short enhanceBassDisable(void);<br>
enhanceBassE*,NULL // unsigned short enhanceBassEnable(void);<br>
enhanceBass,ff // unsigned short enhanceBass(float lr_lev, float bass_lev);<br>
enhanceBass,ffii // unsigned short enhanceBass(float lr_lev, float bass_lev, uint8_t hpf_bypass, uint8_t cutoff);<br>
eqBands,fffff // void eqBands(float bass, float mid_bass, float midrange, float mid_treble, float treble);<br>
eqBands,ff // void eqBands(float bass, float treble);<br>
eqBand,if // unsigned short eqBand(uint8_t bandNum, float n);<br>
eqFilterC*,i // unsigned short eqFilterCount(uint8_t n);<br>
eqS*,i // unsigned short eqSelect(uint8_t n);<br>
h*,i // bool headphoneSelect(int n) {<br>
inputL*,f // bool inputLevel(float n) {return false;}<br>
inputS*,i // bool inputSelect(int n) {<br>
k*,NULL // void killAutomation(void) { semi_automated=false; }<br>
lineI*,ii // bool lineInLevel(uint8_t left, uint8_t right);<br>
lineI*,i // bool lineInLevel(uint8_t n) { return lineInLevel(n, n); }<br>
lineO*,ii // unsigned short lineOutLevel(uint8_t left, uint8_t right);<br>
lineO*,i // unsigned short lineOutLevel(uint8_t n);<br>
mi*,i // bool micGain(unsigned int dB);<br>
muteH*,NULL // bool muteHeadphone(void) { return write(0x0024, ana_ctrl | (1<<4)); }<br>
muteL*,NULL // bool muteLineout(void) { return write(0x0024, ana_ctrl | (1<<8)); }<br>
se*,i // void setAddress(uint8_t level);<br>
surroundSoundD*,NULL // unsigned short surroundSoundDisable(void);<br>
surroundSoundE*,NULL // unsigned short surroundSoundEnable(void);<br>
surroundSound,i // unsigned short surroundSound(uint8_t width);<br>
surroundSound,ii // unsigned short surroundSound(uint8_t width, uint8_t select);<br>
unmuteH*,NULL // bool unmuteHeadphone(void) { return write(0x0024, ana_ctrl & ~(1<<4)); }<br>
unmuteL*,NULL // bool unmuteLineout(void) { return write(0x0024, ana_ctrl & ~(1<<8)); }<br>
v*,ff // bool volume(float left, float right);<br>
v*,f // bool volume(float n) { return volumeInteger(n * 129 + 0.499f); }<br>
<br>
AudioControlTLV320AIC3206<br>
aic_r*,ii // unsigned int aic_readPage(uint8_t page, uint8_t reg);<br>
aic_w*,iii // bool aic_writePage(uint8_t page, uint8_t reg, uint8_t val);<br>
d*,NULL // bool disable(void);<br>
enableA*,;i // bool enableAutoMuteDAC(bool, uint8_t);<br>
enableM*,; // bool enableMicDetect(bool);<br>
enable,NULL // bool enable(void);<br>
getH*,NULL // float getHPCutoff_Hz(void) { return HP_cutoff_Hz; }<br>
getS*,NULL // float getSampleRate_Hz(void) { return sample_rate_Hz; }<br>
inputL*,f // bool inputLevel(float n);  //dummy to be compatible with Teensy Audio Library<br>
inputS*,i // bool inputSelect(int n);   //use AIC3206_INPUT_IN1 or one of other choices defined earlier<br>
o*,i // bool outputSelect(int n);  //use AIC3206_OUTPUT_HEADPHONE_JACK_OUT or one of other choices defined earlier<br>
r*,NULL // int  readMicDetect(void);<br>
setH*,;ff // void setHPFonADC(bool enable, float cutoff_Hz, float fs_Hz);<br>
setIn*,f // bool setInputGain_dB(float n);<br>
setM*,i // bool setMicBias(int n);  //use AIC3206_MIC_BIAS_OFF or AIC3206_MIC_BIAS_2_5 or one of other choices defined earlier<br>
u*,i // bool updateInputBasedOnMicDetect(int setting = AIC3206_INPUT_IN1); //which input to monitor<br>
volume_*,f // bool volume_dB(float n);<br>
volume,f // bool volume(float n);<br>
<br>
AudioControlWM8731<br>
d*,NULL // bool disable(void) { return false; }<br>
e*,NULL // bool enable(void);<br>
inputL*,f // bool inputLevel(float n); // range: 0.0f to 1.0f<br>
inputS*,i // bool inputSelect(int n);<br>
v*,f // bool volume(float n) { return volumeInteger(n * 80.0f + 47.499f); }<br>
<br>
AudioControlWM8731master<br>
e*,NULL // bool enable(void);<br>
<br>
AudioEffectBitcrusher<br>
b*,i // void bits(uint8_t b) {<br>
s*,f // void sampleRate(float hz) {<br>
<br>
AudioEffectDelay<br>
de*,if // void delay(uint8_t channel, float milliseconds) {<br>
di*,i // void disable(uint8_t channel) {<br>
<br>
AudioEffectDelayExternal<br>
de*,if // void delay(uint8_t channel, float milliseconds) {<br>
di*,i // void disable(uint8_t channel) {<br>
<br>
AudioEffectDigitalCombine<br>
s*,i // void setCombineMode(int mode_in) {<br>
<br>
AudioEffectEnvelope<br>
a*,f // void attack(float milliseconds) {<br>
dec*,f // void decay(float milliseconds) {<br>
del*,f // void delay(float milliseconds) {<br>
h*,f // void hold(float milliseconds) {<br>
isA*,NULL // bool isActive();<br>
isS*,NULL // bool isSustain();<br>
noteOf*,NULL // void noteOff();<br>
noteOn,NULL // void noteOn();<br>
releaseN*,f // void releaseNoteOn(float milliseconds) {<br>
release,f // void release(float milliseconds) {<br>
s*,f // void sustain(float level) {<br>
<br>
AudioEffectExpEnvelope<br>
a*,ff // void attack(float milliseconds, float target_factor = TF)<br>
c*,NULL // void close(){<br>
dec*,ff // void decay(float milliseconds, float target_factor = TF)<br>
del*,f // void delay(float milliseconds)<br>
getG*,NULL // float getGain() {return HIRES_TO_FLOAT(mult_hires);}<br>
getS*,NULL // uint8_t getState();<br>
h*,f // void hold(float milliseconds)<br>
isA*,NULL // bool isActive();<br>
isS*,NULL // bool isSustain();<br>
noteOf*,NULL // void noteOff();<br>
noteOn,NULL // void noteOn();<br>
r*,ff // void release(float milliseconds, float target_factor = TF)<br>
s*,f // void sustain(float level)<br>
<br>
AudioEffectFade<br>
fadeI*,i // void fadeIn(uint32_t milliseconds) {<br>
fadeO*,i // void fadeOut(uint32_t milliseconds) {<br>
<br>
AudioEffectFreeverb<br>
d*,f // void damping(float n) {<br>
r*,f // void roomsize(float n) {<br>
<br>
AudioEffectFreeverbStereo<br>
d*,f // void damping(float n) {<br>
r*,f // void roomsize(float n) {<br>
<br>
AudioEffectGranular<br>
beginF*,f // void beginFreeze(float grain_length) {<br>
beginP*,f // void beginPitchShift(float grain_length) {<br>
se*,f // void setSpeed(float ratio) {<br>
st*,NULL // void stop();<br>
<br>
AudioEffectMidSide<br>
d*,NULL // void decode() { encoding = false; }<br>
e*,NULL // void encode() { encoding = true; }<br>
<br>
AudioEffectMultiply<br>
<br>
AudioEffectRectifier<br>
<br>
AudioEffectReverb<br>
r*,f // void reverbTime(float);<br>
<br>
AudioEffectWaveFolder<br>
<br>
AudioEffectWaveshaper<br>
<br>
AudioFilterBiquad<br>
setB*,iff // void setBandpass(uint32_t stage, float frequency, float q = 1.0) {<br>
setHighS*,ifff // void setHighShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f) {<br>
setHighp*,iff // void setHighpass(uint32_t stage, float frequency, float q = 0.7071) {<br>
setLowS*,ifff // void setLowShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f) {<br>
setLowp*,iff // void setLowpass(uint32_t stage, float frequency, float q = 0.7071f) {<br>
setN*,iff // void setNotch(uint32_t stage, float frequency, float q = 1.0) {<br>
<br>
AudioFilterFIR<br>
e*,NULL // void end(void) {<br>
<br>
AudioFilterLadder<br>
f*,f // void frequency(float FC);<br>
inp*,f // void inputDrive(float drv);<br>
int*,i // void interpolationMethod(AudioFilterLadderInterpolation im);<br>
o*,f // void octaveControl(float octaves);<br>
p*,f // void passbandGain(float passbandgain);<br>
r*,f // void resonance(float reson);<br>
<br>
AudioFilterStateVariable<br>
f*,f // void frequency(float freq) {<br>
o*,f // void octaveControl(float n) {<br>
r*,f // void resonance(float q) {<br>
<br>
AudioInputAnalog<br>
<br>
AudioInputAnalogStereo<br>
<br>
AudioInputI2S<br>
<br>
AudioInputI2S2<br>
<br>
AudioInputI2SHex<br>
<br>
AudioInputI2SOct<br>
<br>
AudioInputI2SQuad<br>
<br>
AudioInputPDM<br>
<br>
AudioInputPDM2<br>
<br>
AudioInputSPDIF3<br>
p*,NULL // static bool pllLocked(void);<br>
s*,NULL // static unsigned int sampleRate(void);<br>
<br>
AudioInputTDM<br>
<br>
AudioInputTDM2<br>
<br>
AudioMixer4<br>
g*,if // void gain(unsigned int channel, float gain) {<br>
<br>
AudioOutputADAT<br>
m*,; // static void mute_PCM(const bool mute);<br>
<br>
AudioOutputAnalog<br>
a*,i // void analogReference(int ref);<br>
<br>
AudioOutputAnalogStereo<br>
a*,i // void analogReference(int ref);<br>
<br>
AudioOutputI2S<br>
<br>
AudioOutputI2S2<br>
<br>
AudioOutputI2SHex<br>
<br>
AudioOutputI2SOct<br>
<br>
AudioOutputI2SQuad<br>
<br>
AudioOutputMQS<br>
<br>
AudioOutputPT8211<br>
<br>
AudioOutputPT8211_2<br>
<br>
AudioOutputPWM<br>
<br>
AudioOutputSPDIF<br>
m*,; // static void mute_PCM(const bool mute);<br>
<br>
AudioOutputSPDIF2<br>
m*,; // static void mute_PCM(const bool mute);<br>
<br>
AudioOutputSPDIF3<br>
m*,; // static void mute_PCM(const bool mute);<br>
<br>
AudioOutputTDM<br>
<br>
AudioOutputTDM2<br>
<br>
AudioPlayMemory<br>
i*,NULL // bool isPlaying(void) { return playing; }<br>
l*,NULL // uint32_t lengthMillis(void);<br>
po*,NULL // uint32_t positionMillis(void);<br>
s*,NULL // void stop(void);<br>
<br>
AudioPlayQueue<br>
a*,NULL // bool available(void);<br>
g*,NULL // int16_t * getBuffer(void);<br>
playB*,NULL // void playBuffer(void);<br>
<br>
AudioPlaySdRaw<br>
b*,NULL // void begin(void);<br>
i*,NULL // bool isPlaying(void) { return playing; }<br>
l*,NULL // uint32_t lengthMillis(void);<br>
po*,NULL // uint32_t positionMillis(void);<br>
s*,NULL // void stop(void);<br>
<br>
AudioPlaySdWav<br>
b*,NULL // void begin(void);<br>
isPa*,NULL // bool isPaused(void);<br>
isPl*,NULL // bool isPlaying(void);<br>
isS*,NULL // bool isStopped(void);<br>
l*,NULL // uint32_t lengthMillis(void);<br>
po*,NULL // uint32_t positionMillis(void);<br>
s*,NULL // void stop(void);<br>
t*,NULL // void togglePlayPause(void);<br>
<br>
AudioPlaySerialflashRaw<br>
b*,NULL // void begin(void);<br>
i*,NULL // bool isPlaying(void) { return playing; }<br>
l*,NULL // uint32_t lengthMillis(void);<br>
po*,NULL // uint32_t positionMillis(void);<br>
s*,NULL // void stop(void);<br>
<br>
AudioRecordQueue<br>
a*,NULL // int available(void);<br>
b*,NULL // void begin(void) {<br>
c*,NULL // void clear(void);<br>
e*,NULL // void end(void) {<br>
f*,NULL // void freeBuffer(void);<br>
r*,NULL // int16_t * readBuffer(void);<br>
<br>
AudioSynthKarplusStrong<br>
noteOf*,f // void noteOff(float velocity) {<br>
noteOn,ff // void noteOn(float frequency, float velocity) {<br>
<br>
AudioSynthNoisePink<br>
a*,f // void amplitude(float n) {<br>
<br>
AudioSynthNoiseWhite<br>
a*,f // void amplitude(float n) {<br>
<br>
AudioSynthSimpleDrum<br>
f*,f // void frequency(float freq)<br>
l*,i // void length(int32_t milliseconds)<br>
n*,NULL // void noteOn();<br>
p*,f // void pitchMod(float depth);<br>
s*,f // void secondMix(float level);<br>
<br>
AudioSynthToneSweep<br>
i*,NULL // unsigned char isPlaying(void);<br>
p*,fiif // boolean play(float t_amp,int t_lo,int t_hi,float t_time);<br>
r*,NULL // float read(void) {<br>
<br>
AudioSynthWaveform<br>
am*,f // void amplitude(float n) {	// 0 to 1.0<br>
b*,ffi // void begin(float t_amp, float t_freq, short t_type) {<br>
b*,i // void begin(short t_type) {<br>
f*,f // void frequency(float freq) {<br>
o*,f // void offset(float n) {<br>
ph*,f // void phase(float angle) {<br>
pu*,f // void pulseWidth(float n) {	// 0.0 to 1.0<br>
<br>
AudioSynthWaveformDc<br>
a*,f // void amplitude(float n) {<br>
a*,ff // void amplitude(float n, float milliseconds) {<br>
r*,NULL // float read(void) {<br>
<br>
AudioSynthWaveformModulated<br>
am*,f // void amplitude(float n) {	// 0 to 1.0<br>
b*,ffi // void begin(float t_amp, float t_freq, short t_type) {<br>
b*,i // void begin(short t_type) {<br>
frequencyM*,f // void frequencyModulation(float octaves) {<br>
frequency,f // void frequency(float freq) {<br>
o*,f // void offset(float n) {<br>
p*,f // void phaseModulation(float degrees) {<br>
<br>
AudioSynthWaveformPWM<br>
a*,f // void amplitude(float n) {<br>
f*,f // void frequency(float freq) {<br>
<br>
AudioSynthWaveformSine<br>
a*,f // void amplitude(float n) {<br>
f*,f // void frequency(float freq) {<br>
p*,f // void phase(float angle) {<br>
<br>
AudioSynthWaveformSineHires<br>
a*,f // void amplitude(float n) {<br>
f*,f // void frequency(float freq) {<br>
p*,f // void phase(float angle) {<br>
<br>
AudioSynthWaveformSineModulated<br>
a*,f // void amplitude(float n) {<br>
f*,f // void frequency(float freq) {<br>
p*,f // void phase(float angle) {<br>
<br>
AudioSynthWavetable<br>
a*,f // void amplitude(float v) {<br>
f*,f // static int freqToNote(float freq) {<br>
g*,NULL // envelopeStateEnum getEnvState(void) { return env_state; }<br>
i*,NULL // bool isPlaying(void) { return env_state != STATE_IDLE; }<br>
m*,i // static float midi_volume_transform(int midi_amp) {<br>
n*,i // static float noteToFreq(int note) {<br>
playF*,fi // void playFrequency(float freq, int amp = DEFAULT_AMPLITUDE);<br>
playN*,ii // void playNote(int note, int amp = DEFAULT_AMPLITUDE);<br>
setF*,f // void setFrequency(float freq);<br>
st*,NULL // void stop(void);<br>