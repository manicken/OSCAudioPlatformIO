# OSCdynamicTest
working with TD1.55 @ platformio using latest dynamic-updates branch from h4yn0nnym0u5e/cores


AsyncAudioInputSPDIF3
b*,NULL // void begin();
getA*,NULL // double getAttenuation() const;
getB*,NULL // double getBufferedTime() const;
getH*,NULL // int32_t getHalfFilterLength() const;
getI*,NULL // double getInputFrequency() const;
getT*,NULL // double getTargetLantency() const;
i*,NULL // static bool isLocked();

AudioAmplifier
g*,f // void gain(float n)

AudioAnalyzeEvent
getC*,NULL // uint32_t getCount(void) {return count;}

AudioAnalyzeFFT1024
ava*,NULL // bool available() {
ave*,i // void averageTogether(uint8_t n) {
r*,ii // float read(unsigned int binFirst, unsigned int binLast) {
r*,i // float read(unsigned int binNumber) {

AudioAnalyzeFFT256
ava*,NULL // bool available() {
ave*,i // void averageTogether(uint8_t n) {
r*,ii // float read(unsigned int binFirst, unsigned int binLast) {
r*,i // float read(unsigned int binNumber) {

AudioAnalyzeNoteFrequency
a*,NULL // bool available( void );
b*,f // void begin( float threshold );
p*,NULL // float probability( void );
r*,NULL // float read( void );
t*,f // void threshold( float p );

AudioAnalyzePeak
a*,NULL // bool available(void) {
readP*,NULL // float readPeakToPeak(void) {
read,NULL // float read(void) {

AudioAnalyzePrint
d*,i // void delay(uint32_t num) { delay_length = num; }
l*,i // void length(uint32_t num) { print_length = num; }
t*,NULL // void trigger(void);

AudioAnalyzeRMS
a*,NULL // bool available(void) {
r*,NULL // float read(void);

AudioAnalyzeToneDetect
a*,NULL // bool available(void) {
f*,fi // void frequency(float freq, uint16_t cycles=10) {
r*,NULL // float read(void);
s*,iii // void set_params(int32_t coef, uint16_t cycles, uint16_t len);
t*,f // void threshold(float level) {

AudioControlAK4558
disableI*,NULL // bool disableIn(void);	//powers down ADC
disableO*,NULL // bool disableOut(void);	//powers down DAC
disable,NULL // bool disable(void) { return (disableIn()&&disableOut()); }	//powers down ADC/DAC
enableI*,NULL // bool enableIn(void);	//powers up ADC
enableO*,NULL // bool enableOut(void);	//powers up DAC
enable,NULL // bool enable(void);		//enables the CODEC, does not power up ADC nor DAC (use enableIn() and enableOut() for selective power up)
inputL*,f // bool inputLevel(float n) { return false; }	//not supported by AK4558
inputS*,i // bool inputSelect(int n) { return false; }	//sets inputs to mono left, mono right, stereo (default stereo), not yet implemented
volumeL*,f // bool volumeLeft(float n);	//sets LOUT volume to n (range 0.0 - 1.0)
volumeR*,f // bool volumeRight(float n);	//sets ROUT volume to n (range 0.0 - 1.0)
volume,f // bool volume(float n);	//sets LOUT/ROUT volume to n (range 0.0 - 1.0)

AudioControlCS42448
d*,NULL // bool disable(void) {
e*,NULL // bool enable(void);
inputL*,f // bool inputLevel(float level) {
inputL*,if // bool inputLevel(int channel, float level) {
inputS*,i // bool inputSelect(int n) {
s*,i // void setAddress(uint8_t addr) {
v*,f // bool volume(float level) {
v*,if // bool volume(int channel, float level) {

AudioControlCS4272
da*,ff // bool dacVolume(float left, float right);
da*,f // bool dacVolume(float n) { return volumeInteger(n * 127 + 0.499f); }
disableD*,NULL // bool disableDither(void);
disable,NULL // bool disable(void) { return false; }
enableD*,NULL // bool enableDither(void);
enable,NULL // bool enable(void);
inputL*,f // bool inputLevel(float n) { return false; }
inputS*,i // bool inputSelect(int n) { return false; }
muteI*,NULL // bool muteInput(void);
muteO*,NULL // bool muteOutput(void);
unmuteI*,NULL // bool unmuteInput(void);
unmuteO*,NULL // bool unmuteOutput(void);
v*,ff // bool volume(float left, float right);
v*,f // bool volume(float n) { return volumeInteger(n * 127 + 0.499f); }

AudioControlSGTL5000
adcHighPassFilterD*,NULL // unsigned short adcHighPassFilterDisable(void);
adcHighPassFilterE*,NULL // unsigned short adcHighPassFilterEnable(void);
adcHighPassFilterF*,NULL // unsigned short adcHighPassFilterFreeze(void);
audioPo*,NULL // unsigned short audioPostProcessorEnable(void);
audioPre*,NULL // unsigned short audioPreProcessorEnable(void);
audioPro*,NULL // unsigned short audioProcessorDisable(void);
autoVolumeC*,iiifff // unsigned short autoVolumeControl(uint8_t maxGain, uint8_t lbiResponse, uint8_t hardLimit, float threshold, float attack, float decay);
autoVolumeD*,NULL // unsigned short autoVolumeDisable(void);
autoVolumeE*,NULL // unsigned short autoVolumeEnable(void);
dacVolumeRampD*,NULL // bool dacVolumeRampDisable();
dacVolumeRampL*,NULL // bool dacVolumeRampLinear();
dacVolumeRamp,NULL // bool dacVolumeRamp();
dacVolume,ff // unsigned short dacVolume(float left, float right);
dacVolume,f // unsigned short dacVolume(float n);
di*,NULL // bool disable(void) { return false; }
ena*,NULL // bool enable(void);//For Teensy LC the SGTL acts as master, for all other Teensys as slave.
enhanceBassD*,NULL // unsigned short enhanceBassDisable(void);
enhanceBassE*,NULL // unsigned short enhanceBassEnable(void);
enhanceBass,ff // unsigned short enhanceBass(float lr_lev, float bass_lev);
enhanceBass,ffii // unsigned short enhanceBass(float lr_lev, float bass_lev, uint8_t hpf_bypass, uint8_t cutoff);
eqBands,fffff // void eqBands(float bass, float mid_bass, float midrange, float mid_treble, float treble);
eqBands,ff // void eqBands(float bass, float treble);
eqBand,if // unsigned short eqBand(uint8_t bandNum, float n);
eqFilterC*,i // unsigned short eqFilterCount(uint8_t n);
eqS*,i // unsigned short eqSelect(uint8_t n);
h*,i // bool headphoneSelect(int n) {
inputL*,f // bool inputLevel(float n) {return false;}
inputS*,i // bool inputSelect(int n) {
k*,NULL // void killAutomation(void) { semi_automated=false; }
lineI*,ii // bool lineInLevel(uint8_t left, uint8_t right);
lineI*,i // bool lineInLevel(uint8_t n) { return lineInLevel(n, n); }
lineO*,ii // unsigned short lineOutLevel(uint8_t left, uint8_t right);
lineO*,i // unsigned short lineOutLevel(uint8_t n);
mi*,i // bool micGain(unsigned int dB);
muteH*,NULL // bool muteHeadphone(void) { return write(0x0024, ana_ctrl | (1<<4)); }
muteL*,NULL // bool muteLineout(void) { return write(0x0024, ana_ctrl | (1<<8)); }
se*,i // void setAddress(uint8_t level);
surroundSoundD*,NULL // unsigned short surroundSoundDisable(void);
surroundSoundE*,NULL // unsigned short surroundSoundEnable(void);
surroundSound,i // unsigned short surroundSound(uint8_t width);
surroundSound,ii // unsigned short surroundSound(uint8_t width, uint8_t select);
unmuteH*,NULL // bool unmuteHeadphone(void) { return write(0x0024, ana_ctrl & ~(1<<4)); }
unmuteL*,NULL // bool unmuteLineout(void) { return write(0x0024, ana_ctrl & ~(1<<8)); }
v*,ff // bool volume(float left, float right);
v*,f // bool volume(float n) { return volumeInteger(n * 129 + 0.499f); }

AudioControlTLV320AIC3206
aic_r*,ii // unsigned int aic_readPage(uint8_t page, uint8_t reg);
aic_w*,iii // bool aic_writePage(uint8_t page, uint8_t reg, uint8_t val);
d*,NULL // bool disable(void);
enableA*,;i // bool enableAutoMuteDAC(bool, uint8_t);
enableM*,; // bool enableMicDetect(bool);
enable,NULL // bool enable(void);
getH*,NULL // float getHPCutoff_Hz(void) { return HP_cutoff_Hz; }
getS*,NULL // float getSampleRate_Hz(void) { return sample_rate_Hz; }
inputL*,f // bool inputLevel(float n);  //dummy to be compatible with Teensy Audio Library
inputS*,i // bool inputSelect(int n);   //use AIC3206_INPUT_IN1 or one of other choices defined earlier
o*,i // bool outputSelect(int n);  //use AIC3206_OUTPUT_HEADPHONE_JACK_OUT or one of other choices defined earlier
r*,NULL // int  readMicDetect(void);
setH*,;ff // void setHPFonADC(bool enable, float cutoff_Hz, float fs_Hz);
setIn*,f // bool setInputGain_dB(float n);
setM*,i // bool setMicBias(int n);  //use AIC3206_MIC_BIAS_OFF or AIC3206_MIC_BIAS_2_5 or one of other choices defined earlier
u*,i // bool updateInputBasedOnMicDetect(int setting = AIC3206_INPUT_IN1); //which input to monitor
volume_*,f // bool volume_dB(float n);
volume,f // bool volume(float n);

AudioControlWM8731
d*,NULL // bool disable(void) { return false; }
e*,NULL // bool enable(void);
inputL*,f // bool inputLevel(float n); // range: 0.0f to 1.0f
inputS*,i // bool inputSelect(int n);
v*,f // bool volume(float n) { return volumeInteger(n * 80.0f + 47.499f); }

AudioControlWM8731master
e*,NULL // bool enable(void);

AudioEffectBitcrusher
b*,i // void bits(uint8_t b) {
s*,f // void sampleRate(float hz) {

AudioEffectDelay
de*,if // void delay(uint8_t channel, float milliseconds) {
di*,i // void disable(uint8_t channel) {

AudioEffectDelayExternal
de*,if // void delay(uint8_t channel, float milliseconds) {
di*,i // void disable(uint8_t channel) {

AudioEffectDigitalCombine
s*,i // void setCombineMode(int mode_in) {

AudioEffectEnvelope
a*,f // void attack(float milliseconds) {
dec*,f // void decay(float milliseconds) {
del*,f // void delay(float milliseconds) {
h*,f // void hold(float milliseconds) {
isA*,NULL // bool isActive();
isS*,NULL // bool isSustain();
noteOf*,NULL // void noteOff();
noteOn,NULL // void noteOn();
releaseN*,f // void releaseNoteOn(float milliseconds) {
release,f // void release(float milliseconds) {
s*,f // void sustain(float level) {

AudioEffectExpEnvelope
a*,ff // void attack(float milliseconds, float target_factor = TF)
c*,NULL // void close(){
dec*,ff // void decay(float milliseconds, float target_factor = TF)
del*,f // void delay(float milliseconds)
getG*,NULL // float getGain() {return HIRES_TO_FLOAT(mult_hires);}
getS*,NULL // uint8_t getState();
h*,f // void hold(float milliseconds)
isA*,NULL // bool isActive();
isS*,NULL // bool isSustain();
noteOf*,NULL // void noteOff();
noteOn,NULL // void noteOn();
r*,ff // void release(float milliseconds, float target_factor = TF)
s*,f // void sustain(float level)

AudioEffectFade
fadeI*,i // void fadeIn(uint32_t milliseconds) {
fadeO*,i // void fadeOut(uint32_t milliseconds) {

AudioEffectFreeverb
d*,f // void damping(float n) {
r*,f // void roomsize(float n) {

AudioEffectFreeverbStereo
d*,f // void damping(float n) {
r*,f // void roomsize(float n) {

AudioEffectGranular
beginF*,f // void beginFreeze(float grain_length) {
beginP*,f // void beginPitchShift(float grain_length) {
se*,f // void setSpeed(float ratio) {
st*,NULL // void stop();

AudioEffectMidSide
d*,NULL // void decode() { encoding = false; }
e*,NULL // void encode() { encoding = true; }

AudioEffectMultiply

AudioEffectRectifier

AudioEffectReverb
r*,f // void reverbTime(float);

AudioEffectWaveFolder

AudioEffectWaveshaper

AudioFilterBiquad
setB*,iff // void setBandpass(uint32_t stage, float frequency, float q = 1.0) {
setHighS*,ifff // void setHighShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f) {
setHighp*,iff // void setHighpass(uint32_t stage, float frequency, float q = 0.7071) {
setLowS*,ifff // void setLowShelf(uint32_t stage, float frequency, float gain, float slope = 1.0f) {
setLowp*,iff // void setLowpass(uint32_t stage, float frequency, float q = 0.7071f) {
setN*,iff // void setNotch(uint32_t stage, float frequency, float q = 1.0) {

AudioFilterFIR
e*,NULL // void end(void) {

AudioFilterLadder
f*,f // void frequency(float FC);
inp*,f // void inputDrive(float drv);
int*,i // void interpolationMethod(AudioFilterLadderInterpolation im);
o*,f // void octaveControl(float octaves);
p*,f // void passbandGain(float passbandgain);
r*,f // void resonance(float reson);

AudioFilterStateVariable
f*,f // void frequency(float freq) {
o*,f // void octaveControl(float n) {
r*,f // void resonance(float q) {

AudioInputAnalog

AudioInputAnalogStereo

AudioInputI2S

AudioInputI2S2

AudioInputI2SHex

AudioInputI2SOct

AudioInputI2SQuad

AudioInputPDM

AudioInputPDM2

AudioInputSPDIF3
p*,NULL // static bool pllLocked(void);
s*,NULL // static unsigned int sampleRate(void);

AudioInputTDM

AudioInputTDM2

AudioMixer4
g*,if // void gain(unsigned int channel, float gain) {

AudioOutputADAT
m*,; // static void mute_PCM(const bool mute);

AudioOutputAnalog
a*,i // void analogReference(int ref);

AudioOutputAnalogStereo
a*,i // void analogReference(int ref);

AudioOutputI2S

AudioOutputI2S2

AudioOutputI2SHex

AudioOutputI2SOct

AudioOutputI2SQuad

AudioOutputMQS

AudioOutputPT8211

AudioOutputPT8211_2

AudioOutputPWM

AudioOutputSPDIF
m*,; // static void mute_PCM(const bool mute);

AudioOutputSPDIF2
m*,; // static void mute_PCM(const bool mute);

AudioOutputSPDIF3
m*,; // static void mute_PCM(const bool mute);

AudioOutputTDM

AudioOutputTDM2

AudioPlayMemory
i*,NULL // bool isPlaying(void) { return playing; }
l*,NULL // uint32_t lengthMillis(void);
po*,NULL // uint32_t positionMillis(void);
s*,NULL // void stop(void);

AudioPlayQueue
a*,NULL // bool available(void);
g*,NULL // int16_t * getBuffer(void);
playB*,NULL // void playBuffer(void);

AudioPlaySdRaw
b*,NULL // void begin(void);
i*,NULL // bool isPlaying(void) { return playing; }
l*,NULL // uint32_t lengthMillis(void);
po*,NULL // uint32_t positionMillis(void);
s*,NULL // void stop(void);

AudioPlaySdWav
b*,NULL // void begin(void);
isPa*,NULL // bool isPaused(void);
isPl*,NULL // bool isPlaying(void);
isS*,NULL // bool isStopped(void);
l*,NULL // uint32_t lengthMillis(void);
po*,NULL // uint32_t positionMillis(void);
s*,NULL // void stop(void);
t*,NULL // void togglePlayPause(void);

AudioPlaySerialflashRaw
b*,NULL // void begin(void);
i*,NULL // bool isPlaying(void) { return playing; }
l*,NULL // uint32_t lengthMillis(void);
po*,NULL // uint32_t positionMillis(void);
s*,NULL // void stop(void);

AudioRecordQueue
a*,NULL // int available(void);
b*,NULL // void begin(void) {
c*,NULL // void clear(void);
e*,NULL // void end(void) {
f*,NULL // void freeBuffer(void);
r*,NULL // int16_t * readBuffer(void);

AudioSynthKarplusStrong
noteOf*,f // void noteOff(float velocity) {
noteOn,ff // void noteOn(float frequency, float velocity) {

AudioSynthNoisePink
a*,f // void amplitude(float n) {

AudioSynthNoiseWhite
a*,f // void amplitude(float n) {

AudioSynthSimpleDrum
f*,f // void frequency(float freq)
l*,i // void length(int32_t milliseconds)
n*,NULL // void noteOn();
p*,f // void pitchMod(float depth);
s*,f // void secondMix(float level);

AudioSynthToneSweep
i*,NULL // unsigned char isPlaying(void);
p*,fiif // boolean play(float t_amp,int t_lo,int t_hi,float t_time);
r*,NULL // float read(void) {

AudioSynthWaveform
am*,f // void amplitude(float n) {	// 0 to 1.0
b*,ffi // void begin(float t_amp, float t_freq, short t_type) {
b*,i // void begin(short t_type) {
f*,f // void frequency(float freq) {
o*,f // void offset(float n) {
ph*,f // void phase(float angle) {
pu*,f // void pulseWidth(float n) {	// 0.0 to 1.0

AudioSynthWaveformDc
a*,f // void amplitude(float n) {
a*,ff // void amplitude(float n, float milliseconds) {
r*,NULL // float read(void) {

AudioSynthWaveformModulated
am*,f // void amplitude(float n) {	// 0 to 1.0
b*,ffi // void begin(float t_amp, float t_freq, short t_type) {
b*,i // void begin(short t_type) {
frequencyM*,f // void frequencyModulation(float octaves) {
frequency,f // void frequency(float freq) {
o*,f // void offset(float n) {
p*,f // void phaseModulation(float degrees) {

AudioSynthWaveformPWM
a*,f // void amplitude(float n) {
f*,f // void frequency(float freq) {

AudioSynthWaveformSine
a*,f // void amplitude(float n) {
f*,f // void frequency(float freq) {
p*,f // void phase(float angle) {

AudioSynthWaveformSineHires
a*,f // void amplitude(float n) {
f*,f // void frequency(float freq) {
p*,f // void phase(float angle) {

AudioSynthWaveformSineModulated
a*,f // void amplitude(float n) {
f*,f // void frequency(float freq) {
p*,f // void phase(float angle) {

AudioSynthWavetable
a*,f // void amplitude(float v) {
f*,f // static int freqToNote(float freq) {
g*,NULL // envelopeStateEnum getEnvState(void) { return env_state; }
i*,NULL // bool isPlaying(void) { return env_state != STATE_IDLE; }
m*,i // static float midi_volume_transform(int midi_amp) {
n*,i // static float noteToFreq(int note) {
playF*,fi // void playFrequency(float freq, int amp = DEFAULT_AMPLITUDE);
playN*,ii // void playNote(int note, int amp = DEFAULT_AMPLITUDE);
setF*,f // void setFrequency(float freq);
st*,NULL // void stop(void);