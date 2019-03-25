/*
todo: new things

bus class
pattern oscillator - patternOsc.play([1,3,4,5,1,3],800ms)
gate oscillator - gateOsc.play([1,0,0,0,0,1,0,0], 400ms)

*/
/*
 *  maximilian.h
 *  platform independent synthesis library using portaudio or rtaudio
 *
 *  Created by Mick Grierson on 29/12/2009.
 *  Copyright 2009 Mick Grierson & Strangeloop Limited. All rights reserved.
 *  Thanks to the Goldsmiths Creative Computing Team.
 *  Special thanks to Arturo Castro for the PortAudio implementation.
 *
 *  Permission is hereby granted, free of charge, to any person
 *  obtaining a copy of this software and associated documentation
 *  files (the "Software"), to deal in the Software without
 *  restriction, including without limitation the rights to use,
 *  copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the
 *  Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be
 *  included in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *  OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef MAXIMILIAN_H
#define MAXIMILIAN_H

//#define MAXIMILIAN_PORTAUDIO
#define MAXIMILIAN_RT_AUDIO


#include <iostream>
#include <fstream>
#include <string.h>
#include <cstdlib>
#include "math.h"
#include <vector>
#ifdef _WIN32 //|| _WIN64
#include <algorithm>
#endif

using namespace std;
#ifndef PI
#define PI  3.1415926535897932384626433832795
#endif
#define TWOPI 6.283185307179586476925286766559

class maxiSettings {
public:
  static int sampleRate;
  static int channels;
  static int bufferSize;
  static void setup(int initSampleRate, int initChannels, int initBufferSize) {
    maxiSettings::sampleRate = initSampleRate;
    maxiSettings::channels = initChannels;
    maxiSettings::bufferSize = initBufferSize;
  }

  void setSampleRate(int sampleRate_){
    sampleRate = sampleRate_;
  }

  void setNumChannels(int channels_){
    channels = channels_;
  }

  void setBufferSize(int bufferSize_){
    bufferSize = bufferSize_;
  }

  int getSampleRate() const{
    return sampleRate;
  }

  int getNumChannels() const{
    return channels;
  }

  int getBufferSize() const{
    return bufferSize;
  }
};


class maxiOsc {

  float frequency;
  float phase;
  float startphase;
  float endphase;
  float output;
  float tri;


public:
  maxiOsc();
  float sinewave(float frequency);
  float coswave(float frequency);
  float phasor(float frequency);
  float phasor(float frequency, float startphase, float endphase);
  float saw(float frequency);
  float triangle(float frequency);
  float square(float frequency);
  float pulse(float frequency, float duty);
  float noise();
  float sinebuf(float frequency);
  float sinebuf4(float frequency);
  float sawn(float frequency);
  float rect(float frequency, float duty=0.5);
  void phaseReset(float phaseIn);

};


class maxiEnvelope {

  float period;
  float output;
  float startval;
  float currentval;
  float nextval;
  int isPlaying;

public:
//  float line(int numberofsegments,float segments[100]);
  float line(int numberofsegments, std::vector<float>& segments);

  void trigger(int index,float amp);
  int valindex;
  float amplitude;

  // ------------------------------------------------
  // getters/setters
  void setValindex(int index){
    valindex = index;
  }

  void setAmplitude(float amp){
    amplitude = amp;
  }

  int getValindex() const{
    return valindex;
  }

  float getAmplitude() const{
    return amplitude;
  }
  // ------------------------------------------------
};


class maxiDelayline {
  float frequency;
  int phase;
  float startphase;
  float endphase;
  float output;
  float *memory;

public:
  maxiDelayline();
  ~maxiDelayline();
  float dl(float input, int size, float feedback);
  float dl(float input, int size, float feedback, int position);


};


class maxiFilter {
  float gain;
  float input;
  float output;
  float inputs[10];
  float outputs[10];
  float cutoff1;
  float x;//speed
  float y;//pos
  float z;//pole
  float c;//filter coefficient

public:
  maxiFilter():x(0.0), y(0.0), z(0.0), c(0.0){};
  float cutoff;
  float resonance;
  float lores(float input,float cutoff1, float resonance);
  float hires(float input,float cutoff1, float resonance);
  float bandpass(float input,float cutoff1, float resonance);
  float lopass(float input,float cutoff);
  float hipass(float input,float cutoff);

  // ------------------------------------------------
  // getters/setters
  void setCutoff(float cut){
    cutoff = cut;
  }

  void setResonance(float res){
    resonance = res;
  }

  float getCutoff() const{
    return cutoff;
  }

  float getResonance() const{
    return resonance;
  }
  // ------------------------------------------------
};

class maxiMix  {
  float input;
  float two[2];
  float four[4];
  float eight[8];
public:
//  float x;
//  float y;
//  float z;

  // ------------------------------------------------
  // getters/setters

  // ------------------------------------------------

//  float *stereo(float input,float two[2],float x);
//  float *quad(float input,float four[4], float x,float y);
//  float *ambisonic(float input,float eight[8],float x,float y, float z);

  // should return or just be void function
  void stereo(float input,std::vector<float>& two,float x);
  void quad(float input,std::vector<float>& four, float x,float y);
  void ambisonic(float input,std::vector<float>& eight, float x,float y, float z);
};

//lagging with an exponential moving average
//a lower alpha value gives a slower lag
template <class T>
class maxiLagExp {
public:
  T alpha, alphaReciprocal;
  T val;

  maxiLagExp() {
    init(0.5, 0.0);
  };

  maxiLagExp(T initAlpha, T initVal) {
    init(initAlpha, initVal);
  }

  void init(T initAlpha, T initVal) {
    alpha = initAlpha;
    alphaReciprocal = 1.0 - alpha;
    val = initVal;
  }

  inline void addSample(T newVal) {
    val = (alpha * newVal) + (alphaReciprocal * val);
  }


  // getters/setters
  void setAlpha(T alpha_){
    alpha = alpha_;
  }

  void setAlphaReciprocal(T alphaReciprocal_){
    alphaReciprocal = alphaReciprocal_;
  }

  void setVal(T val_){
    val = val_;
  }

  T getAlpha() const{
    return alpha;
  }

  T getAlphaReciprocal() const{
    return alphaReciprocal;
  }

  inline T value() const{
    return val;
  }


};


class maxiSample  {

private:
  string  myPath;
  int   myChunkSize;
  int mySubChunk1Size;
  int   readChannel;
  short   myFormat;
  int     myByteRate;
  short   myBlockAlign;
  float position, recordPosition;
  float speed;
  float output;
    maxiLagExp<float> loopRecordLag;

public:
  int myDataSize;
  short   myChannels;
  int     mySampleRate;
  long length;
  void getLength(); // why void??
    void setLength(unsigned long numSamples);
    short   myBitsPerSample;

  vector<float> tempVec;
  vector<short> temp;
//  char*   myData;
//    short* temp;

  // get/set for the Path property

  ~maxiSample()
  {
//    if (myData) free(myData);
//        if (temp) free(temp);
    temp.clear();
    tempVec.clear();
        printf("freeing SampleData");

  }

    maxiSample():temp(NULL),position(0), recordPosition(0), myChannels(1), mySampleRate(maxiSettings::sampleRate) {};

    maxiSample& operator=(const maxiSample &source) {
        if (this == &source)
            return *this;
        position=0;
        recordPosition = 0;
        myChannels = source.myChannels;
        mySampleRate = maxiSettings::sampleRate;
//        free(temp);
    temp.clear();
        myDataSize = source.myDataSize;
//        temp = (short*) malloc(myDataSize * sizeof(char));
    temp = source.temp;
//        memcpy(temp, source.temp, myDataSize * sizeof(char));
        length = source.length;
        return *this;
    }

  bool load(string fileName, int channel=0);

    bool loadOgg(string filename,int channel=0);

  // -------------------------
  // js bits
  // as loading is currently asynchronous in js this can be useful
  bool isReady();

  void setSample(vector<float>& temp);
  void setSample(vector<float>& temp, int sampleRate);

//  void setSampleChar(vector<char>& temp);
  void clear(){temp.clear();}
  // -------------------------

  void trigger();

  // read a wav file into this class
  bool read();

//  bool read(vector<char>& fileChars);

  //read an ogg file into this class using stb_vorbis
    bool readOgg();

    void loopRecord(float newSample, const bool recordEnabled, const float recordMix, float start = 0.0, float end = 1.0) {
        loopRecordLag.addSample(recordEnabled);
        if (recordPosition < start * length) recordPosition = start * length;
        if(recordEnabled) {
            float currentSample = temp[(unsigned long)recordPosition] / 32767.0;
            newSample = (recordMix * currentSample) + ((1.0 - recordMix) * newSample);
            newSample *= loopRecordLag.value();
            temp[(unsigned long)recordPosition] = newSample * 32767;
        }
        ++recordPosition;
        if (recordPosition >= end * length)
            recordPosition= start * length;
    }

//    void clear();

    void reset();

    float play();

    float playLoop(float start, float end); // start and end are between 0.0 and 1.0

    float playOnce();

    float playOnce(float speed);

    void setPosition(float newPos); // between 0.0 and 1.0

    float playUntil(float end);

    float play(float speed);

    float play(float frequency, float start, float end, float &pos);

    float play(float frequency, float start, float end);

    float play4(float frequency, float start, float end);
    /*
    float bufferPlay(unsigned char &bufferin,long length);

    float bufferPlay(unsigned char &bufferin,float speed,long length);

    float bufferPlay(unsigned char &bufferin,float frequency, float start, float end);

    float bufferPlay4(unsigned char &bufferin,float frequency, float start, float end);
   */
    bool save() {
        return save(myPath);
    }

  bool save(string filename)
  {
        fstream myFile (filename.c_str(), ios::out | ios::binary);

        // write the wav file per the wav file format
        myFile.seekp (0, ios::beg);
        myFile.write ("RIFF", 4);
        myFile.write ((char*) &myChunkSize, 4);
        myFile.write ("WAVE", 4);
        myFile.write ("fmt ", 4);
        myFile.write ((char*) &mySubChunk1Size, 4);
        myFile.write ((char*) &myFormat, 2);
        myFile.write ((char*) &myChannels, 2);
        myFile.write ((char*) &mySampleRate, 4);
        myFile.write ((char*) &myByteRate, 4);
        myFile.write ((char*) &myBlockAlign, 2);
        myFile.write ((char*) &myBitsPerSample, 2);
        myFile.write ("data", 4);
        myFile.write ((char*) &myDataSize, 4);
//        myFile.write ((char*) temp, myDataSize);
    myFile.write ((char*) temp.data(), myDataSize);
        return true;
  }

  // return a printable summary of the wav file
  char *getSummary()
  {
    char *summary = new char[250];
    sprintf(summary, " Format: %d\n Channels: %d\n SampleRate: %d\n ByteRate: %d\n BlockAlign: %d\n BitsPerSample: %d\n DataSize: %d\n", myFormat, myChannels, mySampleRate, myByteRate, myBlockAlign, myBitsPerSample, myDataSize);
    std::cout << myDataSize;
    return summary;
  }

    void normalise(float maxLevel = 0.99);  //0 < maxLevel < 1.0
    void autoTrim(float alpha = 0.3, float threshold = 6000, bool trimStart = true, bool trimEnd = true); //alpha of lag filter (lower == slower reaction), threshold to mark start and end, < 32767
};


class maxiMap {
public:
    static float inline linlin(float val, float inMin, float inMax, float outMin, float outMax) {
        val = max(min(val, inMax), inMin);
        return ((val - inMin) / (inMax - inMin) * (outMax - outMin)) + outMin;
    }

    static float inline linexp(float val, float inMin, float inMax, float outMin, float outMax) {
        //clipping
        val = max(min(val, inMax), inMin);
        return pow((outMax / outMin), (val - inMin) / (inMax - inMin)) * outMin;
    }

    static float inline explin(float val, float inMin, float inMax, float outMin, float outMax) {
        //clipping
        val = max(min(val, inMax), inMin);
        return (log(val/inMin) / log(inMax/inMin) * (outMax - outMin)) + outMin;
    }

    //changed to templated function, e.g. maxiMap::maxiClamp<int>(v, l, h);
    template<typename T>
    static T inline clamp(T v, const T low, const T high) {
        if (v > high)
            v = high;
        else if (v < low) {
            v = low;
        }
        return v;
    }

};


class maxiDyn {


public:
//  float gate(float input, float threshold=0.9, long holdtime=1, float attack=1, float release=0.9995);
//  float compressor(float input, float ratio, float threshold=0.9, float attack=1, float release=0.9995);
    float gate(float input, float threshold=0.9, long holdtime=1, float attack=1, float release=0.9995);
    float compressor(float input, float ratio, float threshold=0.9, float attack=1, float release=0.9995);
    float compress(float input);

    float input;
  float ratio;
  float currentRatio;
  float threshold;
  float output;
  float attack;
  float release;
  float amplitude;

    void setAttack(float attackMS);
    void setRelease(float releaseMS);
    void setThreshold(float thresholdI);
    void setRatio(float ratioF);
  long holdtime;
  long holdcount;
  int attackphase,holdphase,releasephase;

  // ------------------------------------------------
  // getters/setters
//  int getTrigger() const{
//    return trigger;
//  }

//  void setTrigger(int trigger){
//    this->trigger = trigger;
//  }

  // ------------------------------------------------
};

class maxiEnv {


public:
  float ar(float input, float attack=1, float release=0.9, long holdtime=1, int trigger=0);
  float adsr(float input, float attack=1, float decay=0.99, float sustain=0.125, float release=0.9, long holdtime=1, int trigger=0);
    float adsr(float input,int trigger);
  float input;
  float output;
  float attack;
  float decay;
  float sustain;
  float release;
  float amplitude;

    void setAttack(float attackMS);
    void setRelease(float releaseMS);
    void setDecay(float decayMS);
    void setSustain(float sustainL);
  int trigger;

  long holdtime=1;
  long holdcount;
  int attackphase,decayphase,sustainphase,holdphase,releasephase;


  // ------------------------------------------------
  // getters/setters
  int getTrigger() const{
    return trigger;
  }

  void setTrigger(int trigger){
    this->trigger = trigger;
  }

  // ------------------------------------------------
};

class convert {
public:
  float mtof(int midinote);
};



class maxiDistortion {
public:
    /*atan distortion, see http://www.musicdsp.org/showArchiveComment.php?ArchiveID=104*/
    /*shape from 1 (soft clipping) to infinity (hard clipping)*/
    float atanDist(const float in, const float shape);
    float fastAtanDist(const float in, const float shape);
    float fastatan( float x );
};

inline float maxiDistortion::fastatan(float x)
{
    return (x / (1.0 + 0.28 * (x * x)));
}

inline float maxiDistortion::atanDist(const float in, const float shape) {
    float out;
    out = (1.0 / atan(shape)) * atan(in * shape);
    return out;
}

inline float maxiDistortion::fastAtanDist(const float in, const float shape) {
    float out;
    out = (1.0 / fastatan(shape)) * fastatan(in * shape);
    return out;
}


class maxiFlanger {
public:
    //delay = delay time - ~800 sounds good
    //feedback = 0 - 1
    //speed = lfo speed in Hz, 0.0001 - 10 sounds good
    //depth = 0 - 1
    float flange(const float input, const unsigned int delay, const float feedback, const float speed, const float depth);
    maxiDelayline dl;
    maxiOsc lfo;

};

inline float maxiFlanger::flange(const float input, const unsigned int delay, const float feedback, const float speed, const float depth)
{
    //todo: needs fixing
    float output;
    float lfoVal = lfo.triangle(speed);
    output = dl.dl(input, delay + (lfoVal * depth * delay) + 1, feedback) ;
    float normalise = (1 - fabs(output));
    output *= normalise;
    return (output + input) / 2.0;
}

class maxiChorus {
public:
    //delay = delay time - ~800 sounds good
    //feedback = 0 - 1
    //speed = lfo speed in Hz, 0.0001 - 10 sounds good
    //depth = 0 - 1
    float chorus(const float input, const unsigned int delay, const float feedback, const float speed, const float depth);
    maxiDelayline dl, dl2;
    maxiOsc lfo;
    maxiFilter lopass;

};

inline float maxiChorus::chorus(const float input, const unsigned int delay, const float feedback, const float speed, const float depth)
{
    //this needs fixing
    float output1, output2;
    float lfoVal = lfo.noise();
    lfoVal = lopass.lores(lfoVal, speed, 1.0) * 2.0;
    output1 = dl.dl(input, delay + (lfoVal * depth * delay) + 1, feedback) ;
    output2 = dl2.dl(input, (delay + (lfoVal * depth * delay * 1.02) + 1) * 0.98, feedback * 0.99) ;
    output1 *= (1.0 - fabs(output1));
    output2 *= (1.0 - fabs(output2));
    return (output1 + output2 + input) / 3.0;
}

template<typename T>
class maxiEnvelopeFollowerType {
public:
    maxiEnvelopeFollowerType() {
        setAttack(100);
        setRelease(100);
        env = 0;
    }
    void setAttack(T attackMS) {
        attack = pow( 0.01, 1.0 / (attackMS * maxiSettings::sampleRate * 0.001 ) );
    }
    void setRelease(T releaseMS) {
        release = pow( 0.01, 1.0 / (releaseMS * maxiSettings::sampleRate * 0.001 ) );
    }
    inline T play(T input) {
        input = fabs(input);
        if (input>env)
            env = attack * (env - input) + input;
        else
            env = release * (env - input) + input;
        return env;
    }
    void reset() {env=0;}
    inline T getEnv(){return env;}
    inline void setEnv(T val){env = val;}
private:
    T attack, release, env;
};

typedef maxiEnvelopeFollowerType<float> maxiEnvelopeFollower;
typedef maxiEnvelopeFollowerType<float> maxiEnvelopeFollowerF;

class maxiDCBlocker {
public:
    float xm1, ym1;
    maxiDCBlocker() : xm1(0), ym1(0) {}
    inline float play(float input, float R) {
        ym1 = input - xm1 + R * ym1;
        xm1 = input;
        return ym1;
    }
};

/*
 State Variable Filter

 algorithm from  http://www.cytomic.com/files/dsp/SvfLinearTrapOptimised.pdf
 usage:
 either set the parameters separately as required (to save CPU)

 filter.setCutoff(param1);
 filter.setResonance(param2);

 w = filter.play(w, 0.0, 1.0, 0.0, 0.0);

 or set everything together at once

 w = filter.setCutoff(param1).setResonance(param2).play(w, 0.0, 1.0, 0.0, 0.0);

 */
class maxiSVF {
public:
    maxiSVF() : v0z(0), v1(0), v2(0) { setParams(1000, 1);}

    //20 < cutoff < 20000
    inline maxiSVF& setCutoff(float cutoff) {
        setParams(cutoff, res);
        return *this;
    }

    //from 0 upwards, starts to ring from 2-3ish, cracks a bit around 10
    inline maxiSVF& setResonance(float q) {
        setParams(freq, q);
        return *this;
    }

    //run the filter, and get a mixture of lowpass, bandpass, highpass and notch outputs
    inline float play(float w, float lpmix, float bpmix, float hpmix, float notchmix) {
        float low, band, high, notch;
        float v1z = v1;
        float v2z = v2;
        float v3 = w + v0z - 2.0 * v2z;
        v1 += g1*v3-g2*v1z;
        v2 += g3*v3+g4*v1z;
        v0z = w;
        low = v2;
        band = v1;
        high = w-k*v1-v2;
        notch = w-k*v1;
        return (low * lpmix) + (band * bpmix) + (high * hpmix) + (notch * notchmix);
    }

private:
    inline void setParams(float _freq, float _res) {
        freq = _freq;
        res = _res;
        g = tanf(PI * freq / maxiSettings::sampleRate);
        damping = res == 0 ? 0 : 1.0 / res;
        k = damping;
        ginv = g / (1.0 + g * (g + k));
        g1 = ginv;
        g2 = 2.0 * (g + k) * ginv;
        g3 = g * ginv;
        g4 = 2.0 * ginv;
    }

    float v0z, v1, v2, g, damping, k, ginv, g1, g2, g3 ,g4;
    float freq, res;

};

class maxiKick {

public:
    maxiKick();
    float play();
    void setPitch(float pitch);
    void setRelease(float releaseD);
    void trigger();
    float pitch;
    float output = 0 ;
    float outputD =0 ;
    float envOut;
    bool useDistortion = false;
    bool useLimiter = false;
    bool useFilter = false;
    float distortion = 0;
    bool inverse = false;
    float cutoff;
    float resonance;
    float gain = 1;
    maxiOsc kick;
    maxiEnv envelope;
    maxiDistortion distort;
    maxiFilter filter;
};

class maxiSnare {
public:
    maxiSnare();
    float play();
    void setPitch(float pitch);
    void setRelease(float releaseD);
    void trigger();
    float pitch;
    float output = 0 ;
    float outputD = 0 ;
    float envOut;
    bool useDistortion = false;
    bool useLimiter = false;
    bool useFilter = true;
    float distortion = 0;
    bool inverse = false;
    float cutoff;
    float resonance;
    float gain = 1;
    maxiOsc tone;
    maxiOsc noise;
    maxiEnv envelope;
    maxiDistortion distort;
    maxiFilter filter;



};

class maxiHats {

public:
    maxiHats();
    float play();
    void setPitch(float pitch);
    void setRelease(float releaseD);
    void trigger();
    float pitch;
    float output = 0;
    float outputD = 0;
    float envOut;
    bool useDistortion = false;
    bool useLimiter = false;
    bool useFilter = false;
    float distortion = 0;
    bool inverse = false;
    float cutoff;
    float resonance;
    float gain = 1;
    maxiOsc tone;
    maxiOsc noise;
    maxiEnv envelope;
    maxiDistortion distort;
    maxiSVF filter;


};


class maxiSynth {



};


class granularSynth {



};


class maxiSampler {

public:
    maxiSampler();
    float play();
    void setPitch(float pitch, bool setall=false);
    void midiNoteOn(float pitch, float velocity, bool setall=false);
    void midiNoteOff(float pitch, float velocity, bool setall=false);
    void setAttack(float attackD,bool setall=true);
    void setDecay(float decayD,bool setall=true);
    void setSustain(float sustainD,bool setall=true);
    void setRelease(float releaseD,bool setall=true);
    void setPosition(float positionD,bool setall=true);
    void load(string inFile,bool setall=true);
    void setNumVoices(int numVoices);
    float position;
    void trigger();
    float pitch[32];
    int originalPitch=67;
    float outputs[32];
    float outputD = 0;
    float envOut[32];
    float envOutGain[32];
    float output;
    bool useDistortion = false;
    bool useLimiter = false;
    bool useFilter = false;
    float distortion = 0;
    bool inverse = false;
    float cutoff;
    float resonance;
    float gain = 1;
    int voices;
    int currentVoice=0;
    convert mtof;
    maxiOsc LFO1;
    maxiOsc LFO2;
    maxiOsc LFO3;
    maxiOsc LFO4;
    maxiSample samples[32];
    maxiEnv envelopes[32];
    maxiDistortion distort;
    maxiSVF filters[32];
    bool sustain = true;


};

class maxiClock {
public:
    maxiClock();
    void ticker();
    void setTempo(float bpm);
    void setTicksPerBeat(int ticksPerBeat);
  bool isTick();

    maxiOsc timer;
    int currentCount;
    int lastCount;
    int playHead;
    float bps;
    float bpm;
    int ticks;
    bool tick;

  // SETTERS
  void setCurrentCount(int n){
    this->currentCount = n;
  }

  void setLastCount(int n){
    this->lastCount = n;
  }

  void setPlayHead(int n){
    this->playHead = n;
  }

  void setBps(int bps_){
    this->bps = bps_;
  }

  void setBpm(int bpm_){
    this->bpm = bpm_;
  }

  void setTick(int tick_){
    this->tick = tick_;
  }

  void setTicks(int ticks_){
    this->ticks = ticks_;
  }


  // GETTERS

  int getCurrentCount() const{
    return currentCount;
  }

  int getLastCount() const{
    return lastCount;
  }

  int getPlayHead() const{
    return playHead;
  }

  float getBps() const{
    return bps;
  }

  float getBpm() const{
    return bpm;
  }
  bool getTick() const{
    return tick;
  }
  int getTicks() const{
    return ticks;
  }

};


#endif
