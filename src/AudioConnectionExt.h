#include <Audio.h>

#ifndef AUDIOCONNECTIONS_EXT_H
#define AUDIOCONNECTIONS_EXT_H
typedef struct audio_connection_struct
{
    AudioStream &stream;
    unsigned char portIndex;
} audio_connection;


class AudioHalfConnection
{
  public:
    class MultDest
    {
      private:
        audio_connection *dests;
        unsigned char destCount;
        AudioConnection **acs;
      public:  
        bool connected = false;
        MultDest(audio_connection destinations[], unsigned char destinationCount)
        {
            dests = destinations;
            destCount = destinationCount;
            acs = (AudioConnection **)malloc(destCount*sizeof(audio_connection));
        }
        ~MultDest() {
            disconnect();
            delete dests;// are theese the proper way of doing it
            delete acs; 
        }
        int connect(AudioStream &source, unsigned char sourcePortIndex)
        {
            if (connected == true) return 1;
            for (int i=0;i<destCount;i++)
            {
                acs[i] = new AudioConnection(source, sourcePortIndex, dests[i].stream, dests[i].portIndex);
            }
            connected = true;
            return 0;
        }
        int connect(AudioHalfConnection &source)
        {
            if (1 == source._dir) return 2; // this cannot connect to annother input
            return connect(*source._audioStream, source._audioStreamPortIndex);
        }
        void disconnect () {
            for (int i=0;i<destCount;i++)
            {
                acs[i]->disconnect();
            }
        }
    };

  private:
    AudioConnection *ac;
  public:
    AudioStream *_audioStream;
    unsigned char _audioStreamPortIndex;
    unsigned char _dir;
    bool connected = false;

    AudioHalfConnection(unsigned char dir, AudioStream &audioStream, unsigned char audioStreamPortIndex)
    {
        _dir = dir;
        _audioStream = &audioStream;
        _audioStreamPortIndex = audioStreamPortIndex;
    }
    AudioHalfConnection(AudioHalfConnection &other) {
        _dir = other._dir;
        _audioStream = other._audioStream;
        _audioStreamPortIndex = other._audioStreamPortIndex;
    }
    ~AudioHalfConnection() {
        disconnect();
    }
    
    int connect(AudioStream &audioStream, unsigned char audioStreamPortIndex)
    {
        if (true == connected) return 1;
        if (_dir == 0)
            ac = new AudioConnection(*_audioStream, _audioStreamPortIndex, audioStream, audioStreamPortIndex);
        else // should be 1 but as failsafe allow everything
            ac = new AudioConnection(audioStream, audioStreamPortIndex, *_audioStream, _audioStreamPortIndex);
        connected = true;
        return 0;
    }

    int connect(AudioHalfConnection &other) { // this allows classes to connect together
        if (_dir == other._dir) return 2; // cannot connect similar connections together
        if (true == other.connected) return 1; // other is allready connected to something

        return connect(*other._audioStream, other._audioStreamPortIndex);
    }

    int connect(MultDest &other) {
        if (1 == _dir) return 2; // cannot connect a input to a input
        if (true == other.connected) return 1; // other is allready connected to something
        
        return other.connect(*_audioStream, _audioStreamPortIndex);
    }
    void disconnect()
    {
        ac->disconnect();
    }
};
#endif