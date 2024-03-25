//
//  AudioLibSwitcher_OpenAL.h
//  AudioLibSwitcher_OpenAL
//
//  Created by Rasmus Anthin on 2024-03-25.
//

#pragma once
#include "AudioLibSwitcher/IAudioLibSwitcher.h"

#ifdef _MSC_VER
  #include <OpenAL_Soft/al.h>
  #include <OpenAL_Soft/alc.h>
#else
  #include <AL/al.h>
  #include <AL/alc.h>
#endif

#include <iostream>


namespace audio
{
  
  class AudioLibSwitcher_OpenAL final : IAudioLibSwitcher
  {
    ALCdevice* m_device = nullptr;
    ALCcontext* m_context = nullptr;
    
  public:
    virtual void init() override
    {
      // Initialize OpenAL context and device
      m_device = alcOpenDevice(nullptr);
      if (m_device == nullptr)
      {
        // Handle error: Unable to open audio device
        std::cerr << "ERROR: Unable to open audio device in AudioSourceHandler().\n";
      }
      
      m_context = alcCreateContext(m_device, nullptr);
      if (m_context == nullptr)
      {
        // Handle error: Unable to create audio context
        std::cerr << "ERROR: Unable to create audio context in AudioSourceHandler().\n";
      }
      
      alcMakeContextCurrent(m_context);
    }
    
    virtual void finish() override
    {
      // Clean up OpenAL resources
      alcMakeContextCurrent(nullptr);
      alcDestroyContext(m_context);
      alcCloseDevice(m_device);
    }
    
    virtual int create_source() override
    {
      int source_id = 0;
      // Generate OpenAL source
      alGenSources(1, &source_id);
      return source_id;
    }
    
    virtual void destroy_source(int src_id) override
    {
      alDeleteSources(1, &src_id);
    }
    
    virtual int create_buffer() override
    {
      // Generate OpenAL buffer
      int buffer_id = 0;
      alGenBuffers(1, &buffer_id);
      return buffer_id;
    }
    
    virtual void destroy_buffer(int buf_id) override
    {
      alDeleteBuffers(1, &buf_id);
    }
    
    virtual void play_source(int src_id) override
    {
      alSourcePlay(src_id);
    }
    
    virtual bool is_source_playing(int src_id) override
    {
      ALint source_state = 0;
      alGetSourcei(src_id, AL_SOURCE_STATE, &source_state);
      return source_state == AL_PLAYING;
    }
    
    virtual void pause_source(int src_id) override
    {
      alSourcePause(src_id);
    }
    
    virtual void stop_source(int src_id) override
    {
      alSourceStop(src_id);
    }
    
    virtual void set_source_volume(int src_id, float vol) override
    {
      alSourcef(src_id, AL_GAIN, volume);
    }
    
    virtual void set_source_pitch(int src_id float pitch) override
    {
      alSourcef(src_id, AL_PITCH, pitch);
    }
    
    virtual void set_source_looping(int src_id bool loop) override
    {
      alSourcei(src_id, AL_LOOPING, loop ? AL_TRUE : AL_FALSE);
    }
    
    // #NOTE: Might be deprecated.
    virtual void detach_source(int src_id) override
    {
      alSourcei(src_id, AL_BUFFER, 0);
    }
    
    virtual void set_source_standard_params(int src_id) override
    {
      // Set source parameters (adjust as needed)
      alSourcef(src_id, AL_PITCH, 1.0f);
      alSourcef(src_id, AL_GAIN, 1.0f);
      alSource3f(src_id, AL_POSITION, 0.0f, 0.0f, 0.0f);
      alSource3f(src_id, AL_VELOCITY, 0.0f, 0.0f, 0.0f);
      alSourcei(src_id, AL_LOOPING, AL_FALSE); // Adjust as needed
    }
    
    virtual void set_buffer_data_mono_16(int buf_id, const std::vector<short>& buffer, int sample_rate) override
    {
      alBufferData(buf_id, AL_FORMAT_MONO16, buffer.data(), static_cast<ALsizei>(buffer.size() * sizeof(short)), static_cast<ALsizei>(sample_rate));
    }
    
    virtual void attach_buffer_to_source(int src_id, int buf_id) override
    {
      // Attach buffer to source
      alSourcei(src_id, AL_BUFFER, buf_id);
    }
    
    virtual std::string check_error() override
    {
      std::string err_str;
      // Check for errors
      ALenum error = alGetError();
      if (error != AL_NO_ERROR)
      {
        // Handle error
        err_str = alGetString(error);
      }
      return err_str;
    }
  };

}
