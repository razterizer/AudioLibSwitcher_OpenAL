//
//  AudioLibSwitcher_OpenAL.h
//  AudioLibSwitcher_OpenAL
//
//  Created by Rasmus Anthin on 2024-03-25.
//

#pragma once
#include "../../AudioLibSwitcher/include/AudioLibSwitcher/IAudioLibSwitcher.h"

#ifdef _MSC_VER
  #include <OpenAL_Soft/al.h>
  #include <OpenAL_Soft/alc.h>
#else
  #include <AL/al.h>
  #include <AL/alc.h>
#endif

// Define FLOAT32 format constants if not provided by the OpenAL headers
#ifndef AL_FORMAT_MONO_FLOAT32
    #define AL_FORMAT_MONO_FLOAT32 0x10010
#endif
#ifndef AL_FORMAT_STEREO_FLOAT32
    #define AL_FORMAT_STEREO_FLOAT32 0x10011
#endif

// You might also want to define the double-precision ones for future use
#ifndef AL_FORMAT_MONO_DOUBLE_EXT
    #define AL_FORMAT_MONO_DOUBLE_EXT 0x10012
#endif
#ifndef AL_FORMAT_STEREO_DOUBLE_EXT
    #define AL_FORMAT_STEREO_DOUBLE_EXT 0x10013
#endif

#include <iostream>
#include <algorithm>


namespace audio
{
  
  class AudioLibSwitcher_OpenAL final : public IAudioLibSwitcher
  {
    ALCdevice* m_device = nullptr;
    ALCcontext* m_context = nullptr;
    
    struct SourceDirectivityState
    {
      float alpha            = 0.f;  // [0, 1].
      float sharpness        = 1.f;  // [1, 8].
      int type               = 0; // [0, 3].
      float rear_attenuation = 1.f; // [0, 1]
    };
    std::unordered_map<unsigned int, SourceDirectivityState> m_source_states;
    
  public:
    virtual void init(bool enable_audio = true) override
    {
      // Initialize OpenAL context and device
      m_device = enable_audio ? alcOpenDevice(nullptr) : alcOpenDevice("null");
      if (m_device == nullptr)
      {
        // Handle error: Unable to open audio device
        std::cerr << "ERROR: Unable to open audio device.\n";
      }
      
      m_context = alcCreateContext(m_device, nullptr);
      if (m_context == nullptr)
      {
        // Handle error: Unable to create audio context
        std::cerr << "ERROR: Unable to create audio context.\n";
      }
      
      if (m_context == nullptr || alcMakeContextCurrent(m_context) == ALC_FALSE)
      {
        std::cerr << "ERROR: Failed to create or activate OpenAL context .\n";
      }
    }
    
    virtual void finish() override
    {
      // Clean up OpenAL resources
      alcMakeContextCurrent(nullptr);
      alcDestroyContext(m_context);
      alcCloseDevice(m_device);
    }
    
    virtual unsigned int create_source() override
    {
      unsigned int source_id = 0;
      // Generate OpenAL source
      alGenSources(1, &source_id);
      return source_id;
    }
    
    virtual void destroy_source(unsigned int src_id) override
    {
      alDeleteSources(1, &src_id);
    }
    
    virtual unsigned int create_buffer() override
    {
      // Generate OpenAL buffer
      unsigned int buffer_id = 0;
      alGenBuffers(1, &buffer_id);
      return buffer_id;
    }
    
    virtual void destroy_buffer(unsigned int buf_id) override
    {
      alDeleteBuffers(1, &buf_id);
    }
    
    virtual void play_source(unsigned int src_id) override
    {
      alSourcePlay(src_id);
    }
    
    virtual std::optional<bool> is_source_playing(unsigned int src_id) override
    {
      ALint source_state = 0;
      alGetSourcei(src_id, AL_SOURCE_STATE, &source_state);
      return source_state == AL_PLAYING;
    }
    
    virtual void pause_source(unsigned int src_id) override
    {
      alSourcePause(src_id);
    }
    
    virtual void stop_source(unsigned int src_id) override
    {
      alSourceStop(src_id);
    }
    
    virtual void set_source_volume(unsigned int src_id, float vol) override
    {
      alSourcef(src_id, AL_GAIN, vol);
    }
    
    virtual void set_source_pitch(unsigned int src_id, float pitch) override
    {
      alSourcef(src_id, AL_PITCH, pitch);
    }
    
    virtual void set_source_looping(unsigned int src_id, bool loop) override
    {
      alSourcei(src_id, AL_LOOPING, loop ? AL_TRUE : AL_FALSE);
    }
    
    virtual void set_source_standard_params(unsigned int src_id) override
    {
      // Set source parameters (adjust as needed)
      alSourcef(src_id, AL_PITCH, 1.0f);
      alSourcef(src_id, AL_GAIN, 1.0f);
      alSource3f(src_id, AL_POSITION, 0.0f, 0.0f, 0.0f);
      alSource3f(src_id, AL_VELOCITY, 0.0f, 0.0f, 0.0f);
      alSourcei(src_id, AL_LOOPING, AL_FALSE); // Adjust as needed
    }
    
    virtual bool set_buffer_data_8u(unsigned int buf_id, const std::vector<unsigned char>& buffer, int num_channels, int sample_rate) override
    {
      ALenum format;
      if (num_channels == 1)
        format = AL_FORMAT_MONO8;
      else if (num_channels == 2)
        format = AL_FORMAT_STEREO8;
      else
      {
        std::cerr << "ERROR: Unsupported number of channels for 8u format!" << std::endl;
        return false;
      }
      alBufferData(buf_id, format, buffer.data(), static_cast<ALsizei>(buffer.size() * sizeof(unsigned char)), sample_rate);
      return true;
    }
    
    virtual bool set_buffer_data_16s(unsigned int buf_id, const std::vector<signed short>& buffer, int num_channels, int sample_rate) override
    {
      ALenum format;
      if (num_channels == 1)
        format = AL_FORMAT_MONO16;
      else if (num_channels == 2)
        format = AL_FORMAT_STEREO16;
      else
      {
        std::cerr << "ERROR: Unsupported number of channels for 16s format!" << std::endl;
        return false;
      }
      alBufferData(buf_id, format, buffer.data(), static_cast<ALsizei>(buffer.size() * sizeof(signed short)), sample_rate);
      return true;
    }
    
    virtual bool set_buffer_data_32f(unsigned int buf_id, const std::vector<float>& buffer, int num_channels, int sample_rate) override
    {
      // 32-bit float is not part of the original OpenAL 1.1 core spec.
      // It is available through the common AL_EXT_FLOAT32 extension.
      // You MUST check for support first.
      
      if (!alIsExtensionPresent("AL_EXT_float32"))
      {
        std::cout << "ERROR: 32-bit float format not supported by this OpenAL implementation!" << std::endl;
        return false;
      }
      
      ALenum format;
      if (num_channels == 1)
        format = AL_FORMAT_MONO_FLOAT32;
      else if (num_channels == 2)
        format = AL_FORMAT_STEREO_FLOAT32;
      else
      {
        std::cerr << "ERROR: Unsupported number of channels for 32f format!" << std::endl;
        return false;
      }
      alBufferData(buf_id, format, buffer.data(), static_cast<ALsizei>(buffer.size() * sizeof(float)), sample_rate);
      return true;
    }
    
    virtual void attach_buffer_to_source(unsigned int src_id, unsigned int buf_id) override
    {
      // Attach buffer to source
      alSourcei(src_id, AL_BUFFER, buf_id);
    }
    
    virtual void detach_buffer_from_source(unsigned int src_id) override
    {
      alSourcei(src_id, AL_BUFFER, 0);
    }
    
    virtual void set_source_panning(unsigned int src_id, std::optional<float> pan = std::nullopt) override
    {
      // #FHXFTW!
      if (!pan.has_value())
        return;
        
      // error: use of undeclared identifier 'AL_FORMAT'
      //ALint buffer;
      //alGetSourcei(src_id, AL_BUFFER, &buffer);
      //ALint format;
      //alGetBufferi(buffer, AL_FORMAT, &format);
      //if (format != AL_FORMAT_MONO8 && format != AL_FORMAT_MONO16)
      //  return;
      
      // Clamp pan [-1, 1]
      float p = std::max(-1.f, std::min(1.f, pan.value()));
      
      // Move the mono source along X-axis for simple left-right pan
      alSource3f(src_id, AL_POSITION, p, 0.f, 0.f);
    }
    
    virtual void init_3d_scene() override
    {
      alDopplerFactor(1.0f); // default
    }

    virtual void enable_source_3d_audio(unsigned int src_id, bool enable) override
    {
      // When AL_SOURCE_RELATIVE is true, positions are relative to listener (no world 3D)
      alSourcei(src_id, AL_SOURCE_RELATIVE, enable ? AL_FALSE : AL_TRUE);
    }

    // std::array<float, 9> is a row-major 3x3 matrix.
    // Only valid value for channel is 1.
    virtual bool set_source_3d_state_channel(
        unsigned int src_id, int channel,
        const std::array<float, 9>& /*rot_mtx*/,
        const std::array<float, 3>& pos_world, const std::array<float, 3>& vel_world) override
    {
      if (channel != 1)
      {
        std::cerr << "WARNING in enable_source_3d_audio : 3D state can only be applied to mono sources (channel 1)." << std::endl;
        return false;
      }
      
      alSource3f(src_id, AL_POSITION, pos_world[0], pos_world[1], pos_world[2]);
      alSource3f(src_id, AL_VELOCITY, vel_world[0], vel_world[1], vel_world[2]);
      
      return true;
    }
    
    // std::array<float, 9> is a row-major 3x3 matrix.
    // channel = 0 is the only valid value here.
    virtual bool set_listener_3d_state_channel(
        int channel,
        const std::array<float, 9>& rot_mtx,
        const std::array<float, 3>& pos_world,
        const std::array<float, 3>& vel_world) override
    {
      if (channel != 0) { // OpenAL listener is one entity
        std::cerr << "Warning: OpenAL supports only one listener." << std::endl;
        return false;
      }
      
      // Position & velocity
      alListener3f(AL_POSITION, pos_world[0], pos_world[1], pos_world[2]);
      alListener3f(AL_VELOCITY, vel_world[0], vel_world[1], vel_world[2]);
      
      // Convert rotation matrix to forward & up vectors
      // Assuming rot_mtx is row-major 3x3
      // Row 0 = X axis, Row 1 = Y axis, Row 2 = Z axis
      ALfloat forward[3] = { -rot_mtx[6], -rot_mtx[7], -rot_mtx[8] }; // -Z
      ALfloat up[3]      = { rot_mtx[3],  rot_mtx[4],  rot_mtx[5] };  // Y
      
      ALfloat orientation[6] = {
        forward[0], forward[1], forward[2],
        up[0],      up[1],      up[2]
      };
      alListenerfv(AL_ORIENTATION, orientation);
      
      return true;
    }
    
    virtual bool set_source_speed_of_sound(unsigned int /*src_id*/, float speed_of_sound) override
    {
      alSpeedOfSound(speed_of_sound);
      return alGetError() != AL_NO_ERROR;
    }
    
    virtual std::optional<float> get_source_speed_of_sound(unsigned int /*src_id*/) override
    {
      return std::optional<float>(alGetFloat(AL_SPEED_OF_SOUND));
    }
    
    virtual bool set_source_attenuation_min_distance(unsigned int src_id, float min_dist) override
    {
      alSourcef(src_id, AL_REFERENCE_DISTANCE, min_dist);
      return true;
    }
    
    virtual bool set_source_attenuation_max_distance(unsigned int src_id, float max_dist) override
    {
      alSourcef(src_id, AL_MAX_DISTANCE, max_dist);
      return true;
    }
    
    virtual bool set_source_attenuation_constant_falloff(unsigned int src_id, float const_falloff) override
    {
      // OpenAL does not support separate constant term directly.
      // You could adjust the ROLLOFF_FACTOR slightly to simulate.
      return true;
    }
    
    virtual bool set_source_attenuation_linear_falloff(unsigned int src_id, float lin_falloff) override
    {
      alSourcef(src_id, AL_ROLLOFF_FACTOR, lin_falloff);
      return true;
    }
    
    virtual bool set_source_attenuation_quadratic_falloff(unsigned int src_id, float sq_falloff) override
    {
      // Standard OpenAL does not have separate quadratic factor.
      // You could combine with AL_ROLLOFF_FACTOR to approximate more complex curves,
      // but usually you just choose AL_INVERSE_DISTANCE or AL_INVERSE_DISTANCE_CLAMPED.
      return true;
    }
    
    // directivity_alpha = 0 : Omni.
    // directivity_alpha = 1 : Fully Directional.
    // [0, 1].
    virtual bool set_source_directivity_alpha(unsigned int src_id, float directivity_alpha) override
    {
      directivity_alpha = std::clamp(directivity_alpha, 0.f, 1.f);
      m_source_states[src_id].alpha = directivity_alpha;
      
      // Map to OpenAL cone angles
      float inner_angle = 360.f - 300.f * directivity_alpha; // 0 = omni (360°), 1 = narrow
      float outer_angle = 360.f - 150.f * directivity_alpha;
      alSourcef(src_id, AL_CONE_INNER_ANGLE, inner_angle);
      alSourcef(src_id, AL_CONE_OUTER_ANGLE, outer_angle);
      return true;
    }
    
    // [1, 8]. 8 = sharpest.
    virtual bool set_source_directivity_sharpness(unsigned int src_id, float directivity_sharpness) override
    {
      directivity_sharpness = std::clamp(directivity_sharpness, 1.f, 8.f);
      m_source_states[src_id].sharpness = directivity_sharpness;
      
      // Map to outer gain (linear approximation)
      float outer_gain = std::clamp(1.f - (directivity_sharpness - 1.f) / 7.f, 0.2f, 1.f);
      alSourcef(src_id, AL_CONE_OUTER_GAIN, outer_gain);
      return true;
    }
    
    // 0 = Cardioid, 1 = SuperCardioid, 2 = HalfRectifiedDipole, 3 = Dipole.
    virtual bool set_source_directivity_type(unsigned int src_id, int directivity_type) override
    {
      m_source_states[src_id].type = directivity_type;
      
      // OpenAL cannot model dipoles/cardioids precisely
      // Only rough approximation: narrow cone for directional, omni for type-agnostic
      if (directivity_type == 0 || directivity_type == 1)
      {
        alSourcef(src_id, AL_CONE_INNER_ANGLE, 60.f);
        alSourcef(src_id, AL_CONE_OUTER_ANGLE, 180.f);
        alSourcef(src_id, AL_CONE_OUTER_GAIN, 0.2f);
      }
      else
      {
        // Omni or unsupported type → full 360° cone
        alSourcef(src_id, AL_CONE_INNER_ANGLE, 360.f);
        alSourcef(src_id, AL_CONE_OUTER_ANGLE, 360.f);
        alSourcef(src_id, AL_CONE_OUTER_GAIN, 1.f);
      }
      
      return true;
    }
    
    // [0.f, 1.f]. 0 = Silence, 1 = No Attenuation.
    virtual bool set_source_rear_attenuation(unsigned int src_id, float rear_attenuation) override
    {
      rear_attenuation = std::clamp(rear_attenuation, 0.f, 1.f);
      m_source_states[src_id].rear_attenuation = rear_attenuation;
      
      // Map to OpenAL outer gain: 0 = silent behind, 1 = no attenuation
      float outer_gain = rear_attenuation;
      alSourcef(src_id, AL_CONE_OUTER_GAIN, outer_gain);
      
      return true;
    }
    
    // [0.f, 1.f]. 0 = Silence, 1 = No Attenuation.
    virtual bool set_listener_rear_attenuation(float rear_attenuation) override
    {
      // No native OpenAL support for listener rear attenuation
      // Store locally; can be applied in post-processing if desired
      // m_listener_rear_attenuation = std::clamp(rear_attenuation, 0.f, 1.f);
      return true;
    }
    
    virtual std::string check_error() override
    {
      ALenum error = alGetError();
      switch (error)
      {
        case AL_NO_ERROR: return "";
        case AL_INVALID_NAME: return "AL_INVALID_NAME";
        case AL_INVALID_ENUM: return "AL_INVALID_ENUM";
        case AL_INVALID_VALUE: return "AL_INVALID_VALUE";
        case AL_INVALID_OPERATION: return "AL_INVALID_OPERATION";
        case AL_OUT_OF_MEMORY: return "AL_OUT_OF_MEMORY";
        default: return "UNKNOWN_AL_ERROR";
      }
    }
  };

}
