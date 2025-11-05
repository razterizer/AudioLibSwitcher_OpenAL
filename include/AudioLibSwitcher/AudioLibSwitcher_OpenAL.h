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
    
    struct SourceState
    {
      float alpha            = 0.f;  // [0, 1].
      float sharpness        = 1.f;  // [1, 8].
      int type               = 0; // [0, 3].
      float rear_attenuation = 1.f; // [0, 1]
    };
    std::unordered_map<unsigned int, SourceState> m_source_states;
    
    int listener_cs_conv = 0; // default RH_XRight_YUp_ZBackward
    
    void apply_coordsys_convention(std::array<float, 9>& rot_mtx, int cs_conv) const
    {
      switch (cs_conv)
      {
        case 0: // RH_XRight_YUp_ZBackward.
          // Default convention: No change.
          break;
          
        case 1: // RH_XLeft_YUp_ZForward.
          // Flip X and Z axes.
          rot_mtx[0] = -rot_mtx[0];
          rot_mtx[1] = -rot_mtx[1];
          rot_mtx[2] = -rot_mtx[2];
          rot_mtx[6] = -rot_mtx[6];
          rot_mtx[7] = -rot_mtx[7];
          rot_mtx[8] = -rot_mtx[8];
          break;
          
        case 2: // RH_XRight_YDown_ZForward.
          // Flip Y and Z axes.
          rot_mtx[3] = -rot_mtx[3];
          rot_mtx[4] = -rot_mtx[4];
          rot_mtx[5] = -rot_mtx[5];
          rot_mtx[6] = -rot_mtx[6];
          rot_mtx[7] = -rot_mtx[7];
          rot_mtx[8] = -rot_mtx[8];
          break;
          
        case 3: // RH_XLeft_YDown_ZBackward.
          // Flip X and Y axes.
          rot_mtx[0] = -rot_mtx[0];
          rot_mtx[1] = -rot_mtx[1];
          rot_mtx[2] = -rot_mtx[2];
          rot_mtx[3] = -rot_mtx[3];
          rot_mtx[4] = -rot_mtx[4];
          rot_mtx[5] = -rot_mtx[5];
          break;
          
        case 4: // RH_XRight_YForward_ZUp.
          // Swap Y and Z axes (Y→Z, Z→Y).
        {
          std::array<float, 9> old = rot_mtx;
          rot_mtx[3] = old[6]; rot_mtx[4] = old[7]; rot_mtx[5] = old[8]; // up = old forward
          rot_mtx[6] = old[3]; rot_mtx[7] = old[4]; rot_mtx[8] = old[5]; // forward = old up
          break;
        }
          
        default:
          std::cerr << "ERROR: Invalid coordinate system convention " << cs_conv << std::endl;
      }
    }
    
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
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
      alSourcePlay(src_id);
    }
    
    virtual std::optional<bool> is_source_playing(unsigned int src_id) override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
    
      ALint state = 0;
      alGetSourcei(src_id, AL_SOURCE_STATE, &state);
      
      // Check for OpenAL errors
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      return state == AL_PLAYING;
    }
    
    virtual void pause_source(unsigned int src_id) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
      alSourcePause(src_id);
    }
    
    virtual std::optional<bool> is_source_paused(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
      
      ALint state = 0;
      alGetSourcei(src_id, AL_SOURCE_STATE, &state);
      
      // Check for OpenAL errors
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      return state == AL_PAUSED;
    }
    
    virtual void stop_source(unsigned int src_id) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
        
      alSourceStop(src_id);
    }
    
    virtual void set_source_gain(unsigned int src_id, float gain) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
      alSourcef(src_id, AL_GAIN, gain);
    }
    
    virtual std::optional<float> get_source_gain(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
      
      ALfloat gain = 0.0f;
      alGetSourcef(src_id, AL_GAIN, &gain);
      
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      return gain;
    }
    
    virtual void set_source_pitch(unsigned int src_id, float pitch) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
      alSourcef(src_id, AL_PITCH, pitch);
    }
    
    virtual std::optional<float> get_source_pitch(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
      
      ALfloat pitch = 0.0f;
      alGetSourcef(src_id, AL_PITCH, &pitch);
      
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      return pitch;
    }
    
    virtual void set_source_looping(unsigned int src_id, bool loop) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
      alSourcei(src_id, AL_LOOPING, loop ? AL_TRUE : AL_FALSE);
    }
    
    virtual std::optional<bool> get_source_looping(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
      
      ALint looping = AL_FALSE;
      alGetSourcei(src_id, AL_LOOPING, &looping);
      
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      return static_cast<bool>(looping);
    }
    
    virtual void set_source_standard_params(unsigned int src_id) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
        
      // Set source parameters (adjust as needed)
      alSourcef(src_id, AL_PITCH, 1.0f);
      alSourcef(src_id, AL_GAIN, 1.0f);
      alSource3f(src_id, AL_POSITION, 0.0f, 0.0f, 0.0f);
      alSource3f(src_id, AL_VELOCITY, 0.0f, 0.0f, 0.0f);
      alSourcei(src_id, AL_LOOPING, AL_FALSE); // Adjust as needed
    }
    
    virtual bool set_buffer_data_8u(unsigned int buf_id, const std::vector<unsigned char>& buffer, int num_channels, int sample_rate) override
    {
      if (!alIsBuffer(buf_id))
        return false;  // Invalid handle — doesn't exist.
    
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
      if (!alIsBuffer(buf_id))
        return false;  // Invalid handle — doesn't exist.
    
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
      if (!alIsBuffer(buf_id))
        return false;  // Invalid handle — doesn't exist.
    
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
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
      if (!alIsBuffer(buf_id))
        return;  // Invalid handle — doesn't exist.
    
      // Attach buffer to source.
      alSourcei(src_id, AL_BUFFER, buf_id);
    }
    
    virtual void detach_buffer_from_source(unsigned int src_id) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
      alSourcei(src_id, AL_BUFFER, 0);
    }
    
    virtual void set_source_panning(unsigned int src_id, std::optional<float> pan = std::nullopt) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
      // #FHXFTW!
      if (!pan.has_value())
        return;
        
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
        
      // For safety, only pan mono sources (stereo sources ignore spatialization).
      ALint buffer = 0;
      alGetSourcei(src_id, AL_BUFFER, &buffer);
      
      if (alGetError() != AL_NO_ERROR || buffer == 0)
        return;
      
      ALint channels = 0;
      alGetBufferi(buffer, AL_CHANNELS, &channels);
      
      if (alGetError() != AL_NO_ERROR || channels != 1)
        return; // Only pan mono sources.
      
      // Clamp pan [-1, 1].
      float p = std::max(-1.f, std::min(1.f, pan.value()));
      
      // Move the mono source along X-axis for simple left-right pan.
      alSource3f(src_id, AL_POSITION, p, 0.f, 0.f);
    }
    
    virtual std::optional<float> get_source_panning(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
      
      ALfloat x = 0.f, y = 0.f, z = 0.f;
      alGetSource3f(src_id, AL_POSITION, &x, &y, &z);
      
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      // Clamp and normalize to [-1, 1] range
      return std::max(-1.f, std::min(1.f, x));
    }
    
    virtual void init_3d_scene() override
    {
      alDopplerFactor(1.0f); // default
    }

    virtual void enable_source_3d_audio(unsigned int src_id, bool enable) override
    {
      if (!alIsSource(src_id))
        return;  // Invalid handle — doesn't exist.
    
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
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
    
      if (channel != 1)
      {
        std::cerr << "WARNING in set_source_3d_state_channel() : 3D state can only be applied to mono sources (channel 1)." << std::endl;
        return false;
      }
      
      alSource3f(src_id, AL_POSITION, pos_world[0], pos_world[1], pos_world[2]);
      alSource3f(src_id, AL_VELOCITY, vel_world[0], vel_world[1], vel_world[2]);
      
      return true;
    }
    
    // std::array<float, 9> is a row-major 3x3 matrix.
    virtual bool get_source_3d_state_channel(unsigned int src_id, int channel,
        std::array<float, 9>& rot_mtx,
        std::array<float, 3>& pos_world, std::array<float, 3>& vel_world) const override
    {
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
      
      if (channel != 1)
      {
        std::cerr << "WARNING in get_source_3d_state_channel() : 3D state can only be applied to mono sources (channel 1)." << std::endl;
        return false;
      }
      
      ALfloat x = 0.f, y = 0.f, z = 0.f;
      alGetSource3f(src_id, AL_POSITION, &x, &y, &z);
      pos_world[0] = x;
      pos_world[1] = y;
      pos_world[2] = z;
      alGetSource3f(src_id, AL_VELOCITY, &x, &y, &z);
      vel_world[0] = x;
      vel_world[1] = y;
      vel_world[2] = z;
      
      return true;
    }
    
    // std::array<float, 9> is a row-major 3x3 matrix.
    // channel = 0 is the only valid value here.
    virtual bool set_listener_3d_state_channel(
        int channel,
        const std::array<float, 9>& rot_mtx_in,
        const std::array<float, 3>& pos_world,
        const std::array<float, 3>& vel_world) override
    {
      if (channel != 1)
      {
        std::cerr << "WARNING in set_listener_3d_state_channel() : 3D state can only be applied to mono sources (channel 1)." << std::endl;
        return false;
      }
      
      // Position & velocity.
      alListener3f(AL_POSITION, pos_world[0], pos_world[1], pos_world[2]);
      alListener3f(AL_VELOCITY, vel_world[0], vel_world[1], vel_world[2]);
      
      // Copy and adapt rotation.
      std::array<float, 9> rot_mtx = rot_mtx_in;
      apply_coordsys_convention(rot_mtx, listener_cs_conv);
      
      // Convert rotation matrix to forward & up vectors.
      // Assuming rot_mtx is row-major 3x3.
      // Row 0 = X axis, Row 1 = Y axis, Row 2 = Z axis.
      ALfloat forward[3] = { -rot_mtx[6], -rot_mtx[7], -rot_mtx[8] }; // -Z
      ALfloat up[3]      = { rot_mtx[3],  rot_mtx[4],  rot_mtx[5] };  // Y
      
      ALfloat orientation[6] = {
        forward[0], forward[1], forward[2],
        up[0],      up[1],      up[2]
      };
      alListenerfv(AL_ORIENTATION, orientation);
      
      return true;
    }
    
    // std::array<float, 9> is a row-major 3x3 matrix.
    virtual bool get_listener_3d_state_channel(int channel,
        std::array<float, 9>& rot_mtx,
        std::array<float, 3>& pos_world, std::array<float, 3>& vel_world) const override
    {
      if (channel != 1)
      {
        std::cerr << "WARNING in get_listener_3d_state_channel() : 3D state can only be applied to mono sources (channel 1)." << std::endl;
        return false;
      }
      
      // Position & velocity.
      ALfloat x = 0.f, y = 0.f, z = 0.f;
      alGetListener3f(AL_POSITION, &x, &y, &z);
      pos_world[0] = x;
      pos_world[1] = y;
      pos_world[2] = z;
      alGetListener3f(AL_VELOCITY, &x, &y, &z);
      vel_world[0] = x;
      vel_world[1] = y;
      vel_world[2] = z;
      
      ALfloat orientation[6];
      alGetListenerfv(AL_ORIENTATION, orientation);
      
      std::array<float, 3> forward = { orientation[0], orientation[1], orientation[2] };
      std::array<float, 3> up = { orientation[3], orientation[4], orientation[5] };
      
      std::array<float, 3> right =
      {
        up[1]*forward[2] - up[2]*forward[1],
        up[2]*forward[0] - up[0]*forward[2],
        up[0]*forward[1] - up[1]*forward[0]
      };
      
      rot_mtx =
      {
        right[0],   right[1],   right[2],
        up[0],      up[1],      up[2],
        forward[0], forward[1], forward[2]
      };
      
      // Undo convention transform to return caller’s space.
      apply_coordsys_convention(rot_mtx, listener_cs_conv);
      
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
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
        
      alSourcef(src_id, AL_REFERENCE_DISTANCE, min_dist);
      return true;
    }
    
    virtual std::optional<float> get_source_attenuation_min_distance(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
        
      ALfloat min_dist = 0.0f;
      alGetSourcef(src_id, AL_REFERENCE_DISTANCE, &min_dist);
      
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      return min_dist;
    }
    
    virtual bool set_source_attenuation_max_distance(unsigned int src_id, float max_dist) override
    {
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
    
      alSourcef(src_id, AL_MAX_DISTANCE, max_dist);
      return true;
    }
    
    virtual std::optional<float> get_source_attenuation_max_distance(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
        
      ALfloat max_dist = 0.0f;
      alGetSourcef(src_id, AL_REFERENCE_DISTANCE, &max_dist);
      
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      return max_dist;
    }
    
    virtual bool set_source_attenuation_constant_falloff(unsigned int src_id, float const_falloff) override
    {
      // OpenAL does not support separate constant term directly.
      // You could adjust the ROLLOFF_FACTOR slightly to simulate.
      return true;
    }
    
    virtual std::optional<float> get_source_attenuation_constant_falloff(unsigned int src_id) const override
    {
      return std::nullopt;
    }
    
    virtual bool set_source_attenuation_linear_falloff(unsigned int src_id, float lin_falloff) override
    {
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
    
      alSourcef(src_id, AL_ROLLOFF_FACTOR, lin_falloff);
      return true;
    }
    
    virtual std::optional<float> get_source_attenuation_linear_falloff(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
        
      ALfloat lin_falloff = 0.0f;
      alGetSourcef(src_id, AL_ROLLOFF_FACTOR, &lin_falloff);
      
      if (alGetError() != AL_NO_ERROR)
        return std::nullopt;
      
      return lin_falloff;
    }
    
    virtual bool set_source_attenuation_quadratic_falloff(unsigned int /*src_id*/, float /*sq_falloff*/) override
    {
      // Standard OpenAL does not have separate quadratic factor.
      // You could combine with AL_ROLLOFF_FACTOR to approximate more complex curves,
      // but usually you just choose AL_INVERSE_DISTANCE or AL_INVERSE_DISTANCE_CLAMPED.
      return true;
    }
    
    virtual std::optional<float> get_source_attenuation_quadratic_falloff(unsigned int /*src_id*/) const override
    {
      return std::nullopt;
    }
    
    // directivity_alpha = 0 : Omni.
    // directivity_alpha = 1 : Fully Directional.
    // [0, 1].
    virtual bool set_source_directivity_alpha(unsigned int src_id, float directivity_alpha) override
    {
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
    
      directivity_alpha = std::clamp(directivity_alpha, 0.f, 1.f);
      m_source_states[src_id].alpha = directivity_alpha;
      
      // Map to OpenAL cone angles
      float inner_angle = 360.f - 300.f * directivity_alpha; // 0 = omni (360°), 1 = narrow
      float outer_angle = 360.f - 150.f * directivity_alpha;
      alSourcef(src_id, AL_CONE_INNER_ANGLE, inner_angle);
      alSourcef(src_id, AL_CONE_OUTER_ANGLE, outer_angle);
      return true;
    }
    
    // directivity_alpha = 0 : Omni.
    // directivity_alpha = 1 : Fully Directional.
    // [0, 1].
    virtual std::optional<float> get_source_directivity_alpha(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
        
      auto it = m_source_states.find(src_id);
      if (it != m_source_states.end())
        return it->second.alpha;
        
      return std::nullopt;
        
      //ALfloat inner_angle = 0.0f;
      //alGetSourcef(src_id, AL_CONE_INNER_ANGLE, &inner_angle);
      //
      //if (alGetError() != AL_NO_ERROR)
      //  return std::nullopt;
      //
      //return (360.f - inner_angle) / 300.f;
    }
    
    // [1, 8]. 8 = sharpest.
    virtual bool set_source_directivity_sharpness(unsigned int src_id, float directivity_sharpness) override
    {
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
    
      directivity_sharpness = std::clamp(directivity_sharpness, 1.f, 8.f);
      m_source_states[src_id].sharpness = directivity_sharpness;
      
      // Map to outer gain (linear approximation)
      float outer_gain = std::clamp(1.f - (directivity_sharpness - 1.f) / 7.f, 0.2f, 1.f);
      alSourcef(src_id, AL_CONE_OUTER_GAIN, outer_gain);
      return true;
    }
    
    // [1, 8]. 8 = sharpest.
    virtual std::optional<float> get_source_directivity_sharpness(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
        
      auto it = m_source_states.find(src_id);
      if (it != m_source_states.end())
        return it->second.sharpness;
        
      return std::nullopt;
        
      //ALfloat outer_gain = 0.0f;
      //alGetSourcef(src_id, AL_CONE_OUTER_GAIN, &outer_gain);
      //
      //if (alGetError() != AL_NO_ERROR)
      //  return std::nullopt;
      //
      //return (1.f - outer_gain) * 7.f + 1.f;
    }
    
    // 0 = Cardioid, 1 = SuperCardioid, 2 = HalfRectifiedDipole, 3 = Dipole.
    virtual bool set_source_directivity_type(unsigned int src_id, int directivity_type) override
    {
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
    
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
    
    // 0 = Cardioid, 1 = SuperCardioid, 2 = HalfRectifiedDipole, 3 = Dipole.
    virtual std::optional<int> get_source_directivity_type(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
        
      auto it = m_source_states.find(src_id);
      if (it != m_source_states.end())
        return it->second.type;
        
      return std::nullopt;
    }
    
    // [0.f, 1.f]. 0 = Silence, 1 = No Attenuation.
    virtual bool set_source_rear_attenuation(unsigned int src_id, float rear_attenuation) override
    {
      if (!alIsSource(src_id))
        return false;  // Invalid handle — doesn't exist.
    
      rear_attenuation = std::clamp(rear_attenuation, 0.f, 1.f);
      m_source_states[src_id].rear_attenuation = rear_attenuation;
      
      // Map to OpenAL outer gain: 0 = silent behind, 1 = no attenuation
      float outer_gain = rear_attenuation;
      alSourcef(src_id, AL_CONE_OUTER_GAIN, outer_gain);
      
      return true;
    }
    
    // [0.f, 1.f]. 0 = Silence, 1 = No Attenuation.
    virtual std::optional<float> get_source_rear_attenuation(unsigned int src_id) const override
    {
      if (!alIsSource(src_id))
        return std::nullopt;  // Invalid handle — doesn't exist.
        
      auto it = m_source_states.find(src_id);
      if (it != m_source_states.end())
        return it->second.rear_attenuation;
        
      return std::nullopt;
    }
    
    // [0.f, 1.f]. 0 = Silence, 1 = No Attenuation.
    virtual bool set_listener_rear_attenuation(float /*rear_attenuation*/) override
    {
      // No native OpenAL support for listener rear attenuation
      // Store locally; can be applied in post-processing if desired
      // m_listener_rear_attenuation = std::clamp(rear_attenuation, 0.f, 1.f);
      return true;
    }
    
    // [0.f, 1.f]. 0 = Silence, 1 = No Attenuation.
    virtual std::optional<float> get_listener_rear_attenuation() const override
    {
      return std::nullopt;
    }
    
    // 0 = RH_XRight_YUp_ZBackward, 1 = RH_XLeft_YUp_ZForward, 2 = RH_XRight_YDown_ZForward, 3 = RH_XLeft_YDown_ZBackward, 4 = RH_XRight_YForward_ZUp.
    virtual bool set_source_coordsys_convention(unsigned int src_id, int cs_conv) override
    {
      return false;
    }
    
    // 0 = RH_XRight_YUp_ZBackward, 1 = RH_XLeft_YUp_ZForward, 2 = RH_XRight_YDown_ZForward, 3 = RH_XLeft_YDown_ZBackward, 4 = RH_XRight_YForward_ZUp.
    virtual std::optional<int> get_source_coordsys_convention(unsigned int src_id) const override
    {
      return std::nullopt;
    }
    
    // 0 = RH_XRight_YUp_ZBackward, 1 = RH_XLeft_YUp_ZForward, 2 = RH_XRight_YDown_ZForward, 3 = RH_XLeft_YDown_ZBackward, 4 = RH_XRight_YForward_ZUp.
    virtual bool set_listener_coordsys_convention(int cs_conv) override
    {
      cs_conv = std::clamp(cs_conv, 0, 4);
      listener_cs_conv = cs_conv;
      return true;
    }
    
    // 0 = RH_XRight_YUp_ZBackward, 1 = RH_XLeft_YUp_ZForward, 2 = RH_XRight_YDown_ZForward, 3 = RH_XLeft_YDown_ZBackward, 4 = RH_XRight_YForward_ZUp.
    virtual std::optional<int> get_listener_coordsys_convention() const override
    {
      return listener_cs_conv;
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
