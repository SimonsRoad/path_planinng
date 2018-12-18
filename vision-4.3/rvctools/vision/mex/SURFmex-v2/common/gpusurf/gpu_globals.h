/**
 * @file   gpu_globals.h
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 20 20:16:39 2010
 * 
 * @brief  Global configuration variables and structure definitions.
 * 
 * 
 */
 
/*
Copyright (c) 2010, Paul Furgale and Chi Hay Tong
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
  notice, this list of conditions and the following disclaimer in the 
  documentation and/or other materials provided with the distribution.
* The names of its contributors may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef ASRL_GPU_GLOBALS_FTK_H
#define ASRL_GPU_GLOBALS_FTK_H

/**
 * \def ASRL_SURF_MAX_INTERVALS
 * \brief The maximum number of intervals that device memory is reserved for.
 *
 */
#define ASRL_SURF_MAX_INTERVALS 8

/**
 * \def ASRL_SURF_MAX_OCTAVES
 * \brief The maximum number of octaves that device memory is reserved for.
 *
 */
#define ASRL_SURF_MAX_OCTAVES 8

/**
 * \def ASRL_SURF_MAX_FEATURES
 * \brief The maximum number of features that memory is reserved for.
 *
 */
#define ASRL_SURF_MAX_FEATURES 4096

/**
 * \def ASRL_SURF_MAX_CANDIDATES
 * \brief The maximum number of features (before subpixel interpolation) that memory is reserved for.
 *
 */
#define ASRL_SURF_MAX_CANDIDATES 6120

/**
 * \def ASRL_SURF_DESCRIPTOR_DIM
 * \brief The dimension of the SURF descriptor
 *
 */
#define ASRL_SURF_DESCRIPTOR_DIM 64

namespace asrl
{
  // Forward declaration
  class GpuSurfOctave;

  /** 
   * \class SurfOctaveParameters
   * \brief A structure which holds the constant parameters that describe an octave layout.
   *
   */
  struct SurfOctaveParameters
  {
    /// The width of the octave buffer.
    int x_size;
    /// The height of the octave buffer.
    int y_size;
    /// The number of intervals in the octave.
    int nIntervals;
    /// The size of the octave border in pixels.
    int border;
    /// The step size used in this octave in pixels.
    int step;

    /// Mask sizes derived from the mask parameters
    float mask_width;
    /// Mask sizes derived from the mask parameters
    float mask_height;
    /// Mask sizes derived from the mask parameters
    float dxy_center_offset;
    /// Mask sizes derived from the mask parameters
    float dxy_half_width;
    /// Mask sizes derived from the mask parameters
    float dxy_scale;
  };



  /**
   * \class Keypoint
   * \brief A keypoint class used on the GPU.
   *
   *  It is very difficult to use complicated header files with CUDA.
   *  For example, both boost and opencv cause problems. Hence, we must
   *  create our own keypoint type and use that in the CUDA functions.
   */
  class Keypoint
  {
    public:
    Keypoint():
    x(0.f), y(0.f),size(0.f),response(0.f),angle(0.f),octave(0){}
    float x;
    float y;
    float size;
    float response;
    float angle;
    int octave;
  };

  /**
   * The layout of a keypoint so the elements may be grabbed as an array. 
   * 
   */
  enum KeypointLayout {
    SF_X,
    SF_Y,
    SF_SIZE,
    SF_RESPONSE,
    SF_ANGLE,
    SF_OCTAVE,
    SF_FEATURE_STRIDE
  };



  /** 
   * Initialize global variables used by the SURF detector
   * 
   * @param imWidth   The width of the integral image
   * @param imHeight  The height of the integral image
   * @param octaves   The octave parameters
   */
  void init_globals(int imWidth, int imHeight, GpuSurfOctave * octaves, int nOctaves);

  /** 
   * @return A variable at gpusurf.cu file scope that says if the constant memory has been initialized.
   */
  bool & get_s_initialized();

  /** 
   * 
   * @return A variable at gpusurf.cu file scope that holds the initialized image width
   */
  int & get_s_initWidth();

  /** 
   * 
   * @return A variable at gpusurf.cu file scope that holds the initialized image height
   */
  int & get_s_initHeight();

  /** 
   * 
   * @return A __constant__ variable at gpusurf.cu file scope that holds octave parameters
   */
  SurfOctaveParameters * get_d_octave_params();

  /** 
   * 
   * @return A __constant__ variable at gpusurf.cu file scope that holds the octave scale constants
   */
  
  float * get_d_hessian_scale();

  /** 
   * 
   * @return A __constant__ variable at gpusurf.cu file scope that holds the hessian buffer row stride.
   */
  int * get_d_hessian_stride();  


} // namespace asrl





#endif // ASRL_GPU_GLOBALS_FTK_H
