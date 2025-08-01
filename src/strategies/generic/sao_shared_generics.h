/*****************************************************************************
 * This file is part of Kvazaar HEVC encoder.
 *
 * Copyright (c) 2021, Tampere University, ITU/ISO/IEC, project contributors
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of the Tampere University or ITU/ISO/IEC nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 ****************************************************************************/

#ifndef SAO_BAND_DDISTORTION_H_
#define SAO_BAND_DDISTORTION_H_

// #include "encoder.h"
#include "encoderstate.h"
#include "kvazaar.h"
#include "sao.h"

// Mapping of edge_idx values to eo-classes.
static int sao_calc_eo_cat(kvz_pixel a, kvz_pixel b, kvz_pixel c)
{
  // Mapping relationships between a, b and c to eo_idx.
  static const int sao_eo_idx_to_eo_category[] = { 1, 2, 0, 3, 4 };

  int eo_idx = 2 + SIGN3((int)c - (int)a) + SIGN3((int)c - (int)b);

  return sao_eo_idx_to_eo_category[eo_idx];
}

static int sao_edge_ddistortion_generic(const encoder_control_t* const encoder, 
                                        const kvz_pixel *orig_data,
                                        const kvz_pixel *rec_data,
                                              int32_t    block_width,
                                              int32_t    block_height,
                                              int32_t    eo_class,
                                        const int32_t    offsets[NUM_SAO_EDGE_CATEGORIES])
{
  int y, x;
  int32_t sum = 0;
  vector2d_t a_ofs = g_sao_edge_offsets[eo_class][0];
  vector2d_t b_ofs = g_sao_edge_offsets[eo_class][1];

  const int bit_offset = encoder->bitdepth != 8 ? 1 << (encoder->bitdepth - 9) : 0;

  for (y = 1; y < block_height - 1; y++) {
    for (x = 1; x < block_width - 1; x++) {
      uint32_t c_pos =  y            * block_width + x;
      uint32_t a_pos = (y + a_ofs.y) * block_width + x + a_ofs.x;
      uint32_t b_pos = (y + b_ofs.y) * block_width + x + b_ofs.x;

      kvz_pixel a    =  rec_data[a_pos];
      kvz_pixel b    =  rec_data[b_pos];
      kvz_pixel c    =  rec_data[c_pos];
      kvz_pixel orig = orig_data[c_pos];

      int32_t eo_cat = sao_calc_eo_cat(a, b, c);
      int32_t offset = offsets[eo_cat];

      if (offset != 0) {
        int32_t diff   = (orig - c + bit_offset) >> (encoder->bitdepth - 8);
        int32_t delta  = diff - offset;
        int32_t curr   = delta * delta - diff * diff;

        sum += curr;
      }
    }
  }
  return sum;
}

static int sao_band_ddistortion_generic(const encoder_state_t * const state,
                                        const kvz_pixel *orig_data,
                                        const kvz_pixel *rec_data,
                                        int block_width,
                                        int block_height,
                                        int band_pos,
                                        const int sao_bands[4])
{
  int y, x;
  int shift = state->encoder_control->bitdepth-5;
  int sum = 0;
  for (y = 0; y < block_height; ++y) {
    for (x = 0; x < block_width; ++x) {
      const int32_t curr_pos = y * block_width + x;

      kvz_pixel rec  =  rec_data[curr_pos];
      kvz_pixel orig = orig_data[curr_pos];

      int32_t band = (rec >> shift) - band_pos;
      int32_t offset = 0;
      if (band >= 0 && band <= 3) {
        offset = sao_bands[band];
      }
      // Offset is applied to reconstruction, so it is subtracted from diff.

      int32_t diff  = orig - rec;
      int32_t delta = diff - offset;

      int32_t dmask = (offset == 0) ? -1 : 0;
      diff  &= ~dmask;
      delta &= ~dmask;

      sum += delta * delta - diff * diff;
    }
  }

  return sum;
}

#endif
