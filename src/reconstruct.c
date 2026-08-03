/*****************************************************************************
 * This file is part of Kvazaar HEVC encoder.
 *
 * Copyright (c) 2026, project contributors
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
 * INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
 ****************************************************************************/

/**
 * \ingroup Reconstruction
 * \file
 * Frame-level chroma reconstruction from the final CU tree and coefficients.
 *
 * After the whole frame has been searched, kvazaar's frame->rec chroma can
 * differ from what the decoder reconstructs for some 4:2:2 blocks in inter
 * frames (the search's intra prediction can read a stale work-tree reference).
 * This pass re-runs ONLY the chroma reconstruction in decode order, reading
 * references from the progressively-rebuilt frame->rec and using the final
 * (retained) coefficients, so frame->rec chroma matches the decoder. Luma is
 * left untouched (it is already correct).
 */

#include "global.h"

#include "cu.h"
#include "encoderstate.h"
#include "inter.h"
#include "intra.h"
#include "strategies/strategies-picture.h"
#include "transform.h"
#include "videoframe.h"

/**
 * Reconstruct only the chroma of a leaf CU into lcu->rec using the stored
 * coefficients and reading references from lcu->rec (which mirrors the
 * progressively-rebuilt frame->rec).
 */
static void reconstruct_cu_chroma(encoder_state_t * const state,
                                  int x, int y, int depth,
                                  lcu_t *lcu,
                                  const cu_info_t *cur_cu)
{
  const int cu_width = LCU_WIDTH >> depth;
  const bool has_chroma = state->encoder_control->cfg.chroma_format != KVZ_CSP_400;

  // The dequant/inverse-transform in the recon path uses state->qp, which
  // after the per-LCU search loop is stale. Use the block's own QP so the
  // residual reconstruction matches the decoder.
  state->qp = cur_cu->qp;

  if (cur_cu->type == CU_INTRA) {
    // Intra transform coding always runs, so the per-sub-TU CBFs are always
    // signalled; no overall-CBF gate here.
    kvz_intra_recon_cu(state, x, y, depth,
                       -1, cur_cu->intra.mode_chroma, // skip luma
                       NULL, lcu, true, false);
  } else if (cur_cu->type == CU_INTER) {
    kvz_inter_recon_cu(state, lcu, x, y, cu_width, false, has_chroma);
    // For inter CUs the transform coding (and hence all sub-TU chroma CBFs)
    // is only invoked when the CU's overall CBF is set. The cu_array can carry
    // a stale per-sub-position CBF from a rejected candidate, so gate the whole
    // residual on the CU's own overall CBF to match the decoder.
    if (cbf_is_set_any(cur_cu->cbf, depth)) {
      kvz_quantize_lcu_residual(state, false, has_chroma, x, y, depth,
                                NULL, lcu, false, KVZ_SUBTU_ALL, true);
    }
  }
}

/**
 * Walk the CU tree of an LCU in decode order and reconstruct every leaf's
 * chroma.
 */
static void reconstruct_coding_tree_chroma(encoder_state_t * const state,
                                           int x, int y, int depth,
                                           lcu_t *lcu)
{
  const videoframe_t * const frame = state->tile->frame;
  const cu_info_t *cur_cu = kvz_cu_array_at_const(frame->cu_array, x, y);
  const int cu_width = LCU_WIDTH >> depth;

  if (GET_SPLITDATA(cur_cu, depth)) {
    const int half_cu = cu_width >> 1;
    reconstruct_coding_tree_chroma(state, x,          y,          depth + 1, lcu);
    reconstruct_coding_tree_chroma(state, x + half_cu, y,          depth + 1, lcu);
    reconstruct_coding_tree_chroma(state, x,          y + half_cu, depth + 1, lcu);
    reconstruct_coding_tree_chroma(state, x + half_cu, y + half_cu, depth + 1, lcu);
    return;
  }

  reconstruct_cu_chroma(state, x, y, depth, lcu, cur_cu);
}

/**
 * Copy the current frame->rec chroma region into lcu->rec so that intra chroma
 * references read the already-rebuilt reconstruction. Luma is not copied.
 */
static void copy_frame_rec_chroma_to_lcu(const encoder_state_t * const state,
                                         const int x, const int y, lcu_t *lcu)
{
  const videoframe_t * const frame = state->tile->frame;
  const int pic_width = frame->width;
  const int x_max = MIN(x + LCU_WIDTH, pic_width) - x;
  const int y_max = MIN(y + LCU_WIDTH, frame->height) - y;

  if (state->encoder_control->cfg.chroma_format != KVZ_CSP_400) {
    kvz_pixels_blit(&frame->rec->u[(x >> SHIFT_W) + (y >> SHIFT_H) * (frame->rec->stride >> SHIFT_W)], lcu->rec.u,
                    x_max >> SHIFT_W, y_max >> SHIFT_H, frame->rec->stride >> SHIFT_W, LCU_WIDTH >> SHIFT_W);
    kvz_pixels_blit(&frame->rec->v[(x >> SHIFT_W) + (y >> SHIFT_H) * (frame->rec->stride >> SHIFT_W)], lcu->rec.v,
                    x_max >> SHIFT_W, y_max >> SHIFT_H, frame->rec->stride >> SHIFT_W, LCU_WIDTH >> SHIFT_W);
  }

  // Populate the top reference buffer (chroma pixels above this LCU).
  if (y > 0 && state->encoder_control->cfg.chroma_format != KVZ_CSP_400) {
    // top_ref[0] is the top-left corner, top_ref[1..] the row above.
    // For the leftmost LCU the corner is not available (x_min_in_lcu = 1).
    const int x_min_in_lcu = (x > 0) ? 0 : 1;
    const int src_x_c = (x >> SHIFT_W) + x_min_in_lcu - 1;
    const int x_len_c = MIN(LCU_REF_PX_WIDTH >> SHIFT_W, (pic_width - x) >> SHIFT_W) + (1 - x_min_in_lcu);
    kvz_pixels_blit(&frame->rec->u[src_x_c + ((y - 1) >> SHIFT_H) * (frame->rec->stride >> SHIFT_W)],
                    &lcu->top_ref.u[x_min_in_lcu], x_len_c, 1,
                    frame->rec->stride >> SHIFT_W, x_len_c);
    kvz_pixels_blit(&frame->rec->v[src_x_c + ((y - 1) >> SHIFT_H) * (frame->rec->stride >> SHIFT_W)],
                    &lcu->top_ref.v[x_min_in_lcu], x_len_c, 1,
                    frame->rec->stride >> SHIFT_W, x_len_c);
  }

  // Populate the left reference buffer (chroma pixels to the left of this LCU).
  if (x > 0 && state->encoder_control->cfg.chroma_format != KVZ_CSP_400) {
    // left_ref[0] is the top-left corner, left_ref[1..] the column to the left.
    const int y_min_in_lcu = (y > 0) ? 0 : 1;
    const int src_y_c = (y >> SHIFT_H) + y_min_in_lcu - 1;
    const int y_len_c = MIN(LCU_REF_PX_WIDTH >> SHIFT_H, (frame->height - y) >> SHIFT_H) + (1 - y_min_in_lcu);
    kvz_pixels_blit(&frame->rec->u[((x - 1) >> SHIFT_W) + src_y_c * (frame->rec->stride >> SHIFT_W)],
                    &lcu->left_ref.u[y_min_in_lcu], 1, y_len_c,
                    frame->rec->stride >> SHIFT_W, 1);
    kvz_pixels_blit(&frame->rec->v[((x - 1) >> SHIFT_W) + src_y_c * (frame->rec->stride >> SHIFT_W)],
                    &lcu->left_ref.v[y_min_in_lcu], 1, y_len_c,
                    frame->rec->stride >> SHIFT_W, 1);
  }
}

/**
 * Copy lcu->rec chroma back to frame->rec.
 */
static void copy_lcu_chroma_to_frame_rec(const encoder_state_t * const state,
                                         const int x, const int y, const lcu_t *lcu)
{
  const videoframe_t * const frame = state->tile->frame;
  const int pic_width = frame->width;
  const int x_max = MIN(x + LCU_WIDTH, pic_width) - x;
  const int y_max = MIN(y + LCU_WIDTH, frame->height) - y;

  if (state->encoder_control->cfg.chroma_format != KVZ_CSP_400) {
    kvz_pixels_blit(lcu->rec.u, &frame->rec->u[(x >> SHIFT_W) + (y >> SHIFT_H) * (frame->rec->stride >> SHIFT_W)],
                    x_max >> SHIFT_W, y_max >> SHIFT_H, LCU_WIDTH >> SHIFT_W, frame->rec->stride >> SHIFT_W);
    kvz_pixels_blit(lcu->rec.v, &frame->rec->v[(x >> SHIFT_W) + (y >> SHIFT_H) * (frame->rec->stride >> SHIFT_W)],
                    x_max >> SHIFT_W, y_max >> SHIFT_H, LCU_WIDTH >> SHIFT_W, frame->rec->stride >> SHIFT_W);
  }
}

/**
 * Frame-level post-search chroma reconstruction. Rebuilds frame->rec chroma
 * from the final CU tree and retained coefficients so that it matches the
 * decoder. Luma is left untouched.
 */
void kvz_reconstruct_frame_chroma(encoder_state_t * const state)
{
  const videoframe_t * const frame = state->tile->frame;
  if (state->encoder_control->cfg.chroma_format == KVZ_CSP_400) {
    return;
  }

  for (int y = 0; y < frame->height_in_lcu; ++y) {
    for (int x = 0; x < frame->width_in_lcu; ++x) {
      const int lcu_px_x = x * LCU_WIDTH;
      const int lcu_px_y = y * LCU_WIDTH;

      lcu_t lcu;
      FILL(lcu, 0);
      lcu.rec.chroma_format = state->encoder_control->cfg.chroma_format;
      lcu.ref.chroma_format = state->encoder_control->cfg.chroma_format;

      // Copy the current frame->rec chroma into lcu->rec so intra references
      // read the already-rebuilt reconstruction of neighbouring blocks.
      copy_frame_rec_chroma_to_lcu(state, lcu_px_x, lcu_px_y, &lcu);

      // Load the final CU tree and coefficients for this LCU.
      kvz_cu_array_copy_to_lcu(&lcu, lcu_px_x, lcu_px_y, frame->cu_array);
      if (frame->lcu_coeffs) {
        const int lcu_index = y * frame->width_in_lcu + x;
        lcu.coeff = frame->lcu_coeffs[lcu_index];
      }

      reconstruct_coding_tree_chroma(state, lcu_px_x, lcu_px_y, 0, &lcu);

      copy_lcu_chroma_to_frame_rec(state, lcu_px_x, lcu_px_y, &lcu);
    }
  }
}
