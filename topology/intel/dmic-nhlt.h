/*
  Copyright(c) 2021 Intel Corporation
  All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.
*/

#ifndef __DMIC_NHLT_H
#define __DMIC_NHLT_H

#include <stdint.h>

int dmic_build_nhlt();

void dmic_set_dai_index(uint32_t val);
void dmic_set_num_pdm_active(uint32_t val);
void dmic_set_fifo_word_length(uint32_t val);
void dmic_set_clk_min(uint32_t val);
void dmic_set_clk_max(uint32_t val);
void dmic_set_duty_min(uint32_t val);
void dmic_set_duty_max(uint32_t val);
void dmic_set_sample_rate(uint32_t val);
void dmic_set_unmute_ramp_time_ms(uint32_t val);
void dmic_set_ctrl_id(uint32_t val);
void dmic_set_mic_a_enable(uint32_t val);
void dmic_set_mic_b_enable(uint32_t val);
void dmic_set_polarity_a(uint32_t val);
void dmic_set_polarity_b(uint32_t val);
void dmic_set_clk_edge(uint32_t val);
void dmic_set_skew(uint32_t val);

#endif
