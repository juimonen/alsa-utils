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

#include <stdint.h>

const int32_t fir_int32_02_4375_5100_010_095[101] = {
	-587830,
	-2653881,
	-5154608,
	-4845367,
	-226474,
	4220832,
	2571159,
	-3184700,
	-4043579,
	2206821,
	5554546,
	-750495,
	-6923897,
	-1268580,
	8073364,
	4085184,
	-8546479,
	-7505366,
	8176184,
	11533751,
	-6471060,
	-15704257,
	3359705,
	19852407,
	1635592,
	-23144509,
	-8252640,
	25285011,
	16574477,
	-25723227,
	-26663303,
	23549736,
	38139662,
	-17943368,
	-50446982,
	8141045,
	63090266,
	7051348,
	-75166961,
	-29039893,
	85772628,
	60568976,
	-93167361,
	-106799777,
	94198977,
	180962817,
	-78385599,
	-324820256,
	-12243140,
	742491464,
	1151461314,
	742491464,
	-12243140,
	-324820256,
	-78385599,
	180962817,
	94198977,
	-106799777,
	-93167361,
	60568976,
	85772628,
	-29039893,
	-75166961,
	7051348,
	63090266,
	8141045,
	-50446982,
	-17943368,
	38139662,
	23549736,
	-26663303,
	-25723227,
	16574477,
	25285011,
	-8252640,
	-23144509,
	1635592,
	19852407,
	3359705,
	-15704257,
	-6471060,
	11533751,
	8176184,
	-7505366,
	-8546479,
	4085184,
	8073364,
	-1268580,
	-6923897,
	-750495,
	5554546,
	2206821,
	-4043579,
	-3184700,
	2571159,
	4220832,
	-226474,
	-4845367,
	-5154608,
	-2653881,
	-587830

};

struct pdm_decim pdm_decim_int32_02_4375_5100_010_095 = {
	2, 101, 0, 4375, 5100, 10, 95, fir_int32_02_4375_5100_010_095
};
