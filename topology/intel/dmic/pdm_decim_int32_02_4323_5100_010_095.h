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

const int32_t fir_int32_02_4323_5100_010_095[95] = {
	178179,
	-158757,
	-2195582,
	-5296650,
	-5737416,
	-1057695,
	4405719,
	3336648,
	-3249588,
	-5061179,
	1984305,
	6895125,
	68826,
	-8433396,
	-2933479,
	9499107,
	6882087,
	-9330152,
	-11397510,
	7807097,
	16376076,
	-4402338,
	-21239788,
	-1118085,
	25423993,
	9062534,
	-27935015,
	-19203927,
	28049586,
	31500423,
	-24524863,
	-45191501,
	16582731,
	59861920,
	-2808306,
	-74639091,
	-18696113,
	88054673,
	50505898,
	-98266320,
	-97865783,
	101816481,
	173879965,
	-88042495,
	-320187025,
	-1193013,
	740698712,
	1139586920,
	740698712,
	-1193013,
	-320187025,
	-88042495,
	173879965,
	101816481,
	-97865783,
	-98266320,
	50505898,
	88054673,
	-18696113,
	-74639091,
	-2808306,
	59861920,
	16582731,
	-45191501,
	-24524863,
	31500423,
	28049586,
	-19203927,
	-27935015,
	9062534,
	25423993,
	-1118085,
	-21239788,
	-4402338,
	16376076,
	7807097,
	-11397510,
	-9330152,
	6882087,
	9499107,
	-2933479,
	-8433396,
	68826,
	6895125,
	1984305,
	-5061179,
	-3249588,
	3336648,
	4405719,
	-1057695,
	-5737416,
	-5296650,
	-2195582,
	-158757,
	178179

};

struct pdm_decim pdm_decim_int32_02_4323_5100_010_095 = {
	2, 95, 0, 4323, 5100, 10, 95, fir_int32_02_4323_5100_010_095
};
