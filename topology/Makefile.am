bin_PROGRAMS = \
	alsatplg

if USE_RST2MAN
man_MANS = alsatplg.1
endif

%.1: %.rst
	rst2man $< > $@

alsatplg_SOURCES = topology.c pre-processor.c pre-process-class.c pre-process-object.c \
		    pre-process-dapm.c pre-process-dai.c intel/dmic-nhlt.c intel/dmic/numbers.c

noinst_HEADERS = topology.h pre-processor.h \
		intel/dmic-nhlt.h intel/dmic/format.h intel/dmic/numbers.h \
		intel/dmic/pdm_decim_fir.h intel/dmic/pdm_decim_table.h \
		intel/dmic/pdm_decim_int32_02_4375_5100_010_095.h \
		intel/dmic/pdm_decim_int32_08_4156_5301_010_090.h \
		intel/dmic/pdm_decim_int32_03_4375_5100_010_095.h \
		intel/dmic/pdm_decim_int32_10_4156_5345_010_090.h \
		intel/dmic/pdm_decim_int32_04_4318_5100_010_095.h \
		intel/dmic/pdm_decim_int32_12_4156_5345_010_090.h \
		intel/dmic/pdm_decim_int32_05_4325_5100_010_095.h \
		intel/dmic/pdm_decim_int32_02_4323_5100_010_095.h \
		intel/dmic/pdm_decim_int32_06_4172_5100_010_095.h

AM_CPPFLAGS = \
         -Wall -I$(top_srcdir)/include

alsatplg_LDADD = $(ALSA_TOPOLOGY_LIBS)

EXTRA_DIST = alsatplg.rst

CLEANFILES = alsatplg.1
