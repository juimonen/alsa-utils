#include <stdint.h>
#include <stdio.h>
#include <alsa/input.h>
#include <alsa/output.h>
#include <alsa/conf.h>
#include <alsa/error.h>
#include "pre-processor.h"
#include "dmic-nhlt.h"
#include "dmic/pdm_decim_table.h"
#include "dmic/numbers.h"
#include "dmic/format.h"

#define BIT(b)                  (1UL << (b))

#define MASK(b_hi, b_lo)        \
        (((1ULL << ((b_hi) - (b_lo) + 1ULL)) - 1ULL) << (b_lo))
#define SET_BIT(b, x)           (((x) & 1) << (b))
#define SET_BITS(b_hi, b_lo, x) \
        (((x) & ((1ULL << ((b_hi) - (b_lo) + 1ULL)) - 1ULL)) << (b_lo))

#define EXP_FIXED_INPUT_QY 27
#define EXP_FIXED_OUTPUT_QY 20
#define DB2LIN_FIXED_INPUT_QY 24
#define DB2LIN_FIXED_OUTPUT_QY 20

/* these should all come from topology platform definition */
#define DMIC_HW_VERSION		1
#define DMIC_HW_CONTROLLERS	2
#define DMIC_HW_FIFOS		2
#define DMIC_HW_IOCLK           38400000

#define DMIC_MAX_MODES 50
#define DMIC_FIR_PIPELINE_OVERHEAD 5
#define DMIC_UNMUTE_RAMP_US     1000

/* Parameters used in modes computation */
#define DMIC_HW_BITS_CIC		26
#define DMIC_HW_BITS_FIR_COEF		20
#define DMIC_HW_BITS_FIR_GAIN		20
#define DMIC_HW_BITS_FIR_INPUT		22
#define DMIC_HW_BITS_FIR_OUTPUT		24
#define DMIC_HW_BITS_FIR_INTERNAL	26
#define DMIC_HW_BITS_GAIN_OUTPUT	22
#define DMIC_HW_FIR_LENGTH_MAX		250
#define DMIC_HW_CIC_SHIFT_MIN		-8
#define DMIC_HW_CIC_SHIFT_MAX		4
#define DMIC_HW_FIR_SHIFT_MIN		0
#define DMIC_HW_FIR_SHIFT_MAX		8
#define DMIC_HW_CIC_DECIM_MIN		5
#define DMIC_HW_CIC_DECIM_MAX		31 /* Note: Limited by BITS_CIC */
#define DMIC_HW_FIR_DECIM_MIN		2
#define DMIC_HW_FIR_DECIM_MAX		20 /* Note: Practical upper limit */
#define DMIC_HW_SENS_Q28		Q_CONVERT_FLOAT(1.0, 28) /* Q1.28 */
#define DMIC_HW_PDM_CLK_MIN		100000 /* Note: Practical min value */
#define DMIC_HW_DUTY_MIN		20 /* Note: Practical min value */
#define DMIC_HW_DUTY_MAX		80 /* Note: Practical max value */

/* OUTCONTROL0 bits */
#define OUTCONTROL0_TIE_BIT	BIT(27)
#define OUTCONTROL0_SIP_BIT	BIT(26)
#define OUTCONTROL0_FINIT_BIT	BIT(25)
#define OUTCONTROL0_FCI_BIT	BIT(24)
#define OUTCONTROL0_TIE(x)	SET_BIT(27, x)
#define OUTCONTROL0_SIP(x)	SET_BIT(26, x)
#define OUTCONTROL0_FINIT(x)	SET_BIT(25, x)
#define OUTCONTROL0_FCI(x)	SET_BIT(24, x)
#define OUTCONTROL0_BFTH(x)	SET_BITS(23, 20, x)
#define OUTCONTROL0_OF(x)	SET_BITS(19, 18, x)
#define OUTCONTROL0_TH(x)	SET_BITS(5, 0, x)

/* OUTCONTROL1 bits */
#define OUTCONTROL1_TIE_BIT	BIT(27)
#define OUTCONTROL1_SIP_BIT	BIT(26)
#define OUTCONTROL1_FINIT_BIT	BIT(25)
#define OUTCONTROL1_FCI_BIT	BIT(24)
#define OUTCONTROL1_TIE(x)	SET_BIT(27, x)
#define OUTCONTROL1_SIP(x)	SET_BIT(26, x)
#define OUTCONTROL1_FINIT(x)	SET_BIT(25, x)
#define OUTCONTROL1_FCI(x)	SET_BIT(24, x)
#define OUTCONTROL1_BFTH(x)	SET_BITS(23, 20, x)
#define OUTCONTROL1_OF(x)	SET_BITS(19, 18, x)
#define OUTCONTROL1_TH(x)	SET_BITS(5, 0, x)

#if DMIC_HW_VERSION == 1
/* OUTCONTROL0 bits */
#define OUTCONTROL0_IPM(x)	SET_BITS(17, 16, x)
/* OUTCONTROL1 bits */
#define OUTCONTROL1_IPM(x)	SET_BITS(17, 16, x)
#endif

#if DMIC_HW_VERSION >= 2
/* OUTCONTROL0 bits */
#define OUTCONTROL0_IPM(x)                      SET_BITS(17, 15, x)
#define OUTCONTROL0_IPM_SOURCE_1(x)		SET_BITS(14, 13, x)
#define OUTCONTROL0_IPM_SOURCE_2(x)		SET_BITS(12, 11, x)
#define OUTCONTROL0_IPM_SOURCE_3(x)		SET_BITS(10, 9, x)
#define OUTCONTROL0_IPM_SOURCE_4(x)		SET_BITS(8, 7, x)

/* OUTCONTROL1 bits */
#define OUTCONTROL1_IPM(x)                      SET_BITS(17, 15, x)
#define OUTCONTROL1_IPM_SOURCE_1(x)		SET_BITS(14, 13, x)
#define OUTCONTROL1_IPM_SOURCE_2(x)		SET_BITS(12, 11, x)
#define OUTCONTROL1_IPM_SOURCE_3(x)		SET_BITS(10, 9, x)
#define OUTCONTROL1_IPM_SOURCE_4(x)		SET_BITS(8, 7, x)

#define OUTCONTROLX_IPM_NUMSOURCES		4
#endif

/* CIC_CONTROL bits */
#define CIC_CONTROL_SOFT_RESET_BIT	BIT(16)
#define CIC_CONTROL_CIC_START_B_BIT	BIT(15)
#define CIC_CONTROL_CIC_START_A_BIT	BIT(14)
#define CIC_CONTROL_MIC_B_POLARITY_BIT	BIT(3)
#define CIC_CONTROL_MIC_A_POLARITY_BIT	BIT(2)
#define CIC_CONTROL_MIC_MUTE_BIT	BIT(1)
#define CIC_CONTROL_STEREO_MODE_BIT	BIT(0)

#define CIC_CONTROL_SOFT_RESET(x)	SET_BIT(16, x)
#define CIC_CONTROL_CIC_START_B(x)	SET_BIT(15, x)
#define CIC_CONTROL_CIC_START_A(x)	SET_BIT(14, x)
#define CIC_CONTROL_MIC_B_POLARITY(x)	SET_BIT(3, x)
#define CIC_CONTROL_MIC_A_POLARITY(x)	SET_BIT(2, x)
#define CIC_CONTROL_MIC_MUTE(x)		SET_BIT(1, x)
#define CIC_CONTROL_STEREO_MODE(x)	SET_BIT(0, x)

/* CIC_CONFIG bits */
#define CIC_CONFIG_CIC_SHIFT(x)		SET_BITS(27, 24, x)
#define CIC_CONFIG_COMB_COUNT(x)	SET_BITS(15, 8, x)

/* CIC_CONFIG masks */
#define CIC_CONFIG_CIC_SHIFT_MASK	MASK(27, 24)
#define CIC_CONFIG_COMB_COUNT_MASK	MASK(15, 8)

/* MIC_CONTROL bits */
#define MIC_CONTROL_PDM_EN_B_BIT	BIT(1)
#define MIC_CONTROL_PDM_EN_A_BIT	BIT(0)
#define MIC_CONTROL_PDM_CLKDIV(x)	SET_BITS(15, 8, x)
#define MIC_CONTROL_PDM_SKEW(x)		SET_BITS(7, 4, x)
#define MIC_CONTROL_CLK_EDGE(x)		SET_BIT(3, x)
#define MIC_CONTROL_PDM_EN_B(x)		SET_BIT(1, x)
#define MIC_CONTROL_PDM_EN_A(x)		SET_BIT(0, x)

/* MIC_CONTROL masks */
#define MIC_CONTROL_PDM_CLKDIV_MASK	MASK(15, 8)

/* FIR_CONTROL_A bits */
#define FIR_CONTROL_A_START_BIT			BIT(7)
#define FIR_CONTROL_A_ARRAY_START_EN_BIT	BIT(6)
#define FIR_CONTROL_A_MUTE_BIT			BIT(1)
#define FIR_CONTROL_A_START(x)			SET_BIT(7, x)
#define FIR_CONTROL_A_ARRAY_START_EN(x)		SET_BIT(6, x)
#define FIR_CONTROL_A_DCCOMP(x)			SET_BIT(4, x)
#define FIR_CONTROL_A_MUTE(x)			SET_BIT(1, x)
#define FIR_CONTROL_A_STEREO(x)			SET_BIT(0, x)

/* FIR_CONFIG_A bits */
#define FIR_CONFIG_A_FIR_DECIMATION(x)		SET_BITS(20, 16, x)
#define FIR_CONFIG_A_FIR_SHIFT(x)		SET_BITS(11, 8, x)
#define FIR_CONFIG_A_FIR_LENGTH(x)		SET_BITS(7, 0, x)

/* DC offset compensation time constants */
#define DCCOMP_TC0	0
#define DCCOMP_TC1	1
#define DCCOMP_TC2	2
#define DCCOMP_TC3	3
#define DCCOMP_TC4	4
#define DCCOMP_TC5	5
#define DCCOMP_TC6	6
#define DCCOMP_TC7	7

/* DC_OFFSET_LEFT_A bits */
#define DC_OFFSET_LEFT_A_DC_OFFS(x)		SET_BITS(21, 0, x)

/* DC_OFFSET_RIGHT_A bits */
#define DC_OFFSET_RIGHT_A_DC_OFFS(x)		SET_BITS(21, 0, x)

/* OUT_GAIN_LEFT_A bits */
#define OUT_GAIN_LEFT_A_GAIN(x)			SET_BITS(19, 0, x)

/* OUT_GAIN_RIGHT_A bits */
#define OUT_GAIN_RIGHT_A_GAIN(x)		SET_BITS(19, 0, x)

/* FIR_CONTROL_B bits */
#define FIR_CONTROL_B_START_BIT			BIT(7)
#define FIR_CONTROL_B_ARRAY_START_EN_BIT	BIT(6)
#define FIR_CONTROL_B_MUTE_BIT			BIT(1)
#define FIR_CONTROL_B_START(x)			SET_BIT(7, x)
#define FIR_CONTROL_B_ARRAY_START_EN(x)		SET_BIT(6, x)
#define FIR_CONTROL_B_DCCOMP(x)			SET_BIT(4, x)
#define FIR_CONTROL_B_MUTE(x)			SET_BIT(1, x)
#define FIR_CONTROL_B_STEREO(x)			SET_BIT(0, x)

/* FIR_CONFIG_B bits */
#define FIR_CONFIG_B_FIR_DECIMATION(x)		SET_BITS(20, 16, x)
#define FIR_CONFIG_B_FIR_SHIFT(x)		SET_BITS(11, 8, x)
#define FIR_CONFIG_B_FIR_LENGTH(x)		SET_BITS(7, 0, x)

/* DC_OFFSET_LEFT_B bits */
#define DC_OFFSET_LEFT_B_DC_OFFS(x)		SET_BITS(21, 0, x)

/* DC_OFFSET_RIGHT_B bits */
#define DC_OFFSET_RIGHT_B_DC_OFFS(x)		SET_BITS(21, 0, x)

/* OUT_GAIN_LEFT_B bits */
#define OUT_GAIN_LEFT_B_GAIN(x)			SET_BITS(19, 0, x)

/* OUT_GAIN_RIGHT_B bits */
#define OUT_GAIN_RIGHT_B_GAIN(x)		SET_BITS(19, 0, x)

/* FIR coefficients */
#define FIR_COEF_A(x)				SET_BITS(19, 0, x)
#define FIR_COEF_B(x)				SET_BITS(19, 0, x)

/* Minimum OSR is always applied for 48 kHz and less sample rates */
#define DMIC_MIN_OSR  50

/* These are used as guideline for configuring > 48 kHz sample rates. The
 * minimum OSR can be relaxed down to 40 (use 3.84 MHz clock for 96 kHz).
 */
#define DMIC_HIGH_RATE_MIN_FS	64000
#define DMIC_HIGH_RATE_OSR_MIN	40

/* Used for scaling FIR coefficients for HW */
#define DMIC_HW_FIR_COEF_MAX ((1 << (DMIC_HW_BITS_FIR_COEF - 1)) - 1)
#define DMIC_HW_FIR_COEF_Q (DMIC_HW_BITS_FIR_COEF - 1)

/* Internal precision in gains computation, e.g. Q4.28 in int32_t */
#define DMIC_FIR_SCALE_Q 28

/* Used in unmute ramp values calculation */
#define DMIC_HW_FIR_GAIN_MAX ((1 << (DMIC_HW_BITS_FIR_GAIN - 1)) - 1)

/* Hardwired log ramp parameters. The first value is the initial gain in
 * decibels. The second value is the default ramp time.
 */
#define LOGRAMP_START_DB Q_CONVERT_FLOAT(-90, DB2LIN_FIXED_INPUT_QY)
#define LOGRAMP_TIME_MS 400 /* Default ramp time in milliseconds */

/* Limits for ramp time from topology */
#define LOGRAMP_TIME_MIN_MS 10 /* Min. 10 ms */
#define LOGRAMP_TIME_MAX_MS 1000 /* Max. 1s */

/* Simplify log ramp step calculation equation with this constant term */
#define LOGRAMP_CONST_TERM ((int32_t) \
	((int64_t)-LOGRAMP_START_DB * DMIC_UNMUTE_RAMP_US / 1000))

/* Fractional shift for gain update. Gain format is Q2.30. */
#define Q_SHIFT_GAIN_X_GAIN_COEF \
	(Q_SHIFT_BITS_32(30, DB2LIN_FIXED_OUTPUT_QY, 30))

#define SOF_DAI_INTEL_DMIC_NUM_CTRL 4

/* structs for dmic internal calculations */
struct decim_modes {
        int16_t clkdiv[DMIC_MAX_MODES];
        int16_t mcic[DMIC_MAX_MODES];
        int16_t mfir[DMIC_MAX_MODES];
        int num_of_modes;
};

struct matched_modes {
        int16_t clkdiv[DMIC_MAX_MODES];
        int16_t mcic[DMIC_MAX_MODES];
        int16_t mfir_a[DMIC_MAX_MODES];
        int16_t mfir_b[DMIC_MAX_MODES];
        int num_of_modes;
};

struct dmic_configuration {
        struct pdm_decim *fir_a;
        struct pdm_decim *fir_b;
        int clkdiv;
        int mcic;
        int mfir_a;
        int mfir_b;
        int cic_shift;
        int fir_a_shift;
        int fir_b_shift;
        int fir_a_length;
        int fir_b_length;
        int32_t fir_a_scale;
        int32_t fir_b_scale;
};

/* structs for dmic nhlt blob generation */
struct nhlt_fir_config
{
    uint32_t fir_control;
    uint32_t fir_config;
    uint32_t dc_offset_left;
    uint32_t dc_offset_right;
    uint32_t out_gain_left;
    uint32_t out_gain_right;
    uint32_t reserved2[2];
};

struct nhlt_pdm_ctrl_cfg {
	uint32_t cic_control;
	uint32_t cic_config;
	uint32_t reserved0;
	uint32_t mic_control;
	uint32_t pdmsm;
	uint32_t reuse_fir_from_pdm;
	uint32_t reserved1[2];
	struct nhlt_fir_config fir_config[2];
	uint32_t fir_coeffs[0];
};

struct dmic_nhlt_config_data {
	uint32_t ts_group[4];
	uint32_t clock_on_delay;
	uint32_t channel_ctrl_mask;
	uint32_t chan_ctrl_cfg[2];
	uint32_t channel_pdm_mask;
	struct nhlt_pdm_ctrl_cfg pdm_ctrl_cfg[2];
};

/* helper structs for setting the tokens from topology */
struct sof_ipc_dai_dmic_pdm_ctrl {
        uint32_t reserved0;
        uint16_t id;            /**< PDM controller ID */

        uint16_t enable_mic_a;  /**< Use A (left) channel mic (0 or 1)*/
        uint16_t enable_mic_b;  /**< Use B (right) channel mic (0 or 1)*/

        uint16_t polarity_mic_a; /**< Optionally invert mic A signal (0 or 1) */
        uint16_t polarity_mic_b; /**< Optionally invert mic B signal (0 or 1) */

        uint16_t clk_edge;      /**< Optionally swap data clock edge (0 or 1) */
        uint16_t skew;          /**< Adjust PDM data sampling vs. clock (0..15) */

        uint16_t reserved[3];   /**< Make sure the total size is 4 bytes aligned */
};

struct sof_ipc_dai_dmic_params {
        uint32_t reserved0;
        uint32_t driver_ipc_version;    /**< Version (1..N) */

        uint32_t pdmclk_min;    /**< Minimum microphone clock in Hz (100000..N) */
        uint32_t pdmclk_max;    /**< Maximum microphone clock in Hz (min...N) */

        uint32_t fifo_fs;       /**< FIFO sample rate in Hz (8000..96000) */
        uint32_t reserved_1;    /**< Reserved */
        uint16_t fifo_bits;     /**< FIFO word length (16 or 32) */
        uint16_t fifo_bits_b;   /**< Deprecated since firmware ABI 3.0.1 */

        uint16_t duty_min;      /**< Min. mic clock duty cycle in % (20..80) */
        uint16_t duty_max;      /**< Max. mic clock duty cycle in % (min..80) */

        uint32_t num_pdm_active; /**< Number of active pdm controllers. */
                                 /**< Range is 1..SOF_DAI_INTEL_DMIC_NUM_CTRL */

        uint32_t wake_up_time;      /**< Time from clock start to data (us) */
        uint32_t min_clock_on_time; /**< Min. time that clk is kept on (us) */
        uint32_t unmute_ramp_time;  /**< Length of logarithmic gain ramp (ms) */

        /* reserved for future use */
        uint32_t reserved[5];

        /**< PDM controllers configuration */
        struct sof_ipc_dai_dmic_pdm_ctrl pdm[SOF_DAI_INTEL_DMIC_NUM_CTRL];
};

struct dmic_nhlt_config_data dmic_nhlt;
static struct sof_ipc_dai_dmic_params dmic_prm[DMIC_HW_FIFOS];
int pdm_index = 0;
int dai_index = 0;

/* This function returns a raw list of potential microphone clock and decimation
 * modes for achieving requested sample rates. The search is constrained by
 * decimation HW capabililies and setup parameters. The parameters such as
 * microphone clock min/max and duty cycle requirements need be checked from
 * used microphone component datasheet.
 */
static void find_modes(struct decim_modes *modes, uint32_t fs, int di)
{
	int clkdiv_min;
	int clkdiv_max;
	int clkdiv;
	int c1;
	int du_min;
	int du_max;
	int pdmclk;
	int osr;
	int mfir;
	int mcic;
	int ioclk_test;
	int osr_min = DMIC_MIN_OSR;
	int j;
	int i = 0;

	/* Defaults, empty result */
	modes->num_of_modes = 0;

	/* The FIFO is not requested if sample rate is set to zero. Just
	 * return in such case with num_of_modes as zero.
	 */
	if (fs == 0) {
		tplg_pp_debug("find_modes(): fs not set");
		return;
	}

	/* Override DMIC_MIN_OSR for very high sample rates, use as minimum
	 * the nominal clock for the high rates.
	 */
	if (fs >= DMIC_HIGH_RATE_MIN_FS)
		osr_min = DMIC_HIGH_RATE_OSR_MIN;

	/* Check for sane pdm clock, min 100 kHz, max ioclk/2 */
	if (dmic_prm[di].pdmclk_max < DMIC_HW_PDM_CLK_MIN ||
	    dmic_prm[di].pdmclk_max > DMIC_HW_IOCLK / 2) {
		tplg_pp_debug("find_modes():  pdm clock max not in range");
		return;
	}
	if (dmic_prm[di].pdmclk_min < DMIC_HW_PDM_CLK_MIN ||
	    dmic_prm[di].pdmclk_min > dmic_prm[di].pdmclk_max) {
		tplg_pp_debug("find_modes():  pdm clock min not in range");
		return;
	}

	/* Check for sane duty cycle */
	if (dmic_prm[di].duty_min > dmic_prm[di].duty_max) {
		tplg_pp_debug("find_modes(): duty cycle min > max");
		return;
	}
	if (dmic_prm[di].duty_min < DMIC_HW_DUTY_MIN ||
	    dmic_prm[di].duty_min > DMIC_HW_DUTY_MAX) {
		tplg_pp_debug("find_modes():  pdm clock min not in range");
		return;
	}
	if (dmic_prm[di].duty_max < DMIC_HW_DUTY_MIN ||
	    dmic_prm[di].duty_max > DMIC_HW_DUTY_MAX) {
		tplg_pp_debug("find_modes(): pdm clock max not in range");
		return;
	}

	/* Min and max clock dividers */
	clkdiv_min = ceil_divide(DMIC_HW_IOCLK, dmic_prm[di].pdmclk_max);
	clkdiv_min = MAX(clkdiv_min, DMIC_HW_CIC_DECIM_MIN);
	clkdiv_max = DMIC_HW_IOCLK / dmic_prm[di].pdmclk_min;

	tplg_pp_debug("find_modes():  clkdiv_min %u clkdiv_max %u", clkdiv_min, clkdiv_max);

	/* Loop possible clock dividers and check based on resulting
	 * oversampling ratio that CIC and FIR decimation ratios are
	 * feasible. The ratios need to be integers. Also the mic clock
	 * duty cycle need to be within limits.
	 */
	for (clkdiv = clkdiv_min; clkdiv <= clkdiv_max; clkdiv++) {
		/* Calculate duty cycle for this clock divider. Note that
		 * odd dividers cause non-50% duty cycle.
		 */
		c1 = clkdiv >> 1;
		du_min = 100 * c1 / clkdiv;
		du_max = 100 - du_min;

		/* Calculate PDM clock rate and oversampling ratio. */
		pdmclk = DMIC_HW_IOCLK / clkdiv;
		osr = pdmclk / fs;

		/* Check that OSR constraints is met and clock duty cycle does
		 * not exceed microphone specification. If exceed proceed to
		 * next clkdiv.
		 */
		if (osr < osr_min || du_min < dmic_prm[di].duty_min ||
		    du_max > dmic_prm[di].duty_max)
			continue;

		/* Loop FIR decimation factors candidates. If the
		 * integer divided decimation factors and clock dividers
		 * as multiplied with sample rate match the IO clock
		 * rate the division was exact and such decimation mode
		 * is possible. Then check that CIC decimation constraints
		 * are met. The passed decimation modes are added to array.
		 */
		for (j = 0; fir_list[j]; j++) {
			mfir = fir_list[j]->decim_factor;

			/* Skip if previous decimation factor was the same */
			if (j > 1 && fir_list[j - 1]->decim_factor == mfir)
				continue;

			mcic = osr / mfir;
			ioclk_test = fs * mfir * mcic * clkdiv;

			if (ioclk_test == DMIC_HW_IOCLK &&
			    mcic >= DMIC_HW_CIC_DECIM_MIN &&
			    mcic <= DMIC_HW_CIC_DECIM_MAX &&
			    i < DMIC_MAX_MODES) {
				modes->clkdiv[i] = clkdiv;
				modes->mcic[i] = mcic;
				modes->mfir[i] = mfir;
				i++;
			}
		}
	}

	tplg_pp_debug("find_modes(): mode %d", i);
	modes->num_of_modes = i;
}

/* The previous raw modes list contains sane configuration possibilities. When
 * there is request for both FIFOs A and B operation this function returns
 * list of compatible settings.
 */
static void match_modes(struct matched_modes *c, struct decim_modes *a,
			struct decim_modes *b)
{
	int16_t idx[DMIC_MAX_MODES];
	int idx_length;
	int i;
	int n;
	int m;

	/* Check if previous search got results. */
	c->num_of_modes = 0;
	if (a->num_of_modes == 0 && b->num_of_modes == 0) {
		/* Nothing to do */
		return;
	}

	/* Ensure that num_of_modes is sane. */
	if (a->num_of_modes > DMIC_MAX_MODES ||
	    b->num_of_modes > DMIC_MAX_MODES)
		return;

	/* Check for request only for FIFO A or B. In such case pass list for
	 * A or B as such.
	 */
	if (b->num_of_modes == 0) {
		c->num_of_modes = a->num_of_modes;
		for (i = 0; i < a->num_of_modes; i++) {
			c->clkdiv[i] = a->clkdiv[i];
			c->mcic[i] = a->mcic[i];
			c->mfir_a[i] = a->mfir[i];
			c->mfir_b[i] = 0; /* Mark FIR B as non-used */
		}
		return;
	}

	if (a->num_of_modes == 0) {
		c->num_of_modes = b->num_of_modes;
		for (i = 0; i < b->num_of_modes; i++) {
			c->clkdiv[i] = b->clkdiv[i];
			c->mcic[i] = b->mcic[i];
			c->mfir_b[i] = b->mfir[i];
			c->mfir_a[i] = 0; /* Mark FIR A as non-used */
		}
		return;
	}

	/* Merge a list of compatible modes */
	i = 0;
	for (n = 0; n < a->num_of_modes; n++) {
		/* Find all indices of values a->clkdiv[n] in b->clkdiv[] */
		idx_length = find_equal_int16(idx, b->clkdiv, a->clkdiv[n],
					      b->num_of_modes, 0);
		for (m = 0; m < idx_length; m++) {
			if (b->mcic[idx[m]] == a->mcic[n]) {
				c->clkdiv[i] = a->clkdiv[n];
				c->mcic[i] = a->mcic[n];
				c->mfir_a[i] = a->mfir[n];
				c->mfir_b[i] = b->mfir[idx[m]];
				i++;
			}
		}
		c->num_of_modes = i;
	}
}

/* Finds a suitable FIR decimation filter from the included set */
static struct pdm_decim *get_fir(struct dmic_configuration *cfg, int mfir)
{
	int i;
	int fs;
	int cic_fs;
	int fir_max_length;
	struct pdm_decim *fir = NULL;

	if (mfir <= 0)
		return fir;

	cic_fs = DMIC_HW_IOCLK / cfg->clkdiv / cfg->mcic;
	fs = cic_fs / mfir;
	/* FIR max. length depends on available cycles and coef RAM
	 * length. Exceeding this length sets HW overrun status and
	 * overwrite of other register.
	 */
	fir_max_length = MIN(DMIC_HW_FIR_LENGTH_MAX,
			     DMIC_HW_IOCLK / fs / 2 -
			     DMIC_FIR_PIPELINE_OVERHEAD);

	i = 0;
	/* Loop until NULL */
	while (fir_list[i]) {
		if (fir_list[i]->decim_factor == mfir) {
			if (fir_list[i]->length <= fir_max_length) {
				/* Store pointer, break from loop to avoid a
				 * Possible other mode with lower FIR length.
				 */
				fir = fir_list[i];
				break;
			}
			/* dai_info(dai, "get_fir(), Note length=%d exceeds max=%d",
			 *	 fir_list[i]->length, fir_max_length);
			 */
		}
		i++;
	}

	return fir;
}

/* Calculate scale and shift to use for FIR coefficients. Scale is applied
 * before write to HW coef RAM. Shift will be programmed to HW register.
 */
static int fir_coef_scale(int32_t *fir_scale, int *fir_shift, int add_shift,
			  const int32_t coef[], int coef_length, int32_t gain)
{
	int32_t amax;
	int32_t new_amax;
	int32_t fir_gain;
	int shift;

	/* Multiply gain passed from CIC with output full scale. */
	fir_gain = Q_MULTSR_32X32((int64_t)gain, DMIC_HW_SENS_Q28,
				  DMIC_FIR_SCALE_Q, 28, DMIC_FIR_SCALE_Q);

	/* Find the largest FIR coefficient value. */
	amax = find_max_abs_int32((int32_t *)coef, coef_length);

	/* Scale max. tap value with FIR gain. */
	new_amax = Q_MULTSR_32X32((int64_t)amax, fir_gain, 31,
				  DMIC_FIR_SCALE_Q, DMIC_FIR_SCALE_Q);
	if (new_amax <= 0)
		return -1;

	/* Get left shifts count to normalize the fractional value as 32 bit.
	 * We need right shifts count for scaling so need to invert. The
	 * difference of Q31 vs. used Q format is added to get the correct
	 * normalization right shift value.
	 */
	shift = 31 - DMIC_FIR_SCALE_Q - norm_int32(new_amax);

	/* Add to shift for coef raw Q31 format shift and store to
	 * configuration. Ensure range (fail should not happen with OK
	 * coefficient set).
	 */
	*fir_shift = -shift + add_shift;
	if (*fir_shift < DMIC_HW_FIR_SHIFT_MIN ||
	    *fir_shift > DMIC_HW_FIR_SHIFT_MAX)
		return -1;

	/* Compensate shift into FIR coef scaler and store as Q4.20. */
	if (shift < 0)
		*fir_scale = fir_gain << -shift;
	else
		*fir_scale = fir_gain >> shift;

	return 0;
}

/* This function selects with a simple criteria one mode to set up the
 * decimator. For the settings chosen for FIFOs A and B output a lookup
 * is done for FIR coefficients from the included coefficients tables.
 * For some decimation factors there may be several length coefficient sets.
 * It is due to possible restruction of decimation engine cycles per given
 * sample rate. If the coefficients length is exceeded the lookup continues.
 * Therefore the list of coefficient set must present the filters for a
 * decimation factor in decreasing length order.
 *
 * Note: If there is no filter available an error is returned. The parameters
 * should be reviewed for such case. If still a filter is missing it should be
 * added into the included set. FIR decimation with a high factor usually
 * needs compromizes into specifications and is not desirable.
 */
static int select_mode(struct dmic_configuration *cfg,
		       struct matched_modes *modes)
{
	int32_t g_cic;
	int32_t fir_in_max;
	int32_t cic_out_max;
	int32_t gain_to_fir;
	int16_t idx[DMIC_MAX_MODES];
	int16_t *mfir;
	int mcic;
	int bits_cic;
	int ret;
	int n;
	int found = 0;

	/* If there are more than one possibilities select a mode with a preferred
	 * FIR decimation factor. If there are several select mode with highest
	 * ioclk divider to minimize microphone power consumption. The highest
	 * clock divisors are in the end of list so select the last of list.
	 * The minimum OSR criteria used in previous ensures that quality in
	 * the candidates should be sufficient.
	 */
	if (modes->num_of_modes == 0) {
		/* dai_err(dai, "select_mode(): no modes available"); */
		return -1;
	}

	/* Valid modes presence is indicated with non-zero decimation
	 * factor in 1st element. If FIR A is not used get decimation factors
	 * from FIR B instead.
	 */
	if (modes->mfir_a[0] > 0)
		mfir = modes->mfir_a;
	else
		mfir = modes->mfir_b;

	/* Search fir_list[] decimation factors from start towards end. The found
	 * last configuration entry with searched decimation factor will be used.
	 */
	for (n = 0; fir_list[n]; n++) {
		found = find_equal_int16(idx, mfir, fir_list[n]->decim_factor,
					 modes->num_of_modes, 0);
		if (found)
			break;
	}

	if (!found) {
		/* dai_err(dai, "select_mode(): No filter for decimation found"); */
		return -1;
	}
	n = idx[found - 1]; /* Option with highest clock divisor and lowest mic clock rate */

	/* Get microphone clock and decimation parameters for used mode from
	 * the list.
	 */
	cfg->clkdiv = modes->clkdiv[n];
	cfg->mfir_a = modes->mfir_a[n];
	cfg->mfir_b = modes->mfir_b[n];
	cfg->mcic = modes->mcic[n];
	cfg->fir_a = NULL;
	cfg->fir_b = NULL;

	/* Find raw FIR coefficients to match the decimation factors of FIR
	 * A and B.
	 */
	if (cfg->mfir_a > 0) {
		cfg->fir_a = get_fir(cfg, cfg->mfir_a);
		if (!cfg->fir_a) {
			/* dai_err(dai, "select_mode(): cannot find FIR coefficients, mfir_a = %u",
			 *	cfg->mfir_a);
			 */
			return -1;
		}
	}

	if (cfg->mfir_b > 0) {
		cfg->fir_b = get_fir(cfg, cfg->mfir_b);
		if (!cfg->fir_b) {
			/* dai_err(dai, "select_mode(): cannot find FIR coefficients, mfir_b = %u",
			 *	cfg->mfir_b);
			 */
			return -1;
		}
	}

	/* Calculate CIC shift from the decimation factor specific gain. The
	 * gain of HW decimator equals decimation factor to power of 5.
	 */
	mcic = cfg->mcic;
	g_cic = mcic * mcic * mcic * mcic * mcic;
	if (g_cic < 0) {
		/* Erroneous decimation factor and CIC gain */
		/* dai_err(dai, "select_mode(): erroneous decimation factor and CIC gain"); */
		return -1;
	}

	bits_cic = 32 - norm_int32(g_cic);
	cfg->cic_shift = bits_cic - DMIC_HW_BITS_FIR_INPUT;

	/* Calculate remaining gain to FIR in Q format used for gain
	 * values.
	 */
	fir_in_max = INT_MAX(DMIC_HW_BITS_FIR_INPUT);
	if (cfg->cic_shift >= 0)
		cic_out_max = g_cic >> cfg->cic_shift;
	else
		cic_out_max = g_cic << -cfg->cic_shift;

	gain_to_fir = (int32_t)((((int64_t)fir_in_max) << DMIC_FIR_SCALE_Q) /
		cic_out_max);

	/* Calculate FIR scale and shift */
	if (cfg->mfir_a > 0) {
		cfg->fir_a_length = cfg->fir_a->length;
		ret = fir_coef_scale(&cfg->fir_a_scale, &cfg->fir_a_shift,
				     cfg->fir_a->shift, cfg->fir_a->coef,
				     cfg->fir_a->length, gain_to_fir);
		if (ret < 0) {
			/* Invalid coefficient set found, should not happen. */
			/* dai_err(dai, "select_mode(): invalid coefficient set found"); */
			return -1;
		}
	} else {
		cfg->fir_a_scale = 0;
		cfg->fir_a_shift = 0;
		cfg->fir_a_length = 0;
	}

	if (cfg->mfir_b > 0) {
		cfg->fir_b_length = cfg->fir_b->length;
		ret = fir_coef_scale(&cfg->fir_b_scale, &cfg->fir_b_shift,
				     cfg->fir_b->shift, cfg->fir_b->coef,
				     cfg->fir_b->length, gain_to_fir);
		if (ret < 0) {
			/* Invalid coefficient set found, should not happen. */
			/* dai_err(dai, "select_mode(): invalid coefficient set found"); */
			return -1;
		}
	} else {
		cfg->fir_b_scale = 0;
		cfg->fir_b_shift = 0;
		cfg->fir_b_length = 0;
	}

	return 0;
}

/* The FIFO input packer mode (IPM) settings are somewhat different in
 * HW versions. This helper function returns a suitable IPM bit field
 * value to use.
 */

static inline void ipm_helper1(int *ipm, int di)
{
	int pdm[DMIC_HW_CONTROLLERS];
	int i;

	/* Loop number of PDM controllers in the configuration. If mic A
	 * or B is enabled then a pdm controller is marked as active for
	 * this DAI.
	 */
	for (i = 0; i < DMIC_HW_CONTROLLERS; i++) {
		if (dmic_prm[di].pdm[i].enable_mic_a ||
		    dmic_prm[di].pdm[i].enable_mic_b)
			pdm[i] = 1;
		else
			pdm[i] = 0;
	}

	/* Set IPM to match active pdm controllers. */
	*ipm = 0;

	if (pdm[0] == 0 && pdm[1] > 0)
		*ipm = 1;

	if (pdm[0] > 0 && pdm[1] > 0)
		*ipm = 2;
}

#if DMIC_HW_VERSION >= 2

static inline void ipm_helper2(int source[], int *ipm, int di)
{
	int pdm[DMIC_HW_CONTROLLERS];
	int i;
	int n = 0;

	for (i = 0; i < OUTCONTROLX_IPM_NUMSOURCES; i++)
		source[i] = 0;

	/* Loop number of PDM controllers in the configuration. If mic A
	 * or B is enabled then a pdm controller is marked as active.
	 * The function returns in array source[] the indice of enabled
	 * pdm controllers to be used for IPM configuration.
	 */
	for (i = 0; i < DMIC_HW_CONTROLLERS; i++) {
		if (dmic_prm[di].pdm[i].enable_mic_a ||
		    dmic_prm[di].pdm[i].enable_mic_b) {
			pdm[i] = 1;
			source[n] = i;
			n++;
		} else {
			pdm[i] = 0;
		}
	}

	/* IPM bit field is set to count of active pdm controllers. */
	*ipm = pdm[0];
	for (i = 1; i < DMIC_HW_CONTROLLERS; i++)
		*ipm += pdm[i];
}
#endif

/* Loop number of PDM controllers in the configuration. The function
 * checks if the controller should operate as stereo or mono left (A)
 * or mono right (B) mode. Mono right mode is setup as channel
 * swapped mono left.
 */
static int stereo_helper(int stereo[], int swap[])
{
	int cnt;
	int i;
	int swap_check;
	int ret = 0;

	for (i = 0; i < DMIC_HW_CONTROLLERS; i++) {
		cnt = 0;
		if (dmic_prm[0].pdm[i].enable_mic_a ||
		    dmic_prm[1].pdm[i].enable_mic_a)
			cnt++;

		if (dmic_prm[0].pdm[i].enable_mic_b ||
		    dmic_prm[1].pdm[i].enable_mic_b)
			cnt++;

		/* Set stereo mode if both mic A anc B are enabled. */
		cnt >>= 1;
		stereo[i] = cnt;

		/* Swap channels if only mic B is used for mono processing. */
		swap[i] = (dmic_prm[0].pdm[i].enable_mic_b ||
			dmic_prm[1].pdm[i].enable_mic_b) && !cnt;

		/* Check that swap does not conflict with other DAI request */
		swap_check = dmic_prm[1].pdm[i].enable_mic_a ||
			dmic_prm[0].pdm[i].enable_mic_a;

		if (swap_check && swap[i])
			ret = -1;
	}
	return ret;
}

static int configure_registers(struct dmic_configuration *cfg)
{
	int stereo[DMIC_HW_CONTROLLERS];
	int swap[DMIC_HW_CONTROLLERS];
	uint32_t val;
	int32_t ci;
	uint32_t cu;
	int ipm;
	int of0;
	int of1;
	int fir_decim;
	int fir_length;
	int length;
	int edge;
	int soft_reset;
	int cic_mute;
	int fir_mute;
	int i;
	int j;
	int ret;
	/* int di = dai->index; */
	int di = 0; /* where to get this */
	int dccomp = 1;
	int array_a = 0;
	int array_b = 0;
	int bfth = 3; /* Should be 3 for 8 entries, 1 is 2 entries */
	int th = 0; /* Used with TIE=1 */

	/* Normal start sequence */
	soft_reset = 1;
	cic_mute = 1;
	fir_mute = 1;

#if (DMIC_HW_VERSION == 2 && DMIC_HW_CONTROLLERS > 2) || DMIC_HW_VERSION == 3
	int source[OUTCONTROLX_IPM_NUMSOURCES];
#endif

	/* OUTCONTROL0 and OUTCONTROL1 */
	of0 = (dmic_prm[0].fifo_bits == 32) ? 2 : 0;

#if DMIC_HW_FIFOS > 1
	of1 = (dmic_prm[1].fifo_bits == 32) ? 2 : 0;
#else
	of1 = 0;
#endif

#if DMIC_HW_VERSION == 1 || (DMIC_HW_VERSION == 2 && DMIC_HW_CONTROLLERS <= 2)
	if (di == 0) {
		ipm_helper1(&ipm, 0);
		val = OUTCONTROL0_TIE(0) |
			OUTCONTROL0_SIP(0) |
			OUTCONTROL0_FINIT(1) |
			OUTCONTROL0_FCI(0) |
			OUTCONTROL0_BFTH(bfth) |
			OUTCONTROL0_OF(of0) |
			OUTCONTROL0_IPM(ipm) |
			OUTCONTROL0_TH(th);
	} else {
		ipm_helper1(&ipm, 1);
		val = OUTCONTROL1_TIE(0) |
			OUTCONTROL1_SIP(0) |
			OUTCONTROL1_FINIT(1) |
			OUTCONTROL1_FCI(0) |
			OUTCONTROL1_BFTH(bfth) |
			OUTCONTROL1_OF(of1) |
			OUTCONTROL1_IPM(ipm) |
			OUTCONTROL1_TH(th);
	}
#endif

#if DMIC_HW_VERSION == 3 || (DMIC_HW_VERSION == 2 && DMIC_HW_CONTROLLERS > 2)
	if (di == 0) {
		ipm_helper2(source, &ipm, 0);
		val = OUTCONTROL0_TIE(0) |
			OUTCONTROL0_SIP(0) |
			OUTCONTROL0_FINIT(1) |
			OUTCONTROL0_FCI(0) |
			OUTCONTROL0_BFTH(bfth) |
			OUTCONTROL0_OF(of0) |
			OUTCONTROL0_IPM(ipm) |
			OUTCONTROL0_IPM_SOURCE_1(source[0]) |
			OUTCONTROL0_IPM_SOURCE_2(source[1]) |
			OUTCONTROL0_IPM_SOURCE_3(source[2]) |
			OUTCONTROL0_IPM_SOURCE_4(source[3]) |
			OUTCONTROL0_TH(th);
	} else {
		ipm_helper2(source, &ipm, 1);
		val = OUTCONTROL1_TIE(0) |
			OUTCONTROL1_SIP(0) |
			OUTCONTROL1_FINIT(1) |
			OUTCONTROL1_FCI(0) |
			OUTCONTROL1_BFTH(bfth) |
			OUTCONTROL1_OF(of1) |
			OUTCONTROL1_IPM(ipm) |
			OUTCONTROL1_IPM_SOURCE_1(source[0]) |
			OUTCONTROL1_IPM_SOURCE_2(source[1]) |
			OUTCONTROL1_IPM_SOURCE_3(source[2]) |
			OUTCONTROL1_IPM_SOURCE_4(source[3]) |
			OUTCONTROL1_TH(th);
	}
#endif

	dmic_nhlt.chan_ctrl_cfg[di] = val;

	ret = stereo_helper(stereo, swap);
	if (ret < 0) {
		/* dai_err(dai, "configure_registers(): enable conflict"); */
		return ret;
	}

	for (i = 0; i < DMIC_HW_CONTROLLERS; i++) {

		/* CIC */
		val = CIC_CONTROL_SOFT_RESET(soft_reset) |
			CIC_CONTROL_CIC_START_B(0) |
			CIC_CONTROL_CIC_START_A(0) |
			CIC_CONTROL_MIC_B_POLARITY(dmic_prm[di].pdm[i].polarity_mic_b) |
			CIC_CONTROL_MIC_A_POLARITY(dmic_prm[di].pdm[i].polarity_mic_a) |
			CIC_CONTROL_MIC_MUTE(cic_mute) |
			CIC_CONTROL_STEREO_MODE(stereo[i]);
		dmic_nhlt.pdm_ctrl_cfg[i].cic_control = val;

		val = CIC_CONFIG_CIC_SHIFT(cfg->cic_shift + 8) |
			CIC_CONFIG_COMB_COUNT(cfg->mcic - 1);
		dmic_nhlt.pdm_ctrl_cfg[i].cic_config = val;

		/* Mono right channel mic usage requires swap of PDM channels
		 * since the mono decimation is done with only left channel
		 * processing active.
		 */
		edge = dmic_prm[di].pdm[i].clk_edge;
		if (swap[i])
			edge = !edge;

		val = MIC_CONTROL_PDM_CLKDIV(cfg->clkdiv - 2) |
			MIC_CONTROL_PDM_SKEW(dmic_prm[di].pdm[i].skew) |
			MIC_CONTROL_CLK_EDGE(edge) |
			MIC_CONTROL_PDM_EN_B(0) |
			MIC_CONTROL_PDM_EN_A(0);
		dmic_nhlt.pdm_ctrl_cfg[i].mic_control = val;

		if (di == 0) {
			/* FIR A */
			fir_decim = MAX(cfg->mfir_a - 1, 0);
			fir_length = MAX(cfg->fir_a_length - 1, 0);
			val = FIR_CONTROL_A_START(0) |
				FIR_CONTROL_A_ARRAY_START_EN(array_a) |
				FIR_CONTROL_A_DCCOMP(dccomp) |
				FIR_CONTROL_A_MUTE(fir_mute) |
				FIR_CONTROL_A_STEREO(stereo[i]);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].fir_control = val;

			val = FIR_CONFIG_A_FIR_DECIMATION(fir_decim) |
				FIR_CONFIG_A_FIR_SHIFT(cfg->fir_a_shift) |
				FIR_CONFIG_A_FIR_LENGTH(fir_length);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].fir_config = val;

			val = DC_OFFSET_LEFT_A_DC_OFFS(DCCOMP_TC0);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].dc_offset_left = val;

			val = DC_OFFSET_RIGHT_A_DC_OFFS(DCCOMP_TC0);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].dc_offset_right = val;

			val = OUT_GAIN_LEFT_A_GAIN(0);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].out_gain_left = val;

			val = OUT_GAIN_RIGHT_A_GAIN(0);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].out_gain_right = val;

			/* Write coef RAM A with scaled coefficient in reverse order */
			length = cfg->fir_a_length;
			for (j = 0; j < length; j++) {
				ci = (int32_t)Q_MULTSR_32X32((int64_t)cfg->fir_a->coef[j],
							     cfg->fir_a_scale, 31,
							     DMIC_FIR_SCALE_Q, DMIC_HW_FIR_COEF_Q);
				cu = FIR_COEF_A(ci);
				dmic_nhlt.pdm_ctrl_cfg[i].fir_coeffs[j] = cu;
			}
		}

		if (di == 1) {
			/* FIR B */
			fir_decim = MAX(cfg->mfir_b - 1, 0);
			fir_length = MAX(cfg->fir_b_length - 1, 0);
			val = FIR_CONTROL_B_START(0) |
				FIR_CONTROL_B_ARRAY_START_EN(array_b) |
				FIR_CONTROL_B_DCCOMP(dccomp) |
				FIR_CONTROL_B_MUTE(fir_mute) |
				FIR_CONTROL_B_STEREO(stereo[i]);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].fir_control = val;

			val = FIR_CONFIG_B_FIR_DECIMATION(fir_decim) |
				FIR_CONFIG_B_FIR_SHIFT(cfg->fir_b_shift) |
				FIR_CONFIG_B_FIR_LENGTH(fir_length);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].fir_config = val;
			val = DC_OFFSET_LEFT_B_DC_OFFS(DCCOMP_TC0);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].dc_offset_left = val;

			val = DC_OFFSET_RIGHT_B_DC_OFFS(DCCOMP_TC0);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].dc_offset_right = val;

			val = OUT_GAIN_LEFT_B_GAIN(0);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].out_gain_left = val;

			val = OUT_GAIN_RIGHT_B_GAIN(0);
			dmic_nhlt.pdm_ctrl_cfg[i].fir_config[di].out_gain_right = val;

			/* Write coef RAM B with scaled coefficient in reverse order */
			length = cfg->fir_b_length;
			for (j = 0; j < length; j++) {
				ci = (int32_t)Q_MULTSR_32X32((int64_t)cfg->fir_b->coef[j],
							     cfg->fir_b_scale, 31,
							     DMIC_FIR_SCALE_Q, DMIC_HW_FIR_COEF_Q);
				cu = FIR_COEF_B(ci);
				dmic_nhlt.pdm_ctrl_cfg[i].fir_coeffs[j] = cu;
			}
		}
	}

	return 0;
}

void print_dmic_nhlt_bytes(struct dmic_configuration *cfg)
{
	uint8_t *p;
	int i, length;
	int line = 1;

	p = (uint8_t*)&dmic_nhlt;

	length = sizeof(struct dmic_nhlt_config_data) +
		cfg->fir_a_length * sizeof(uint32_t) +
		cfg->fir_b_length * sizeof(uint32_t);

	tplg_pp_debug("dmic nhlt bytes, length %d", length);

	/* todo check the last line length */
	for (i = 0; i < length; i += 8) {
		tplg_pp_debug("%d 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
			      line, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6),
			      *(p+7));
		line++;
		p += 8;
	}
}

static void print_dmic_nhlt(struct dmic_configuration *cfg)
{
	int i;
	int line;

	tplg_pp_debug("printing dmic nhlt blob");

	tplg_pp_debug("ts_group: %u %u %u %u", dmic_nhlt.ts_group[0], dmic_nhlt.ts_group[1],
		      dmic_nhlt.ts_group[2], dmic_nhlt.ts_group[3]);
	tplg_pp_debug("clock_on_delay: %u", dmic_nhlt.clock_on_delay);
	tplg_pp_debug("channel_ctrl_mask: %u", dmic_nhlt.channel_ctrl_mask);
	tplg_pp_debug("chan_ctrl_cfg: %u %u", dmic_nhlt.chan_ctrl_cfg[0],
		      dmic_nhlt.chan_ctrl_cfg[1]);
	tplg_pp_debug("channel_pdm_mask: %u", dmic_nhlt.channel_pdm_mask);

	tplg_pp_debug("pdm_ctrl_cfg 0");
	tplg_pp_debug("cic_control: %u", dmic_nhlt.pdm_ctrl_cfg[0].cic_control);
	tplg_pp_debug("cic_config: %u", dmic_nhlt.pdm_ctrl_cfg[0].cic_config);
	tplg_pp_debug("mic_control: %u", dmic_nhlt.pdm_ctrl_cfg[0].mic_control);
	tplg_pp_debug("pdmsm: %u", dmic_nhlt.pdm_ctrl_cfg[0].pdmsm);
	tplg_pp_debug("reuse_fir_from_pdm: %u", dmic_nhlt.pdm_ctrl_cfg[0].reuse_fir_from_pdm);

	tplg_pp_debug("fir_config 0");
	tplg_pp_debug("fir_control: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[0].fir_control);
	tplg_pp_debug("fir_config: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[0].fir_config);
	tplg_pp_debug("dc_offset_left: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[0].dc_offset_left);
	tplg_pp_debug("dc_offset_right: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[0].dc_offset_right);
	tplg_pp_debug("out_gain_left: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[0].out_gain_left);
	tplg_pp_debug("out_gain_right: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[0].out_gain_right);

	tplg_pp_debug("fir_config 1");
	tplg_pp_debug("fir_control: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[1].fir_control);
	tplg_pp_debug("fir_config: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[1].fir_config);
	tplg_pp_debug("dc_offset_left: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[1].dc_offset_left);
	tplg_pp_debug("dc_offset_right: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[1].dc_offset_right);
	tplg_pp_debug("out_gain_left: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[1].out_gain_left);
	tplg_pp_debug("out_gain_right: %u", dmic_nhlt.pdm_ctrl_cfg[0].fir_config[1].out_gain_right);

	tplg_pp_debug("fir_coeffs a:");
	/* todo check the last line length */
	line = 1;
	for (i = 0; i < cfg->fir_a_length; i += 8, line++) {
		tplg_pp_debug("%d %u %u %u %u %u %u %u %u", line,
			      dmic_nhlt.pdm_ctrl_cfg[0].fir_coeffs[i],
			      dmic_nhlt.pdm_ctrl_cfg[0].fir_coeffs[i+1],
			      dmic_nhlt.pdm_ctrl_cfg[0].fir_coeffs[i+2],
			      dmic_nhlt.pdm_ctrl_cfg[0].fir_coeffs[i+3],
			      dmic_nhlt.pdm_ctrl_cfg[0].fir_coeffs[i+4],
			      dmic_nhlt.pdm_ctrl_cfg[0].fir_coeffs[i+5],
			      dmic_nhlt.pdm_ctrl_cfg[0].fir_coeffs[i+6],
			      dmic_nhlt.pdm_ctrl_cfg[0].fir_coeffs[i+7]);
	}

	tplg_pp_debug("pdm_ctrl_cfg 1");
	tplg_pp_debug("cic_control: %u", dmic_nhlt.pdm_ctrl_cfg[1].cic_control);
	tplg_pp_debug("cic_config: %u", dmic_nhlt.pdm_ctrl_cfg[1].cic_config);
	tplg_pp_debug("mic_control: %u", dmic_nhlt.pdm_ctrl_cfg[1].mic_control);
	tplg_pp_debug("pdmsm: %u", dmic_nhlt.pdm_ctrl_cfg[1].pdmsm);
	tplg_pp_debug("reuse_fir_from_pdm: %u", dmic_nhlt.pdm_ctrl_cfg[1].reuse_fir_from_pdm);

	tplg_pp_debug("fir_config 0");
	tplg_pp_debug("fir_control: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[0].fir_control);
	tplg_pp_debug("fir_config: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[0].fir_config);
	tplg_pp_debug("dc_offset_left: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[0].dc_offset_left);
	tplg_pp_debug("dc_offset_right: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[0].dc_offset_right);
	tplg_pp_debug("out_gain_left: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[0].out_gain_left);
	tplg_pp_debug("out_gain_right: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[0].out_gain_right);

	tplg_pp_debug("fir_config 1");
	tplg_pp_debug("fir_control: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[1].fir_control);
	tplg_pp_debug("fir_config: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[1].fir_config);
	tplg_pp_debug("dc_offset_left: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[1].dc_offset_left);
	tplg_pp_debug("dc_offset_right: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[1].dc_offset_right);
	tplg_pp_debug("out_gain_left: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[1].out_gain_left);
	tplg_pp_debug("out_gain_right: %u", dmic_nhlt.pdm_ctrl_cfg[1].fir_config[1].out_gain_right);

	tplg_pp_debug("fir_coeffs b:");
		/* todo check the last line length */
	line = 1;
	for (i = 0; i < cfg->fir_b_length; i += 8, line++) {
		tplg_pp_debug("%d %u %u %u %u %u %u %u %u", line,
			      dmic_nhlt.pdm_ctrl_cfg[1].fir_coeffs[i],
			      dmic_nhlt.pdm_ctrl_cfg[1].fir_coeffs[i+1],
			      dmic_nhlt.pdm_ctrl_cfg[1].fir_coeffs[i+2],
			      dmic_nhlt.pdm_ctrl_cfg[1].fir_coeffs[i+3],
			      dmic_nhlt.pdm_ctrl_cfg[1].fir_coeffs[i+4],
			      dmic_nhlt.pdm_ctrl_cfg[1].fir_coeffs[i+5],
			      dmic_nhlt.pdm_ctrl_cfg[1].fir_coeffs[i+6],
			      dmic_nhlt.pdm_ctrl_cfg[1].fir_coeffs[i+7]);
	}

	print_dmic_nhlt_bytes(cfg);
}

int dmic_build_nhlt()
{
	/* struct dmic_pdata *dmic = dai_get_drvdata(dai); */
	struct matched_modes modes_ab;
	struct dmic_configuration cfg;
	struct decim_modes modes_a;
	struct decim_modes modes_b;
	int32_t unmute_ramp_time_ms;
	int ret = 0;
	int di = dai_index;

	/* Compute unmute ramp gain update coefficient. Use the value from
	 * topology if it is non-zero, otherwise use default length.
	 */
	if (dmic_prm[di].unmute_ramp_time)
		unmute_ramp_time_ms = dmic_prm[di].unmute_ramp_time;
	else
		unmute_ramp_time_ms = LOGRAMP_TIME_MS;

	if (unmute_ramp_time_ms < LOGRAMP_TIME_MIN_MS ||
	    unmute_ramp_time_ms > LOGRAMP_TIME_MAX_MS) {
		tplg_pp_debug("dmic_set_config(): Illegal ramp time = %d",
			      unmute_ramp_time_ms);
		ret = -1;
		goto out;
	}

	if (di >= DMIC_HW_FIFOS) {
		tplg_pp_debug("dmic_set_config(): dai->index exceeds number of FIFOs");
		ret = -1;
		goto out;
	}

	if (dmic_prm[di].num_pdm_active > DMIC_HW_CONTROLLERS) {
		tplg_pp_debug("dmic_set_config(): the requested PDM controllers count exceeds platform capability");
		ret = -1;
		goto out;
	}

	switch (dmic_prm[di].fifo_bits) {
	case 0:
	case 16:
	case 32:
		break;
	default:
		tplg_pp_debug("dmic_set_config(): fifo_bits EINVAL");
		ret = -1;
		goto out;
	}

	/* Match and select optimal decimators configuration for FIFOs A and B
	 * paths. This setup phase is still abstract. Successful completion
	 * points struct cfg to FIR coefficients and contains the scale value
	 * to use for FIR coefficient RAM write as well as the CIC and FIR
	 * shift values.
	 */
	tplg_pp_debug("dmic_set_config(): find modes 1");
	find_modes(&modes_a, dmic_prm[0].fifo_fs, di);
	if (modes_a.num_of_modes == 0 && dmic_prm[0].fifo_fs > 0) {
		tplg_pp_debug("dmic_set_config(): No modes found found for FIFO A");
		ret = -1;
		goto out;
	}

	tplg_pp_debug("dmic_set_config(): find modes 2");
	find_modes(&modes_b, dmic_prm[1].fifo_fs, di);
	if (modes_b.num_of_modes == 0 && dmic_prm[1].fifo_fs > 0) {
		tplg_pp_debug("dmic_set_config(): No modes found for FIFO B");
		ret = -1;
		goto out;
	}

	tplg_pp_debug("dmic_set_config(): match modes");
	match_modes(&modes_ab, &modes_a, &modes_b);
	ret = select_mode(&cfg, &modes_ab);
	if (ret < 0) {
		tplg_pp_debug("dmic_set_config(): select_mode() failed");
		ret = -1;
		goto out;
	}

	/* Struct reg contains a mirror of actual HW registers. Determine
	 * register bits configuration from decimator configuration and the
	 * requested parameters.
	 */
	ret = configure_registers(&cfg);
	if (ret < 0) {
		tplg_pp_debug("dmic_set_config(): cannot configure registers");
		ret = -1;
		goto out;
	}

	print_dmic_nhlt(&cfg);
out:
	return ret;
}

void dmic_set_dai_index(uint32_t val)
{
	dai_index = val;
}

void dmic_set_num_pdm_active(uint32_t val)
{
	dmic_prm[dai_index].num_pdm_active = val;
}

void dmic_set_fifo_word_length(uint32_t val)
{
	dmic_prm[dai_index].fifo_bits = val;
}

void dmic_set_clk_min(uint32_t val)
{
	dmic_prm[dai_index].pdmclk_min = val;
}

void dmic_set_clk_max(uint32_t val)
{
	dmic_prm[dai_index].pdmclk_max = val;
}

void dmic_set_duty_min(uint32_t val)
{
	dmic_prm[dai_index].duty_min = val;
}

void dmic_set_duty_max(uint32_t val)
{
	dmic_prm[dai_index].duty_max = val;
}

void dmic_set_sample_rate(uint32_t val)
{
	dmic_prm[dai_index].fifo_fs = val;
}

void dmic_set_unmute_ramp_time_ms(uint32_t val)
{
	dmic_prm[dai_index].unmute_ramp_time = val;
}

void dmic_set_ctrl_id(uint32_t val)
{
	pdm_index = val;
}

void dmic_set_mic_a_enable(uint32_t val)
{
	dmic_prm[dai_index].pdm[pdm_index].enable_mic_a = val;
}

void dmic_set_mic_b_enable(uint32_t val)
{
	dmic_prm[dai_index].pdm[pdm_index].enable_mic_b = val;
}

void dmic_set_polarity_a(uint32_t val)
{
	dmic_prm[dai_index].pdm[pdm_index].polarity_mic_a = val;
}

void dmic_set_polarity_b(uint32_t val)
{
	dmic_prm[dai_index].pdm[pdm_index].polarity_mic_b = val;
}

void dmic_set_clk_edge(uint32_t val)
{
	dmic_prm[dai_index].pdm[pdm_index].clk_edge = val;
}

void dmic_set_skew(uint32_t val)
{
	dmic_prm[dai_index].pdm[pdm_index].skew = val;
}
