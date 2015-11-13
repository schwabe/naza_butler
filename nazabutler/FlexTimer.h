/* FlexTimer.h - Partial FlexTimer header definitions.
 * Copyright (c) 2014, Robert Collins http://www.rcollins.org
 *
 * Boiler Plate BS:
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*---------------------------------------------------------------------
 * FTM_SC Register definition.  Defined in https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf, page 780.
 * Bits[31:08]	= Reserved
 * Bits[07:07]	= TOF 	: Timer Overflow Flag (interrupt flag)
 *                  0 : FTM Counter has not overflowed.
 * 					1 : FTM Counter has overflowed.
 * Bits[06:06]	= TOIE	: Timer Overflow Interrupt Enable
 * 					0 : Disable TOF Interrupt
 * 					1 : Enable TOF Interrupt
 * Bits[05:05]	= CPWMS	: Center-Aligned PWM Select
 * 					0 : FTM counter operates in Up Counting mode.
 * 					1 : FTM counter operates in Up-Down Counting mode.
 * Bits[04:03]	= CLKS	: Clock Select
 * 					00 : No clock selected. This in effect disables the FTM counter
 * 					01 : System Clock
 * 					10 : Fixed frequency Clock
 *                  11 : External Clock
 * Bits[02:00]	= PS	: Prescale Factor Selection
 * 					000 : Divide by 1
 * 					001 : Divide by 2
 * 					...
 *                  110 : Divide by 64
 * 					111 : Divide by 128
 * ------------------------------------------------------------------*/
/* FTM SC Register Definition						*/
/* 	Bit positions									*/
#define	FTM_SC_TOF_POS			7
#define FTM_SC_TOIE_POS			6
#define FTM_SC_CPWMS_POS   		5
#define FTM_SC_CLKS_POS			3
#define FTM_SC_PS_POS			0

/* Mask values										*/
#define	FTM_SC_TOF_MASK			1
#define FTM_SC_TOIE_MASK		1
#define FTM_SC_CPWMS_MASK   	1
#define FTM_SC_CLKS_MASK		3
#define FTM_SC_PS_MASK			7

/* Definition macros								*/
#define	FTM_SC_TOF_VAL(n)		(((n) & FTM_SC_TOF_MASK)		<< FTM_SC_TOF_POS)
#define	FTM_SC_TOF_INT			1														/* Interrupt occurred	*/

#define FTM_SC_TOIE_VAL(n)		(((n) & FTM_SC_TOIE_MASK)		<< FTM_SC_TOIE_POS)
#define	FTM_SC_TOIE_IE			1							                            /* Interrupt Enabled	*/

#define FTM_SC_CPWMS_VAL(n)		(((n) & FTM_SC_CPWMS_MASK)		<< FTM_SC_CPWMS_POS)
#define FTM_SC_CPWMS_UP_ONLY	0                                                       /* FTM counter operates in Up Counting mode.		*/
#define FTM_SC_CPWMS_UP_DOWN	1                                                       /* FTM counter operates in Up-Down Counting mode.	*/

#define FTM_SC_CLKS_VAL(n)		(((n) & FTM_SC_CLKS_MASK)		<< FTM_SC_CLKS_POS)
#define FTM_SC_CLKS_OFF			0                                                       /* No clock selected. This in effect disables the FTM counter	*/
#define FTM_SC_CLKS_SYSTEM		1                                                       /* System Clock													*/
#define FTM_SC_CLKS_FIXED		2                                                       /* Fixed frequency Clock										*/
#define FTM_SC_CLKS_EXTERNAL	3                                                       /* External Clock												*/

#define FTM_SC_PS_VAL(n)		(((n) & FTM_SC_PS_MASK)			<< FTM_SC_PS_POS)
#define FTM_SC_PS_1				0                                                       /* Divide by 1			*/
#define FTM_SC_PS_2             1                                                       /* Divide by 2          */
#define FTM_SC_PS_4             2                                                       /* Divide by 4          */
#define FTM_SC_PS_8             3                                                       /* Divide by 8          */
#define FTM_SC_PS_16            4                                                       /* Divide by 16         */
#define FTM_SC_PS_32            5                                                       /* Divide by 32         */
#define FTM_SC_PS_64            6                                                       /* Divide by 64         */
#define FTM_SC_PS_128           7                                                       /* Divide by 128        */

/*---------------------------------------------------------------------
 * FTM_CnSC Register definition.  Defined in https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf, page 783.
 * Bits[31:08]	= Reserved
 * Bits[07:07]	= CHF 	: Channel Flag
 *                  0 : No channel event occurred (interrupt)
 * 					1 : A channel event has occurred (interrupt)
 * Bits[06:06]	= CHIE	: Channel Interrupt Enable
 * 					0 : Disable channel interrupt
 * 					1 : Enable channel interrupt
 * Bits[05:04]	= MSx	: Channel Mode Select
 * 					00 : Input capture
 * 					01 : Output Compare
 * 					1X : Edge-aligned PWM
 * 					XX : Center aligned PWM, Combine PWM
 * Bits[03:02]	= ELSx	: Edge or Level Select (context dependant)
 * Bits[01:01]	= Reserved
 * Bits[00:00]	= DMA Enable
 * 					0 : Disable DMA transfers
 * 					1 : Enable DMA transfers
 * ------------------------------------------------------------------*/

/* FTM CnSC Register Definition						*/
/* 	Bit positions									*/
#define	FTM_CnSC_CHF_POS			7
#define FTM_CnSC_CHIE_POS			6
#define FTM_CnSC_MSx_POS			4
#define FTM_CnSC_ELSx_POS			2
#define FTM_CnSC_RSVD01_POS			1
#define FTM_CnSC_DMA_POS			0

/* Mask values										*/
#define	FTM_CnSC_CHF_MASK			1
#define FTM_CnSC_CHIE_MASK			1
#define FTM_CnSC_MSx_MASK			3
#define FTM_CnSC_ELSx_MASK			3
#define FTM_CnSC_RSVD01_MASK		1
#define FTM_CnSC_DMA_MASK			1

/* Definition macros								*/
#define	FTM_CnSC_CHF_VAL(n)			(((n) & FTM_CnSC_CHF_MASK)		<< FTM_CnSC_CHF_POS)
#define FTM_CnSC_CHF_INT			1														/* Interrupt Occurred	*/

#define FTM_CnSC_CHIE_VAL(n)		(((n) & FTM_CnSC_CHIE_MASK)		<< FTM_CnSC_CHIE_POS)
#define FTM_CnSC_CHIE_ENABLE		1														/* Interrupt Enable		*/
#define FTM_CnSC_CHIE_DISABLE		0														/* Interrupt Disable	*/

#define FTM_CnSC_MSx_VAL(n)			(((n) & FTM_CnSC_MSx_MASK)		<< FTM_CnSC_MSx_POS)
#define FTM_CnSC_MSx_INPUT			0														/* Input capture		*/
#define FTM_CnSC_MSx_OUTPUT			1														/* Output compare		*/
#define	FTM_CnSC_MSx_EDGE_PWM		2														/* Edge-aligned PWM		*/
#define FTM_CnSC_MSx_COMBI_PWM		3														/* Center aligned, or combined PWM	*/

#define FTM_CnSC_ELSx_VAL(n)		(((n) & FTM_CnSC_ELSx_MASK)		<< FTM_CnSC_ELSx_POS)
#define FTM_CnSC_ELSx_INPUT_NONE	0														/* No input	*/
#define FTM_CnSC_ELSx_INPUT_RISING	1														/* Input capture, rising edge	*/
#define FTM_CnSC_ELSx_INPUT_FALLING	2														/* Input capture, falling edge	*/
#define FTM_CnSC_ELSx_INPUT_EDGE	3														/* Input capture, rising or falling edge	*/
#define	FTM_CnSC_ELSx_OUTPUT_TOGGLE	1														/* Output compare, Toggle output pin		*/
#define	FTM_CnSC_ELSx_OUTPUT_CLEAR	2														/* Output compare, Clear output pin		*/
#define	FTM_CnSC_ELSx_OUTPUT_SET	3														/* Output compare, Set output pin		*/

#define FTM_CnSC_RSVD01_VAL(n)		(((n) & FTM_CnSC_RSVD01_MASK)	<< FTM_CnSC_RSVD01_POS)
#define FTM_CnSC_DMA_VAL(n)			(((n) & FTM_CnSC_DMA_MASK)		<< FTM_CnSC_DMA_POS)
#define FTM_DMA_ENA					1														/* Enable DMA transfers	*/
#define FTM_DMA_DISA				0														/* Disable DMA transfers	*/

struct	FlexTimerBase_Struct {
	volatile	uint32_t	SC;			/* Status and Control			*/
	volatile	uint32_t	CNT;		/* Counter						*/
	volatile	uint32_t    MOD;		/* Modulo						*/
	volatile	uint32_t    C0SC;		/* Status and Control Register	*/
	volatile	uint32_t	C0V;		/* Channel Value Register		*/
	volatile	uint32_t    C1SC;		/* Status and Control Register	*/
	volatile	uint32_t	C1V;		/* Channel Value Register		*/
	volatile	uint32_t    C2SC;		/* Status and Control Register	*/
	volatile	uint32_t	C2V;		/* Channel Value Register		*/
	volatile	uint32_t    C3SC;		/* Status and Control Register	*/
	volatile	uint32_t	C3V;		/* Channel Value Register		*/
	volatile	uint32_t    C4SC;		/* Status and Control Register	*/
	volatile	uint32_t	C4V;		/* Channel Value Register		*/
	volatile	uint32_t    C5SC;		/* Status and Control Register	*/
	volatile	uint32_t	C5V;		/* Channel Value Register		*/
	volatile	uint32_t    C6SC;		/* Status and Control Register	*/
	volatile	uint32_t	C6V;		/* Channel Value Register		*/
	volatile	uint32_t    C7SC;		/* Status and Control Register	*/
	volatile	uint32_t	C7V;		/* Channel Value Register		*/
	volatile	uint32_t	CNTIN;		/* Counter initial value		*/
	volatile	uint32_t    STATUS;		/* Capture and Compare status	*/
	volatile	uint32_t    MODE;		/* Features Mode selection		*/
	volatile	uint32_t    SYNC;		/* Synchronization				*/
	volatile	uint32_t    OUTINIT;	/* Initial state for channels output	*/
	volatile	uint32_t    OUTMASK;	/* Output mask					*/
	volatile	uint32_t    COMBINE;	/* Function for linked channels	*/
	volatile	uint32_t    DEADTIME;	/* Deadtime insertion control	*/
	volatile	uint32_t    EXTTRIG;	/* FTM external trigger			*/
	volatile	uint32_t    POL;		/* Channels polarity			*/
	volatile	uint32_t	FMS;		/* Fault Mode Status			*/
	volatile	uint32_t    FILTER;		/* Input capture filter			*/
	volatile	uint32_t    FLTCTRL;	/* Fault Control				*/
	volatile	uint32_t    QDCTRL;		/* Quadrature Decoder Control & Status	*/
	volatile	uint32_t    CONF;		/* Configuration				*/
	volatile	uint32_t	FLTPOL;		/* FTM Fault Input Polarity		*/
	volatile	uint32_t    SYNCONF;	/* Syncronization Configuration	*/
	volatile	uint32_t    INVCTRL;	/* FTM Inverting Control		*/
	volatile	uint32_t    SWOCTRL;	/* FTM Software Output Control	*/
	volatile	uint32_t	PWMCTRL;	/* FTM PWM Load					*/
};

struct	FlexTimerChannel_Struct {
	volatile	uint32_t    CnSC;		/* Status and Control Register	*/
	volatile	uint32_t	CnV;		/* Channel Value Register		*/
};
