/*
 * Copyright (c) 2021 Ievgenii Meshcheriakov.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APPLICATION_SRC_AX5X43_REGS_H_
#define APPLICATION_SRC_AX5X43_REGS_H_
#include <sys/util.h>

/* SPI Status Bits. */
#define AX5X43_STATUS_POWERED       BIT(15)
#define AX5X43_STAT_PLL_LOCK        BIT(14)
#define AX5X43_STAT_FIFO_OVER       BIT(13)
#define AX5X43_STAT_FIFO_UNDER      BIT(12)
#define AX5X43_STAT_THRES_FREE      BIT(11)
#define AX5X43_STAT_THRES_COUNT     BIT(10)
#define AX5X43_STAT_FIFO_FULL       BIT(9)
#define AX5X43_STAT_FIFO_EMPTY      BIT(8)
#define AX5X43_STAT_PWRGOOD         BIT(7)
#define AX5X43_STAT_PWR_INT_PEND    BIT(6)
#define AX5X43_STAT_RADIO_EVT_PEND  BIT(5)
#define AX5X43_STAT_XTAL_OSC_RUNN   BIT(4)
#define AX5X43_STAT_WAKEUP_INT_PEND BIT(3)
#define AX5X43_STAT_LPOSC_INT_PEND  BIT(2)
#define AX5X43_STAT_GPADC_INT_PEND  BIT(1)

/* FIFO Chunk Types */
#define AX5X43_CHUNK_NOP           0x00
#define AX5X43_CHUNK_RSSI          0x31
#define AX5X43_CHUNK_TXCTRL        0x3C
#define AX5X43_CHUNK_FREQOFFS      0x52
#define AX5X43_CHUNK_ANTRSSI2      0x55
#define AX5X43_CHUNK_REPEATDATA    0x62
#define AX5X43_CHUNK_TIMER         0x70
#define AX5X43_CHUNK_RFFREQOFFS    0x73
#define AX5X43_CHUNK_DATARATE      0x74
#define AX5X43_CHUNK_ANTRSSI3      0x75
#define AX5X43_CHUNK_DATA          0xE1
#define AX5X43_CHUNK_TXPWR         0xFD

#define AX5X43_REPEATDATA_PKTSTART BIT(0)
#define AX5X43_REPEATDATA_PKTEND   BIT(1)
#define AX5X43_REPEATDATA_RESIDUE  BIT(2)
#define AX5X43_REPEATDATA_NOCRC    BIT(3)
#define AX5X43_REPEATDATA_RAW      BIT(4)
#define AX5X43_REPEATDATA_UNENC    BIT(5)

#define AX5X43_DATA_PKTSTART       BIT(0)
#define AX5X43_DATA_PKTEND         BIT(1)
#define AX5X43_DATA_RESIDUE        BIT(2)
#define AX5X43_DATA_NOCRC          BIT(3)
#define AX5X43_DATA_RAW            BIT(4)
#define AX5X43_DATA_UNENC          BIT(5)

/* Interrupt request indices */
#define AX5X43_IRQ_FIFONOTEMPTY    BIT(0)
#define AX5X43_IRQ_FIFONOTFULL     BIT(1)
#define AX5X43_IRQ_FIFOTHRCNT      BIT(2)
#define AX5X43_IRQ_FIFOTHRFREE     BIT(3)
#define AX5X43_IRQ_FIFOERROR       BIT(4)
#define AX5X43_IRQ_PLLUNLOCK       BIT(5)
#define AX5X43_IRQ_RADIOCTRL       BIT(6)
#define AX5X43_IRQ_POWER           BIT(7)
#define AX5X43_IRQ_XTALREADY       BIT(8)
#define AX5X43_IRQ_WAKEUPTIMER     BIT(9)
#define AX5X43_IRQ_LPOSC           BIT(10)
#define AX5X43_IRQ_GPADC           BIT(11)
#define AX5X43_IRQ_PLLRNGDONE      BIT(12)

/** Last register that can be accessed using short address. */
#define AX5X43_LAST_DYN_REG        0x06F

/* Revision & Interface Probing */
#define AX5X43_REG_REVISION        0x000 /* Silicon Revision */
#define AX5X43_REG_SCRATCH         0x001 /* Scratch Register */

/* Operating Mode */
#define AX5X43_REG_PWRMODE         0x002 /* Power Mode */

/* Voltage Regulator */
#define AX5X43_REG_POWSTAT         0x003 /* Power Management Status */
#define AX5X43_REG_POWSTICKYSTAT   0x004 /* Power Management Sticky Status */
#define AX5X43_REG_POWIRQMASK      0x005 /* Power Management Interrupt Mask */

/* Interrupt Control */
#define AX5X43_REG_IRQMASK         0x006 /* IRQ Mask */
#define AX5X43_REG_RADIOEVENTMASK  0x008 /* Radio Event Mask */
#define AX5X43_REG_IRQINVERSION    0x00A /* IRQ Inversion */
#define AX5X43_REG_IRQREQUEST      0x00C /* IRQ Request */
#define AX5X43_REG_RADIOEVENTREQ   0x00E /* Radio Event Request */

/* Modulation & Framing */
#define AX5X43_REG_MODULATION      0x010 /* Modulation */
#define AX5X43_REG_ENCODING        0x011 /* Encoder/Decoder Settings */
#define AX5X43_REG_FRAMING         0x012 /* Framing Settings */
#define AX5X43_REG_CRCINIT         0x013 /* CRC Initialization Data */

/* Forward Error Correction */
#define AX5X43_REG_FEC             0x018 /* FEC (Viterbi) Configuration */
#define AX5X43_REG_FECSYNC         0x019 /* Interleaver Synchronization Threshold */
#define AX5X43_REG_FECSTATUS       0x01A /* FEC Status */

/* Status */
#define AX5X43_REG_RADIOSTATE      0x01C /* Radio Controller State */
#define AX5X43_REG_XTALSTATUS      0x01D /* Crystal Oscillator Status */

/* Pin Configuration */
#define AX5X43_REG_PINSTATE        0x020 /* Pin State */
#define AX5X43_REG_PINFUNCSYSCLK   0x021 /* SYSCLK Pin Function */
#define AX5X43_REG_PINFUNCDCLK     0x022 /* DCLK Pin Function */
#define AX5X43_REG_PINFUNCDATA     0x023 /* DATA Pin Function */
#define AX5X43_REG_PINFUNCIRQ      0x024 /* IRQ Pin Function */
#define AX5X43_REG_PINFUNCTCXO_EN  0x026 /* TCXO_EN Pin Function */
#define AX5X43_REG_PWRAMP          0x027 /* PWRAMP Control */

/* FIFO */
#define AX5X43_REG_FIFOSTAT        0x028 /* FIFO Control */
#define AX5X43_REG_FIFODATA        0x029 /* FIFO Data */
#define AX5X43_REG_FIFOCOUNT       0x02A /* Number of Words currently in FIFO */
#define AX5X43_REG_FIFOFREE        0x02C /* Number of Words that can be written to FIFO */
#define AX5X43_REG_FIFOTHRESH      0x02F /* FIFO Threshold */

/* Synthesizer */
#define AX5X43_REG_PLLLOOP         0x030 /* PLL Loop Filter Settings */
#define AX5X43_REG_PLLCPI          0x031 /* PLL Charge Pump Current (Boosted) */
#define AX5X43_REG_PLLVCODIV       0x032 /* PLL Divider Settings */
#define AX5X43_REG_PLLRANGINGA     0x033 /* PLL Autoraging */
#define AX5X43_REG_FREQA           0x034 /* Synthesizer Frequency */
#define AX5X43_REG_PLLLOOPBOOST    0x038 /* PLL Loop Filter Settings (Boosted) */
#define AX5X43_REG_PLLCPIBOOST     0x039 /* PLL Charge Pump Current */
#define AX5X43_REG_PLLRANGINGB     0x03B /* PLL Autoranging */
#define AX5X43_REG_FREQB           0x03C /* Synthesizer Frequency */

/* Signal Strenght */
#define AX5X43_REG_RSSI            0x040 /* Received Signal Strength Indicator */
#define AX5X43_REG_BGNDRSSI        0x041 /* Background RSSI */
#define AX5X43_REG_DIVERSITY       0x042 /* Antenna Diversity Configuration */
#define AX5X43_REG_AGCCOUNTER      0x043 /* AGC Current Value */

/* Receiver Tracking */
#define AX5X43_REG_TRKDATARATE     0x045 /* Datarate Tracking */
#define AX5X43_REG_TRKAMPL         0x048 /* Amplitude Tracking */
#define AX5X43_REG_TRKPHASE        0x04A /* Phase Tracking */
#define AX5X43_REG_TRKRFFREQ       0x04D /* RF Frequency Tracking */
#define AX5X43_REG_TRKFREQ         0x050 /* Frequency Tracking */
#define AX5X43_REG_TRKFSKDEMOD     0x052 /* FSK Demodulator Tracking */
#define AX5X43_REG_TRKAFSKDEMOD    0x054 /* AFSK Demodulator Tracking */

/* Timer */
#define AX5X43_REG_TIMER           0x059 /* 1 MHz Timer */

/* Wakeup Timer */
#define AX5X43_REG_WAKEUPTIMER     0x068 /* Wakeup Timer */
#define AX5X43_REG_WAKEUP          0x06A /* Wakeup Time */
#define AX5X43_REG_WAKEUPFREQ      0x06C /* Wakeup Frequency */
#define AX5X43_REG_WAKEUPXOEARLY   0x06E /* Wakeup Crystal Oscillator Early */

/* Physical Layer Parameters */

/* Receiver Parameters */
#define AX5X43_REG_IFFREQ          0x100 /* 2nd LO / IF Frequency */
#define AX5X43_REG_DECIMATION      0x102 /* Decimation Factor */
#define AX5X43_REG_RXDATARATE      0x103 /* Receiver Datarate */
#define AX5X43_REG_MAXDROFFSET     0x106 /* Maximum Receiver Datarate Offset */
#define AX5X43_REG_MAXRFOFFSET     0x109 /* Maximum Receiver RF Offset */
#define AX5X43_REG_FSKDMAX         0x10C /* Four FSK RX Deviation */
#define AX5X43_REG_FSKDMIN         0x10E /* Four FSK RX Deviation */
#define AX5X43_REG_AFSKSPACE       0x110 /* AFSK Space (0) Frequency */
#define AX5X43_REG_AFSKMARK        0x112 /* AFSK Mark (1) Frequency */
#define AX5X43_REG_AFSKCTRL        0x114 /* AFSK Control */
#define AX5X43_REG_AMPLFILTER      0x115 /* Aplitude Filter */
#define AX5X43_REG_FREQUENCYLEAK   0x116 /* Baseband Frequency Recovery Loop Leakiness */
#define AX5X43_REG_RXPARAMSETS     0x117 /* Receiver Parameter Set Indirection */
#define AX5X43_REG_RXPARAMCURSET   0x118 /* Receiver Current Parameter Set */

/* Receiver Parameter Sets */
#define AX5X43_RX_PARAM_SET0       0x120
#define AX5X43_RX_PARAM_SET1       0x130
#define AX5X43_RX_PARAM_SET2       0x140
#define AX5X43_RX_PARAM_SET3       0x150

/* Receiver Parameter Register Offsets */
#define AX5X43_RX_AGCGAIN          0x000 /* AGC Speed */
#define AX5X43_RX_AGCTARGET        0x001 /* AGC Target */
#define AX5X43_RX_AGCAHYST         0x002 /* AGC Digital Threshold Range */
#define AX5X43_RX_AGCMINMAX        0x003 /* AGC Digital Min/Max Set Points */
#define AX5X43_RX_TIMEGAIN         0x004 /* Timing Gain */
#define AX5X43_RX_DRGAIN           0x005 /* Data Rate Gain */
#define AX5X43_RX_PHASEGAIN        0x006 /* Filter Index, Phase Gain */
#define AX5X43_RX_FREQGAIN         0x007 /* Frequency Gain */
#define AX5X43_RX_AMPLGAIN         0x00B /* Aplitude Gain */
#define AX5X43_RX_FREQDEV          0x00C /* Receiver Frequency Deviation */
#define AX5X43_RX_FOURFSK          0x00E /* Four FSK Control */
#define AX5X43_RX_BBOFFSRES        0x00F /* Baseband Offset Compensation Resitiors */

/* Transmitter Parameters */
#define AX5X43_REG_MODCFGF         0x160 /* Modulator Configuration F */
#define AX5X43_REG_FSKDEV          0x161 /* FSK Frequency Deviation */
#define AX5X43_REG_MODCFGA         0x164 /* Modulator Configuration A */
#define AX5X43_REG_TXRATE          0x165 /* Transmitter Bitrate */
#define AX5X43_REG_TXPWRCOEFFA     0x168 /* Transmitter Predistortio Coefficient A */
#define AX5X43_REG_TXPWRCOEFFB     0x16A /* Transmitter Predistortio Coefficient B */
#define AX5X43_REG_TXPWRCOEFFC     0x16C /* Transmitter Predistortio Coefficient C */
#define AX5X43_REG_TXPWRCOEFFD     0x16E /* Transmitter Predistortio Coefficient D */
#define AX5X43_REG_TXPWRCOEFFE     0x170 /* Transmitter Predistortio Coefficient E */

/* PLL Parameters */
#define AX5X43_REG_PLLVCOI         0x180 /* VCO Current */
#define AX5X43_REG_PLLVCOIR        0x181 /* VCO Current Readback */
#define AX5X43_REG_PLLLOCKDET      0x182 /* PLL Lock Detect Delay */
#define AX5X43_REG_PLLRNGCLK       0x183 /* PLL Ranging Clock */

/* Crystal Oscillator */
#define AX5X43_REG_XTALCAP         0x184 /* Crystal Oscillator Load Capacitance */

/* Baseband */
#define AX5X43_REG_BBTUNE          0x188 /* Baseband Tuning */
#define AX5X43_REG_BBOFFSCAP       0x189 /* Baseband Offset Compensation Capacitors */

/* MAC Layer Parameters */

/* Packet Format */
#define AX5X43_REG_PKTADDRCFG      0x200 /* Packet Address Config */
#define AX5X43_REG_PKTLENCFG       0x201 /* Packet Length Config */
#define AX5X43_REG_PKTLENOFFSET    0x202 /* Packet Length Offset */
#define AX5X43_REG_PKTMAXLEN       0x203 /* Packet Maximum Length */
#define AX5X43_REG_PKTADDR         0x204 /* Packet Address */
#define AX5X43_REG_PKTADDRMASK     0x208 /* Packet Address Mask */

/* Pattern Match */
#define AX5X43_REG_MATCH0PAT       0x210 /* Pattern Match Unit 0, Pattern */
#define AX5X43_REG_MATCH0LEN       0x214 /* Pattern Match Unit 0, Pattern Length */
#define AX5X43_REG_MATCH0MIN       0x215 /* Pattern Match Unit 0, Minimum Match */
#define AX5X43_REG_MATCH0MAX       0x216 /* Pattern Match Unit 0, Maximum Match */
#define AX5X43_REG_MATCH1PAT       0x218 /* Pattern Match Unit 1, Pattern */
#define AX5X43_REG_MATCH1LEN       0x21C /* Pattern Match Unit 1, Pattern Length */
#define AX5X43_REG_MATCH1MIN       0x21D /* Pattern Match Unit 1, Minimum Match */
#define AX5X43_REG_MATCH1MAX       0x21E /* Pattern Match Unit 1, Maximum Match */

/* Packet Controller */
#define AX5X43_REG_TMGTXBOOST      0x220 /* Transmit PLL Boost Time */
#define AX5X43_REG_TMGTXSETTLE     0x221 /* Transmit PLL (post boost) Settling Time */
#define AX5X43_REG_TMGRXBOOST      0x223 /* Receive PLL Boost Time */
#define AX5X43_REG_TMGRXSETTLE     0x224 /* Receive PLL (post boost) Settling Time */
#define AX5X43_REG_TMGRXOFFSACQ    0x225 /* Receive Baseband DC Offset Acquisition Time */
#define AX5X43_REG_TMGRXCOARSEAGC  0x226 /* Receive Coarse AGC Time */
#define AX5X43_REG_TMGRXAGC        0x227 /* Receiver AGC Settling Time */
#define AX5X43_REG_TMGRXRSSI       0x228 /* Receiver RSSI Settling Time */
#define AX5X43_REG_TMGRXPREAMBLE1  0x229 /* Receiver Preamble 1 Timeout */
#define AX5X43_REG_TMGRXPREAMBLE2  0x22A /* Receiver Preamble 2 Timeout */
#define AX5X43_REG_TMGRXPREAMBLE3  0x22B /* Receiver Preamble 3 Timeout */
#define AX5X43_REG_RSSIREFERENCE   0x22C /* RSSI Offset */
#define AX5X43_REG_RSSIABSTHR      0x22D /* RSSI Absolute Threshold */
#define AX5X43_REG_BGNDRSSIGAIN    0x22E /* Background RSSI Averaging Time Constant */
#define AX5X43_REG_BGNDRSSITHR     0x22F /* Background RSSI Relative Threshold */
#define AX5X43_REG_PKTCHUNKSIZE    0x230 /* Packet Chunk Size */
#define AX5X43_REG_PKTMISCFLAGS    0x231 /* Packet Controller Miscellaneous Flags */
#define AX5X43_REG_PKTSTOREFLAGS   0x232 /* Packet Controller Store Flags */
#define AX5X43_REG_PKTACCEPTFLAGS  0x233 /* Packet Controller Accept Flags */

/* Special Functions */

/* General Purpose ADC */
#define AX5X43_REG_GPADCCTRL       0x300 /* General Purpose ADC Control */
#define AX5X43_REG_GPADCPERIOD     0x301 /* GPADC Sampling Period */
#define AX5X43_REG_GPADC13VALUE    0x308 /* GPADC13 Value */

/* Low Power Oscillator Calibration */
#define AX5X43_REG_LPOSCCONFIG     0x310 /* Low Power Oscillator Configuration */
#define AX5X43_REG_LPOSCSTATUS     0x311 /* Low Power Oscillator Status */
#define AX5X43_REG_LPOSCKFILT      0x312 /* Low Power Oscillator Calibration Filter Constant */
#define AX5X43_REG_LPOSCREF        0x314 /* Low Power Oscillator Calibration Reference */
#define AX5X43_REG_LPOSCFREQ       0x317 /* Low Power Oscillator Calibration Frequency */
#define AX5X43_REG_LPOSCPER        0x319 /* Low Power Oscillator Calibration Period */

/* DAC */
#define AX5X43_REG_DACVALUE        0x330 /* DAC Value */
#define AX5X43_REG_DACCONFIG       0x332 /* DAC Configuration */

/* Performance Tuning */
#define AX5X43_REG_XTALOSC         0xF10 /* Ibias */
#define AX5X43_REG_XTALAMP         0xF11 /* Ibias autoreg. */

#define AX5X43_REVISION_VALUE        0x51

#define AX5X43_SCRATCH_VALUE         0xC5

#define AX5X43_PWRMODE_PWRMODE_MASK  BIT_MASK(4)
#define AX5X43_PWRMODE_POWERDOWN     0x0
#define AX5X43_PWRMODE_DEEPSLEEP     0x1
#define AX5X43_PWRMODE_STANDBY       0x5
#define AX5X43_PWRMODE_FIFOON        0x7
#define AX5X43_PWRMODE_SYNTHRX       0x8
#define AX5X43_PWRMODE_FULLRX        0x9
#define AX5X43_PWRMODE_WORRX         0xB
#define AX5X43_PWRMODE_SYNTHTX       0xC
#define AX5X43_PWRMODE_FULLTX        0xD
#define AX5X43_PWRMODE_WDS           BIT(4)
#define AX5X43_PWRMODE_XOEN          BIT(5)
#define AX5X43_PWRMODE_REFEN         BIT(6)
#define AX5X43_PWRMODE_RST           BIT(7)

#define AX5X43_PLLRANGING_RNGSTART   BIT(4)
#define AX5X43_PLLRANGING_RNGERR     BIT(5)
#define AX5X43_PLLRANGING_PLLLOCK    BIT(6)
#define AX5X43_PLLRANGING_STICKYLOCK BIT(7)

#define AX5X43_XTALSTATUS_XTALRUN    BIT(0)

#define AX5X43_MODULATION_MSK        0x7

#define AX5X43_ENCODING_INV          BIT(0)
#define AX4X43_ENCODING_DIFF         BIT(1)
#define AX4X43_ENCODING_SCRAM        BIT(2)
#define AX4X43_ENCODING_MANCH        BIT(3)
#define AX4X43_ENCODING_NOSYNC       BIT(4)

#define AX5X43_FRMMODE_HDLC          (0x2 << 1)
#define AX5X43_CRCMODE_CCITT         (0x1 << 4)
#define AX5X43_CRCMODE_CRC16         (0x2 << 4)

#define AX5X43_ACCPT_RESIDUE         BIT(0)
#define AX5X43_ACCPT_ABRT            BIT(1)
#define AX5X43_ACCPT_CRCF            BIT(2)
#define AX5X43_ACCPT_ADDRF           BIT(3)
#define AX5X43_ACCPT_SZF             BIT(4)
#define AX5X43_ACCPT_LRGP            BIT(5)

#define AX5X43_ST_TIMER              BIT(0)
#define AX5X43_ST_FOFFS              BIT(1)
#define AX5X43_ST_RFOFFS             BIT(2)
#define AX5X43_ST_DR                 BIT(3)
#define AX5X43_ST_RSSI               BIT(4)
#define AX5X43_ST_CRCB               BIT(5)
#define AX5X43_ST_ANT_RSSI           BIT(6)

#define AX5X43_FIFOCMD_NOP           0x00
#define AX5X43_FIFOCMD_COMMIT        0x04

#define AX5X43_PKTCHUNKSIZE_1        0x1
#define AX5X43_PKTCHUNKSIZE_2        0x2
#define AX5X43_PKTCHUNKSIZE_4        0x3
#define AX5X43_PKTCHUNKSIZE_8        0x4
#define AX5X43_PKTCHUNKSIZE_16       0x5
#define AX5X43_PKTCHUNKSIZE_32       0x6
#define AX5X43_PKTCHUNKSIZE_64       0x7
#define AX5X43_PKTCHUNKSIZE_96       0x8
#define AX5X43_PKTCHUNKSIZE_128      0x9
#define AX5X43_PKTCHUNKSIZE_160      0xA
#define AX5X43_PKTCHUNKSIZE_192      0xB
#define AX5X43_PKTCHUNKSIZE_224      0xC
#define AX5X43_PKTCHUNKSIZE_240      0xD

#endif
