/*
 * emif.c
 *
 *  Created on: 2022��7��7��
 *      Author: Lenovo
 */
#include "F28x_Project.h"
#include "emif.h"
#include "bsp_emif.h"

extern void setup_emif1_pinmux_async_16bit(Uint16 cpu_sel);

void bsp_emif_init(void)
{
	//
	//Configure to run EMIF1 on 1/2 Rate (EMIF1CLK = CPU1SYSCLK/2)
	//
	  EALLOW;
	  ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 0x1;
	  EDIS;

	  EALLOW;
	//


	// Grab EMIF1 For CPU1
	//
	  Emif1ConfigRegs.EMIF1MSEL.all = MSEL_EMIF1_CPU1; //0x93A5CE71;

	//
	//Disable Access Protection (CPU_FETCH/CPU_WR/DMA_WR)
	//
	  Emif1ConfigRegs.EMIF1ACCPROT0.all = 0x0;


	//
	// Commit the configuration related to protection. Till this bit remains set
	// content of EMIF1ACCPROT0 register can't be changed.
	//
	  Emif1ConfigRegs.EMIF1COMMIT.all = 0x1;


	//
	// Lock the configuration so that EMIF1COMMIT register can't be
	// changed any more.
	//
	  Emif1ConfigRegs.EMIF1LOCK.all = 0x1;

	  EDIS;

	//
	//Configure GPIO pins for EMIF1
	//
	  setup_emif1_pinmux_async_16bit(0);

	//
	//Configure the access timing for CS2 space
	//

	    Emif1Regs.ASYNC_CS3_CR.all =  (EMIF_ASYNC_ASIZE_16    | // 16Bit Memory
	                                                            // Interface
	                                   EMIF_ASYNC_TA_4        | // Turn Around time
	                                                            // of 2 Emif Clock
	                                   EMIF_ASYNC_RHOLD_4     | // Read Hold time
	                                                            // of 2 Emif Clock
	                                   EMIF_ASYNC_RSTROBE_12   | // Read Strobe time
	                                                            // of 4 Emif Clock
	                                   EMIF_ASYNC_RSETUP_4    | // Read Setup time
	                                                            // of 2 Emif Clock
	                                   EMIF_ASYNC_WHOLD_4     | // Write Hold time
	                                                            // of 2 Emif Clock
	                                   EMIF_ASYNC_WSTROBE_4   | // Write Strobe time
	                                                            // of 4 Emif Clock
	                                   EMIF_ASYNC_WSETUP_4    | // Write Setup time
	                                                            // of 2 Emif Clock
	                                   EMIF_ASYNC_EW_DISABLE  | // Extended Wait
	                                                            // Disable.
	                                   EMIF_ASYNC_SS_DISABLE    // Strobe Select Mode
	                                                            // Disable.
	                                  );

}


