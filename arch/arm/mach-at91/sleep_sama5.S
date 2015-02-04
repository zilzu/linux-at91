/*
 * Copyright (C) 2014 Atmel,
 *
 * This program is free software,you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/linkage.h>
#include <mach/at91_pmc.h>
#include <mach/at91sam9_ddrsdr.h>

/*
 * void at91_cortexa5_do_wfi(void)
 *
 * This function is to disable the Processor Clock
 * and put the core to the WFI state.
 */
ENTRY(at91_cortexa5_do_wfi)
	stmfd	sp!, {lr}

	/*
	 * Execute an ISB instruction to ensure that all of the
	 * CP15 register changes have been committed.
	 */
	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * by any CPU in the cluster have completed.
	 */
	dsb
	dmb

	/* Disable the Processor Clock */
	bl	at91_get_pmc_base
	mov	r2, r0
	mov	r1, #AT91_PMC_PCK
	str	r1, [r2, #AT91_PMC_SCDR]

	/*
	 * Execute a WFI instruction and wait until the
	 * STANDBYWFI output is asserted to indicate that the
	 * CPU is in idle and low power state. CPU can specualatively
	 * prefetch the instructions so add NOPs after WFI. Sixteen
	 * NOPs as per Cortex-A5 pipeline.
	 */
	wfi	@ Wait For Interrupt
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	ldmfd	sp!, {pc}
ENDPROC(at91_cortexa5_do_wfi)

/*
 * unsigned int at91_ddrc_enter_self_refresh(void)
 *
 * The function is to put the DDR SDRAM to enter the self-refresh mode
 *  @return: DDRC low-power register setting.
 */
ENTRY(at91_ddrc_enter_self_refresh)
	stmfd	sp!, {r4 - r12, lr}

	bl	at91_get_ramc0_base
	mov	r2, r0
	ldr	r1, [r2, #AT91_DDRSDRC_LPR]
	mov	r0, r1
	bic	r1, #AT91_DDRSDRC_LPCB
	orr	r1, #AT91_DDRSDRC_LPCB_SELF_REFRESH
	str	r1, [r2, #AT91_DDRSDRC_LPR]

	ldmfd	sp!, {r4 - r12, pc}
ENDPROC(at91_ddrc_enter_self_refresh)

/*
 * void at91_ddrc_exit_self_refresh(unsigned int lpr_setting)
 *
 * The funtion is to make the DDR SDRAM exit the self-refresh mode.
 * @lpr_setting: DDRC low-power register setting.
 */

ENTRY(at91_ddrc_exit_self_refresh)
	stmfd	sp!, {r4 - r12, lr}

	mov	r1, r0
	bl	at91_get_ramc0_base
	mov	r2, r0
	str	r1, [r2, #AT91_DDRSDRC_LPR]

	ldmfd	sp!, {r4 - r12, pc}
ENDPROC(at91_ddrc_exit_self_refresh)

/*
 * void at91_cortexa5_standby(void)
 *
 * The function is to make the core perform suspend to standby mode.
 */
ENTRY(at91_cortexa5_standby)
	stmfd	sp!, {r4 - r12, lr}

	bl	at91_ddrc_enter_self_refresh
	push	{r0}

	bl	at91_cortexa5_do_wfi

	pop	{r0}
	bl	at91_ddrc_exit_self_refresh

	ldmfd	sp!, {r4 - r12, pc}
ENDPROC(at91_cortexa5_standby)
