/*--------------------------------------------------------------/
/  Startup Module for NXP LPC1100 Microcontrollers              /
/                                                               /
/ * This module defines vector table, startup code, default     /
/   exception handlers, main stack and miscellanous functions.  /
/ * This file is a non-copyrighted public domain software.      /
/--------------------------------------------------------------*/

#include "LPC1100.h"

/*-------------------------------------------------------------------*/
/* �X�^�b�N�E�T�C�Y�ƃN���b�N�̐ݒ�                                  */

#define STACK_SIZE 512  /* �X�^�b�N�E�T�C�Y[�o�C�g] (8�̔{���ł��邱��. 0��ݒ肷���RAM�̍Ō������g��) */

#define CLK_SEL 3           /* ���C���E�N���b�N�� = 0:IRC(12MHz), 1:PLL-in, 2:WDT or 3:PLL-out */
#define OSC_SEL 0           /* PLL���� = 0:IRC osc or 1:Main osc */
#define F_OSC   12000000    /* ���U����g��(IRC���g���ꍇ12M) */
#define PLL_M   4           /* PLL���{�� = 1..32 */
#define MCLK    48000000    /* ���C���E�N���b�N�̑z��l = F_OSC * (CLK_SEL == 3 ? PLL_M : 1) */
#define	SYSCLK	(MCLK / 1)	/* �V�X�e���E�N���b�N (MCLK��1/n) */

/*-------------------------------------------------------------------*/


#if MCLK != F_OSC * (CLK_SEL == 3 ? PLL_M : 1)
#error MCLK does not match calcurated value
#endif

#if CLK_SEL == 3
#if F_OSC < 10000000 || F_OSC > 25000000
#error F_OSC is out of range for PLL input
#endif
#if MCLK * 2 >= 156000000
#define P_SEL 0
#elif MCLK * 4 >= 156000000
#define P_SEL 1
#elif MCLK * 8 >= 156000000
#define P_SEL 2
#else
#define P_SEL 3
#endif
#endif

#if   SYSCLK <= 20000000
#define FLASH_WAIT 0
#elif SYSCLK <= 40000000
#define FLASH_WAIT 1
#else
#define FLASH_WAIT 2
#endif



/*--------------------------------------------------------------------/
/ �e��錾                                                            /
/--------------------------------------------------------------------*/

/* �O���V���{�� */
extern long _sidata[], _sdata[], _edata[], _sbss[], _ebss[], _endof_sram[]; /* �e�Z�N�V���� (�����J�E�X�N���v�g�Œ�`) */
extern int main (void);

/* ���荞�݃n���h���̐錾 */
void Reset_Handler (void)      __attribute__ ((noreturn, naked));
void NMI_Handler (void)        __attribute__ ((weak, alias ("Exception_Trap")));
void HardFault_Hander (void)   __attribute__ ((weak, alias ("Exception_Trap")));
void SVC_Handler (void)        __attribute__ ((weak, alias ("Exception_Trap")));
void PendSV_Handler (void)     __attribute__ ((weak, alias ("Exception_Trap")));
void SysTick_Handler (void)    __attribute__ ((weak, alias ("Exception_Trap")));
void PIO0_0_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_1_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_2_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_3_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_4_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_5_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_6_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_7_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_8_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_9_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_10_IRQHandler (void) __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO0_11_IRQHandler (void) __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO1_0_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void C_CAN_IRQHandler (void)   __attribute__ ((weak, alias ("IRQ_Trap")));
void SPI1_IRQHandler (void)    __attribute__ ((weak, alias ("IRQ_Trap")));
void I2C_IRQHandler (void)     __attribute__ ((weak, alias ("IRQ_Trap")));
void CT16B0_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void CT16B1_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void CT32B0_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void CT32B1_IRQHandler (void)  __attribute__ ((weak, alias ("IRQ_Trap")));
void SPI0_IRQHandler (void)    __attribute__ ((weak, alias ("IRQ_Trap")));
void UART_IRQHandler (void)    __attribute__ ((weak, alias ("IRQ_Trap")));
void ADC_IRQHandler (void)     __attribute__ ((weak, alias ("IRQ_Trap")));
void WDT_IRQHandler (void)     __attribute__ ((weak, alias ("IRQ_Trap")));
void BOD_IRQHandler (void)     __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO_3_IRQHandler (void)   __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO_2_IRQHandler (void)   __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO_1_IRQHandler (void)   __attribute__ ((weak, alias ("IRQ_Trap")));
void PIO_0_IRQHandler (void)   __attribute__ ((weak, alias ("IRQ_Trap")));



/*--------------------------------------------------------------------/
/ ���C���E�X�^�b�N (�X�^�b�N�̈�ɔz�u)                               /
/--------------------------------------------------------------------*/

#if STACK_SIZE > 0  /* �X�^�b�N�̈���m�ۂ���Ƃ� */
static
char mstk[STACK_SIZE] __attribute__ ((aligned(8), section(".STACK")));
#define INITIAL_MSP	&mstk[STACK_SIZE]

#else               /* �X�^�b�N�̈���m�ۂ��Ȃ��Ƃ� */
#define INITIAL_MSP	_endof_sram
#endif



/*--------------------------------------------------------------------/
/ �x�N�^�E�e�[�u�� (ROM�擪�ɔz�u)                                    /
/--------------------------------------------------------------------*/

void* const vector[] __attribute__ ((section(".VECTOR"))) =
{
    INITIAL_MSP, /* MSP�̏����l (8�o�C�g���E�ł��邱��) */
    Reset_Handler,
    NMI_Handler,
    HardFault_Hander,
    0,
    0,
    0,
    0, /* �`�F�b�N�E�T�� (�擪���炱���܂ł����Z����0�ɂȂ�悤�Ƀt���b�V���E�v���O���}���ݒ�) */
    0,
    0,
    0,
    SVC_Handler,
    0,
    0,
    PendSV_Handler,
    SysTick_Handler,

    PIO0_0_IRQHandler,
    PIO0_1_IRQHandler,
    PIO0_2_IRQHandler,
    PIO0_3_IRQHandler,
    PIO0_4_IRQHandler,
    PIO0_5_IRQHandler,
    PIO0_6_IRQHandler,
    PIO0_7_IRQHandler,
    PIO0_8_IRQHandler,
    PIO0_9_IRQHandler,
    PIO0_10_IRQHandler,
    PIO0_11_IRQHandler,
    PIO1_0_IRQHandler,
    C_CAN_IRQHandler,
    SPI1_IRQHandler,
    I2C_IRQHandler,
    CT16B0_IRQHandler,
    CT16B1_IRQHandler,
    CT32B0_IRQHandler,
    CT32B1_IRQHandler,
    SPI0_IRQHandler,
    UART_IRQHandler,
    0,
    0,
    ADC_IRQHandler,
    WDT_IRQHandler,
    BOD_IRQHandler,
    0,
    PIO_3_IRQHandler,
    PIO_2_IRQHandler,
    PIO_1_IRQHandler,
    PIO_0_IRQHandler
};



/*---------------------------------------------------------------------/
/ ���Z�b�g�E�n���h��                                                   /
/---------------------------------------------------------------------*/

void Reset_Handler (void)
{
    long *s, *d;

    BODCTRL = 0x13;   /* BOD�̐ݒ� (2.7V�Ń��Z�b�g) */

   /* �N���b�N����̏����� */

    MAINCLKSEL = 0;  /* ���C���E�N���b�N�Ƃ��Ĉꎞ�I��IRC��I�� */
    MAINCLKUEN = 0; MAINCLKUEN = 1;

    FLASHCFG = (FLASHCFG & 0xFFFFFFFC) | FLASH_WAIT;  /* �t���b�V���E�������̃E�F�C�g�� */

#if CLK_SEL == 1 || (CLK_SEL == 3 && OSC_SEL == 1)   /* �K�v�Ȃ甭�U��H���N�� */
    SYSOSCCTRL = (F_OSC >= 17500000) ? 2 : 0;
    PDRUNCFG &= ~0x20;
#endif
#if CLK_SEL == 2  /* �K�v�Ȃ�WDT���U����N�� */
    PDRUNCFG &= ~0x40;
#endif
#if CLK_SEL == 3  /* �K�v�Ȃ�PLL�������� */
    SYSPLLCLKSEL = OSC_SEL;
    SYSPLLCLKUEN = 0; SYSPLLCLKUEN = 1;
    SYSPLLCTRL = (PLL_M - 1) | (P_SEL << 6);
    PDRUNCFG &= ~0x80;
    while ((SYSPLLSTAT & 1) == 0) ; /* PLL�̃��b�N��҂� */
#endif

    SYSAHBCLKDIV = MCLK / SYSCLK; /* �V�X�e���E�N���b�N�������ݒ� */
    MAINCLKSEL = CLK_SEL;         /* �ړI�̃N���b�N�E�\�[�X��I�� */
    MAINCLKUEN = 0; MAINCLKUEN = 1;

    SYSAHBCLKCTRL = 0x1005F;  /* ���Ӄ��W���[���ւ̃N���b�N���� (SYS, ROM, RAM, FLASH, GPIO �� IOCON �̂�) */


    /* .data/.bss �Z�N�V�����̏�����(����ȍ~�ÓI�ϐ����g�p��) */
    for (s = _sidata, d = _sdata; d < _edata; *d++ = *s++) ;
    for (d = _sbss; d < _ebss; *d++ = 0) ;


    main();  /* main()�֐�����ڂ� */

    for (;;) ;
}



/*--------------------------------------------------------------------/
/ �Ӑ}���Ȃ���O�̃g���b�v                                            /
/--------------------------------------------------------------------*/

void Exception_Trap (void)
{
    for (;;) ;
}


void IRQ_Trap (void)
{
    for (;;) ;
}

