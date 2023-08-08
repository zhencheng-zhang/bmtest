// testcase_efuse.c
#include <stdio.h>
#include <boot_test.h>
#include <system_common.h>
#include "efuse.h"
#include "efuse_info.h"
#include "string.h"

static void print_efuse()
{
	u32 val,i;

	uartlog("0 efuse_ecc_read:\n");
	for(i=0; i < 14; i++){
		val = efuse_ecc_read(i);
		uartlog(" %8x", val);
		if ((i + 1) % 4 == 0)
			uartlog("\n");
	}

	uartlog("\n 0 efuse_embedded_read:\n");
	for (i = 0; i < 16; i++) {
		val = efuse_embedded_read(i);
		uartlog(" %8x", val);
		if ((i + 1) % 4 == 0)
	 		 uartlog("\n");
	}

	efuse_embedded_write(0x6,0x12567f);

	uartlog("\n 1 efuse had been changed by embedwrite, efuse_ecc_read:\n");
	for(i=0; i < 14; i++){
		val = efuse_ecc_read(i);
		uartlog(" %8x", val);
		if ((i + 1) % 4 == 0)
			uartlog("\n");
	}

	uartlog("\n 1 efuse had been changed by embedwrite, efuse_embedded_read:\n");
	for (i = 0; i < 16; i++) {
		val = efuse_embedded_read(i);
		uartlog(" %8x", val);
		if ((i + 1) % 4 == 0)
			uartlog("\n");
	}

#if 1
	// bit 6 in soft_reset_reg
	writel(SOFT_RESET_REG0,readl(SOFT_RESET_REG1)&~(1<<SOFT_RESET_EFUSE0_BIT));
	mdelay(1);
	writel(SOFT_RESET_REG0,readl(SOFT_RESET_REG1)|(1<<SOFT_RESET_EFUSE0_BIT));
#endif

	efuse_embedded_write(0x5,0x12567f);
	uartlog("\n 2 efuse_embedded_read:\n");
	for (i = 0; i < 128; i++) {
		val = efuse_embedded_read(i);
		uartlog(" %8x", val);
		if ((i + 1) % 4 == 0)
	  		uartlog("\n");
	}
	uartlog("\n 2 efuse_ecc_read:\n");
	for(i=0; i < 14; i++){
		val = efuse_ecc_read(i);
		uartlog(" %8x", val);
		if ((i + 1) % 4 == 0)
	 		uartlog("\n");
	}
}

static void print_efuse_info()
{
	efuse_info_t e;
	efuse_info_read(&e);
	uartlog("efuse_info_read:\n"
			"  bad_npu0: %d\n"
			"  bad_npu1: %d\n"
			"  serdes_pcs: 0x%02x%02x%02x%02x%02x\n"
			"  signature: %s\n",
			e.bad_npus[0],
			e.bad_npus[1],
			e.serdes_pcs[4],
			e.serdes_pcs[3],
			e.serdes_pcs[2],
			e.serdes_pcs[1],
			e.serdes_pcs[0],
			e.signature);
}

int testcase_efuse(void)
{
	//efuse_embedded_write(4, (u32)0x80000000);
	//efuse_embedded_write(11, (u32)0x20505958);
	writel(SOFT_RESET_REG0,readl(SOFT_RESET_REG0)&~(1<<SOFT_RESET_EFUSE0_BIT));
	mdelay(1);
	writel(SOFT_RESET_REG0,readl(SOFT_RESET_REG0)|(1<<SOFT_RESET_EFUSE0_BIT));

	uartlog("--after soft_reset_reg0---\n");

	//efuse_info_t e;
	//efuse_info_init(&e);
	//e.bad_npus[0] = 10;
	//e.bad_npus[1] = 24;
	//memset(e.serdes_pcs, 0x00, sizeof(e.serdes_pcs));
	//strcpy(e.signature, "'s Chip");
	// addr
	efuse_embedded_write(0x4,0x12345677);
	efuse_embedded_write(0x5,0x1234567f);

	uartlog("--after efuse_embedded_write--\n");

	writel(CLK_EN_REG0,readl(CLK_EN_REG0)&~(1<<CLK_EN_EFUSE_BIT));
	mdelay(1);
	uartlog("cyx clk gating\n");
	writel(CLK_EN_REG0,readl(CLK_EN_REG0)|(1<<CLK_EN_EFUSE_BIT));

	uartlog("---In efuse case----\n");

	//efuse_info_write(&e);
	print_efuse();
	print_efuse_info();

	return 0;
}

#ifndef BUILD_TEST_CASE_ALL
int testcase_main()
{
  return testcase_efuse();
}
#endif




//efuse.h
/*
 * NOTE: Efuse is organized as efuse_num_cells() 32-bit
 *       words, named cells.
 */

u32 efuse_num_cells();

#define SOFT_RESET_REG0 	0x30013000
#define SOFT_RESET_REG1 	0x30013004
#define SOFT_RESET_EFUSE0_BIT   6
#define SOFT_RESET_EFUSE1_BIT   7

#define CLK_EN_REG0 		0x7030012000
#define CLK_EN_EFUSE_BIT    20
#define CLK_EN_APB_EFUSE_BIT    21



/*
 * address: [0, efuse_num_cells() - 1]
 */
u32 efuse_embedded_read(u32 address);
void efuse_embedded_write(u32 address, u32 val);
u32 efuse_ecc_read(u32 address);


//efuse.c
#include "system_common.h"
#include "efuse.h"

/*
 * Organization of EFUSE_ADR register:
 *   [  i:   0]: i <= 6, address of 32-bit cell
 *   [i+6: i+1]: 5-bit bit index within the cell
 */

#define NUM_ADDRESS_BITS 7

// 070:3000:0000 - efuse0
// 070:3000:1000 - efuse1
#define EFUSE_BASE 07030000000
// verified
static const u64 EFUSE_MODE = EFUSE_BASE;
static const u64 EFUSE_ADR = EFUSE_BASE + 0x4;
static const u64 EFUSE_RD_DATA = EFUSE_BASE + 0xc;
static const u64 EFUSE_ECCSRAM_ADR = EFUSE_BASE + 0x10;
static const u64 EFUSE_ECCSRAM_RDPORT = EFUSE_BASE + 0x14;


/**
 * EFUSE_MD
  2’b00:IDLE, switch to ECC mode when ECC_READ enable.
  2’b01:DIR
  2’b10:Embedded read
  2’b11:Embedded write
*/
static void efuse_mode_md_write(u32 val)
{
  u32 mode = cpu_read32(EFUSE_MODE);
  u32 new = (mode & 0xfffffffc) | (val & 0b11);
  cpu_write32(EFUSE_MODE, new);
}

static void efuse_mode_wait_ready()
{
  // bit[7]: efuse_rdy. after reset this bit is 0
  while (cpu_read32(EFUSE_MODE) == 0x80)
    ;
}

static void efuse_mode_reset()
{
  cpu_write32(EFUSE_MODE, 0);
  efuse_mode_wait_ready();
}

static u32 make_adr_val(u32 address, u32 bit_i)
{
  const u32 address_mask = (1 << NUM_ADDRESS_BITS) - 1;
  return (address & address_mask) |
      ((bit_i & 0x1f) << NUM_ADDRESS_BITS);
}

// static void efuse_set_bit(u32 address, u32 bit_i)
// {
//   efuse_mode_reset();
//   u32 adr_val = make_adr_val(address, bit_i);
//   cpu_write32(EFUSE_ADR, adr_val);
//   efuse_mode_md_write(0b11);
//   efuse_mode_wait_ready();
//   // while (cpu_read32(EFUSE_MODE) & 0x3);
// }

static void efuse_set_bit(uint32_t address, uint32_t bit_i)
{
    const uint32_t address_mask = (1 << NUM_ADDRESS_BITS) - 1;
    uint32_t adr_val;
    int loop = 100;

    if (address > ((1 << NUM_ADDRESS_BITS) - 1))
        return;

    // embedded write
    adr_val = (address & address_mask) | ((bit_i & 0x1f) << NUM_ADDRESS_BITS);                                                                                                                                                                    
    cpu_write32(EFUSE_MODE, adr_val);
    cpu_write32(EFUSE_MODE, cpu_read32(EFUSE_MODE) | 0x3);
    while (cpu_read32(EFUSE_MODE) & 0x3) {
        if (loop-- > 0)
            opdelay(1);
        else
            break;
    }
}

/**
 * Step 1 : Program EFUSE_ADR
  Step 2 : Program EFUSE_MODE[1:0] = 2’b10
  Step 3 : Wait EFUSE_MODE[1:0] return back to 2’b00.
  eFuse controller will read 32-bit data specified by EFUSE_ADR[6:0].
  As finished, return EFUSE_MODE[1:0] back to 2’b00.
  Step 4 : Get those 4 read out bytes at EFUSE_RD_DATA[31:0]
*/
u32 efuse_embedded_read(u32 address)
{
  efuse_mode_reset();
  u32 adr_val = make_adr_val(address, 0);
  cpu_write32(EFUSE_ADR, adr_val);
  efuse_mode_md_write(0b10);
  efuse_mode_wait_ready();
  // while (cpu_read32(EFUSE_MODE) & 0x3);
  uartlog("--embedded read success\n");
  
  return cpu_read32(EFUSE_RD_DATA);
}

void efuse_embedded_write(u32 address, u32 val)
{
  for (int i = 0; i < 32; i++)
    if ((val >> i) & 1)
      efuse_set_bit(address, i);
  // cpu_write32((u64)address, val);
  // efuse_mode_md_write(0b11);
  // efuse_mode_wait_ready();
  // // while (cpu_read32(EFUSE_MODE) & 0x3);
}


u32 efuse_ecc_read(u32 address)
{
   static int ecc_read_cnt = 0;
   efuse_mode_reset();
   u32 adr_val = make_adr_val(address, 0);


  /**
   * 31 RW  0  ECC_READ.The signal which is about switching to ECC Mode.
   * 30 RW  1  ECC_EN,Enable the ECC engine to decode read out data.
   * 28 WO  1  Mask ECC data.MASK=H for original data,MASK=L for repairing data.
  */
   //read out efuse and write into sram, once is ok
  //  if(0 == ecc_read_cnt) {
	// 	cpu_write32(EFUSE_MODE, (cpu_read32(EFUSE_MODE) | (0x3 << 30) | (0x1 << 28)));
	// 	while (cpu_read32(EFUSE_MODE) & 0x80000000);
	// 	ecc_read_cnt++;
  //  }

   /**Step 1 : Make sure EFUSE_MODE[1:0] = 2’b00
      Step 1 : Program EFUSE_MODE[31:30] = 2’b11
      Step 3 : Wait EFUSE_MODE[31] return back to 1’b0.
              eFuse controller will read out the eFuse content from Address_8 ~ Address_127.
              Decode them by Hamming code N(31.26) and write the decoded data into the
              ECCSRAM(30x26).
              Those eFuse content are divided into 30 blocks. The associated ECC decoded status
              Are stored in EFUSE_ECC_STATUS0/1 registers.
      Step 4 : Read out ECCSRAM data
              Program EFUSE_ECCSRAM_ADR. Range from 0 ~ 29
              Read the EFUSE_ECCSRAM_RDPORT to retrieve the ECCSRAM content. LSB 26 bits
              are valid for each read.
    */
  if(0 == ecc_read_cnt) {
    cpu_write32(EFUSE_MODE, (cpu_read32(EFUSE_MODE) | 0x3));
    uartlog("--- write efuse mode success--\n");
    while (cpu_read32(EFUSE_MODE) & 0x80000000);
    ecc_read_cnt++;
  }

  uartlog("---ecc read success--\n");

  cpu_write32(EFUSE_ECCSRAM_ADR, adr_val);
  return cpu_read32(EFUSE_ECCSRAM_RDPORT);
}


u32 efuse_num_cells()
{
  return (u32)1 << NUM_ADDRESS_BITS;
}



// testcase_watchdog.c
#include <stdio.h>
#include "boot_test.h"
#include "system_common.h"
#include "timer.h"
#include "testcase_watchdog.h"
#include "mmio.h"

#define SOFT_RESET_REG0 		0x30013000	//bit[10] is watchdog
#define SOFT_RESET_REG1 		0x30013004
#define CLK_EN_REG0 			0x7030012000
#define CLK_EN_REG1 			0x7030012004
#define TEST_WITH_TNTERRUPT  	1
#define WATCHDOG_INTR	 		99


#define REG_WDT				0x07030004000
#define WDT_CR				0x00
#define WDT_TORR			0x04
#define WDT_CCVR			0x08
#define WDT_CRR				0x0C
#define WDT_STAT			0x10
#define WDT_EOI				0x14
#define WDT_COMP_PARAM_5	0xE4

#define WDT_COMP_PARAM_4	0xE8
#define WDT_COMP_PARAM_3	0xEC
#define WDT_COMP_PARAM_2	0xF0
#define WDT_COMP_PARAM_1	0xF4
#define WDT_COMP_VERSION	0xF8
#define WDT_COMP_TYPE		0xFC

#define GPIO0DATA						0x50027000
#define GPIO0DIRECTION					0x50027004
#define PINMUXGPIO0						0x70300110f8

// Top Misc Control Registers in sg2042
#define REG_TOP_CONTROL					0x7030010000 + 0x008



static u32 us1,us2,us3;

#if TEST_WITH_TNTERRUPT
int watchdog_irq_handler(int irqn, void *priv)
{
    u32 int_status;
	int_status = readl(REG_WDT + WDT_STAT);

	/** Interrupt Clear Register:  
	 * Clears the watchdog interrupt. 
	 * This can be used to clear the interrupt without restarting the watchdog counter.
	 */
	/*clear int stop reboot; or dont clear let it reboot*/
	// mmio_read_32(REG_WDT + WDT_EOI);
	us3 = timer_meter_get_us();

	uartlog("irq us1=%d us2=%d us3=%d\n", us1,us2,us3);
	uartlog("%s  irqn=%d int_status=0x%x  \n",__func__, irqn, int_status);
	return 0;
}
#endif


int testcase_watchdog(void)
{
	uartlog("%s\n", __func__);
	u32 rpl;

	//reg_sw_root_reset_en
	mmio_write_32(REG_TOP_CONTROL,mmio_read_32(REG_TOP_CONTROL) | 0x04);

	//set pluse lenth 0b'100 �C 32 pclk cycles; at least 4pclk cycles
	rpl = 4;

	// Control Bit 10010: 8 pclk cycles
	//mode :0 reset system,1 first generate interrupt second timeout reset system
	//disable :0,  enable 1
	mmio_write_32(REG_WDT+WDT_CR, (rpl<<2) | 1<<1 | 0 );
	//mmio_write_32(REG_WDT+WDT_CR, (rpl<<2)  );
#ifdef	PLATFORM_PALLADIUM
	mmio_write_32(REG_WDT+WDT_TORR, 0x0 );
#else
	// Timeout Range Register
	//2^(16 + i)
	mmio_write_32(REG_WDT+WDT_TORR, 0x9 );
#endif

	uartlog("----TOP: %u   CR: %u    TORR:%u---\n", mmio_read_32(REG_TOP_CONTROL), 
													mmio_read_32(REG_WDT+WDT_CR),
													mmio_read_32(REG_WDT+WDT_TORR));

#if TEST_WITH_TNTERRUPT
	request_irq(WATCHDOG_INTR, watchdog_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_MASK, "timer int", NULL);
#endif

	//Pin Mux Selector for GPIO0
	cpu_write32(PINMUXGPIO0,cpu_read32(PINMUXGPIO0) & ~(0x3<<20));
	writel(GPIO0DIRECTION, 0xffffffff);
	writel(GPIO0DATA,0xffffffff);

	// enable watch dog
	timer_meter_start();
	uartlog("%s 0\n", __func__);

	// sg2042: bit[27]: clk_apb_wdt
	writel(CLK_EN_REG0, readl(CLK_EN_REG0)&~(1<<27));
	writel(CLK_EN_REG0, readl(CLK_EN_REG0)|(1<<27));

	// WDT enable
	mmio_write_32(REG_WDT+WDT_CR, (mmio_read_32(REG_WDT+WDT_CR)) | 0x1 );
	us1 = timer_meter_get_us();

#ifdef	PLATFORM_PALLADIUM
	//while(0)
#else
	while((timer_meter_get_us() -us1) < 5000)
#endif
	{
		mmio_write_32(REG_WDT+WDT_CRR,0x76);
	}
	us2 = timer_meter_get_us();

#if !TEST_WITH_TNTERRUPT
	writel(GPIO0DATA,0);
	//writel(CLK_EN_REG1, readl(CLK_EN_REG1)&~(1<<4));
	//mdelay(1);
	//writel(CLK_EN_REG1, readl(CLK_EN_REG1)|(1<<4));

	while (mmio_read_32(REG_WDT+WDT_STAT) == 0);

	us3 = timer_meter_get_us();
	writel(GPIO0DATA,0xffffffff);
 	uartlog(" us1=%d us2=%d us3=%d\n", us1,us2,us3);
#endif

  uartlog("need check whether cpu is reset!!!!\n");

  return 0;
}

#ifndef BUILD_TEST_CASE_ALL
int testcase_main()
{
  return testcase_watchdog();
}
#endif




// testcase_timer.c
#include <stdio.h>
#include "boot_test.h"
#include "system_common.h"
#include "timer.h"
#include "mmio.h"
#include "command.h"
#include "cli.h"

#define TEST_WITH_TNTERRUPT  	1
#define TIMER_INTR	 			100
#define SOFT_RESET_REG0 		0x30013000		//bit[9] all timer reset
#define SOFT_RESET_REG1 		0x30013004
#define CLK_EN_REG1 			0x7030012004


#define REG_TIMER_BASE				0x07030003000

#define REG_TIMER1_BASE				(REG_TIMER_BASE+0x00)
#define REG_TIMER2_BASE				(REG_TIMER_BASE+0x14)
#define REG_TIMER3_BASE				(REG_TIMER_BASE+0x28)
#define REG_TIMER4_BASE				(REG_TIMER_BASE+0x3C)
#define REG_TIMER5_BASE				(REG_TIMER_BASE+0x50)
#define REG_TIMER6_BASE				(REG_TIMER_BASE+0x64)
#define REG_TIMER7_BASE				(REG_TIMER_BASE+0x78)
#define REG_TIMER8_BASE				(REG_TIMER_BASE+0x8C)
#define REG_TIMERS_INTSTATUS		(REG_TIMER_BASE+0xA0)
#define REG_TIMERS_EOI				(REG_TIMER_BASE+0xA4)
#define REG_TIMERS_RAW_INTSTATUS	(REG_TIMER_BASE+0xA8)
#define REG_TIMERS_COMP_VERSION		(REG_TIMER_BASE+0xAC)
#define REG_TIMERN_LOADCOUNT2_BASE	(REG_TIMER_BASE+0xB0)

#define REG_TIMER1_LOADCONNT		REG_TIMER1_BASE
#define REG_TIMER2_LOADCONNT		REG_TIMER2_BASE
#define REG_TIMER3_LOADCONNT		REG_TIMER3_BASE
#define REG_TIMER4_LOADCONNT		REG_TIMER4_BASE
#define REG_TIMER5_LOADCONNT		REG_TIMER5_BASE
#define REG_TIMER6_LOADCONNT		REG_TIMER6_BASE
#define REG_TIMER7_LOADCONNT		REG_TIMER7_BASE
#define REG_TIMER8_LOADCONNT		REG_TIMER8_BASE

#define REG_TIMER1_LOADCONNT2		(REG_TIMERN_LOADCOUNT2_BASE+0x00)
#define REG_TIMER2_LOADCONNT2		(REG_TIMERN_LOADCOUNT2_BASE+0x04)
#define REG_TIMER3_LOADCONNT2		(REG_TIMERN_LOADCOUNT2_BASE+0x08)
#define REG_TIMER4_LOADCONNT2		(REG_TIMERN_LOADCOUNT2_BASE+0x0C)
#define REG_TIMER5_LOADCONNT2		(REG_TIMERN_LOADCOUNT2_BASE+0x10)
#define REG_TIMER6_LOADCONNT2		(REG_TIMERN_LOADCOUNT2_BASE+0x14)
#define REG_TIMER7_LOADCONNT2		(REG_TIMERN_LOADCOUNT2_BASE+0x18)
#define REG_TIMER8_LOADCONNT2		(REG_TIMERN_LOADCOUNT2_BASE+0x1C)

#define REG_TIMER1_CURRENT_VALUE		(REG_TIMER1_BASE+0x04)
#define REG_TIMER2_CURRENT_VALUE		(REG_TIMER2_BASE+0x04)
#define REG_TIMER3_CURRENT_VALUE		(REG_TIMER3_BASE+0x04)
#define REG_TIMER4_CURRENT_VALUE		(REG_TIMER4_BASE+0x04)
#define REG_TIMER5_CURRENT_VALUE		(REG_TIMER5_BASE+0x04)
#define REG_TIMER6_CURRENT_VALUE		(REG_TIMER6_BASE+0x04)
#define REG_TIMER7_CURRENT_VALUE		(REG_TIMER7_BASE+0x04)
#define REG_TIMER8_CURRENT_VALUE		(REG_TIMER8_BASE+0x04)


// bit[0]: Timer Enable
// bit[1]: Timer Mode
// bit[2]: Timer Interrupt Mask
// bit[3]: PWM
#define REG_TIMER1_CONTROL		(REG_TIMER1_BASE+0x08)
#define REG_TIMER2_CONTROL		(REG_TIMER2_BASE+0x08)
#define REG_TIMER3_CONTROL		(REG_TIMER3_BASE+0x08)
#define REG_TIMER4_CONTROL		(REG_TIMER4_BASE+0x08)
#define REG_TIMER5_CONTROL		(REG_TIMER5_BASE+0x08)
#define REG_TIMER6_CONTROL		(REG_TIMER6_BASE+0x08)
#define REG_TIMER7_CONTROL		(REG_TIMER7_BASE+0x08)
#define REG_TIMER8_CONTROL		(REG_TIMER8_BASE+0x08)


#define REG_TIMER1_EOI		(REG_TIMER1_BASE+0x0C)
#define REG_TIMER2_EOI		(REG_TIMER2_BASE+0x0C)
#define REG_TIMER3_EOI		(REG_TIMER3_BASE+0x0C)
#define REG_TIMER4_EOI		(REG_TIMER4_BASE+0x0C)
#define REG_TIMER5_EOI		(REG_TIMER5_BASE+0x0C)
#define REG_TIMER6_EOI		(REG_TIMER6_BASE+0x0C)
#define REG_TIMER7_EOI		(REG_TIMER7_BASE+0x0C)
#define REG_TIMER8_EOI		(REG_TIMER8_BASE+0x0C)

#define REG_TIMER1_INTSTATUS		(REG_TIMER1_BASE+0x10)
#define REG_TIMER2_INTSTATUS		(REG_TIMER2_BASE+0x10)
#define REG_TIMER3_INTSTATUS		(REG_TIMER3_BASE+0x10)
#define REG_TIMER4_INTSTATUS		(REG_TIMER4_BASE+0x10)
#define REG_TIMER5_INTSTATUS		(REG_TIMER5_BASE+0x10)
#define REG_TIMER6_INTSTATUS		(REG_TIMER6_BASE+0x10)
#define REG_TIMER7_INTSTATUS		(REG_TIMER7_BASE+0x10)
#define REG_TIMER8_INTSTATUS		(REG_TIMER8_BASE+0x10)

#define TIMER_CLK	(50 * 1000 * 1000)


#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

#define PINMUXGPIO0						0x70300110f8
#define GPIO0DATA						0x50027000
#define GPIO0DIRECTION					0x50027004

static int us1, us2;
//static int need_loop;

 __attribute__ ((noinline))  void sleep(int count)
{
	int i = 0;

	for (i=0; i < count; i++) {
		;/*loop*/
	}
}

void BM1684_timer_delay(int delay, int num)
{
	// disable & user-defined count mode  ~0x01)|0x2
	// disable & free-running count mode  ~0x03)|0
	mmio_write_32((REG_TIMER1_CONTROL + 0x14*num),(mmio_read_32(REG_TIMER1_CONTROL + 0x14*num) & ~0x03)|0);//0x01)|0x2);	 //0x03)|0);
	// set count
	mmio_write_32((REG_TIMER1_LOADCONNT + 0x14*num), delay);

	us1 = timer_meter_get_us();
	//need_loop = 1;
	// enable
	mmio_write_32((REG_TIMER1_CONTROL + 0x14*num),mmio_read_32(REG_TIMER1_CONTROL + 0x14*num)  | 0x01);

	//writel(CLK_EN_REG0, readl(CLK_EN_REG0)&~(1<<20));
	//mdelay(1);
	//writel(CLK_EN_REG0, readl(CLK_EN_REG0)|(1<<20));


#if !TEST_WITH_TNTERRUPT
	u32 curr_cnt1, curr_cnt2=0;
	while (1) {
		while ((mmio_read_32(REG_TIMER1_INTSTATUS + 0x14*num) == 0) &&
			(readl(REG_TIMERS_INTSTATUS) == 0) &&
			(readl(REG_TIMERS_RAW_INTSTATUS) == 0) );

	    us2 = timer_meter_get_us();
		uartlog("loop get int,us1=%d us2=%d,intn=0x%x, 0x%x 0x%x\n",us1, us2, readl(REG_TIMER1_INTSTATUS),
				readl(REG_TIMERS_INTSTATUS),readl(REG_TIMERS_RAW_INTSTATUS));
		mmio_read_32(REG_TIMER1_EOI + 0x14*num);
	}
#endif

}


#if TEST_WITH_TNTERRUPT
int timer_irq_handler(int irqn, void *priv)
{
    u32 int_status;
	int_status = readl(REG_TIMERS_RAW_INTSTATUS);

	uartlog("---In IRQ---\n");
	/*clear int*/
	mmio_read_32(REG_TIMERS_EOI);
	us2 = timer_meter_get_us();
	uartlog(" us1=%d us2=%d 0x%x 0x%x \n", us1, us2, readl(REG_TIMER1_INTSTATUS), readl(REG_TIMERS_INTSTATUS));
	uartlog("%s  irqn=0x%x int_status=0x%x  0x%x num=%d \n",__func__, irqn, int_status,readl(REG_TIMERS_RAW_INTSTATUS), *(u32 *)priv);
	return 0;
}
#endif

static int test_timer(int argc, char **argv)
{
	uartlog("%s\n", __func__);
	u32 num;
	num = strtoul(argv[1], NULL, 10);
	if(num > 7) {
		uartlog("test timer, invalid args, num: 0~7\n");
		return 0;
	}

	writel(SOFT_RESET_REG0, readl(SOFT_RESET_REG0)&~(1<<9));
	sleep(1000);
	writel(SOFT_RESET_REG0, readl(SOFT_RESET_REG0)|(1<<9));

#if TEST_WITH_TNTERRUPT
	request_irq(TIMER_INTR, timer_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_MASK, "timer int", (void *)&num);
#endif

	u32 count;
	cpu_write32(PINMUXGPIO0,cpu_read32(PINMUXGPIO0) & ~(0x3<<20));
	writel(GPIO0DIRECTION, 0xffffffff);

	uartlog("FPGA check GPIO0 in J7 C4 11\n");
	count =1 * TIMER_CLK/1;  //fpga 50Mclk  5* 1��
#ifdef	PLATFORM_PALLADIUM
	count = 1000 * 50;       //pld 50M clk�� 1000΢��
#endif

	uartlog("count %d\n", count);
	writel(GPIO0DATA,0xffffffff);
	BM1684_timer_delay(count, num);
	writel(GPIO0DATA,0x0);

  return 0;
}


static struct cmd_entry test_cmd_list[] __attribute__ ((unused)) = {
	{"timer", test_timer, 1, "timer [num]"},
	{NULL, NULL, 0, NULL}
};

int  testcase_timer (void) {
	int i = 0;
	for(i = 0;i < ARRAY_SIZE(test_cmd_list) - 1;i++) {
		command_add(&test_cmd_list[i]);
	}
	cli_simple_loop();

	return 0;
}

#ifndef BUILD_TEST_CASE_ALL
int testcase_main()
{
  return testcase_timer();
}
#endif



// testcase_spi.c
#include <stdio.h>
#include <string.h>
#include <boot_test.h>
#include <system_common.h>
#include "reg_spi.h"
#include "smem_utils.h"
#include "spi_flash.h"
#include "bm_spi_nand.h"

// sg2042
#define SOFT_RESET_REG0 	0x30013000
#define SOFT_RESET_REG1 	0x30013004     //bit[1]: spi0   bit[2]: spi1

#define CLK_EN_REG0 		0x7030012000
#define CLK_EN_REG1 		0x7030012004

#define SOFT_RESET_SPI0_BIT 1
#define SOFT_RESET_SPI1_BIT 2

#define CLK_EN_SPI_BIT      25


int spi_flash_spic_fifo_rw_test(u64 spi_base)
{
  u32 data_dw = 0x76543210;
  u16 data_w = 0xabcd;
  u8 data_b = 0xef;
  uartlog("\n--spi ctrl fifo rw test\n");

  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);    //do flush FIFO before test

  writel(spi_base + REG_BM1680_SPI_FIFO_PORT, data_dw);
  uartlog("data_dw filled 0x%x, fifo pt: 0x%08x\n", data_dw, readl(spi_base + REG_BM1680_SPI_FIFO_PT));

  writew(spi_base + REG_BM1680_SPI_FIFO_PORT, data_w);
  uartlog("data_w filled 0x%x, fifo pt: 0x%08x\n", data_w, readl(spi_base + REG_BM1680_SPI_FIFO_PT));

  writeb(spi_base + REG_BM1680_SPI_FIFO_PORT, data_b);
  uartlog("data_b filled 0x%x, fifo pt: 0x%08x\n", data_b, readl(spi_base + REG_BM1680_SPI_FIFO_PT));
  data_b = 0x89;
  writeb(spi_base + REG_BM1680_SPI_FIFO_PORT, data_b);
  uartlog("data_b filled 0x%x, fifo pt: 0x%08x\n", data_b, readl(spi_base + REG_BM1680_SPI_FIFO_PT));

  data_b = readb(spi_base + REG_BM1680_SPI_FIFO_PORT);
  uartlog("data_b read 0x%x, fifo pt: 0x%08x\n", data_b, readl(spi_base + REG_BM1680_SPI_FIFO_PT));
  data_b = readb(spi_base + REG_BM1680_SPI_FIFO_PORT);
  uartlog("data_b read 0x%x, fifo pt: 0x%08x\n", data_b, readl(spi_base + REG_BM1680_SPI_FIFO_PT));

  data_w = readw(spi_base + REG_BM1680_SPI_FIFO_PORT);
  uartlog("data_w read 0x%x, fifo pt: 0x%08x\n", data_w, readl(spi_base + REG_BM1680_SPI_FIFO_PT));

  data_dw = readl(spi_base + REG_BM1680_SPI_FIFO_PORT);
  uartlog("data_dw read 0x%x, fifo pt: 0x%08x\n", data_dw, readl(spi_base + REG_BM1680_SPI_FIFO_PT));

  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);    //do flush FIFO after test

  uartlog("%s done!\n", __func__);

  return 0;
}

int spi_flash_rw_test(u64 spi_base)
{
  u32 sector_num = 1;
  u32 sector_addr = 0x0;
  do_sector_erase(spi_base, sector_addr, sector_num);

  u8 wdata[SPI_PAGE_SIZE];
  for (int i = 0; i < SPI_PAGE_SIZE; i++) {
    wdata[i] = 0x5a;//rand();
  }

  u32 count = 16;//(SPI_SECTOR_SIZE * sector_num) / SPI_PAGE_SIZE;

  for (int i = 0; i < count; i++) {
    u32 off = i * SPI_PAGE_SIZE;
    spi_flash_write_by_page(spi_base, sector_addr + off, wdata, SPI_PAGE_SIZE);

#if 1
	writel(CLK_EN_REG1,readl(CLK_EN_REG1)&~(0x4));
	//mdelay(1);
	uartlog(" clk gating\n");
	writel(CLK_EN_REG1,readl(CLK_EN_REG1)|0x4);

	//writel(SOFT_RESET_REG1,readl(SOFT_RESET_REG1)&~(1<<5));
	//mdelay(1);
	//writel(SOFT_RESET_REG1,readl(SOFT_RESET_REG1)|(1<<5));
	uartlog(" val=0x%x 0x%x 0x%x\n",readl(spi_base),readl(spi_base+0x4),readl(spi_base+0x14));
#endif

    u8 rdata[SPI_PAGE_SIZE];
    memset(rdata, 0x0, SPI_PAGE_SIZE);
    spi_flash_read_by_page(spi_base, sector_addr + off, rdata, SPI_PAGE_SIZE);
    int ret = memcmp(rdata, wdata, SPI_PAGE_SIZE);

    if (ret != 0) {
      uartlog("page program test memcmp failed: 0x%08x\n", ret);
      dump_hex((char *)"wdata", (void *)wdata, SPI_PAGE_SIZE);
      dump_hex((char *)"rdata", (void *)rdata, SPI_PAGE_SIZE);
      return ret;
    }
	uartlog("page count %d rd wr cmp ok\n",i);
  }
  uartlog("%s done!\n", __func__);

  return 0;
}

int spi_flash_full_chip_scan(u64 spi_base)
{
  u32 off = 0;
  u32 xfer_size = 0;
  u32 size = SPI_MAX_SIZE;
  u8 *wdata = (u8 *)DO_SPI_FW_PROG_BUF_ADDR;
  u32 sector_addr = 0x0;

  do_sector_erase(spi_base, sector_addr, 16*256);

  while (off < size) {
    if ((size - off) >= SPI_PAGE_SIZE)
      xfer_size = SPI_PAGE_SIZE;
    else
      xfer_size = size - off;
    spi_flash_write_by_page(spi_base, sector_addr+off, wdata + off, xfer_size);

    u8 rdata[SPI_PAGE_SIZE];
    memset(rdata, 0x0, SPI_PAGE_SIZE);
    spi_flash_read_by_page(spi_base, sector_addr+off, rdata, xfer_size);
    int ret = memcmp(rdata, wdata + off, SPI_PAGE_SIZE);
    if (ret != 0) {
      uartlog("page program test memcmp failed: 0x%08x\n", ret);
      dump_hex((char *)"wdata", (void *)wdata, xfer_size);
      dump_hex((char *)"rdata", (void *)rdata, xfer_size);
      return ret;
    }

    uartlog("page read compare ok @%d\n", off);
    off += xfer_size;
  }

  uartlog("%s done!\n", __func__);
  return 0;
}

int spi_flash_basic_test(void)
{
  u64 spi_base = spi_flash_map_enable(0);

  uartlog(" before spi flash init\n");
  spi_flash_init(spi_base);

  int ret = 0;
  ret |= spi_flash_id_check(spi_base);

  spi_flash_spic_fifo_rw_test(spi_base);

  ret |= spi_flash_rw_test(spi_base);
 // ret |= spi_flash_full_chip_scan(spi_base);

  writel(SOFT_RESET_REG1,readl(SOFT_RESET_REG1)&~(1<<5));
  mdelay(1);
  writel(SOFT_RESET_REG1,readl(SOFT_RESET_REG1)|(1<<5));

  spi_flash_set_dmmr_mode(spi_base, 1);

  uartlog("%s done!\n", __func__);

  return ret;
}

int spi_flash_program_test(void)
{
  uartlog("%s\n", __func__);
  int err = 0;
  err = spi_flash_program_pdl();

  return err;
}

int testcase_spi(void)
{
  int err = 0;

	writel(CLK_EN_REG0,readl(CLK_EN_REG0)&~(1 << CLK_EN_SPI_BIT));
	mdelay(1);
	writel(CLK_EN_REG0,readl(CLK_EN_REG0)|(1 << CLK_EN_SPI_BIT));

	writel(SOFT_RESET_REG1,readl(SOFT_RESET_REG1)&~(1<<SOFT_RESET_SPI0_BIT));
	mdelay(1);
	writel(SOFT_RESET_REG1,readl(SOFT_RESET_REG1)|(1<<SOFT_RESET_SPI0_BIT));



  //err |= spi_flash_program_test();
 // uartlog("spi_flash_program_test   %s\n",err ? "failed!":"passed!");
  err |= spi_flash_basic_test();
  uartlog("spi_flash_basic_test   %s\n", err ? "failed!":"passed!");

  return err;
}

#ifndef BUILD_TEST_CASE_ALL
int testcase_main()
{
  return testcase_spi();
}
#endif



//spi.h
#ifndef _SPI_FLASH_
#define _SPI_FLASH_

#include "system_common.h"

#define SPI0_BASE   07040004000
#define SPI1_BASE   07040005000

#define SPI_BASE    SPI0_BASE

#define DO_SPI_FW_PROG
#define DO_SPI_FW_PROG_BUF_ADDR     0x10100000 //0x45000000
#define DO_SPI_FW_PROG_BUF_SIZE     0x400 		//0x10000

#define PLATFORM_ASIC

#ifdef PLATFORM_ASIC
#define SPI_PAGE_SIZE           256
#define SPI_SECTOR_SIZE         (SPI_PAGE_SIZE * 16)    //4KB
#define SPI_MAX_SIZE            (SPI_BLOCK1_SIZE * 256) //16MB
#elif defined(PLATFORM_FPGA) || defined(PLATFORM_FPGA_MUL)
#define SPI_PAGE_SIZE           256
#define SPI_SECTOR_SIZE         (SPI_PAGE_SIZE * 1024)
#define SPI_MAX_SIZE            (SPI_SECTOR_SIZE * 64)
#elif defined(PLATFORM_PALLADIUM)
#define SPI_PAGE_SIZE           256
#define SPI_SECTOR_SIZE         (SPI_PAGE_SIZE * 1024)
#define SPI_MAX_SIZE            (SPI_SECTOR_SIZE * 64)
#else
#error "Undefined PLATFORM"
#endif

#define SPI_BLOCK0_SIZE         (SPI_SECTOR_SIZE * 8)   //32KB
#define SPI_BLOCK1_SIZE         (SPI_SECTOR_SIZE * 16)  //64KB
#define SPI_FLASH_SERIAL_NUMBER_START_ADDR  (SPI_MAX_SIZE - SPI_SECTOR_SIZE)
#define SPI_FLASH_CONV_RESULT_START_ADDR    (SPI_MAX_SIZE - SPI_BLOCK1_SIZE * 4 - SPI_SECTOR_SIZE)

/*spi flash model*/
#define SPI_ID_M25P128          0x00182020
#define SPI_ID_N25Q128          0x0018ba20
#define SPI_ID_W25Q128FV        0x001840ef

/* cmd for M25P128 on FPGA */
#define SPI_CMD_WREN            0x06
#define SPI_CMD_WRDI            0x04
#define SPI_CMD_RDID            0x9F

#define SPI_CMD_RDSR            0x05
#define SPI_CMD_RDSR2           0x35
#define SPI_CMD_RDSR3           0x15

#define SPI_CMD_WRSR            0x01
#define SPI_CMD_WRSR2           0x31
#define SPI_CMD_WRSR3           0x11

#define SPI_CMD_READ            0x03
#define SPI_CMD_FAST_READ       0x0B
#define SPI_CMD_PP              0x02
#define SPI_CMD_SE              0xD8
#define SPI_CMD_BE              0xC7


#define SPI_STATUS_WIP          (0x01 << 0)
#define SPI_STATUS_WEL          (0x01 << 1)
#define SPI_STATUS_BP0          (0x01 << 2)
#define SPI_STATUS_BP1          (0x01 << 3)
#define SPI_STATUS_BP2          (0x01 << 4)
#define SPI_STATUS_SRWD         (0x01 << 7)

u8 spi_reg_status(u64 spi_base, u8 cmd);
u64 spi_flash_map_enable(u8 enable);

void spi_flash_init(u64 spi_base);
void spi_flash_set_dmmr_mode(u64 spi_base, u32 en);
int spi_flash_id_check(u64 spi_base);
int do_sector_erase(u64 spi_base, u32 addr, u32 sector_num);
int do_full_chip_erase(u64 spi_base);
//int spi_flash_program(u64 spi_base, u32 sector_addr);

int spi_flash_write_by_page(u64 spi_base, u32 fa, u8 *data, u32 size);
void spi_flash_read_by_page(u64 spi_base, u32 fa, u8 *data, u32 size);


int spi_flash_program(u64 spi_base, u32 sector_addr, u32 len);
int do_page_prog(u64 spi_base, u8 *src_buf, u32 addr, u32 size);
u8 spi_data_read(u64 spi_base, u8* dst_buf, u32 addr, u32 size);
#endif




// spi_flash.c
#include <stdio.h>
#include <string.h>
// #include "plat_def.h"
#include "reg_spi.h"
#include "spi_flash.h"

//#define writel(a, v)		writel((u64)a, v)
//#define readl(a)		readl((u64)a)

static inline u32 _check_reg_bits(volatile u32 *addr, u64 offset, u32 mask, u32 wait_loop)
{
  u32 try_loop = 0;
  u32 reg_val;
  while (1){
    reg_val = *((volatile u32 *)((u8 *)addr + offset));
    if ((reg_val & mask) != 0) {
      *((volatile u32 *)((u8 *)addr + offset)) = reg_val & mask;
      return reg_val & mask;
    }
    if (wait_loop != 0) {
      try_loop++;    //wait_loop == 0 means wait for ever
      if (try_loop >= wait_loop) {
        break;      //timeout break
      }
    }
  }
  return 0;
}

u32 spi_flash_model_support_list[] = { \
        SPI_ID_M25P128,  \
        SPI_ID_N25Q128,  \
        SPI_ID_W25Q128FV,\
};

u8 spi_non_data_tran(u64 spi_base, u8* cmd_buf, u32 with_cmd, u32 addr_bytes)
{
  u32* p_data = (u32*)cmd_buf;

  if (addr_bytes > 3) {
    uartlog("non-data: addr bytes should be less than 3 (%d)\n", addr_bytes);
    return -1;
  }

  /* init tran_csr */
  u32 tran_csr = 0;
  tran_csr = readl(spi_base + REG_BM1680_SPI_TRAN_CSR);
  tran_csr &= ~(BM1680_SPI_TRAN_CSR_TRAN_MODE_MASK
                  | BM1680_SPI_TRAN_CSR_ADDR_BYTES_MASK
                  | BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_MASK
                  | BM1680_SPI_TRAN_CSR_WITH_CMD);
  tran_csr |= (addr_bytes << BM1680_SPI_TRAN_CSR_ADDR_BYTES_SHIFT);
  tran_csr |= BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_1_BYTE;
  tran_csr |= (with_cmd) ? BM1680_SPI_TRAN_CSR_WITH_CMD : 0;

  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);    //do flush FIFO before filling fifo

  writel(spi_base + REG_BM1680_SPI_FIFO_PORT, p_data[0]);

  /* issue tran */
  writel(spi_base + REG_BM1680_SPI_INT_STS, 0);   //clear all int
  tran_csr |= BM1680_SPI_TRAN_CSR_GO_BUSY;
  writel(spi_base + REG_BM1680_SPI_TRAN_CSR, tran_csr);

  /* wait tran done */
  u32 int_stat = _check_reg_bits((volatile u32*)spi_base, REG_BM1680_SPI_INT_STS,
                      BM1680_SPI_INT_TRAN_DONE, 100000);
  if (int_stat == 0) {
    uartlog("non data timeout, int stat: 0x%08x\n", int_stat);
    return -1;
  }
  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);    //should flush FIFO after tran

  return 0;
}

u8 spi_data_out_tran(u64 spi_base, u8* src_buf, u8* cmd_buf, u32 with_cmd, u32 addr_bytes, u32 data_bytes)
{
  u32* p_data = (u32*)cmd_buf;
  u32 cmd_bytes = addr_bytes + ((with_cmd) ? 1 : 0);

  if (data_bytes > 65535) {
    uartlog("data out overflow, should be less than 65535 bytes(%d)\n", data_bytes);
    return -1;
  }

  /* init tran_csr */
  u32 tran_csr = 0;
  tran_csr = readl(spi_base + REG_BM1680_SPI_TRAN_CSR);
  tran_csr &= ~(BM1680_SPI_TRAN_CSR_TRAN_MODE_MASK
                  | BM1680_SPI_TRAN_CSR_ADDR_BYTES_MASK
                  | BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_MASK
                  | BM1680_SPI_TRAN_CSR_WITH_CMD);
  tran_csr |= (addr_bytes << BM1680_SPI_TRAN_CSR_ADDR_BYTES_SHIFT);
  tran_csr |= (with_cmd) ? BM1680_SPI_TRAN_CSR_WITH_CMD : 0;
  tran_csr |= BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_8_BYTE;
  tran_csr |= BM1680_SPI_TRAN_CSR_TRAN_MODE_TX;

  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);    //do flush FIFO before filling fifo
  if (with_cmd) {
    for (int i = 0; i < ((cmd_bytes - 1) / 4 + 1); i++) {
      writel(spi_base + REG_BM1680_SPI_FIFO_PORT, p_data[i]);
    }
  }

  /* issue tran */
  writel(spi_base + REG_BM1680_SPI_INT_STS, 0);   //clear all int
  writel(spi_base + REG_BM1680_SPI_TRAN_NUM, data_bytes);
  tran_csr |= BM1680_SPI_TRAN_CSR_GO_BUSY;
  writel(spi_base + REG_BM1680_SPI_TRAN_CSR, tran_csr);
  while ((readl(spi_base + REG_BM1680_SPI_FIFO_PT) & 0xf) != 0) {};   //wait for cmd issued

  /* fill data */
  p_data = (u32*)src_buf;
  u32 off = 0;
  u32 xfer_size = 0;
  while (off < data_bytes) {
    if ((data_bytes - off) >= BM1680_SPI_MAX_FIFO_DEPTH) {
      xfer_size = BM1680_SPI_MAX_FIFO_DEPTH;
    } else {
      xfer_size = data_bytes - off;
    }

    int wait = 0;
    while ((readl(spi_base + REG_BM1680_SPI_FIFO_PT) & 0xf) != 0) {
      wait++;
      if (wait > 10000000) {
        uartlog("wait to write FIFO timeout\n");
        return -1;
      }
    }

    for (int i = 0; i < ((xfer_size - 1) / 4 + 1); i++) {
      writel(spi_base + REG_BM1680_SPI_FIFO_PORT, p_data[off / 4 + i]);
    }
    off += xfer_size;
  }

  /* wait tran done */
  u32 int_stat = _check_reg_bits((volatile u32*)spi_base, REG_BM1680_SPI_INT_STS,
                      BM1680_SPI_INT_TRAN_DONE, 100000);
  if (int_stat == 0) {
    uartlog("data out timeout, int stat: 0x%08x\n", int_stat);
    return -1;
  }
  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);  //should flush FIFO after tran
  return 0;
}

u8 spi_data_in_tran(u64 spi_base, u8* dst_buf, u8* cmd_buf, u32 with_cmd, u32 addr_bytes, u32 data_bytes)
{
  u32* p_data = (u32*)cmd_buf;
  u32 cmd_bytes = addr_bytes + ((with_cmd) ? 1 : 0);

  if (data_bytes > 65535) {
    uartlog("data in overflow, should be less than 65535 bytes(%d)\n", data_bytes);
    return -1;
  }

  /* init tran_csr */
  u32 tran_csr = 0;
  tran_csr = readl(spi_base + REG_BM1680_SPI_TRAN_CSR);
  tran_csr &= ~(BM1680_SPI_TRAN_CSR_TRAN_MODE_MASK
                  | BM1680_SPI_TRAN_CSR_ADDR_BYTES_MASK
                  | BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_MASK
                  | BM1680_SPI_TRAN_CSR_WITH_CMD);
  tran_csr |= (addr_bytes << BM1680_SPI_TRAN_CSR_ADDR_BYTES_SHIFT);
  tran_csr |= (with_cmd) ? BM1680_SPI_TRAN_CSR_WITH_CMD : 0;
  tran_csr |= BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_8_BYTE;
  tran_csr |= BM1680_SPI_TRAN_CSR_TRAN_MODE_RX;

  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);    //do flush FIFO before filling fifo
  if (with_cmd) {
    for (int i = 0; i < ((cmd_bytes - 1) / 4 + 1); i++) {
      writel(spi_base + REG_BM1680_SPI_FIFO_PORT, p_data[i]);
    }
  }

  /* issue tran */
  writel(spi_base + REG_BM1680_SPI_INT_STS, 0);   //clear all int
  writel(spi_base + REG_BM1680_SPI_TRAN_NUM, data_bytes);
  tran_csr |= BM1680_SPI_TRAN_CSR_GO_BUSY;
  writel(spi_base + REG_BM1680_SPI_TRAN_CSR, tran_csr);

  /* check rd int to make sure data out done and in data started */
  u32 int_stat = _check_reg_bits((volatile u32*)spi_base, REG_BM1680_SPI_INT_STS,
                      BM1680_SPI_INT_RD_FIFO, 10000000);
  if (int_stat == 0) {
    uartlog("no read FIFO int\n");
    return -1;
  }

  /* get data */
  p_data = (u32*)dst_buf;
  u32 off = 0;
  u32 xfer_size = 0;
  while (off < data_bytes) {
    if ((data_bytes - off) >= BM1680_SPI_MAX_FIFO_DEPTH) {
      xfer_size = BM1680_SPI_MAX_FIFO_DEPTH;
    } else {
      xfer_size = data_bytes - off;
    }

    int wait = 0;
    while (readl(spi_base + REG_BM1680_SPI_FIFO_PT) != xfer_size) {
      wait++;
      if (wait > 10000000) {
        uartlog("wait to read FIFO timeout\n");
        return -1;
      }
    }
    for (int i = 0; i < ((xfer_size - 1) / 4 + 1); i++) {
      p_data[off / 4 + i] = readl(spi_base + REG_BM1680_SPI_FIFO_PORT);
    }
    off += xfer_size;
  }

  /* wait tran done */
  int_stat = _check_reg_bits((volatile u32*)spi_base, REG_BM1680_SPI_INT_STS,
                      BM1680_SPI_INT_TRAN_DONE, 100000);
  if (int_stat == 0) {
    uartlog("data in timeout, int stat: 0x%08x\n", int_stat);
    return -1;
  }
  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);  //should flush FIFO after tran
  return 0;
}

/*
 * spi_in_out_tran is a workaround fucntion for current 32-bit access to spic fifo:
 * AHB bus could only do 32-bit access to spic fifo, so cmd without 3-bytes addr will leave 3-byte
 * data in fifo, so set tx to mark that these 3-bytes data would be sent out.
 * So send_bytes should be 3 (wirte 1 dw into fifo) or 7(write 2 dw), get_bytes sould be the same value.
 * software would mask out unuseful data in get_bytes.
 */
u8 spi_in_out_tran(u64 spi_base, u8* dst_buf, u8* src_buf,  u32 with_cmd, u32 addr_bytes, u32 send_bytes, u32 get_bytes)
{
  u32* p_data = (u32*)src_buf;

  if (send_bytes != get_bytes) {
    uartlog("data in&out: get_bytes should be the same as send_bytes\n");
    return -1;
  }

  if ((send_bytes > BM1680_SPI_MAX_FIFO_DEPTH) || (get_bytes > BM1680_SPI_MAX_FIFO_DEPTH)) {
    uartlog("data in&out: FIFO will overflow\n");
    return -1;
  }

  /* init tran_csr */
  u32 tran_csr = 0;
  tran_csr = readl(spi_base + REG_BM1680_SPI_TRAN_CSR);
  tran_csr &= ~(BM1680_SPI_TRAN_CSR_TRAN_MODE_MASK
                  | BM1680_SPI_TRAN_CSR_ADDR_BYTES_MASK
                  | BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_MASK
                  | BM1680_SPI_TRAN_CSR_WITH_CMD);
  tran_csr |= (addr_bytes << BM1680_SPI_TRAN_CSR_ADDR_BYTES_SHIFT);
  tran_csr |= BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_1_BYTE;
  tran_csr |= BM1680_SPI_TRAN_CSR_WITH_CMD;
  tran_csr |= BM1680_SPI_TRAN_CSR_TRAN_MODE_TX;
  tran_csr |= BM1680_SPI_TRAN_CSR_TRAN_MODE_RX;

  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);    //do flush FIFO before filling fifo
  u32 total_out_bytes = addr_bytes + send_bytes +((with_cmd) ? 1 : 0);
  for (int i = 0; i < ((total_out_bytes - 1) / 4 + 1); i++) {
    writel(spi_base + REG_BM1680_SPI_FIFO_PORT, p_data[i]);
  }

  /* issue tran */
  writel(spi_base + REG_BM1680_SPI_INT_STS, 0);   //clear all int
  writel(spi_base + REG_BM1680_SPI_TRAN_NUM, get_bytes);
  tran_csr |= BM1680_SPI_TRAN_CSR_GO_BUSY;
  writel(spi_base + REG_BM1680_SPI_TRAN_CSR, tran_csr);

  /* wait tran done and get data */
  u32 int_stat = _check_reg_bits((volatile u32*)spi_base, REG_BM1680_SPI_INT_STS,
                      BM1680_SPI_INT_TRAN_DONE, 100000);
  if (int_stat == 0) {
    uartlog("data in timeout\n");
    return -1;
  }

  p_data = (u32*)dst_buf;
  for (int i = 0; i < ((get_bytes - 1) / 4 + 1); i++) {
    p_data[i] = readl(spi_base + REG_BM1680_SPI_FIFO_PORT);
  }
  writel(spi_base + REG_BM1680_SPI_FIFO_PT, 0);  //should flush FIFO after tran

  return 0;
}

u8 spi_write_en(u64 spi_base)
{
  u8 cmd_buf[4];
  memset(cmd_buf, 0, sizeof(cmd_buf));

  cmd_buf[0] = SPI_CMD_WREN;
  spi_non_data_tran(spi_base, cmd_buf, 1, 0);

  return 0;
}

u8 spi_write_dis(u64 spi_base)
{
  u8 cmd_buf[4];
  memset(cmd_buf, 0, sizeof(cmd_buf));

  cmd_buf[0] = SPI_CMD_WRDI;
  spi_non_data_tran(spi_base, cmd_buf, 1, 0);

  return 0;
}

u32 spi_flash_read_id(u64 spi_base)
{
  u8 cmd_buf[4];
  u8 data_buf[4];
  memset(cmd_buf, 0, sizeof(cmd_buf));
  memset(data_buf, 0, sizeof(data_buf));

  cmd_buf[0] = SPI_CMD_RDID;
  cmd_buf[3] = 0;
  cmd_buf[2] = 0;
  cmd_buf[1] = 0;

  spi_in_out_tran(spi_base, data_buf, cmd_buf, 1, 0, 3, 3);
  u32 read_id = 0;
  read_id = (data_buf[2] << 16) | (data_buf[1] << 8) | (data_buf[0]);

  return read_id;
}

int spi_flash_id_check(u64 spi_base)
{
  uartlog("\n--%s\n", __func__);

  u32 flash_id = 0;

  flash_id = spi_flash_read_id(spi_base);
  u32 i = 0;
  for (i=0; i < sizeof(spi_flash_model_support_list)/sizeof(u32); i++) {
    if (flash_id == spi_flash_model_support_list[i]) {
      uartlog("read id test success, read val:0x%08x\n", flash_id);
      return 0;
    }
  }

  uartlog("read id check failed, read val:0x%08x\n", flash_id);
  return 1;
}

static u8 spi_read_status(u64 spi_base)
{
  u8 cmd_buf[4];
  u8 data_buf[4];
  memset(cmd_buf, 0, sizeof(cmd_buf));
  memset(data_buf, 0, sizeof(data_buf));

  cmd_buf[0] = SPI_CMD_RDSR;
  cmd_buf[3] = 0;
  cmd_buf[2] = 0;
  cmd_buf[1] = 0;
  spi_in_out_tran(spi_base, data_buf, cmd_buf, 1, 0, 3, 3);

  return data_buf[0];
}

u8 spi_reg_status(u64 spi_base, u8 cmd)
{
  u8 cmd_buf[4];
  u8 data_buf[4];
  memset(cmd_buf, 0, sizeof(cmd_buf));
  memset(data_buf, 0, sizeof(data_buf));

  cmd_buf[0] = cmd;
  cmd_buf[3] = 0;
  cmd_buf[2] = 0;
  cmd_buf[1] = 0;
  spi_in_out_tran(spi_base, data_buf, cmd_buf, 1, 0, 3, 3);

  return data_buf[0];
}

u8 spi_data_read(u64 spi_base, u8* dst_buf, u32 addr, u32 size)
{
  u8 cmd_buf[4];

  cmd_buf[0] = SPI_CMD_READ;
  cmd_buf[1] = ((addr) >> 16) & 0xFF;
  cmd_buf[2] = ((addr) >> 8) & 0xFF;
  cmd_buf[3] = (addr) & 0xFF;
  spi_data_in_tran(spi_base, dst_buf, cmd_buf, 1, 3, size);

  return 0;
}

u8 spi_flash_page_program(u64 spi_base, u8 *src_buf, u32 addr, u32 size)
{
  u8 cmd_buf[4];
  memset(cmd_buf, 0, sizeof(cmd_buf));

  cmd_buf[0] = SPI_CMD_PP;
  cmd_buf[1] = (addr >> 16) & 0xFF;
  cmd_buf[2] = (addr >> 8) & 0xFF;
  cmd_buf[3] = addr & 0xFF;

  spi_data_out_tran(spi_base, src_buf, cmd_buf, 1, 3, size);

  return 0;
}

void spi_flash_sector_erase(u64 spi_base, u32 addr)
{
  u8 cmd_buf[4];
  memset(cmd_buf, 0, sizeof(cmd_buf));

  cmd_buf[0] = SPI_CMD_SE;
  cmd_buf[1] = (addr >> 16) & 0xFF;
  cmd_buf[2] = (addr >> 8) & 0xFF;
  cmd_buf[3] = addr & 0xFF;
  spi_non_data_tran(spi_base, cmd_buf, 1, 3);

  return;
}

void spi_flash_bulk_erase(u64 spi_base)
{
  u8 cmd_buf[4];
  memset(cmd_buf, 0, sizeof(cmd_buf));

  cmd_buf[0] = SPI_CMD_BE;
  spi_non_data_tran(spi_base, cmd_buf, 1, 0);

  return;
}

int do_full_chip_erase(u64 spi_base)
{
  spi_write_en(spi_base);
  u8 spi_status = spi_read_status(spi_base);
  if ((spi_status & SPI_STATUS_WEL) == 0) {
    uartlog("write en failed, get status: 0x%02x\n", spi_status);
    return -1;
  }

  spi_flash_bulk_erase(spi_base);

  while(1) {
    u32 wait = 0;
    spi_status = spi_read_status(spi_base);
    if (((spi_status * SPI_STATUS_WIP) == 0) || (wait > 100000000000)) {
      uartlog("full chip erase done, get status: 0x%02x, wait: %d\n", spi_status, wait);
      break;
    }
    if ((wait++ % 100000) == 0)
      uartlog("device busy, get status: 0x%02x\n", spi_status);
  }

  return 0;
}

int do_sector_erase(u64 spi_base, u32 addr, u32 sector_num)
{
  u32 sector_addr = addr - (addr % SPI_SECTOR_SIZE);
  uartlog("do sector erase @0x%08x (sector_addr:0x%08x)\n", addr, sector_addr);

  for (int i = 0; i < sector_num; i++) {
    spi_write_en(spi_base);
    u32 spi_status = spi_read_status(spi_base);
    if ((spi_status & SPI_STATUS_WEL) == 0) {
      uartlog("write en failed, get status: 0x%02x i=%d\n", spi_status,i);
      return -1;
    }

    u32 offset = i * SPI_SECTOR_SIZE;
    spi_flash_sector_erase(spi_base, sector_addr + offset);
    u32 wait = 0;
    while(1) {
      spi_status = spi_read_status(spi_base);
      if (((spi_status & SPI_STATUS_WIP) == 0 ) || (wait > 10000000000)) {
        uartlog("sector erase done, get status: 0x%02x, wait: %d\n", spi_status, wait);
        break;
      }
      if ((wait++ % 100000) == 0)
        uartlog("device busy, get status: 0x%02x\n", spi_status);
    }
  }

  return 0;
}

int do_page_prog(u64 spi_base, u8 *src_buf, u32 addr, u32 size)
{
  if (size > SPI_PAGE_SIZE) {
    uartlog("size larger than a page\n");
    return -1;
  }

  if ((addr % SPI_PAGE_SIZE) != 0) {
    uartlog("addr not alignned to page\n");
    return -1;
  }

  spi_write_en(spi_base);

  u8 spi_status = spi_read_status(spi_base);
  if ((spi_status & SPI_STATUS_WEL) == 0) {
    uartlog("write en failed, get status: 0x%02x\n", spi_status);
    return -1;
  }

  if (spi_status != 0x02) {
    uartlog("spi status check failed, get status: 0x%02x\n", spi_status);
    return -1;
  }

  spi_flash_page_program(spi_base, src_buf, addr, size);

  u32 wait = 0;
  while(1) {
    spi_status = spi_read_status(spi_base);
    if (((spi_status & SPI_STATUS_WIP) == 0 ) || (wait > 10000000)) {
      uartlog("page prog done, get status: 0x%02x\n", spi_status);
      break;
    }
    wait++;
    if ((wait % 10000) == 0) {
      uartlog("device busy, get status: 0x%02x\n", spi_status);
    }
  }

  return 0;
}

int spi_flash_write_by_page(u64 spi_base, u32 fa, u8 *data, u32 size)
{
  u8 cmp_buf[SPI_PAGE_SIZE];
  memset(cmp_buf, 0x11, sizeof(cmp_buf));

  u8 page_num = size / SPI_PAGE_SIZE;
  if (size % SPI_PAGE_SIZE) {
    page_num++;
  }

  u32 offset = 0;
  for (int i = 0; i < page_num; i++) {
    offset = i * SPI_PAGE_SIZE;
    do_page_prog(spi_base, data + offset, fa + offset, SPI_PAGE_SIZE);

#ifndef DEBUG
    spi_data_read(spi_base, cmp_buf, fa + offset, SPI_PAGE_SIZE);
    int ret = memcmp(data, cmp_buf, SPI_PAGE_SIZE);
    if (ret != 0) {
      uartlog("page program test memcmp failed: 0x%08x\n", ret);
      //dump_hex((char *)"src_buf", (void *)data, SPI_PAGE_SIZE);
      //dump_hex((char *)"cmp_buf", (void *)cmp_buf, SPI_PAGE_SIZE);
      return ret;
    }else{
	  uartlog("page program test memcmp success: 0x%08x\n", ret);
    }
#endif
  }

  return 0;
}

#define CLK_EN_REG1 			0x7030012004
void spi_flash_read_by_page(u64 spi_base, u32 fa, u8 *data, u32 size)
{
  u8 page_num = size / SPI_PAGE_SIZE;

  u32 offset = 0;
  for (int i = 0; i < page_num; i++) {
    offset = i * SPI_PAGE_SIZE;
    spi_data_read(spi_base, data + offset, fa + offset, SPI_PAGE_SIZE);
  }

  u32 remainder = size % SPI_PAGE_SIZE;
  if (remainder) {
    offset = page_num * SPI_PAGE_SIZE;
    spi_data_read(spi_base, data + offset, fa + offset, remainder);
  }
#ifdef DEBUG
  //dump_hex((char *)"cmp_buf", (void *)data, size);
#endif
}

void spi_flash_soft_reset(u64 spi_base)
{
  //0x8C003 is default value, 0x200000 is softrst
  writel(spi_base + REG_BM1680_SPI_CTRL, readl(spi_base + REG_BM1680_SPI_CTRL) | 0x1<<21 | 0x3);
  //uartlog("%s:%d\n", __func__, __LINE__);
  return;
}

void spi_flash_set_dmmr_mode(u64 spi_base, u32 en)
{
  u32 reg_val = (en) ? BM1680_SPI_DMMR_EN : 0;

  writel(spi_base + REG_BM1680_SPI_DMMR, reg_val);
 // uartlog("%s:%d\n", __func__, __LINE__);
  return;
}

u64 spi_flash_map_enable(u8 enable)
{
  u64 spi_base = 0;
  //u32 reg = 0;
  if (enable) {
    //reg = readl(TOP_CTLR_BASE_ADDR + REG_BM1680_TOP_IP_EN);
    //reg &= ~BM1680_CHL_IP_EN_SF_REMAP_EN;
    //writel(TOP_CTLR_BASE_ADDR + REG_BM1680_TOP_IP_EN, reg);
#if 0
    spi_base = SPI_CTLR_BASE_ADDR; //0xFFF00000
#else
    spi_base = SPI_BASE; //SPI_CTLR_BASE_ADDR_REMAP; //0x44000000
#endif
  } else {
    //reg = readl(TOP_CTLR_BASE_ADDR + REG_BM1680_TOP_IP_EN);
    //reg |= BM1680_CHL_IP_EN_SF_REMAP_EN;
    //writel(TOP_CTLR_BASE_ADDR + REG_BM1680_TOP_IP_EN, reg);
    spi_base = SPI_BASE;//SPI_CTLR_BASE_ADDR_REMAP; //0x44000000
  }

  return spi_base;
}

void spi_flash_init(u64 spi_base)
{
  u32 tran_csr = 0;

  spi_flash_set_dmmr_mode(spi_base, 0);
  spi_flash_soft_reset(spi_base);

  //uartlog("%s:%d\n", __func__, __LINE__);

  /* conf spi controller regs */
  tran_csr |= (0x03 << BM1680_SPI_TRAN_CSR_ADDR_BYTES_SHIFT);
  tran_csr |= BM1680_SPI_TRAN_CSR_FIFO_TRG_LVL_4_BYTE;
  tran_csr |= BM1680_SPI_TRAN_CSR_WITH_CMD;
  writel(spi_base + REG_BM1680_SPI_TRAN_CSR, tran_csr);
  //uartlog("%s:%d\n", __func__, __LINE__);
#ifdef DEBUG
  printf("check spi reg con[0x%08x]: 0x%08x\n", spi_base + REG_BM1680_SPI_CTRL,
           readl(spi_base + REG_BM1680_SPI_CTRL));
  printf("check spi reg tran csr[0x%08x]: 0x%08x\n", spi_base + REG_BM1680_SPI_TRAN_CSR,
           readl(spi_base + REG_BM1680_SPI_TRAN_CSR));
#endif
}

int spi_flash_program(u64 spi_base, u32 sector_addr, u32 len)
{
  u32 off = 0;
  u32 xfer_size = 0;
  u32 size = len; //DO_SPI_FW_PROG_BUF_SIZE;
  u8 *fw_buf = (u8 *)DO_SPI_FW_PROG_BUF_ADDR;

  while (off < size) {
    if ((size - off) >= SPI_PAGE_SIZE)
      xfer_size = SPI_PAGE_SIZE;
    else
      xfer_size = size - off;

    uartlog("page prog (%d / %d)\n", off, size);
    if (do_page_prog(spi_base, fw_buf + off, sector_addr + off, xfer_size) != 0) {
      uartlog("page prog failed @0x%lx\n", spi_base + sector_addr + off);
      return -1;
    }

    u8 cmp_buf[SPI_PAGE_SIZE];
    memset(cmp_buf, 0x0, SPI_PAGE_SIZE);
    spi_data_read(spi_base, cmp_buf, sector_addr + off, xfer_size);
    int ret = memcmp(fw_buf + off, cmp_buf, xfer_size);
    if (ret != 0) {
      uartlog("memcmp failed\n");
      //dump_hex((char *)"fw_buf", (void *)(fw_buf + off), 16);
      //dump_hex((char *)"cmp_buf", (void *)cmp_buf, 32);
      return ret;
    }
    uartlog("page read compare ok @%d\n", off);
    off += xfer_size;
  }

  uartlog("--%s done!\n", __func__);

  return 0;
}


