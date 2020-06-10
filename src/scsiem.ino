//
// SxSI direct access device emulator
// ( SASI interface is not supported yet. :-P )
//
// 1.0: initial release
// 1.1: multiple drive supported(SCSI_ID0..5)


#define	PRODUCT_NAME	"SCSI direct access device EMulator (scsiem) " __DATE__
#define	VERSION    	1
#define	VERSION_SUB	1

#ifndef	debug_level
#  define	debug_level	(4)
#endif
#include	"logging.h"

static const char *HDIMG_FILE_NAMES[8]={
	"SCSI_ID0.HDS",
	"SCSI_ID1.HDS",
	"SCSI_ID2.HDS",
	"SCSI_ID3.HDS",
	"SCSI_ID4.HDS",
	"SCSI_ID5.HDS",
	"SASI_ID1.HDS",
	"SASI_ID0.HDS",
};

#define SECperTRK (63)  // number of sectors per track(63..1)
#define TRKperCYL (65)  // number of heads=tracks per cylinder(255..1)

#include	"SPI.h"
#include	"SdFat.h"

#ifdef ENABLE_EXTENDED_TRANSFER_CLASS
#  if defined(SD_FAT_VERSION) && ((SD_FAT_VERSION/10000) == 1) && (ENABLE_EXTENDED_TRANSFER_CLASS)
#    define SdFat1 1
// enabling ENABLE_EXTENDED_TRANSFER_CLASS for SdFat V.1
// in ~/.platformio/lib/SdFat/SdFatConfig.h or current project directory.
SPIClass	SPI_1(1); //  SdFat V.1
SdFatEX 	SD(&SPI_1);
#  else
#    error "*** Invalid SD_FAT_VERSION or undefined ENABLE_EXTENDED_TRANSFER_CLASS."
#  endif
#else
// #  ifdef SD_FAT_VERSION == 2
#    warning "*** Invalid SD_FAT_VERSION or undefined ENABLE_EXTENDED_TRANSFER_CLASS."
#    define SdFat2 2
#    define SD1_CONFIG SdSpiConfig(PA4, SHARED_SPI, SD_SCK_MHZ(SPI_FULL_SPEED), &SPI) 
SdFs	SD; //  SdFat V.2
// #  else
// #    error "*** Undefined SdFat version."
// #  endif
#endif

#if ( !defined(log_err) || !defined(logln_err) )
#  define	log_err(...)
#  define	logln_err(...)
#  define	log_warn(...)
#  define	logln_warn(...)
#  define	log_info(...)
#  define	logln_info(...)
#  define	log_debug(...)
#  define	logln_debug(...)
#endif

//
// I/O definition
//
#define	BUS_H	0
#define	BUS_L	1

#define	isHigh(XX)	((XX) == BUS_H)
#define	isLow(XX)	((XX) != BUS_H)

#define	gpio_mode(pin,mode)	gpio_set_mode(PIN_MAP[pin].gpio_device,  PIN_MAP[pin].gpio_bit, mode)
#define	gpio_write(pin,val)	gpio_write_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, val)
#define	gpio_read(pin)  	gpio_read_bit(PIN_MAP[pin].gpio_device,  PIN_MAP[pin].gpio_bit)

#define	TIM     	Timer3
#define	TIM_CH  	TIMER_CH4
#define	BUZ     	PB1     // TIM3_4 out
#define	BUZoff  	TIM.pause()
#define	BUZon   	TIM.resume()
#define BUZ_FREQ(f) { \
  TIM.pause(); \
  TIM.setPrescaleFactor(72); \
  TIM.setOverflow(1000000/f); \
  TIM.setCompare(TIM_CH, 500000/f); \
  TIM.setMode(TIM_CH, TIMER_PWM); \
  BUZoff; \
  gpio_mode(BUZ, GPIO_AF_OUTPUT_PP); \
}

#define	SD_CS   	PA4      // SDCARD:CS
#define	SD_MD   	PB5      // SDCARD:MediaDetect#

#define	LED     	PC13     // LED(green: H=off L=on)
#define	LED2    	PB0      // LED(bi-color: HiZ=off H=Green L=Red)

#define	USER_BTN	PB2      // abort unit switch.

// LED(green: H=off L=on)
#define	LEDoff  	gpio_write(LED, HIGH);
#define	LEDon   	gpio_write(LED, LOW);

// LED2(bi-color: HiZ=off H=Green L=Red)
#define	LED2off 	{ gpio_mode(LED2, GPIO_INPUT_FLOATING); }
#define	LED2RED 	{ gpio_mode(LED2, GPIO_OUTPUT_PP); gpio_write(LED2, LOW); }
#define	LED2GRN 	{ gpio_mode(LED2, GPIO_OUTPUT_PP); gpio_write(LED2, HIGH); }

// SCSI_BUS_DATA_BUS -->> see SCSI_BUS_get_byte() & SCSI_BUS_put_byte()
//#define	DBP     	PB7      // SCSI:DBP
//#define	DB0     	PB8      // SCSI:DB0
//#define	DB1     	PB9      // SCSI:DB1
//#define	DB2     	PB10     // SCSI:DB2
//#define	DB3     	PB11     // SCSI:DB3
//#define	DB4     	PB12     // SCSI:DB4
//#define	DB5     	PB13     // SCSI:DB5
//#define	DB6     	PB14     // SCSI:DB6
//#define	DB7     	PB15     // SCSI:DB7

// SCSI_BUS_CONTROL_LINES -->> see SCSI BUS operation section.
#define	ATN     	PB6      // SCSI:ATN
#define	BSY     	PA15     // SCSI:BSY
#define	ACK     	PB3      // SCSI:ACK
#define	RST     	PA8      // SCSI:RST
#define	MSG     	PA0      // SCSI:-MSG
#define	SEL     	PB4      // SCSI:SEL
#define	C_D     	PA1      // SCSI:-C/D
#define	REQ     	PA2      // SCSI:-REQ
#define	I_O     	PA3      // SCSI:-I/O

//
// global variables
//
union wk {
	uint32_t	uint32;
	uint16_t	uint16[2];
	uint8_t 	uint8[4];
} __packed;

#define	BLOCKSIZE	512	// default block size 512bytes/block
static	byte    	SCSI_id;
static	uint32_t	FIL_IDX;
static	struct  {
	char    	*HDIMG_FILE_NAME;
#ifdef SdFat1
	File    	HDIMG_FILE; 	// HDimage file object
	uint32_t	FILE_SIZE;  	// HDimage file size in byte.
#else
#  ifdef SdFat2
	FsFile    	HDIMG_FILE; 	// HDimage file object
	uint64_t	FILE_SIZE;  	// HDimage file size in byte.
#  endif
#endif
	union wk	BLOCK_CNT;  	// HDimage file size in block.
	union wk	BLOCK_SIZ;
} FIL[8];

byte    	BLK_BUF[BLOCKSIZE];	// disk block I/O buffer

static	byte    	SENSE_KEY= 0;
static	volatile bool SCSI_BUS_RST_FLAG= false;

static	byte    	MSG_CNT;
static	byte    	MSG_BUF[256];

//
// primitive read/write SCSI DataBus w/Parity 
//
static inline int parity_even(byte val) {
  uint32_t x;
  x= (val & 0x55) + ((val & 0xaa) >> 1);
  x= (x & 0x33) + ((x & 0xcc) >> 2);
  x= (x & 0x0f) + ((x & 0xf0) >> 4);
  return x & 0x01;
}

static inline byte SCSI_BUS_get_byte(void){ // PB15..7 is INPUT mode
  GPIOB->regs->CRL = ( GPIOB->regs->CRL & 0x0fffffff ) | 0x40000000;
  GPIOB->regs->CRH = 0x44444444;   // SET INPUT W/O PUPD on PB15-PB8
  return ( ( (~(GPIOB->regs->IDR)) >> 8) & 0xff );
}

static inline void SCSI_BUS_put_byte(byte v){// PB15..7 is OUTPUT mode
  GPIOB->regs->BSRR = 0xff800000 | (( (~v) & 0xff)<<8) | (parity_even(~v)<<7);
  GPIOB->regs->CRL = ( GPIOB->regs->CRL & 0x0fffffff ) | 0x70000000;  // SET OUTPUT W/ PUPD on PB7 50MHz
  GPIOB->regs->CRH = 0x77777777;  // SET OUTPUT-OD W/ PUPD on PB15-PB8 50MHz
}

//
// SCSI BUS operation
//
#if 0
 H init.-> target
 L target -> init.
     H
     L
         H  Data
         L  Stat/Cmd
             H  Data/Stat/Cmd
             L  Msg
PA3 PA2 PA1 PA0
I/O REQ C/D MSG
 L1  *   L1  L1 MsgIn
 H0  *   L1  L1 MsgOut
 L1  *   L1  H0 Status
 H0  *   L1  H0 Command
 L1  *   H0  H0 DataIn
 H0  *   H0  H0 DataOut
typedef enum gpio_pin_mode {
    /** Output push-pull. */
    GPIO_OUTPUT_PP      = GPIO_CR_CNF_OUTPUT_PP | GPIO_CR_MODE_OUTPUT_50MHZ,
    /** Output open-drain. */
    GPIO_OUTPUT_OD      = GPIO_CR_CNF_OUTPUT_OD | GPIO_CR_MODE_OUTPUT_50MHZ,
    /** Alternate function output push-pull. */
    GPIO_AF_OUTPUT_PP   = GPIO_CR_CNF_AF_OUTPUT_PP | GPIO_CR_MODE_OUTPUT_50MHZ,
    /** Alternate function output open drain. */
    GPIO_AF_OUTPUT_OD   = GPIO_CR_CNF_AF_OUTPUT_OD | GPIO_CR_MODE_OUTPUT_50MHZ,
    /** Analog input. */
    GPIO_INPUT_ANALOG   = GPIO_CR_CNF_INPUT_ANALOG | GPIO_CR_MODE_INPUT,
    /** Input floating. */
    GPIO_INPUT_FLOATING = GPIO_CR_CNF_INPUT_FLOATING | GPIO_CR_MODE_INPUT,
    /** Input pull-down. */
    GPIO_INPUT_PD       = GPIO_CR_CNF_INPUT_PU_PD | GPIO_CR_MODE_INPUT,
    /** Input pull-up. */
    GPIO_INPUT_PU, /* (treated a special case, for ODR twiddling) */
} gpio_pin_mode;
#endif

#define	BUS_CTRL(x)     { GPIOA->regs->BSRR = 0x000f0000 | ((x)&0x0f); }
#define	SCSI_BUS_FREE   { BUS_CTRL(0); /* -I/O,-REQ,-C/D,-MSG */ \
  SCSI_BUS_NOTBUSY; \
  gpio_mode(ATN, GPIO_INPUT_FLOATING); \
  gpio_mode(ACK, GPIO_INPUT_FLOATING); \
  gpio_mode(RST, GPIO_INPUT_FLOATING); \
  gpio_mode(SEL, GPIO_INPUT_FLOATING); \
}

#define	SCSI_BUS_BUSY     { \
  gpio_mode(BSY, GPIO_OUTPUT_OD); \
  gpio_write(BSY, BUS_H); \
}
#define	SCSI_BUS_NOTBUSY  { \
  gpio_write(BSY, BUS_H); \
  gpio_mode(BSY, GPIO_INPUT_FLOATING); \
}

#define	isSCSI_BUS_BSY    isLow(gpio_read(BSY))
#define	isSCSI_BUS_SEL    isLow(gpio_read(SEL))
#define	isSCSI_BUS_ATN    isLow(gpio_read(ATN))
#define	isSCSI_BUS_ACK    isLow(gpio_read(ACK))
#define	isSCSI_BUS_RST    isLow(gpio_read(RST))

#define	SCSI_BUS_REQ      { GPIOA->regs->BSRR = 0x00000004; }
#define	SCSI_BUS_REQN     { GPIOA->regs->BSRR = 0x00040000; }

#define	BUS_PHASE(x)      { GPIOA->regs->BSRR = 0x000b0000 | ((x)&0x0b); }
#define	SCSI_PHASE_CMD    BUS_PHASE(0x02) /*  I/O, * ,-C/D, MSG */
#define	SCSI_PHASE_STAT   BUS_PHASE(0x0a) /* -I/O, * ,-C/D, MSG */
#define	SCSI_PHASE_DTIN   BUS_PHASE(0x08) /* -I/O, * , C/D, MSG */
#define	SCSI_PHASE_DTOUT  BUS_PHASE(0x00) /*  I/O, * , C/D, MSG */
#define	SCSI_PHASE_MSGIN  BUS_PHASE(0x0b) /* -I/O, * ,-C/D,-MSG */
#define	SCSI_PHASE_MSGOUT BUS_PHASE(0x03) /*  I/O, * ,-C/D,-MSG */

static const uint32_t SCSI_CMD_group_length_tbl[8]={ 6, 10, 10, 1, 1, 12, 1, 1 };

//
// ********************** SCSI CMDs functions
//
#include "scsi_cmd.inc"

//
// ********************** setup()
//
void setup(void){
uint32_t i;
  // only use PA14,PA13 for debug
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  // setting User button(s)
  gpio_write(USER_BTN, HIGH);
  gpio_mode(USER_BTN, GPIO_INPUT_PU);

  // open serial for debug
  debugout.begin(115200);
  while (!debugout){
    LED2GRN;
    delay(50);
    LED2off;
    delay(150);
  };

  BUZ_FREQ(3000);

  // setting LED & LED2
  gpio_mode(LED, GPIO_OUTPUT_OD);
  LEDoff;
  LED2off;

  gpio_write(SD_MD, HIGH);
  gpio_mode(SD_MD, GPIO_INPUT_PU);


  logln_info("\r\n\r\nInfo: " PRODUCT_NAME);
  log_info("Info: version ");
  log_info(VERSION);
  log_info(".");
  log_info(VERSION_SUB);
  log_info(" debug_level=");
  logln_info(debug_level);

 // setting GPIO(SCSI BUS) I/O
  SCSI_BUS_get_byte(); // set input mode
  delay(100);
  BUZon;

 // setting GPIO(SCSI BUS) OUTPUT
  SCSI_BUS_FREE;
  gpio_mode(MSG, GPIO_OUTPUT_PP);
  gpio_mode(C_D, GPIO_OUTPUT_PP);
  gpio_mode(REQ, GPIO_OUTPUT_PP);
  gpio_mode(I_O, GPIO_OUTPUT_PP);
  LED2GRN;

  // set interrupt from RST fall edge sensed
  attachInterrupt(PIN_MAP[RST].gpio_bit, SCSI_BUS_RST_assert, FALLING);
  LED2RED;

  while (gpio_read(SD_MD)) {
    BUZon;
    logln_warn("Warning: SD/TFcard not present.");
    delay(50);
    BUZoff;
    delay(2950);
  };
  delay(100);
  BUZon;

  if (!SD.begin(SD_CS,SPI_FULL_SPEED)) {
    logln_err("Error: SD/TFcard initialization failed!");
    RIP(200);
  }

  LEDon;

  SCSI_id= 0x00; // clear ID map
  FIL_IDX= 0;    // temporary counts emulate number of drives.
  for(i=0;i<6;i++){
    char *FileName = (char *)HDIMG_FILE_NAMES[i];
    log_info("Info: attempt to open HD image '");
    log_info( FileName );
    logln_info("'.");
    FIL[i].HDIMG_FILE = SD.open( FileName, O_RDWR );
    if( FIL[i].HDIMG_FILE ){
      FIL_IDX += 1;
      FIL[i].HDIMG_FILE_NAME= FileName;
      SCSI_id |= 1<<i;
      if(i<6){
        log_info("Info: >>> SCSI_ID= ");
        log_info(i);
      } else {
        log_info("Info: >>> SASI_# = ");
        log_info(7-i);
      };
      logln_info(" is assumed.");
      FIL[i].FILE_SIZE = FIL[i].HDIMG_FILE.size();
      log_info("Info: open an HD Image File. '");
      log_info(FIL[i].HDIMG_FILE_NAME);
      logln_info("'.");
      log_info("Info: ");
      log_info(FIL[i].FILE_SIZE);
      log_info(" byte (");
      log_info(FIL[i].FILE_SIZE / 1024 / 1024);
      log_info("MiB = ");
      if(SCSI_id & 0xc0){
        FIL[i].BLOCK_SIZ.uint32= 256;
      }else{
        FIL[i].BLOCK_SIZ.uint32= BLOCKSIZE;
      };
      FIL[i].BLOCK_CNT.uint32= FIL[i].FILE_SIZE / FIL[i].BLOCK_SIZ.uint32;
      log_info(FIL[i].BLOCK_CNT.uint32);
      log_info("BLKs @ ");
      log_info(FIL[i].BLOCK_SIZ.uint32);
      logln_info("B/BLK)");
      if (FIL[i].FILE_SIZE % FIL[i].BLOCK_SIZ.uint32) {
        logln_warn("Warning: File size is not aligned by block size.");
      };
    };
  };
  log_warn("Info: scsiem will emulate number of drives: ");
  logln_warn(FIL_IDX);
  if(! SCSI_id){
    logln_err("Error: HDimage files not found.");
    RIP(600);
  };

  BUZ_FREQ(4000);
  delay(100);
  LEDoff;
  LED2off; // OK! go on
}

//
// ********************** RIP (system halt in place)
//
void RIP(uint32_t delaytime){
  SCSI_BUS_FREE;
  gpio_mode(BUZ, GPIO_OUTPUT_PP);
  while(!gpio_read(SD_MD)) {
    LED2RED;
    LEDoff;
    gpio_write(BUZ, LOW);
    delay(delaytime);
    LED2off;
    LEDon;
    gpio_write(BUZ, HIGH);
    delay(delaytime);
  };
  while(true) {
    ;;
  };
}

//
// ********************** MAIN loops: loop()
//
#if debug_level > 2
void test_func(void);
static uint32_t CNT;
static uint32_t DELAY_CNT = 1000000;
#endif

void loop(void) {
int msg;
int stat;
uint32_t len;
uint32_t i;
byte cmd_buf[12];
	msg= 0;
	len= 0;
	stat= 0;

#if debug_level > 2
	test_func();
	if(CNT){
		CNT -= 1;
//		logln_debug(CNT);
		if(0==CNT){
			logln_debug("[BUS free]");
			SCSI_BUS_get_byte();
			SCSI_BUS_FREE;
		};
		return;
	};
#endif

  if (! gpio_read(USER_BTN) ){
    for(i=0;i<6;i++){
      if(SCSI_id & (1<<i)){
        FIL[i].HDIMG_FILE.close();
        log_info("Info: closed an HD image '");
        log_info(FIL[i].HDIMG_FILE_NAME);
        logln_info("'.");
      };
    };
    logln_info("... and HALT ...");
    RIP(1500);
  };

  if( !isSCSI_BUS_BSY || isSCSI_BUS_SEL ) {
    return;
  };

  byte db = SCSI_BUS_get_byte();
  if( (db & SCSI_id) == 0 ) {
    return;
  };
  for(i=0;i<6;i++){
    if( (1<<i) & (db & SCSI_id) ){
      FIL_IDX=i;
      break;;
    };
  };
  log_debug("select target_ID=");
  logln_debug(i);

  logln_debug("Selection phase");
  SCSI_BUS_RST_FLAG = false;
  LEDon; // SCSI_BUS hold

  SCSI_BUS_BUSY;
  while(!isSCSI_BUS_SEL) {
    if(SCSI_BUS_RST_FLAG) goto lbl_bus_free;
  };

  if(!isSCSI_BUS_ATN) { // indication has a message for the target (MSGOUT)
    if(SCSI_BUS_attention()) goto lbl_bus_free;
  };

  logln_debug("Command out phase");
  SCSI_PHASE_CMD;

  cmd_buf[0] = SCSI_BUS_ItoT();
  log_debug(cmd_buf[0],HEX);
  log_debug(':');
  len= SCSI_CMD_group_length_tbl[ cmd_buf[0] >> 5 ];
  for (i = 1; i < len; i++ ) {
    cmd_buf[i] = SCSI_BUS_ItoT();
    log_debug(cmd_buf[i],HEX);
    log_debug(',');
  };
  logln_debug("");

  stat= do_SCSI_CMD(cmd_buf);

  if(SCSI_BUS_RST_FLAG) goto lbl_bus_free;

  logln_debug("Status in phase");
  SCSI_PHASE_STAT;
  SCSI_BUS_IfromT(stat);
  if(SCSI_BUS_RST_FLAG) goto lbl_bus_free;

  logln_debug("Message In phase");
  SCSI_PHASE_MSGIN;
  SCSI_BUS_IfromT(msg);

lbl_bus_free:
  logln_debug("bus_free phase");
  SCSI_BUS_RST_FLAG = false;
  SCSI_BUS_FREE;
  LEDoff; // SCSI_BUS released
}

//
// ********************** MAIN loops: test_func():low level SCSI_BUS_debug features
//
#if debug_level > 2
#  include "test_func.inc"
#endif
