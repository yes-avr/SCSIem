//
// a part of SxSI direct access device emulator
//

int do_SCSI_CMD(byte *p);

void SCSI_BUS_RST_assert(void);
byte SCSI_BUS_ItoT(void);
void SCSI_BUS_IfromT(byte dat);
void SCSI_PROT_DTIN_phase(uint32_t len, byte* p);
void SCSI_PROT_DTIN_direct(uint32_t adds, uint32_t len);
void SCSI_PROT_DTOUT_direct(uint32_t adds, uint32_t len);
void SCSI_PROT_MSGIN_phase(byte msg);
void SCSI_PROT_MSGOUT_phase(void);
uint32_t SCSI_BUS_attention(void);

void SCSI_CMD_RequestSense(byte *p);
void SCSI_CMD_ReadCapacity(void);
byte SCSI_CMD_Read6(byte *p);
byte SCSI_CMD_Read10(byte *p);
byte SCSI_CMD_Write6(byte *p);
byte SCSI_CMD_Write10(byte *p);
void SCSI_CMD_ModeSense(byte *p);

//
// ********************** SCSI_CMD_Inquiry (constant table)
// see - >
// www.seagate.com/files/staticfiles/support/docs/manual/Interface%20manuals/100293068j.pdf
//
static const byte InqResponse[37]=
 "\x00"     // device type:Direct access block device
 "\x00"     // Fixed Media (Removable Media bit(MSB)=0)
 "\x01"     // SCSI version = 1:(obsolete SCSI-1)
 "\x01"     // RESPONSE DATA FORMAT=1:(obsolete SCSI-1)
 "\x1f"     // ADDITIONAL LENGTH (N-4)
 "\x00"     // supports features(no SCCS ACC )
 "\x00"     // supports features(no MULTIP)
 "\x00"     // supports features(no CMDcue)
 "NECKLESS" // T10 VENDOR IDENTIFICATION(8 bytes) for NEC vender protection
 "SCSI_HDDemulator" // PRODUCT IDENTIFICATION(16bytes)
 "0100"     // PRODUCT REVISION LEVEL(4 bytes)
 // asciz NULLend
;

static const char *SCSI_CMD_NAME_basic[0x30]={ // refer to: //www.t10.org/lists/op-num.htm
	"TEST UNIT READY",                  // 0x00
	"REWIND/REZERO UNIT",               // 0x01
	"*vender unique*",                  // 0x02
	"REQUEST SENSE",                    // 0x03
	"FORMAT UNIT/MEDIUM",               // 0x04
	"READ BLOCK LIMITS",                // 0x05
	"*vender unique*",                  // 0x06
	"REASSIGN BLOCKS",                  // 0x07
	"READ(6)",                          // 0x08
	"*vender unique*",                  // 0x09
	"WRITE(6)/SEND(6)/PRINT",           // 0x0a
	"SEEK(6)",                          // 0x0b
	"*vender unique*",                  // 0x0c
	"*vender unique*",                  // 0x0d
	"*vender unique*",                  // 0x0e
	"READ REVERSE(6)",                  // 0x0f
	"WRITE FILEMARKS(6)",               // 0x10
	"SPACE(6)",                         // 0x11
	"INQUIRY",                          // 0x12
	"VERIFY(6)",                        // 0x13
	"RECOVER BUFFERED DATA",            // 0x14
	"MODE SELECT(6)",                   // 0x15
	"RESERVE(6)",                       // 0x16
	"RELEASE(6)",                       // 0x17
	"COPY",                             // 0x18
	"ERASE(6)",                         // 0x19
	"MODE SENSE(6)",                    // 0x1a
	"START STOP UNIT",                  // 0x1b
	"*vender unique*",                  // 0x1c
	"*vender unique*",                  // 0x1d
	"*vender unique*",                  // 0x1e
	"*vender unique*",                  // 0x1f
	"*vender unique*",                  // 0x20
	"*vender unique*",                  // 0x21
	"*vender unique*",                  // 0x22
	"READ FORMAT CAPACITIES",           // 0x23
	"SET WINDOW",                       // 0x24
	"READ CAPACITY(10)",                // 0x25
	"*vender unique*",                  // 0x26
	"*vender unique*",                  // 0x27
	"READ(10)",                         // 0x28
	"READ GENERATION",                  // 0x29
	"WRITE(10)/SEND(10)"                // 0x2a
	"SEEK(10)",                         // 0x2b
	"ERASE(10)",                        // 0x2c
	"READ UPDATED BLOCK",               // 0x2d
	"WRITE AND VERIFY(10)",             // 0x2e
	"VERIFY(10)",                       // 0x2f
};
int do_SCSI_CMD(byte *p){
  switch(*p) { // refer to: //www.t10.org/lists/op-num.htm
/* basic (6) commands */
  case 0x03:
    logln_debug("[REQUEST SENSE]");
    SCSI_CMD_RequestSense(p);
    break;
  case 0x08:
    logln_debug("[READ(6)]");
    return SCSI_CMD_Read6(p);
    break;
  case 0x0a:
    logln_debug("[WRITE(6)/SEND(6)/PRINT]");
    return SCSI_CMD_Write6(p);
    break;
  case 0x12:
    logln_debug("[INQUIRY]");
    SCSI_PROT_DTIN_phase((uint32_t) (*(p+4) < 36 ? *(p+4) : 36), (byte *)InqResponse);
    break;
  case 0x1a:
    logln_debug("[MODE SENSE(6)]");
    SCSI_CMD_ModeSense(p);
    break;
/* basic (10) commands */
  case 0x25:
    logln_debug("[READ CAPACITY(10)]");
    SCSI_CMD_ReadCapacity();
    break;
  case 0x28:
    logln_debug("[READ(10)]");
    return SCSI_CMD_Read10(p);
    break;
  case 0x2a:
    logln_debug("[WRITE(10)/SEND(10)]");
    return SCSI_CMD_Write10(p);
    break;
  case 0x55:
    logln_debug("[MODE SELECT(10)]");
    break;
  case 0x5a:
    logln_debug("[MODE SENSE(10)]");
    SCSI_CMD_ModeSense(p);
    break;
  default:
    if(*p < 0x30){
      log_debug('{');
      log_debug(SCSI_CMD_NAME_basic[*p]);
      logln_debug('}');
    }else{
      log_err("Error: [CMD=0x");
      log_err( *p, HEX);
      logln_err(" invalid command]");
      SENSE_KEY = 5;
      return 2;
    };
    break;
  };
  return 0;
}


//
// ********************** SCSI_BUS_RST_assert
//
void SCSI_BUS_RST_assert(void){
  if(!isSCSI_BUS_RST) {
    if(isSCSI_BUS_RST) {
      logln_debug("BusReset!");
      SCSI_BUS_RST_FLAG = true;
    };
  };
}

//
// ********************** SCSI_BUS_ItoT
//
byte SCSI_BUS_ItoT(void){
byte ret;
  SCSI_BUS_REQ;
  while(isSCSI_BUS_ACK) {
    if(SCSI_BUS_RST_FLAG) return 0;
  };
  ret = SCSI_BUS_get_byte();
  SCSI_BUS_REQN;
  while(!isSCSI_BUS_ACK) {
    if(SCSI_BUS_RST_FLAG) return 0;
  };
  return ret;
}

//
// ********************** SCSI_BUS_IfromT
//
void SCSI_BUS_IfromT(byte dat){
  SCSI_BUS_put_byte(dat);
  SCSI_BUS_REQ;
  while(isSCSI_BUS_ACK) {
    if(SCSI_BUS_RST_FLAG) return;
  };
  SCSI_BUS_REQN;
  while(!isSCSI_BUS_ACK) {
    if(SCSI_BUS_RST_FLAG) return;
  };
}

//
// ********************** SCSI_PROT_DTIN_phase
//
void SCSI_PROT_DTIN_phase(uint32_t len, byte* p){
uint32_t i;
  logln_debug("DATAIN PHASE");
  SCSI_PHASE_DTIN;
  for (i = 0; i < len; i++) {
    if(SCSI_BUS_RST_FLAG) return;
    SCSI_BUS_IfromT(p[i]);
  };
}

//
// ********************** SCSI_PROT_DTIN_direct
//
void SCSI_PROT_DTIN_direct(uint32_t adds, uint32_t len){
uint32_t pos;
uint32_t i,j;
  logln_debug("DATAIN PHASE(SD)");
  pos = adds * FIL[FIL_IDX].BLOCK_SIZ.uint32;
  FIL[FIL_IDX].HDIMG_FILE.seek(pos);
  SCSI_PHASE_DTIN;
  for (i = 0; i < len; i++) {
    FIL[FIL_IDX].HDIMG_FILE.read(BLK_BUF, FIL[FIL_IDX].BLOCK_SIZ.uint32);
    for (j = 0; j < FIL[FIL_IDX].BLOCK_SIZ.uint32; j++) {
      if(SCSI_BUS_RST_FLAG) return;
      SCSI_BUS_IfromT(BLK_BUF[j]);
    };
  };
}

//
// ********************** SCSI_PROT_DTOUT_direct
//
void SCSI_PROT_DTOUT_direct(uint32_t adds, uint32_t len){
uint32_t pos;
uint32_t i,j;
  logln_debug("DATAOUT PHASE(SD)");
  pos = adds * FIL[FIL_IDX].BLOCK_SIZ.uint32;
  FIL[FIL_IDX].HDIMG_FILE.seek(pos);
  SCSI_PHASE_DTOUT;
  for (i = 0; i < len; i++) {
    for (j = 0; j < FIL[FIL_IDX].BLOCK_SIZ.uint32; j++) {
      if(SCSI_BUS_RST_FLAG) return;
      BLK_BUF[j] = SCSI_BUS_ItoT();
    };
    FIL[FIL_IDX].HDIMG_FILE.write(BLK_BUF, FIL[FIL_IDX].BLOCK_SIZ.uint32);
  };
  FIL[FIL_IDX].HDIMG_FILE.flush();
}

//
// ********************** SCSI_PROT_MSGIN_phase
//
void SCSI_PROT_MSGIN_phase(byte msg){
  logln_debug("SCSI_PROT_MSGIN_phase");
  SCSI_PHASE_MSGIN;
  SCSI_BUS_IfromT(msg);
}

//
// ********************** SCSI_PROT_MSGOUT_phase
//
void SCSI_PROT_MSGOUT_phase(void){
  logln_debug("SCSI_PROT_MSGOUT_phase");
  SCSI_PHASE_MSGOUT;
  MSG_BUF[MSG_CNT] = SCSI_BUS_ItoT();
  MSG_CNT += 1;
}

//
// ********************** SCSI_BUS_attention (MSGOUT)
//
uint32_t SCSI_BUS_attention(void) {
int i;
int sync_period = 50;
int sync_offset = 0;
    MSG_CNT = 0;
    memset(MSG_BUF, 0x00, sizeof(MSG_BUF));
    while(!isSCSI_BUS_ATN) {
      SCSI_PROT_MSGOUT_phase();
    };
    for (i = 0; i < MSG_CNT; i++) {
      // NOP
      if (MSG_BUF[i] == 0x00) ;;
      // Save Date Pointer / Restore Pointers
      if (MSG_BUF[i] == 0x02 || MSG_BUF[i] == 0x03) ;;
      // DISCONNECT
      if (MSG_BUF[i] == 0x04) return 1;
      // ABORT
      if (MSG_BUF[i] == 0x06 || MSG_BUF[i] == 0x23) return 1;
      // MESSAGE ERROR / REJECT
      if (MSG_BUF[i] == 0x09 || MSG_BUF[i] == 0x07) return 1;
      // BUS DEVICE RESET
      if (MSG_BUF[i] == 0x0c) return 1;
      // Extended IDENTIFY
      if (MSG_BUF[i] >= 0x80) ;; 
      // Synchronous Data Transfer Request
      if (MSG_BUF[i] == 0x01) {
        if ( MSG_BUF[i + 2] != 0x01 ) {
          SCSI_PROT_MSGIN_phase(0x07); // Message Rejection
          break;
        };
        // set Synchronous period (limit up to 50)
        sync_period = MSG_BUF[i + 3];
        if (sync_period > 50) {
          sync_period = 50;
        };
        // set REQ/ACK offset
        sync_offset = MSG_BUF[i + 4];
        if (sync_offset > 16) {
          sync_offset = 16;
        };
        // Synchronous Data Transfer Response
        SCSI_PROT_MSGIN_phase(0x01); // Synchronous Data Transfer Acknowlidge
        SCSI_PROT_MSGIN_phase(0x03); // 
        SCSI_PROT_MSGIN_phase(0x01); // 
        SCSI_PROT_MSGIN_phase(sync_period);
        SCSI_PROT_MSGIN_phase(sync_offset);
        break;
      };
    };
  return 0;
}

//
// ********************** SCSI_CMD_RequestSense
//
void SCSI_CMD_RequestSense(byte *p){
union wk len;
byte buf[18];
  memset((void *)buf,(int)0,sizeof(buf));
  len.uint32 = (uint32_t) *(p+4);
  buf[0] = 0x70; // RESPONSE CODE = 0x70:CHECK CONDITION RESPONSE
  buf[2] = SENSE_KEY;
  buf[7] = sizeof(buf)-8; // ADDITIONAL LENGTH (N-8)
  SENSE_KEY = 0;
  SCSI_PROT_DTIN_phase(len.uint32 < 18 ? len.uint32 : 18, buf);
}

//
// ********************** SCSI_CMD_ReadCapacity
//
void SCSI_CMD_ReadCapacity(void){
byte buf[8];
  buf[0] = (byte)(FIL[FIL_IDX].BLOCK_CNT.uint8[3]);
  buf[1] = (byte)(FIL[FIL_IDX].BLOCK_CNT.uint8[2]);
  buf[2] = (byte)(FIL[FIL_IDX].BLOCK_CNT.uint8[1]);
  buf[3] = (byte)(FIL[FIL_IDX].BLOCK_CNT.uint8[0]);
  buf[4] = (byte)(FIL[FIL_IDX].BLOCK_SIZ.uint8[3]);
  buf[5] = (byte)(FIL[FIL_IDX].BLOCK_SIZ.uint8[2]);
  buf[6] = (byte)(FIL[FIL_IDX].BLOCK_SIZ.uint8[1]);
  buf[7] = (byte)(FIL[FIL_IDX].BLOCK_SIZ.uint8[0]);
  SCSI_PROT_DTIN_phase(8, buf);
}

//
// ********************** SCSI_CMD_Read6
//
byte SCSI_CMD_Read6(byte *p){
union wk addr,len;
  addr.uint32= 0;
  len.uint32 = (uint32_t) *(p+4);
  if( 0 == *(p+4) ) len.uint8[1]= 0x01;
  addr.uint8[2]= *(p+1) & 0x1f;
  addr.uint8[1]= *(p+2);
  addr.uint8[0]= *(p+3);
  log_debug("Read6 @ 0x");
  log_debug(addr.uint32,HEX);
  log_debug(" len(0x");
  log_debug(len.uint32,HEX);
  logln_debug(")");
  LED2GRN;
  SCSI_PROT_DTIN_direct(addr.uint32, len.uint32);
  LED2off;
  return 0;
}

//
// ********************** SCSI_CMD_Read10
//

byte SCSI_CMD_Read10(byte *p){
union wk addr,len;
  addr.uint32= 0;
  len.uint32 = 0;
  addr.uint8[3]= *(p+2);
  addr.uint8[2]= *(p+3);
  addr.uint8[1]= *(p+4);
  addr.uint8[0]= *(p+5);
  len.uint8[1] = *(p+7);
  len.uint8[0] = *(p+8);
  log_debug("Read10 @ 0x");
  log_debug(addr.uint32,HEX);
  log_debug(" len(0x");
  log_debug(len.uint32,HEX);
  logln_debug(")");
  LED2GRN;
  SCSI_PROT_DTIN_direct(addr.uint32, len.uint32);
  LED2off;
  return 0;
}

//
// ********************** SCSI_CMD_Write6
//
byte SCSI_CMD_Write6(byte *p){
union wk addr,len;
  addr.uint32= 0;
  len.uint32 = (uint32_t) *(p+4);
  if( 0 == *(p+4) ) len.uint8[1]= 0x01;
  addr.uint8[2]= *(p+1) & 0x1f;
  addr.uint8[1]= *(p+2) & 0x1f;
  addr.uint8[0]= *(p+3) & 0x1f;
  log_debug("Write6 @ 0x");
  log_debug(addr.uint32,HEX);
  log_debug(" len(0x");
  log_debug(len.uint32,HEX);
  logln_debug(")");
  LED2RED;
  SCSI_PROT_DTOUT_direct(addr.uint32, len.uint32);
  LED2off;
  return 0;
}

//
// ********************** SCSI_CMD_Write10
//
byte SCSI_CMD_Write10(byte *p){
union wk addr,len;
  addr.uint32= 0;
  len.uint32 = 0;
  addr.uint8[3]= *(p+2);
  addr.uint8[2]= *(p+3);
  addr.uint8[1]= *(p+4);
  addr.uint8[0]= *(p+5);
  len.uint8[1] = *(p+7);
  len.uint8[0] = *(p+8);
  log_debug("Write10 @ 0x");
  log_debug(addr.uint32,HEX);
  log_debug(" len(0x");
  log_debug(len.uint32,HEX);
  logln_debug(")");
  LED2RED;
  SCSI_PROT_DTOUT_direct(addr.uint32, len.uint32);
  LED2off;
  return 0;
}

//
// ********************** SCSI_CMD_ModeSense
//
void SCSI_CMD_ModeSense(byte *p){
byte pcd;
uint32_t len;
uint32_t i;
union wk cnv;
  pcd= *(p+2) & 0x3f;          // pagecode
  len= ( 0x1a != *p ) ? (uint32_t)((*(p+7) << 8) | *(p+8)) : (uint32_t)(*(p+4)) ; // allocation length
  memset(BLK_BUF, 0, sizeof(BLK_BUF));
  i = 4;
  if( 0 == (*(p+1) & 0x08) ) { // DBD is cleared?
    BLK_BUF[3] = 0x08;
    BLK_BUF[i++]= (byte)(FIL[FIL_IDX].BLOCK_CNT.uint8[3]);
    BLK_BUF[i++]= (byte)(FIL[FIL_IDX].BLOCK_CNT.uint8[2]);
    BLK_BUF[i++]= (byte)(FIL[FIL_IDX].BLOCK_CNT.uint8[1]);
    BLK_BUF[i++]= (byte)(FIL[FIL_IDX].BLOCK_CNT.uint8[0]);
    BLK_BUF[i++]= (byte)(FIL[FIL_IDX].BLOCK_SIZ.uint8[3]);
    BLK_BUF[i++]= (byte)(FIL[FIL_IDX].BLOCK_SIZ.uint8[2]);
    BLK_BUF[i++]= (byte)(FIL[FIL_IDX].BLOCK_SIZ.uint8[1]);
    BLK_BUF[i++]= (byte)(FIL[FIL_IDX].BLOCK_SIZ.uint8[0]);
  };
  switch(pcd) { // support for SES-3 standard
    case 0x03:  // Format Parameters page (03h)
      BLK_BUF[i+0] = 0x03; // page code
      BLK_BUF[i+1] = 0x16; // 
      BLK_BUF[i+11] = SECperTRK;   // SECTORS PER TRACK
      BLK_BUF[i+12] = FIL[FIL_IDX].BLOCK_SIZ.uint8[1];   // DATA BYTES PER PHYSICAL SECTOR(MSB)
      BLK_BUF[i+13] = FIL[FIL_IDX].BLOCK_SIZ.uint8[0];   // DATA BYTES PER PHYSICAL SECTOR(LSB)
      i += 24;  // Format Parameters page length = 24
      break;
    case 0x04:  // Rigid Drive Geometry Parameters page (04h)
      cnv.uint32 = FIL[FIL_IDX].BLOCK_CNT.uint32 / TRKperCYL / SECperTRK;
      BLK_BUF[i+0] = 0x04;
      BLK_BUF[i+1] = 0x16;
      BLK_BUF[i+2] = cnv.uint8[2];          // NUMBER OF CYLINDERS (MSB)
      BLK_BUF[i+3] = cnv.uint8[1];
      BLK_BUF[i+4] = cnv.uint8[0];          // NUMBER OF CYLINDERS (LSB)
      BLK_BUF[i+5] = TRKperCYL;             // NUMBER OF HEADS
      i += 24;  // Rigid Drive Geometry Parameters page length = 24
      break;
    default:
      break;
  }
  BLK_BUF[0] = i - 1;
  SCSI_PROT_DTIN_phase(len < i ? len : i, BLK_BUF);
}

