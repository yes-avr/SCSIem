//
// a part of SxSI direct access device emulator
//

void test_func(void) {
char CHR;
uint32_t i;
	while( debugin.available() ){
		BUZon;
		log_debug("[");
		CHR=debugin.read();
		log_debug(CHR);
		log_debug("] ");
		switch(CHR){
		  case('C'):
			CNT= DELAY_CNT;
			SCSI_PHASE_CMD;
			logln_debug("CMD phase");
			break;;
		  case('I'):
			CNT= DELAY_CNT;
			SCSI_PHASE_DTIN;
			logln_debug("DTIN phase");
			break;;
		  case('M'):
			CNT= DELAY_CNT;
			SCSI_PHASE_MSGIN;
			logln_debug("MSGIN phase");
			break;;
		  case('N'):
			CNT= DELAY_CNT;
			SCSI_BUS_REQN;
			logln_debug("REQ Negate");
			break;;
		  case('O'):
			CNT= DELAY_CNT;
			SCSI_PHASE_DTOUT;
			logln_debug("DTOUT phase");
			break;;
		  case('R'):
			CNT= DELAY_CNT;
			SCSI_BUS_REQ;
			logln_debug("REQ assert");
			break;;
		  case('S'):
			CNT= DELAY_CNT;
			SCSI_PHASE_STAT;
			logln_debug("STAT phase");
			break;;
		  case('U'):
			CNT= DELAY_CNT;
			SCSI_PHASE_MSGOUT;
			logln_debug("MSGOUT phase");
			break;;
		  case('a'):
			CNT= DELAY_CNT;
			log_debug("ACK is ");
			logln_debug(isSCSI_BUS_ACK);
			break;;
		  case('b'):
			CNT= DELAY_CNT;
			log_debug("BUSY is ");
			logln_debug(isSCSI_BUS_BSY);
			break;;
		  case('r'):
			CNT= DELAY_CNT;
			log_debug("RST is ");
			logln_debug(isSCSI_BUS_RST);
			break;;
		  case('s'):
			CNT= DELAY_CNT;
			log_debug("SEL is ");
			logln_debug(isSCSI_BUS_SEL);
			break;;
		  case('t'):
			CNT= DELAY_CNT;
			log_debug("ATN is ");
			logln_debug(isSCSI_BUS_ATN);
			break;;
		  case('x'):
			logln_debug("\n\rGPIOA:");
			log_debug("\t.CRH ");
			log_debug(GPIOA->regs->CRH,HEX);
			log_debug("\t.CRL ");
			log_debug(GPIOA->regs->CRL,HEX);
			log_debug("\t.ODR ");
			log_debug(GPIOA->regs->ODR,HEX);
			log_debug("\t.IDR ");
			log_debug(GPIOA->regs->IDR,HEX);
			logln_debug("\n\rGPIOB:");
			log_debug("\t.CRH ");
			log_debug(GPIOB->regs->CRH,HEX);
			log_debug("\t.CRL ");
			log_debug(GPIOB->regs->CRL,HEX);
			log_debug("\t.ODR ");
			log_debug(GPIOB->regs->ODR,HEX);
			log_debug("\t.IDR ");
			logln_debug(GPIOB->regs->IDR,HEX);
			break;;
		  case('z'):
			log_debug(" SCSI_ID map: 0b");
			for(i=0;i<8;i++){
				log_debug(((SCSI_id >> (7-i)) & 1));
			};
			logln_debug("\r\nHD Image file(s) info:");
			for(i=0;i<8;i++){
				if(SCSI_id & (1<<i)){
					log_debug("ID=");
					log_debug(i);
					log_debug(" FILE='");
					log_debug(FIL[i].HDIMG_FILE_NAME);
					log_debug("' SIZE=");
					log_debug(FIL[i].FILE_SIZE);
					log_debug(" BLKS=");
					log_debug(FIL[i].BLOCK_CNT.uint32);
					log_debug(" BLKSIZE=");
					logln_debug(FIL[i].BLOCK_SIZ.uint32);
				};
			};
			break;;
		  case('+'):
			if (DELAY_CNT < 64000000 )DELAY_CNT *= 2;
			log_debug("delay=");
			logln_debug(DELAY_CNT);
			break;;
		  case('-'):
			if (DELAY_CNT > 62500 )DELAY_CNT /= 2;
			log_debug("delay=");
			logln_debug(DELAY_CNT);
			break;;
		  case('='):
			DELAY_CNT = 1000000;
			log_debug("\n\rdelay=");
			log_debug(DELAY_CNT);
			log_debug(" / CNT=");
			logln_debug(CNT);
			break;;
		  case('0'):
		  case('1'):
		  case('2'):
		  case('3'):
		  case('4'):
		  case('5'):
		  case('6'):
		  case('7'):
			CNT= DELAY_CNT;
			SCSI_BUS_put_byte(1<<(CHR-'0'));
			break;;
		  case('8'):
			CNT= DELAY_CNT;
			SCSI_BUS_put_byte(0);
			break;;
		  case('9'):
			CNT= DELAY_CNT;
			log_debug(SCSI_BUS_get_byte(),HEX);
			break;;
		  default:
			break;;
		};
		delay(30);
		BUZoff;
	};
}

