/*
 * MFRC522.cpp - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
 * NOTE: Please also check the comments in MFRC522.h - they provide useful hints and background information.
 * Released into the public domain.
 */

#include "MFRC522.h"
#include "bcm2835.h"
#include <linux/types.h>
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <string>

#define RSTPIN RPI_V2_GPIO_P1_22

using namespace std;

/**
 * Constructor.
 * Prepares the output pins.
 */
MFRC522::MFRC522(){
  
  if (!bcm2835_init()) {
    printf("Failed to initialize. This tool needs root access, use sudo.\n");
  }
  bcm2835_gpio_fsel(RSTPIN, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(RSTPIN, LOW);
  // Set SPI bus to work with MFRC522 chip.
  setSPIConfig();
} // End constructor

/**
 * Set SPI bus to work with MFRC522 chip.
 * Please call this function if you have changed the SPI config since the MFRC522 constructor was run.
 */
void MFRC522::setSPIConfig() {
  
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);    // ~ 4 MHz
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
	
} // End setSPIConfig()

/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_WriteRegister(	byte reg,		///< The register to write to. One of the PCD_Register enums.
					byte value		///< The value to write.
					) {

  char data[2];
  data[0] = reg & 0x7E;
  data[1] = value;
  bcm2835_spi_transfern(data, 2);
  
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_WriteRegister(	byte reg,		///< The register to write to. One of the PCD_Register enums.
					byte count,		///< The number of bytes to write to the register
					byte *values	///< The values to write. Byte array.
					) {
  for (byte index = 0; index < count; index++) {
  	PCD_WriteRegister(reg, values[index]);
	}

} // End PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
byte MFRC522::PCD_ReadRegister(	byte reg	///< The register to read from. One of the PCD_Register enums.
				) {
  
  char data[2];
  data[0] = 0x80 | ((reg) & 0x7E);
  bcm2835_spi_transfern(data,2);
  return (byte)data[1];
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522::PCD_ReadRegister(	byte reg,		///< The register to read from. One of the PCD_Register enums.
				byte count,		///< The number of bytes to read
				byte *values,	///< Byte array to store the values in.
				byte rxAlign	///< Only bit positions rxAlign..7 in values[0] are updated.
				) {
  if (count == 0) {
    return;
  }
  //Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
  byte address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  byte index = 0;							// Index in values array.
  count--;								// One read is performed outside of the loop
  bcm2835_spi_transfer(address);
  while (index < count) {
    if (index == 0 && rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
      // Create bit mask for bit positions rxAlign..7
      byte mask = 0;
      for (byte i = rxAlign; i <= 7; i++) {
	mask |= (1 << i);
      }
      // Read value and tell that we want to read the same address again.
      byte value = bcm2835_spi_transfer(address);
      // Apply mask to both current value of values[0] and the new data in value.
      values[0] = (values[index] & ~mask) | (value & mask);
    }
    else { // Normal case
      values[index] = bcm2835_spi_transfer(address);
    }
    index++;
  }
  values[index] = bcm2835_spi_transfer(0);			// Read the final byte. Send 0 to stop reading.
} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void MFRC522::PCD_SetRegisterBitMask(	byte reg,	///< The register to update. One of the PCD_Register enums.
					byte mask	///< The bits to set.
					) { 
  byte tmp;
  tmp = PCD_ReadRegister(reg);
  PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void MFRC522::PCD_ClearRegisterBitMask(	byte reg,	///< The register to update. One of the PCD_Register enums.
					byte mask	///< The bits to clear.
					) {
  byte tmp;
  tmp = PCD_ReadRegister(reg);
  PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PCD_CalculateCRC(	byte *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
				byte length,	///< In: The number of bytes to transfer.
				byte *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
				) {
  PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
  PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
  PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
  PCD_WriteRegister(FIFODataReg, length, data);	// Write data to the FIFO
  PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	
  // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.
  word i = 5000;
  byte n;
  while (1) {
    n = PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    if (n & 0x04) {						// CRCIRq bit set - calculation done
      break;
    }
    if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
      return STATUS_TIMEOUT;
    }
  }
  PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.
	
  // Transfer the result from the registers to the result buffer
  result[0] = PCD_ReadRegister(CRCResultRegL);
  result[1] = PCD_ReadRegister(CRCResultRegH);
  return STATUS_OK;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void MFRC522::PCD_Init() {
  if (bcm2835_gpio_lev(RSTPIN) == LOW) {	//The MFRC522 chip is in power down mode.
    bcm2835_gpio_write(RSTPIN, HIGH);		// Exit power down mode. This triggers a hard reset.
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
    delay(50);
  }
  else { // Perform a soft reset
    PCD_Reset();
  }
	
  // When communicating with a PICC we need a timeout if something goes wrong.
  // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
  // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
  PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
  PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25�s.
  PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
  PCD_WriteRegister(TReloadRegL, 0xE8);
	
  PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
  PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
  PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void MFRC522::PCD_Reset() {
  PCD_WriteRegister(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
  // The datasheet does not mention how long the SoftRest command takes to complete.
  // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
  // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74�s. Let us be generous: 50ms.
  delay(50);
  // Wait for the PowerDown bit in CommandReg to be cleared
  while (PCD_ReadRegister(CommandReg) & (1<<4)) {
    // PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
  }
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void MFRC522::PCD_AntennaOn() {
  byte value = PCD_ReadRegister(TxControlReg);
  if ((value & 0x03) != 0x03) {
    PCD_WriteRegister(TxControlReg, value | 0x03);
  }
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void MFRC522::PCD_AntennaOff() {
  PCD_ClearRegisterBitMask(TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 * 
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
byte MFRC522::PCD_GetAntennaGain() {
  return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void MFRC522::PCD_SetAntennaGain(byte mask) {
  if (PCD_GetAntennaGain() != mask) {						// only bother if there is a change
    PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
    PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
  }
} // End PCD_SetAntennaGain()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PCD_TransceiveData(	byte *sendData,		///< Pointer to the data to transfer to the FIFO.
					byte sendLen,		///< Number of bytes to transfer to the FIFO.
					byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
					byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
					byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
					byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
					bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
					) {
  byte waitIRq = 0x30;		// RxIRq and IdleIRq
  return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PCD_CommunicateWithPICC(	byte command,		///< The command to execute. One of the PCD_Command enums.
					byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
					byte *sendData,		///< Pointer to the data to transfer to the FIFO.
					byte sendLen,		///< Number of bytes to transfer to the FIFO.
					byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
					byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
					byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
					byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
					bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
					) {
  byte n, _validBits;
  unsigned int i;
	
  // Prepare values for BitFramingReg
  byte txLastBits = validBits ? *validBits : 0;
  byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
  PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
  PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
  PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
  PCD_WriteRegister(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
  PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
  PCD_WriteRegister(CommandReg, command);				// Execute the command
  if (command == PCD_Transceive) {
    PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
  }
	
  // Wait for the command to complete.
  // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
  // Each iteration of the do-while-loop takes 17.86�s.
  i = 2000;
  while (1) {
    n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    if (n & waitIRq) {					// One of the interrupts that signal success has been set.
      break;
    }
    if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
      return STATUS_TIMEOUT;
    }
    if (--i == 0) {						// The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
      return STATUS_TIMEOUT;
    }
  }
	
  // Stop now if any errors except collisions were detected.
  byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
    return STATUS_ERROR;
  }	

  // If the caller wants data back, get it from the MFRC522.
  if (backData && backLen) {
    n = PCD_ReadRegister(FIFOLevelReg);			// Number of bytes in the FIFO
    if (n > *backLen) {
      return STATUS_NO_ROOM;
    }
    *backLen = n;											// Number of bytes returned
    PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
    _validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
    if (validBits) {
      *validBits = _validBits;
    }
  }
	
  // Tell about collisions
  if (errorRegValue & 0x08) {		// CollErr
    return STATUS_COLLISION;
  }
	
  // Perform CRC_A validation if requested.
  if (backData && backLen && checkCRC) {
    // In this case a MIFARE Classic NAK is not OK.
    if (*backLen == 1 && _validBits == 4) {
      return STATUS_MIFARE_NACK;
    }
    // We need at least the CRC_A value and all 8 bits of the last byte must be received.
    if (*backLen < 2 || _validBits != 0) {
      return STATUS_CRC_WRONG;
    }
    // Verify CRC_A - do our own calculation and store the control in controlBuffer.
    byte controlBuffer[2];
    n = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
    if (n != STATUS_OK) {
      return n;
    }
    if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
      return STATUS_CRC_WRONG;
    }
  }
	
  return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PICC_RequestA(byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
			    byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
			    ) {
  return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PICC_WakeupA(	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
				byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
				) {
  return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
byte MFRC522::PICC_REQA_or_WUPA(	byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
					byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
					byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
					) {
  byte validBits;
  byte status;
	
  if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
    return STATUS_NO_ROOM;
  }
  PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
  validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
  status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
  if (status != STATUS_OK) {
    return status;
  }
  if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  }
  return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MFRC522::PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
				byte validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
				) {
  bool uidComplete;
  bool selectDone;
  bool useCascadeTag;
  byte cascadeLevel = 1;
  byte result;
  byte count;
  byte index;
  byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
  signed char currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
  byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
  byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
  byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
  byte *responseBuffer;
  byte responseLength;
	
  // Description of buffer structure:
  //		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
  //		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
  //		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
  //		Byte 3: UID-data
  //		Byte 4: UID-data
  //		Byte 5: UID-data
  //		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
  //		Byte 7: CRC_A
  //		Byte 8: CRC_A
  // The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
  //
  // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
  //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
  //		========	=============	=====	=====	=====	=====
  //		 4 bytes		1			uid0	uid1	uid2	uid3
  //		 7 bytes		1			CT		uid0	uid1	uid2
  //						2			uid3	uid4	uid5	uid6
  //		10 bytes		1			CT		uid0	uid1	uid2
  //						2			CT		uid3	uid4	uid5
  //						3			uid6	uid7	uid8	uid9
	
  // Sanity checks
  if (validBits > 80) {
    return STATUS_INVALID;
  }
	
  // Prepare MFRC522
  PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
  // Repeat Cascade Level loop until we have a complete UID.
  uidComplete = false;
  while (!uidComplete) {
    // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
    switch (cascadeLevel) {
    case 1:
      buffer[0] = PICC_CMD_SEL_CL1;
      uidIndex = 0;
      useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
      break;
			
    case 2:
      buffer[0] = PICC_CMD_SEL_CL2;
      uidIndex = 3;
      useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
      break;
			
    case 3:
      buffer[0] = PICC_CMD_SEL_CL3;
      uidIndex = 6;
      useCascadeTag = false;						// Never used in CL3.
      break;
			
    default:
      return STATUS_INTERNAL_ERROR;
      break;
    }
		
    // How many UID bits are known in this Cascade Level?
    currentLevelKnownBits = validBits - (8 * uidIndex);
    if (currentLevelKnownBits < 0) {
      currentLevelKnownBits = 0;
    }
    // Copy the known bits from uid->uidByte[] to buffer[]
    index = 2; // destination index in buffer[]
    if (useCascadeTag) {
      buffer[index++] = PICC_CMD_CT;
    }
    byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if (bytesToCopy) {
      byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
      if (bytesToCopy > maxBytes) {
	bytesToCopy = maxBytes;
      }
      for (count = 0; count < bytesToCopy; count++) {
	buffer[index++] = uid->uidByte[uidIndex + count];
      }
    }
    // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
    if (useCascadeTag) {
      currentLevelKnownBits += 8;
    }
		
    // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
    selectDone = false;
    while (!selectDone) {
      // Find out how many bits and bytes to send and receive.
      if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
	//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
	buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
	// Calculate BCC - Block Check Character
	buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
	if (result != STATUS_OK) {
	  return result;
	}
	txLastBits		= 0; // 0 => All 8 bits are valid.
	bufferUsed		= 9;
	// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
	responseBuffer	= &buffer[6];
	responseLength	= 3;
      }
      else { // This is an ANTICOLLISION.
	//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
	txLastBits		= currentLevelKnownBits % 8;
	count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
	index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
	buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
	bufferUsed		= index + (txLastBits ? 1 : 0);
	// Store response in the unused part of buffer
	responseBuffer	= &buffer[index];
	responseLength	= sizeof(buffer) - index;
      }
			
      // Set bit adjustments
      rxAlign = txLastBits;											// Having a seperate variable is overkill. But it makes the next line easier to read.
      PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
      // Transmit the buffer and receive the response.
      result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
      if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
	result = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
	if (result & 0x20) { // CollPosNotValid
	  return STATUS_COLLISION; // Without a valid collision position we cannot continue
	}
	byte collisionPos = result & 0x1F; // Values 0-31, 0 means bit 32.
	if (collisionPos == 0) {
	  collisionPos = 32;
	}
	if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
	  return STATUS_INTERNAL_ERROR;
	}
	// Choose the PICC with the bit set.
	currentLevelKnownBits = collisionPos;
	count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
	index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
	buffer[index]	|= (1 << count);
      }
      else if (result != STATUS_OK) {
	return result;
      }
      else { // STATUS_OK
	if (currentLevelKnownBits >= 32) { // This was a SELECT.
	  selectDone = true; // No more anticollision 
	  // We continue below outside the while.
	}
	else { // This was an ANTICOLLISION.
	  // We now have all 32 bits of the UID in this Cascade Level
	  currentLevelKnownBits = 32;
	  // Run loop again to do the SELECT.
	}
      }
    } // End of while (!selectDone)
		
    // We do not check the CBB - it was constructed by us above.
		
    // Copy the found UID bytes from buffer[] to uid->uidByte[]
    index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
    bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
    for (count = 0; count < bytesToCopy; count++) {
      uid->uidByte[uidIndex + count] = buffer[index++];
    }
		
    // Check response SAK (Select Acknowledge)
    if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
      return STATUS_ERROR;
    }
    // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
    result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
    if (result != STATUS_OK) {
      return result;
    }
    if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
      return STATUS_CRC_WRONG;
    }
    if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
      cascadeLevel++;
    }
    else {
      uidComplete = true;
      uid->sak = responseBuffer[0];
    }
  } // End of while (!uidComplete)
	
  // Set correct uid->size
  uid->size = 3 * cascadeLevel + 1;
	
  return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
byte MFRC522::PICC_HaltA() {
  byte result;
  byte buffer[4];
	
  // Build command buffer
  buffer[0] = PICC_CMD_HLTA;
  buffer[1] = 0;
  // Calculate CRC_A
  result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
  if (result != STATUS_OK) {
    return result;
  }
	
  // Send the command.
  // The standard says:
  //		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
  //		HLTA command, this response shall be interpreted as 'not acknowledge'.
  // We interpret that this way: Only STATUS_TIMEOUT is an success.
  result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0);
  if (result == STATUS_TIMEOUT) {
    return STATUS_OK;
  }
  if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
    return STATUS_ERROR;
  }
  return result;
} // End PICC_HaltA()
