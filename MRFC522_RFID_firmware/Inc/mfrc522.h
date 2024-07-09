#ifndef __MFRC522_H__
#define __MFRC522_H__

#include"stm32f4xx.h"
#include"spi.h"

#define PB0_CS_PIN			(1U<<0)
#define PA8_RST_PIN			(1U<<8)

#define MFRC522_CS_LOW					GPIOB->ODR &=~PB0_CS_PIN
#define MFRC522_CS_HIGH					GPIOB->ODR |=PB0_CS_PIN

typedef enum{
	MI_OK =0,
	MI_NOTAGERR, //No Tag Error
	MI_ERR
}status_t;


/* Mifare_One card command word */
#define PICC_REQIDL						0x26
#define PICC_ANTICOLL					0x93
#define PICC_SELECTTAG					0x93
#define PICC_AUTHENT1A					0x60
#define PICC_AUTHENT1B					0x61
#define PICC_READ						0x30
#define PICC_WRITE						0xA0
#define PICC_HALT						0x50


/* MFRC522 Commands */
#define PCD_IDLE						0x00
#define PCD_AUTHENT						0x0E
#define PCD_TRANSCEIVE					0x0C
#define PCD_RESETPHASE					0x0F
#define PCD_CALCCRC						0x03

/* MFRC522 Registers */
//Page 0: Command and Status
#define MFRC522_REG_COMMAND				0x01
#define MFRC522_REG_COMM_IE_N			0x02
#define MFRC522_REG_COMM_IRQ			0x04
#define MFRC522_REG_DIV_IRQ				0x05
#define MFRC522_REG_ERROR				0x06
#define MFRC522_REG_STATUS2				0x08
#define MFRC522_REG_FIFO_DATA			0x09
#define MFRC522_REG_FIFO_LEVEL			0x0A
#define MFRC522_REG_CONTROL				0x0C
#define MFRC522_REG_BIT_FRAMING			0x0D

//Page 1: Command
#define MFRC522_REG_MODE				0x11
#define MFRC522_REG_TX_CONTROL			0x14
#define MFRC522_REG_TX_AUTO				0x15

//Page 2: CFG
#define MFRC522_REG_CRC_RESULT_M		0x21
#define MFRC522_REG_CRC_RESULT_L		0x22
#define MFRC522_REG_RF_CFG				0x26
#define MFRC522_REG_T_MODE				0x2A
#define MFRC522_REG_T_PRESCALER			0x2B
#define MFRC522_REG_T_RELOAD_H			0x2C
#define MFRC522_REG_T_RELOAD_L			0x2D
#define MFRC522_MAX_LEN					16


#define CARD_ID_LEN						5


/**
 * @brief Initializes the MFRC522 RFID reader module.
 */
void mfrc522_init(void);

/**
 * @brief Checks for the presence of an RFID card and reads its ID.
 *
 * @param id Pointer to store the read card ID.
 * @return status_t MI_OK if an RFID card is detected, other status codes for different scenarios.
 */
status_t mfrc522_check(uint8_t* id);

/**
 * @brief Compares two RFID card IDs.
 *
 * @param current_card_id Pointer to the ID of the detected card.
 * @param compare_id Pointer to the ID to compare against.
 * @return status_t MI_OK if both IDs match, MI_ERR if not.
 */
status_t mfrc522_compare(uint8_t* current_card_id, uint8_t* compare_id);

/**
 * @brief Initializes the GPIO pins used by the MFRC522 module.
 */
void mfrc522_init_pins(void);

/**
 * @brief Writes a value to a register in the MFRC522.
 *
 * @param addr Address of the register.
 * @param val Value to be written.
 */
void mfrc522_write_register(uint8_t addr, uint8_t val);

/**
 * @brief Reads a value from a register in the MFRC522.
 *
 * @param addr Address of the register.
 * @return uint8_t Value read from the register.
 */
uint8_t mfrc522_read_register(uint8_t addr);

/**
 * @brief Sets specific bits in a register.
 *
 * @param reg Register to modify.
 * @param mask Bitmask specifying bits to set.
 */
void mfrc522_set_bit_mask(uint8_t reg, uint8_t mask);

/**
 * @brief Clears specific bits in a register.
 *
 * @param reg Register to modify.
 * @param mask Bitmask specifying bits to clear.
 */
void mfrc522_clear_bit_mask(uint8_t reg, uint8_t mask);

/**
 * @brief Turns on the antenna to enable RFID communication.
 */
void mfrc522_antenna_on(void);

/**
 * @brief Turns off the antenna.
 */
void mfrc522_antenna_off(void);

/**
 * @brief Resets the MFRC522 module.
 */
void mfrc522_reset(void);

/**
 * @brief Sends a request to the RFID card.
 *
 * @param req_mode Request mode.
 * @param tag_type Pointer to store the tag type.
 * @return status_t Result of the operation.
 */
status_t mfrc522_request(uint8_t req_mode, uint8_t* tag_type);

/**
 * @brief Communicates with the RFID card.
 *
 * @param command Command to send.
 * @param send_data Data to be sent.
 * @param send_len Length of the data to be sent.
 * @param back_data Buffer to store the response.
 * @param back_len Pointer to store the length of the response.
 * @return status_t Result of the communication.
 */
status_t mfrc522_to_card(uint8_t command, uint8_t* send_data, uint8_t send_len, uint8_t* back_data, uint16_t* back_len);

/**
 * @brief Detects collision and selects a single RFID card.
 *
 * @param ser_num Serial number of the card.
 * @return status_t Result of the anti-collision check.
 */
status_t mfrc522_anticoll(uint8_t* ser_num);

/**
 * @brief Calculates the CRC of the data.
 *
 * @param p_in_data Pointer to the data.
 * @param len Length of the data.
 * @param p_out_data Pointer to store the calculated CRC.
 */
void mfrc522_calculate_crc(uint8_t* p_in_data, uint8_t len, uint8_t* p_out_data);

/**
 * @brief Selects an RFID card for communication.
 *
 * @param ser_num Serial number of the card.
 * @return uint8_t Size of the response.
 */
uint8_t mfrc522_select_tag(uint8_t* ser_num);

/**
 * @brief Authenticates a sector of the RFID card.
 *
 * @param auth_mode Authentication mode.
 * @param block_addr Block address.
 * @param sector_key Key for the sector.
 * @param ser_num Serial number of the card.
 * @return status_t Result of the authentication.
 */
status_t mfrc522_auth(uint8_t auth_mode, uint8_t block_addr, uint8_t* sector_key, uint8_t* ser_num);

/**
 * @brief Reads data from a block of the RFID card.
 *
 * @param blockAddr Address of the block.
 * @param recvData Buffer to store the received data.
 * @return status_t Result of the read operation.
 */
status_t mfrc522_read(uint8_t blockAddr, uint8_t* recvData);

/**
 * @brief Writes data to a block of the RFID card.
 *
 * @param blockAddr Address of the block.
 * @param writeData Data to be written.
 * @return status_t Result of the write operation.
 */
status_t mfrc522_write(uint8_t blockAddr, uint8_t* writeData);

/**
 * @brief Puts the RFID card in a halt state.
 */
void mfrc522_halt(void);

#endif


