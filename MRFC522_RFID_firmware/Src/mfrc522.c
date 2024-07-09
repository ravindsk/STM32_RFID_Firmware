#include"mfrc522.h"

#define GPIOAEN (1U<<0)
#define GPIOBEN (1U<<1)
/*****RC522 PIN
 *
 * VCC   :3V3
 * RST   :PA8(D7)
 * GND   :GND
 * MOSI  :PA7(D11)
 * MISO  :PA6(D12)
 * SCK   :PA5(D13)
 * CS    :PB0(A3)
 * IRQ   : unconnected
 *
 */

/*Configure PA8 as output pin for RST*/
static void rst_pin_init(void)
{
	/*Enable clock access to gpioa*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA8 as output pin*/
	GPIOA->MODER |=(1U<<16);
	GPIOA->MODER &=~(1U<<17);
}

static void cs_pin_init(void)
{
	/*Enable clock access to gpioa*/
	RCC->AHB1ENR |= GPIOBEN
;

	/*Set PB0 as output pin*/
	GPIOB->MODER |=(1U<<0);
	GPIOB->MODER &=~(1U<<1);
}

void pseudo_dly(int dly)
{
	for(int i= 0;i < dly;i++){}
}

void mfrc522_init_pins(void) {

	/*Initialize spi*/
	spi_init();

	//Init CS Pin
	cs_pin_init();

	//Init RST Pin
	rst_pin_init();

	MFRC522_CS_HIGH;

	/*Set RST pin LOW*/
	GPIOA->ODR &=~PA8_RST_PIN;
	pseudo_dly(100000);

	/*Set RST pin HIGH*/
	GPIOA->ODR |=PA8_RST_PIN;
	pseudo_dly(100000);
}

void mfrc522_write_register(uint8_t addr, uint8_t val) {

	//CS low
	MFRC522_CS_LOW;
	uint8_t tmp_addr[1];
	uint8_t tmp_val[1];

	//Prepare address
	tmp_addr[0] = (addr << 1) & 0x7E;
	tmp_val[0]  = val;

	//Send address
	spi_transmit(tmp_addr,1);

	//Send data
	spi_transmit(tmp_val,1);

	//CS high
	MFRC522_CS_HIGH;


	/****Understanding above command
		 *
		 * To write a command, the LSB needs to be 0, MSB needs to be 0,
		 *  and bits 1 to 6 represent actual register address
		 *
		 * reg<<1 shifts it by one ensuring that LSB is 0, indicating write
		 * 0x7E binary is : (0111 1110)
		 * masking it with 0x7E ensures that LSB and MSB is always 0
		 *
		 * from & operator truth table it will only be 1 when both both are 1
		 *
		 * Example:
		 * reg     = 0x1A (0001 1010)
		 * 0x1A<<1 = 0x34 (0011 0100) (LSB is 0)
		 *
		 * (0x1A<<1) & 0x7E is = (0011 0100) &
		 * 						 (0111 1110)
		 * 						 ------------
		 * 						 (0011 0100) which is basically 0x34
		 * 	the & operator just check if MSB and LSB are 0 and bits 1-6 are used
		 * 	 (kind of a comparator)
		 */
}

uint8_t mfrc522_read_register(uint8_t addr) {

	//CS low
	MFRC522_CS_LOW;

	uint8_t rcvd_data = 0;

	uint8_t tmp_addr[1];

	tmp_addr[0] = (((addr << 1) & 0x7E) | 0x80);

	spi_transmit(tmp_addr,1);

	rcvd_data = spi_receive_byte();


	//CS high
	MFRC522_CS_HIGH;

	return rcvd_data;

	/****Understanding above command
		 *
		 * To read a command, the LSB needs to be 0, MSB needs to be 1,
		 *  and bits 1 to 6 represent actual register address
		 *
		 * reg<<1 shifts it by one ensuring that LSB is 0, indicating write
		 * 0x7E binary is : (0111 1110)
		 * 0x80 binary is : (1000 0000)
		 *
		 * masking it with 0x7E ensures that LSB and MSB is always 0,
		 * bits 1-6 hold the actual address
		 *
		 * The or operator sets MSB as 1
		 *
		 * from & operator truth table it will only be 1 when both both are 1
		 * from 1 operator truth table it will only be 0 when both both are 0
		 *
		 * Example:
		 * reg     = 0x1A (0001 1010)
		 * 0x1A<<1 = 0x34 (0011 0100) (LSB is 0)
		 *
		 * (0x1A<<1) & 0x7E is = (0011 0100) &
		 * 						 (0111 1110)
		 * 						 ------------
		 * 						 (0011 0100) |
	 	 	 	 	 	 	 	 (1000 0000)
	 	 	 	 	 	 	 	 ------------
	 	 	 	 	 	 	 	 (1011 0100) //MSB is set to 1
		 * 	the & operator just check if MSB and LSB are 0 and bits 1-6 are used
		 * 	 (kind of a comparator)
		 */
}

void mfrc522_set_bit_mask(uint8_t reg, uint8_t mask) {

    // Set specific bits in the register
    mfrc522_write_register(reg, mfrc522_read_register(reg) | mask);
}

void mfrc522_clear_bit_mask(uint8_t reg, uint8_t mask){

    // Clear specific bits in the register
    mfrc522_write_register(reg, mfrc522_read_register(reg) & (~mask));
}


void mfrc522_init(void) {

    // Initialize pins for MFRC522
    mfrc522_init_pins();

    // Reset the MFRC522 module
    mfrc522_reset();

    //Set internal timer to Auto Mode and
    //configure higher 4 bits of prescaler
    mfrc522_write_register(MFRC522_REG_T_MODE, 0x8D);

    // Configure the  lower 4 bits of prescaler
    mfrc522_write_register(MFRC522_REG_T_PRESCALER, 0x3E);

    // Configure the  Reload Low register
    mfrc522_write_register(MFRC522_REG_T_RELOAD_L, 30);

    // Configure the Reload High register
    mfrc522_write_register(MFRC522_REG_T_RELOAD_H, 0);

    // Set RF configuration for 48dB gain
    mfrc522_write_register(MFRC522_REG_RF_CFG, 0x70);

    // Configure the TX Auto register: Controls transmit modulation settings.
    mfrc522_write_register(MFRC522_REG_TX_AUTO, 0x40);

    // Configure general mode for transmitting and receiving
    // CRCPreset, TxWaitRF
    mfrc522_write_register(MFRC522_REG_MODE, 0x3D);

    // Turn on the antenna
    mfrc522_antenna_on();
}

void mfrc522_antenna_on(void) {
    uint8_t temp;

    // Read the current value of the TX Control register
    temp = mfrc522_read_register(MFRC522_REG_TX_CONTROL);

    // Enable the antenna if not already enabled
    if (!(temp & 0x03)) {
        mfrc522_set_bit_mask(MFRC522_REG_TX_CONTROL, 0x03);
    }
}

void mfrc522_antenna_off(void) {
	mfrc522_clear_bit_mask(MFRC522_REG_TX_CONTROL, 0x03);
}

void mfrc522_reset(void) {
	mfrc522_write_register(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}

status_t mfrc522_to_card(uint8_t command,
		                 uint8_t* send_data,
						 uint8_t send_len,
						 uint8_t* back_data,
						 uint16_t* back_len)
{
    status_t status = MI_ERR;
    uint8_t irq_en = 0x00;
    uint8_t wait_irq = 0x00;
    uint8_t last_bits;
    uint8_t n;
    uint16_t i;

    // Configure interrupt request depending on the command
    switch (command) {
        case PCD_AUTHENT: {
            irq_en = 0x12;  /*Value for enabling AUTH relevant interrupts*/
            wait_irq = 0x10;
            break;
        }
        case PCD_TRANSCEIVE: {
            irq_en = 0x77;  /*Value for enabling TRANSCEIVE relevant interrupts*/
            wait_irq = 0x30;
            break;
        }
        default:
            break;
    }

    // Set up interrupts and FIFO
    mfrc522_write_register(MFRC522_REG_COMM_IE_N, irq_en | 0x80);
    mfrc522_clear_bit_mask(MFRC522_REG_COMM_IRQ, 0x80);

    //Clear FIFO's read and write pointers
    mfrc522_set_bit_mask(MFRC522_REG_FIFO_LEVEL, 0x80);

    // Prepare to execute the command
    mfrc522_write_register(MFRC522_REG_COMMAND, PCD_IDLE);

    // Writing data to the FIFO
    for (i = 0; i < send_len; i++) {
        mfrc522_write_register(MFRC522_REG_FIFO_DATA, send_data[i]);
    }

    // Execute the command
    mfrc522_write_register(MFRC522_REG_COMMAND, command);
    if (command == PCD_TRANSCEIVE) {
        mfrc522_set_bit_mask(MFRC522_REG_BIT_FRAMING, 0x80);
    }

    // Wait for data reception to complete
    i = 2500;
    do {
        n = mfrc522_read_register(MFRC522_REG_COMM_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & wait_irq));

    mfrc522_clear_bit_mask(MFRC522_REG_BIT_FRAMING, 0x80);

    if (i != 0) {
        if (!(mfrc522_read_register(MFRC522_REG_ERROR) & 0x1B)) {
            status = MI_OK;
            if (n & irq_en & 0x01) {
                status = MI_NOTAGERR;
            }

            if (command == PCD_TRANSCEIVE) {
                n = mfrc522_read_register(MFRC522_REG_FIFO_LEVEL);
                last_bits = mfrc522_read_register(MFRC522_REG_CONTROL) & 0x07;
                *back_len = last_bits ? (n - 1) * 8 + last_bits : n * 8;

                n = (n == 0) ? 1 : (n > MFRC522_MAX_LEN ? MFRC522_MAX_LEN : n);

                // Read the received data from FIFO
                for (i = 0; i < n; i++) {
                    back_data[i] = mfrc522_read_register(MFRC522_REG_FIFO_DATA);
                }
            }
        } else {
            status = MI_ERR;
        }
    }

    return status;
}


status_t mfrc522_anticoll(uint8_t* ser_num) {
    status_t status;
    uint8_t ser_num_check = 0;
    uint16_t un_len;

    // Set the Bit Framing register for anti-collision
    mfrc522_write_register(MFRC522_REG_BIT_FRAMING, 0x00);

    // Prepare buffer for ANTI-COLLISION command
    ser_num[0] = PICC_ANTICOLL;
    ser_num[1] = 0x20;
    // Send ANTI-COLLISION command to the card
    status = mfrc522_to_card(PCD_TRANSCEIVE, ser_num, 2, ser_num, &un_len);

    if (status == MI_OK) {
        // Check card serial number
        for (uint8_t i = 0; i < 4; i++) {
            ser_num_check ^= ser_num[i];
        }
        if (ser_num_check != ser_num[4]) {
            return MI_ERR;
        }
    }
    return status;
}

void mfrc522_calculate_crc(uint8_t* p_in_data, uint8_t len, uint8_t* p_out_data) {
    uint8_t i, n;

    mfrc522_clear_bit_mask(MFRC522_REG_DIV_IRQ, 0x04);
    mfrc522_set_bit_mask(MFRC522_REG_FIFO_LEVEL, 0x80);

    // Writing data to the FIFO
    for (i = 0; i < len; i++) {
        mfrc522_write_register(MFRC522_REG_FIFO_DATA, p_in_data[i]);
    }
    // Initiate CRC calculation
    mfrc522_write_register(MFRC522_REG_COMMAND, PCD_CALCCRC);

    // Wait for CRC calculation to complete
    i = 0xFF;
    do {
        n = mfrc522_read_register(MFRC522_REG_DIV_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x04));

    // Read CRC calculation result
    p_out_data[0] = mfrc522_read_register(MFRC522_REG_CRC_RESULT_L);
    p_out_data[1] = mfrc522_read_register(MFRC522_REG_CRC_RESULT_M);
}

uint8_t mfrc522_select_tag(uint8_t* ser_num) {
    uint8_t i;
    status_t status;
    uint8_t size;
    uint16_t recv_bits;
    uint8_t buffer[9];

    // Prepare buffer for SELECT command
    buffer[0] = PICC_SELECTTAG;
    buffer[1] = 0x70;
    for (i = 0; i < 5; i++) {
        buffer[i + 2] = ser_num[i];
    }
    // Calculate CRC for the command
    mfrc522_calculate_crc(buffer, 7, &buffer[7]);
    // Send command to the card
    status = mfrc522_to_card(PCD_TRANSCEIVE, buffer, 9, buffer, &recv_bits);

    // Check if the operation is successful
    size = (status == MI_OK && recv_bits == 0x18) ? buffer[0] : 0;
    return size;
}

status_t mfrc522_auth(uint8_t auth_mode,
		              uint8_t block_addr,
					  uint8_t* sector_key,
					  uint8_t* ser_num)
{
    status_t status;
    uint16_t recv_bits;
    uint8_t buff[12];

    // Prepare buffer for AUTH command
    buff[0] = auth_mode;
    buff[1] = block_addr;
    for (uint8_t i = 0; i < 6; i++) {
        buff[i + 2] = sector_key[i];
    }
    for (uint8_t i = 0; i < 5; i++) {
        buff[i + 8] = ser_num[i];
    }
    // Send AUTH command to the card
    status = mfrc522_to_card(PCD_AUTHENT, buff, 12, buff, &recv_bits);

    // Check if the operation is successful
    if ((status != MI_OK) || (!(mfrc522_read_register(MFRC522_REG_STATUS2) & 0x08))) {
        status = MI_ERR;
    }

    return status;
}


status_t mfrc522_read(uint8_t block_addr, uint8_t* recv_data) {
    status_t status;
    uint16_t un_len;

    // Prepare buffer for READ command
    recv_data[0] = PICC_READ;
    recv_data[1] = block_addr;

    // Calculate CRC for the command
    mfrc522_calculate_crc(recv_data, 2, &recv_data[2]);

    // Send READ command to the card and receive the data
    status = mfrc522_to_card(PCD_TRANSCEIVE, recv_data, 4, recv_data, &un_len);

    // Check if the operation is successful and the received data length is correct
    if ((status != MI_OK) || (un_len != 0x90)) {
        return MI_ERR;
    }

    return status;
}

status_t mfrc522_write(uint8_t block_addr, uint8_t* write_data) {
    status_t status;
    uint16_t recv_bits;
    uint8_t buff[18];

    // Prepare buffer for WRITE command
    buff[0] = PICC_WRITE;
    buff[1] = block_addr;

    // Calculate CRC for the command
    mfrc522_calculate_crc(buff, 2, &buff[2]);

    // Send command to the card
    status = mfrc522_to_card(PCD_TRANSCEIVE, buff, 4, buff, &recv_bits);

    // Check if the operation is successful
    if ((status != MI_OK) || (recv_bits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
        return MI_ERR;
    }

    if (status == MI_OK) {

        // Write data to the temp buff
        for (uint8_t i = 0; i < 16; i++) {
            buff[i] = write_data[i];
        }
        // Calculate CRC for the data
        mfrc522_calculate_crc(buff, 16, &buff[16]);

        // Send data to the card
        status = mfrc522_to_card(PCD_TRANSCEIVE, buff, 18, buff, &recv_bits);

        if ((status != MI_OK) || (recv_bits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
            return MI_ERR;
        }
    }

    return status;
}


void mfrc522_halt(void) {
    uint16_t un_len;
    uint8_t buff[4];

    // Prepare buffer for HALT command
    buff[0] = PICC_HALT;
    buff[1] = 0;

    // Calculate CRC for the command
    mfrc522_calculate_crc(buff, 2, &buff[2]);

    // Send HALT command to the card
    mfrc522_to_card(PCD_TRANSCEIVE, buff, 4, buff, &un_len);
}

status_t mfrc522_compare(uint8_t* current_card_id, uint8_t* compare_id){
    for (uint8_t i = 0; i < 5; i++) {
        if (current_card_id[i] != compare_id[i]) {
            return MI_ERR;
        }
    }
    // If all bytes match, return OK
    return MI_OK;
}

status_t mfrc522_request(uint8_t req_mode, uint8_t* tag_type) {
    status_t status;
    uint16_t back_bits;  // The received data bits

    // Set the Bit Framing register for transmitting the last bits
    mfrc522_write_register(MFRC522_REG_BIT_FRAMING, 0x07);

    // Set request mode
    tag_type[0] = req_mode;

    // Send data to the card and receive the response
    status = mfrc522_to_card(PCD_TRANSCEIVE, tag_type, 1, tag_type, &back_bits);

    // Check if the operation is successful and the received bits are as expected
    if ((status != MI_OK) || (back_bits != 0x10)) {
        status = MI_ERR;
    }

    return status;
}

status_t mfrc522_check(uint8_t* id) {
	status_t status;

    // Attempt to find cards and return the card type
    status = mfrc522_request(PICC_REQIDL, id);

    if (status == MI_OK) {
        // If a card is detected, perform anti-collision to get the card's serial number
        status = mfrc522_anticoll(id);
    }

    mfrc522_halt();

    return status;
}
