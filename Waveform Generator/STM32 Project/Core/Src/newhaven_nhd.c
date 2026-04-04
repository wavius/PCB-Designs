#include "newhaven_nhd.h"

void data_write(uint8_t d) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);

    for (uint8_t n = 0; n < 8; n++) {
        HAL_GPIO_WritePin(SI_GPIO_Port, SI_Pin, (d & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        d <<= 1;
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void comm_write(uint8_t d) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);

    for (uint8_t n = 0; n < 8; n++) {
        HAL_GPIO_WritePin(SI_GPIO_Port, SI_Pin, (d & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        d <<= 1;
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SC_GPIO_Port, SC_Pin, GPIO_PIN_RESET);
    }

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void init_LCD(void) {
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
    comm_write(0xA0); // ADC select
    comm_write(0xAE); // Display OFF
    comm_write(0xC8); // COM direction scan
    comm_write(0xA2); // LCD bias set
    comm_write(0x2F); // Power Control set
    comm_write(0x21); // Resistor Ratio Set
    comm_write(0x81); // Electronic Volume Command (contrast)
    comm_write(0x20); // Volume value
    comm_write(0xAF); // Display ON
}

void ClearLCD(void) {
    for (uint8_t page = 0xB0; page < 0xB4; page++) {
        comm_write(page);
        comm_write(0x10);
        comm_write(0x00);
        for (uint8_t col = 0; col < 128; col++)
            data_write(0x00);
    }
    comm_write(0xAF);
}

void DispPic(const uint8_t *lcd_string) {
    for (uint8_t page = 0xB0; page < 0xB4; page++) {
        comm_write(page);
        comm_write(0x10);
        comm_write(0x00);
        for (uint8_t col = 0; col < 128; col++)
            data_write(*lcd_string++);
    }
    comm_write(0xAF);
    // HAL_Delay(100);
}

void PicLoop(void) {
	for (int i = 0; i < 106; i++) {
        DispPic(frames[i]);
        HAL_Delay(25);
    }
}
