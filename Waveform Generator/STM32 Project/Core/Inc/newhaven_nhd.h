#ifndef __NEWHAVEN_NHD_H
#define __NEWHAVEN_NHD_H

void data_write(uint8_t d);
void comm_write(uint8_t d);
void init_LCD(void);
void ClearLCD(void);
void DispPic(const uint8_t *lcd_string);
void PicLoop(void);


#endif /*__NEWHAVEN_NHD_H*/
