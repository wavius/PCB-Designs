#ifndef __AD9833_DDS_H
#define __AD9833_DDS_H

#define SINE     1
#define SQUARE 	 2
#define TRIANGLE 3
#define SAWTOOTH 4

void GetWave(uint8_t wave_type);
void SetWave(uint8_t wave_type);


#endif /*__AD9833_DDS_H*/
