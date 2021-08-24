#include "AP_RangeFinder_A02YYUW.h"

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

#define A02YYUW_SERIAL_LV_BAUD_RATE 9600

extern const AP_HAL::HAL& hal;

// read - return last value measured by sensor
bool AP_RangeFinder_A02YYUW::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) return false;

    unsigned char data[4]={};
    int checksum;
    float distance;
    bool status = 0;

    int32_t sum = 0;
    int16_t nbytes = uart->available();
    uint16_t count = 0;
    uint8_t i = 1;
    
    while (nbytes-- > 0)
        switch (status)
        {
            case 0:
                data[0]=uart->read();
                if(data[0]==255) status = 1;
                break;
            case 1:
                data[i]=uart->read();
                i++;
                if (i==4)
                {
                    i = 1;
                    status = 0;
                    checksum=(data[0]+data[1]+data[2])&0x00FF;
                    if(checksum==data[3])
                    {
                        distance=(data[1]<<8)+data[2];
                        if(distance>30)
                        {
                            sum+=distance/10;
                            count++;
                        }
                    }
                }
                break;
        }

    if (count == 0) return false;

    reading_cm = sum / count;
    
    return true;
}