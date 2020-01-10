#include "Robot.h"
#include "mbed.h"

I2C i2c(I2C_SDA, I2C_SCL );
const int addr8bit = 20 << 1; // 7bit I2C address is 20; 8bit I2C address is 40 (decimal).

int16_t countsLeft = 0;
int16_t countsRight = 0;


void setSpeeds(int16_t leftSpeed, int16_t rightSpeed)
{
    char buffer[5];
    
    buffer[0] = 0xA1;
    memcpy(&buffer[1], &leftSpeed, sizeof(leftSpeed));
    memcpy(&buffer[3], &rightSpeed, sizeof(rightSpeed));

    i2c.write(addr8bit, buffer, 5); // 5 bytes
}

void setLeftSpeed(int16_t speed)
{
    char buffer[3];
    
    buffer[0] = 0xA2;
    memcpy(&buffer[1], &speed, sizeof(speed));

    i2c.write(addr8bit, buffer, 3); // 3 bytes
}

void setRightSpeed(int16_t speed)
{
    char buffer[3];
    
    buffer[0] = 0xA3;
    memcpy(&buffer[1], &speed, sizeof(speed));

    i2c.write(addr8bit, buffer, 3); // 3 bytes
}

void getCounts()
{
    char write_buffer[2];
    char read_buffer[4];
    
    write_buffer[0] = 0xA0;
    i2c.write(addr8bit, write_buffer, 1); wait_us(100);
    i2c.read( addr8bit, read_buffer, 4);
    countsLeft = (int16_t((read_buffer[0]<<8)|read_buffer[1]));
    countsRight = (int16_t((read_buffer[2]<<8)|read_buffer[3]));
}

void getCountsAndReset()
{
    char write_buffer[2];
    char read_buffer[4];
    
    write_buffer[0] = 0xA4;
    i2c.write(addr8bit, write_buffer, 1); wait_us(100);
    i2c.read( addr8bit, read_buffer, 4);
    countsLeft = (int16_t((read_buffer[0]<<8)|read_buffer[1]));
    countsRight = (int16_t((read_buffer[2]<<8)|read_buffer[3]));
}


