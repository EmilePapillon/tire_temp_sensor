#include "i2c_adapter.hh"

int I2CAdapter::init(int freq) {
    wire_.begin();
    wire_.setClock(1000 * freq); // freq in kHz
    wire_.delayMicroseconds(1000); // wait for 5 ms
    return 0;
}

void I2CAdapter::set_frequency(int freq) {
    wire_.setClock(1000 * freq); // freq in kHz
} 

int I2CAdapter::read(uint8_t device_address,
                      uint16_t start_register,
                      std::size_t length,
                      uint16_t* buffer) {
    char cmd[2] = {0,0};
    uint16_t *p; 
    p = buffer;
    
    int total_bytes = 2*length;
    const int div = 32;
    int factor = (int)(total_bytes)/div;
    int remainder = (total_bytes)%div;
    if(remainder!=0) factor++;
    
    for(int j = 0 ; j < factor ; j++){
        uint16_t address = start_register+(j*div)/2;
        cmd[0] = address >> 8;
        cmd[1] = address & 0x00FF;

        wire_.endTransmission();
        wire_.delayMicroseconds(5);
        wire_.beginTransmission(device_address);
        
        int written1 = wire_.write(cmd[0]);
        int written2 = wire_.write(cmd[1]);
        
        int end = wire_.endTransmission(0);
        if(end == 2 || end == 3) return -1;
        if(end == 1 || end == 4) return -2;
            
        int num_bytes = div;
        if(j == (factor-1) && remainder!=0) num_bytes = remainder;
            
        num_bytes = wire_.requestFrom((int)device_address,num_bytes);
        if(!num_bytes) return -1;
        
        for(int i = 0 ; i < num_bytes/2 ; i++){
            if(wire_.available()){		
                 uint16_t high = wire_.read()<<8;
                 uint16_t low = wire_.read();
                 *p++ = high + low;
            }
        }
    }
    return 0;
}

int I2CAdapter::write(uint8_t device_address,
                       uint16_t reg,
                       uint16_t value) {
    char cmd[4] = {0,0,0,0};
    static uint16_t dataCheck;

    cmd[0] = reg >> 8;
    cmd[1] = reg & 0x00FF;
    cmd[2] = value >> 8;
    cmd[3] = value & 0x00FF;
    
    wire_.endTransmission();
    wire_.beginTransmission(device_address);
    
    wire_.delayMicroseconds(5);

    wire_.write(cmd,4);
    
    wire_.endTransmission(); 
    
    if(read(device_address, reg, 1, &dataCheck) != 0) return -1;
    
    if ( dataCheck != value)
    {
        return -2;
    }    
    
    return 0;
}