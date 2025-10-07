
#include <cstdint>
#include "i_wire.hh"

class Wire : public IWire
{
public:
    ~Wire() = default; 
    void begin() override;
    void setClock(uint32_t freq) override;
    int endTransmission(bool stop)  override;
    void beginTransmission(uint8_t address) override;
    uint8_t requestFrom(uint8_t address, std::size_t quantity) override;
    std::size_t write(uint8_t data) override;
    std::size_t write(const char* data, std::size_t quantity) override;
    int available() override;
    int read() override;
    int peek() override;
    void flush() override;
    void delayMicroseconds(int us) override;
};