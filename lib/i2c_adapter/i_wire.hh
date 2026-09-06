#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

// "Wire" shape: the subset of Arduino's TwoWire API that I2CAdapter needs.
//
// Portable: no Arduino dependency. Implementations are resolved at compile time
// (I2CAdapter<WireT>), never through a vtable. Two implementations exist:
//   - ArduinoWire (include/arduino_wire.hh) wraps the real TwoWire.
//   - MockWire (test/mocks/mock_wire.hh) is the scripted test double.
//
// Required members:
//     void        begin();
//     void        set_clock(uint32_t freq_hz);
//     void        begin_transmission(uint8_t address);
//     int         end_transmission(bool stop);      // Arduino status code: 0 = ok
//     std::size_t request_from(uint8_t address, std::size_t quantity);  // bytes received
//     std::size_t write(uint8_t data);
//     std::size_t write(const uint8_t* data, std::size_t quantity);
//     int         available();
//     int         read();
//     void        delay_microseconds(uint32_t us);

template <typename T, typename = void>
struct is_wire : std::false_type {};

template <typename T>
struct is_wire<T, std::void_t<
    decltype(std::declval<T&>().begin()),
    decltype(std::declval<T&>().set_clock(std::declval<uint32_t>())),
    decltype(std::declval<T&>().begin_transmission(std::declval<uint8_t>())),
    decltype(std::declval<T&>().end_transmission(std::declval<bool>())),
    decltype(std::declval<T&>().request_from(std::declval<uint8_t>(), std::declval<std::size_t>())),
    decltype(std::declval<T&>().write(std::declval<uint8_t>())),
    decltype(std::declval<T&>().write(std::declval<const uint8_t*>(), std::declval<std::size_t>())),
    decltype(std::declval<T&>().available()),
    decltype(std::declval<T&>().read()),
    decltype(std::declval<T&>().delay_microseconds(std::declval<uint32_t>()))
>> : std::true_type {};
