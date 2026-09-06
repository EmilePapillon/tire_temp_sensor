#include "bluefruit_ble_peripheral.hh"

bool BluefruitBlePeripheral::begin() {
    return Bluefruit.begin();
}

void BluefruitBlePeripheral::set_device_name(const char* name) {
    Bluefruit.setName(name);
}

void BluefruitBlePeripheral::add_service(ble::Uuid16 uuid) {
    if (service_count_ >= max_services) {
        return;
    }
    BLEService& service = services_[service_count_++];
    service.setUuid(BLEUuid(uuid));
    service.begin();
}

void BluefruitBlePeripheral::add_characteristic(ble::Uuid16 uuid, const ble::CharacteristicProps& props) {
    if (characteristic_count_ >= max_characteristics) {
        return;
    }
    uint8_t bluefruit_props = 0;
    if (props.read) {
        bluefruit_props |= CHR_PROPS_READ;
    }
    if (props.notify) {
        bluefruit_props |= CHR_PROPS_NOTIFY;
    }
    if (props.indicate) {
        bluefruit_props |= CHR_PROPS_INDICATE;
    }
    if (props.write) {
        bluefruit_props |= CHR_PROPS_WRITE;
    }
    if (props.write_without_response) {
        bluefruit_props |= CHR_PROPS_WRITE_WO_RESP;
    }
    const bool writable = props.write || props.write_without_response;

    characteristic_uuids_[characteristic_count_] = uuid;
    BLECharacteristic& chr = characteristics_[characteristic_count_++];
    chr.setUuid(BLEUuid(uuid));
    chr.setProperties(bluefruit_props);
    chr.setPermission(SECMODE_OPEN, writable ? SECMODE_OPEN : SECMODE_NO_ACCESS);
    if (props.fixed_len != 0) {
        chr.setFixedLen(props.fixed_len);
    }
    chr.begin();  // attaches to the most recently begun BLEService
}

bool BluefruitBlePeripheral::notify(ble::Uuid16 characteristic, const uint8_t* data, std::size_t len) {
    BLECharacteristic* chr = find_characteristic(characteristic);
    if (chr == nullptr) {
        return false;
    }
    return chr->notify(data, static_cast<uint16_t>(len));
}

void BluefruitBlePeripheral::start_advertising(const ble::AdvertisingParams& params) {
    Bluefruit.setTxPower(params.tx_power_dbm);
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    for (std::size_t i = 0; i < service_count_; i++) {
        Bluefruit.Advertising.addService(services_[i]);
    }
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(params.restart_on_disconnect);
    Bluefruit.Advertising.setInterval(params.interval_fast, params.interval_slow);
    Bluefruit.Advertising.setFastTimeout(params.fast_timeout_s);
    Bluefruit.Advertising.start(params.timeout_s);
}

bool BluefruitBlePeripheral::is_connected() {
    return Bluefruit.connected() > 0;
}

void BluefruitBlePeripheral::poll() {
    // Bluefruit services the SoftDevice from its own FreeRTOS task.
}

std::array<uint8_t, 6> BluefruitBlePeripheral::mac_address() const {
    std::array<uint8_t, 6> mac{};
    Bluefruit.getAddr(mac.data());
    return mac;
}

BLECharacteristic* BluefruitBlePeripheral::find_characteristic(ble::Uuid16 uuid) {
    for (std::size_t i = 0; i < characteristic_count_; i++) {
        if (characteristic_uuids_[i] == uuid) {
            return &characteristics_[i];
        }
    }
    return nullptr;
}
