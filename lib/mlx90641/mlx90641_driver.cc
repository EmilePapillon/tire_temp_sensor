#include "mlx90641_driver.hh"
#include "mlx90641_eeprom_parser.hh"
#include <cstring>
#include <cmath>

namespace mlx90641 {

MLX90641Sensor::MLX90641Sensor(I2CAdapter& i2c_adapter, uint8_t i2c_addr, Logger* logger_ptr)
    : i2c_(i2c_adapter), i2c_addr_(i2c_addr), ambient_(0.0f), logger_(logger_ptr)
{
    temps_.fill(0.0f);
    ee_data_.fill(0);
    frame_data_.fill(0);
    std::memset(&calibration_parameters_, 0, sizeof(calibration_parameters_));
}

bool MLX90641Sensor::init()
{
    log(Logger::Level::DEBUG, "Starting MLX90641 sensor initialization");
    
    log(Logger::Level::DEBUG, "Initializing I2C adapter");
    if (!i2c_.init(400)) {
        log(Logger::Level::ERROR, "Failed to initialize I2C adapter");
        return false;
    }
    log(Logger::Level::DEBUG, "I2C adapter initialized successfully");
    
    log(Logger::Level::DEBUG, "Dumping EEPROM data");
    int ee_result = dump_ee();
    if (ee_result != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to dump EEPROM data, error: %d", ee_result);
        log(Logger::Level::ERROR, msg);
        return false;
    }
    log(Logger::Level::DEBUG, "EEPROM data dumped successfully");
    
    log(Logger::Level::DEBUG, "Extracting calibration parameters");
    int param_result = extract_parameters();
    if (param_result != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to extract parameters, error: %d", param_result);
        log(Logger::Level::ERROR, msg);
        return false;
    }
    log(Logger::Level::DEBUG, "Calibration parameters extracted successfully");
    
    // TODO: allow configuration of refresh rate and resolution
    log(Logger::Level::DEBUG, "Setting resolution to 17-bit (0x03)");
    int res_result = set_resolution(0x03);     // 17-bit resolution
    if (res_result != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to set resolution, error: %d", res_result);
        log(Logger::Level::WARN, msg);
    } else {
        log(Logger::Level::DEBUG, "Resolution set successfully");
    }
    
    log(Logger::Level::DEBUG, "Setting refresh rate to 16Hz (0x06)");
    int rate_result = set_refresh_rate(0x06);     // 16Hz refresh
    if (rate_result != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "Failed to set refresh rate, error: %d", rate_result);
        log(Logger::Level::WARN, msg);
    } else {
        log(Logger::Level::DEBUG, "Refresh rate set successfully");
    }

    log(Logger::Level::INFO, "MLX90641 sensor initialization completed successfully");
    return true;
}

bool MLX90641Sensor::read_frame()
{
    if (get_frame_data() < 0)
        return false;
    ambient_ = get_ta();
    return true;
}

void MLX90641Sensor::calculate_temps()
{
    float emissivity = get_emissivity();
    float tr = ambient_;
    calculate_to(emissivity, tr);
    bad_pixels_correction();
}

std::array<float, MLX90641Sensor::num_pixels> MLX90641Sensor::get_temps() const
{
    return temps_;
}

float MLX90641Sensor::get_ambient() const
{
    return ambient_;
}

// ------------------- Private member functions -------------------

int MLX90641Sensor::dump_ee()
{
    bool success = i2c_.read(i2c_addr_, 0x2400, 832, ee_data_.data());
    
    int hamming_decode_error_code = 0;
    if (success)
        hamming_decode_error_code = hamming_decode();
    return success ? hamming_decode_error_code : -1;
}

int MLX90641Sensor::hamming_decode()
{
    int error = 0;
    int16_t parity[5];
    int8_t d[16];
    int16_t check;
    uint16_t data;
    uint16_t mask;
    
    for (int addr = 16; addr < 832; addr++)
    {
        parity[0] = -1;
        parity[1] = -1;
        parity[2] = -1;
        parity[3] = -1;
        parity[4] = -1;
        
        data = ee_data_[addr];
        mask = 1;
        for (int i = 0; i < 16; i++)
        {          
            d[i] = (data & mask) >> i;
            mask = mask << 1;
        }
        
        parity[0] = d[0]^d[1]^d[3]^d[4]^d[6]^d[8]^d[10]^d[11];
        parity[1] = d[0]^d[2]^d[3]^d[5]^d[6]^d[9]^d[10]^d[12];
        parity[2] = d[1]^d[2]^d[3]^d[7]^d[8]^d[9]^d[10]^d[13];
        parity[3] = d[4]^d[5]^d[6]^d[7]^d[8]^d[9]^d[10]^d[14];
        parity[4] = d[0]^d[1]^d[2]^d[3]^d[4]^d[5]^d[6]^d[7]^d[8]^d[9]^d[10]^d[11]^d[12]^d[13]^d[14]^d[15];
       
        if ((parity[0] != 0) || (parity[1] != 0) || (parity[2] != 0) || (parity[3] != 0) || (parity[4] != 0))
        {        
            check = (parity[0]<<0) + (parity[1]<<1) + (parity[2]<<2) + (parity[3]<<3) + (parity[4]<<4);

            if ((check > 15) && (check < 32))
            {
                switch (check)
                {    
                    case 16: d[15] = 1 - d[15]; break;
                    case 24: d[14] = 1 - d[14]; break;
                    case 20: d[13] = 1 - d[13]; break;
                    case 18: d[12] = 1 - d[12]; break;
                    case 17: d[11] = 1 - d[11]; break;
                    case 31: d[10] = 1 - d[10]; break;
                    case 30: d[9] = 1 - d[9]; break;
                    case 29: d[8] = 1 - d[8]; break;
                    case 28: d[7] = 1 - d[7]; break;
                    case 27: d[6] = 1 - d[6]; break;
                    case 26: d[5] = 1 - d[5]; break;
                    case 25: d[4] = 1 - d[4]; break;
                    case 23: d[3] = 1 - d[3]; break;
                    case 22: d[2] = 1 - d[2]; break;
                    case 21: d[1] = 1 - d[1]; break;
                    case 19: d[0] = 1 - d[0]; break;
                }
                if (error == 0)
                    error = -9;
                data = 0;
                mask = 1;
                for (int i = 0; i < 16; i++)
                {                    
                    data = data + d[i]*mask;
                    mask = mask << 1;
                }
            }
            else
            {
                error = -10;                
            }   
        }
        ee_data_[addr] = data & 0x07FF;
    }
    return error;
}

int MLX90641Sensor::get_frame_data()
{
    uint16_t data_ready = 1;
    uint16_t control_register_1;
    uint16_t status_register;
    int error = 1;
    uint8_t cnt = 0;
    uint8_t sub_page = 0;
    
    data_ready = 0;
    while (data_ready == 0)
    {
        error = i2c_.read(i2c_addr_, 0x8000, 1, &status_register);
        if (error != 0)
            return error;
        data_ready = status_register & 0x0008;
    }
    sub_page = status_register & 0x0001;
        
    while (data_ready != 0 && cnt < 5)
    { 
        error = i2c_.write(i2c_addr_, 0x8000, 0x0030);
        if (error == -1)
            return error;
        if (sub_page == 0)
        { 
            error = i2c_.read(i2c_addr_, 0x0400, 32, frame_data_.data()); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x0440, 32, frame_data_.data()+32); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x0480, 32, frame_data_.data()+64); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x04C0, 32, frame_data_.data()+96); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x0500, 32, frame_data_.data()+128); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x0540, 32, frame_data_.data()+160); 
            if (error != 0) return error;
        }
        else
        {
            error = i2c_.read(i2c_addr_, 0x0420, 32, frame_data_.data()); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x0460, 32, frame_data_.data()+32); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x04A0, 32, frame_data_.data()+64); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x04E0, 32, frame_data_.data()+96); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x0520, 32, frame_data_.data()+128); 
            if (error != 0) return error;
            error = i2c_.read(i2c_addr_, 0x0560, 32, frame_data_.data()+160); 
            if (error != 0) return error;
        }
        error = i2c_.read(i2c_addr_, 0x0580, 48, frame_data_.data()+192); 
        if (error != 0) return error;
        error = i2c_.read(i2c_addr_, 0x8000, 1, &status_register);
        if (error != 0) return error;
        data_ready = status_register & 0x0008;
        sub_page = status_register & 0x0001;
        cnt = cnt + 1;
    }
    if (cnt > 4)
        return -8;
    error = i2c_.read(i2c_addr_, 0x800D, 1, &control_register_1);
    frame_data_[240] = control_register_1;
    frame_data_[241] = status_register & 0x0001;
    if (error != 0)
        return error;
    return frame_data_[241];
}

int MLX90641Sensor::extract_parameters()
{
    int error = check_eeprom_valid();
    bool extractions_successful = false;
    if(error == 0)
    {
        if (logger_) {
            char debug_msg[512];
            snprintf(debug_msg, sizeof(debug_msg), 
                "Raw EEPROM - [34]: 0x%04X, [52]: 0x%04X, [53]: 0x%04X, [54]: 0x%04X, [45]: 0x%04X, [256]: 0x%04X", 
                ee_data_[34],   // KsTa
                ee_data_[52],   // ksTo scale
                ee_data_[53],   // ksTo[0]
                ee_data_[54],   // ksTo[1]
                ee_data_[45],   // cpAlpha
                ee_data_[256]); // alpha[0]
            log(Logger::Level::DEBUG, debug_msg);
        }

        extractions_successful = MLX90641EEpromParser(ee_data_).extract_all(calibration_parameters_);
    
        if (logger_) {
            char debug_msg[256];
            snprintf(debug_msg, sizeof(debug_msg), 
                "Critical params - ksTo[1]: %.6f, tgc: %.6f, cpAlpha: %.6f, alpha[0]: %.6f", 
                calibration_parameters_.ksTo[1],
                calibration_parameters_.tgc,
                calibration_parameters_.cpAlpha,
                calibration_parameters_.alpha[0]);
            log(Logger::Level::DEBUG, debug_msg);
        }
    }

    const bool success = extractions_successful && (error == 0);
    return success ? 0 : -1;
}

int MLX90641Sensor::set_resolution(uint8_t resolution)
{
    uint16_t control_register_1;
    int value;
    int error;
    
    value = (resolution & 0x03) << 10;
    
    error = i2c_.read(i2c_addr_, 0x800D, 1, &control_register_1);
    if (error != 0)
    {
        log(Logger::Level::ERROR, "Failed to read control register for setting resolution");
    } 
    if(error == 0)
    {
        value = (control_register_1 & 0xF3FF) | value;
        error = i2c_.write(i2c_addr_, 0x800D, value);        
    }    
    if (error != 0)
    {
        log(Logger::Level::ERROR, "Failed to write control register for setting resolution");
    }
    return error;
}

int MLX90641Sensor::get_cur_resolution() const
{
    uint16_t control_register_1;
    int resolution_ram;
    int error;
    
    error = i2c_.read(i2c_addr_, 0x800D, 1, &control_register_1);
    if(error != 0)
    {
        return error;
    }    
    resolution_ram = (control_register_1 & 0x0C00) >> 10;
    
    return resolution_ram; 
}

int MLX90641Sensor::set_refresh_rate(uint8_t refresh_rate)
{
    uint16_t control_register_1;
    int value;
    int error;
    
    value = (refresh_rate & 0x07)<<7;
    
    error = i2c_.read(i2c_addr_, 0x800D, 1, &control_register_1);
    if(error == 0)
    {
        value = (control_register_1 & 0xFC7F) | value;
        error = i2c_.write(i2c_addr_, 0x800D, value);
    }    
    
    return error;
}

int MLX90641Sensor::get_refresh_rate() const
{
    uint16_t control_register_1;
    int refresh_rate;
    int error;
    error = i2c_.read(i2c_addr_, 0x800D, 1, &control_register_1);
    if(error != 0)
    {
        return error;
    }    
    refresh_rate = (control_register_1 & 0x0380) >> 7;
    
    return refresh_rate;
}

void MLX90641Sensor::calculate_to(float emissivity, float tr)
{
    float vdd;
    float ta;
    float ta4;
    float tr4;
    float ta_tr;
    float gain;
    float ir_data_cp;
    float ir_data;
    float alpha_compensated;
    float sx;
    float to;
    float alpha_corr_r[8];
    int8_t range;
    uint16_t sub_page;
    
    sub_page = frame_data_[241];
    vdd = get_vdd();
    ta = get_ta();
    ta4 = pow((ta + 273.15), (double)4);
    tr4 = pow((tr + 273.15), (double)4);
    ta_tr = tr4 - (tr4-ta4)/emissivity;
    
    alpha_corr_r[1] = 1 / (1 + calibration_parameters_.ksTo[1] * 20);
    alpha_corr_r[0] = alpha_corr_r[1] / (1 + calibration_parameters_.ksTo[0] * 20);
    alpha_corr_r[2] = 1 ;
    alpha_corr_r[3] = (1 + calibration_parameters_.ksTo[2] * calibration_parameters_.ct[2]);
    alpha_corr_r[4] = alpha_corr_r[3] * (1 + calibration_parameters_.ksTo[3] * (calibration_parameters_.ct[4] - calibration_parameters_.ct[3]));
    alpha_corr_r[5] = alpha_corr_r[4] * (1 + calibration_parameters_.ksTo[4] * (calibration_parameters_.ct[5] - calibration_parameters_.ct[4]));
    alpha_corr_r[6] = alpha_corr_r[5] * (1 + calibration_parameters_.ksTo[5] * (calibration_parameters_.ct[6] - calibration_parameters_.ct[5]));
    alpha_corr_r[7] = alpha_corr_r[6] * (1 + calibration_parameters_.ksTo[6] * (calibration_parameters_.ct[7] - calibration_parameters_.ct[6]));

    //------------------------- Gain calculation -----------------------------------
    gain = frame_data_[202];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }

    gain = calibration_parameters_.gainEE / gain;

//------------------------- To calculation -------------------------------------        
    ir_data_cp = frame_data_[200];  
    if(ir_data_cp > 32767)
    {
        ir_data_cp = ir_data_cp - 65536;
    }
    ir_data_cp = ir_data_cp * gain;

    ir_data_cp = ir_data_cp - calibration_parameters_.cpOffset * (1 + calibration_parameters_.cpKta * (ta - 25)) * (1 + calibration_parameters_.cpKv * (vdd - 3.3));
    
    for( int pixel_number = 0; pixel_number < 192; pixel_number++)
    {      
        ir_data = frame_data_[pixel_number];
        if(ir_data > 32767)
        {
            ir_data = ir_data - 65536;
        }
        ir_data = ir_data * gain;

        ir_data = ir_data - calibration_parameters_.offset[sub_page][pixel_number]*(1 + calibration_parameters_.kta[pixel_number]*(ta - 25))*(1 + calibration_parameters_.kv[pixel_number]*(vdd - 3.3));

        ir_data = ir_data - calibration_parameters_.tgc * ir_data_cp;

        ir_data = ir_data / emissivity;

        alpha_compensated = (calibration_parameters_.alpha[pixel_number] - calibration_parameters_.tgc * calibration_parameters_.cpAlpha)*(1 + calibration_parameters_.KsTa * (ta - 25));

        sx = alpha_compensated * alpha_compensated * alpha_compensated * (ir_data + alpha_compensated * ta_tr);
        sx = sqrt(sqrt(sx)) * calibration_parameters_.ksTo[1];

        to = sqrt(sqrt(ir_data/(alpha_compensated * (1 - calibration_parameters_.ksTo[1] * 273.15) + sx) + ta_tr)) - 273.15;


        if(to < calibration_parameters_.ct[1])
        {
            range = 0;
        }
        else if(to < calibration_parameters_.ct[2])
        {
            range = 1;
        }
        else if(to < calibration_parameters_.ct[3])
        {
            range = 2;            
        }
        else if(to < calibration_parameters_.ct[4])
        {
            range = 3;            
        }
        else if(to < calibration_parameters_.ct[5])
        {
            range = 4;            
        }
        else if(to < calibration_parameters_.ct[6])
        {
            range = 5;            
        }
        else if(to < calibration_parameters_.ct[7])
        {
            range = 6;
        }
        else
        {
            range = 7;
        }

        to = sqrt(sqrt(ir_data / (alpha_compensated * alpha_corr_r[range] * (1 + calibration_parameters_.ksTo[range] * (to - calibration_parameters_.ct[range]))) + ta_tr)) - 273.15;
        temps_[pixel_number] = to;
    }
}

void MLX90641Sensor::get_image()
{
    float vdd;
    float ta;
    float gain;
    float ir_data_cp;
    float ir_data;
    float alpha_compensated;
    float image;
    uint16_t sub_page;
    
    sub_page = frame_data_[241];
    
    vdd = get_vdd();
    ta = get_ta();
    
//------------------------- Gain calculation -----------------------------------    
    gain = frame_data_[202];
    if(gain > 32767)
    {
        gain = gain - 65536;
    }
    
    gain = calibration_parameters_.gainEE / gain; 
  
//------------------------- Image calculation -------------------------------------    
    ir_data_cp = frame_data_[200];  
    if(ir_data_cp > 32767)
    {
        ir_data_cp = ir_data_cp - 65536;
    }
    ir_data_cp = ir_data_cp * gain;

    ir_data_cp = ir_data_cp - calibration_parameters_.cpOffset * (1 + calibration_parameters_.cpKta * (ta - 25)) * (1 + calibration_parameters_.cpKv * (vdd - 3.3));
    
    for( int pixel_number = 0; pixel_number < 192; pixel_number++)
    {
        ir_data = frame_data_[pixel_number];
        if(ir_data > 32767)
        {
            ir_data = ir_data - 65536;
        }
        ir_data = ir_data * gain;

        ir_data = ir_data - calibration_parameters_.offset[sub_page][pixel_number]*(1 + calibration_parameters_.kta[pixel_number]*(ta - 25))*(1 + calibration_parameters_.kv[pixel_number]*(vdd - 3.3));

        ir_data = ir_data - calibration_parameters_.tgc * ir_data_cp;

        alpha_compensated = (calibration_parameters_.alpha[pixel_number] - calibration_parameters_.tgc * calibration_parameters_.cpAlpha);

        image = ir_data/alpha_compensated;
            
        temps_[pixel_number] = image;
    }
}

float MLX90641Sensor::get_vdd() const
{
    float vdd;
    float resolution_correction;
    
    int resolution_ram;    
    
    vdd = frame_data_[234];
    if(vdd > 32767)
    {
        vdd = vdd - 65536;
    }
    resolution_ram = (frame_data_[240] & 0x0C00) >> 10;
    resolution_correction = pow(2, (double)calibration_parameters_.resolutionEE) / pow(2, (double)resolution_ram);
    vdd = (resolution_correction * vdd - calibration_parameters_.vdd25) / calibration_parameters_.kVdd + 3.3;

    return vdd;
}

float MLX90641Sensor::get_ta() const
{
    float ptat;
    float ptat_art;
    float vdd;
    float ta;
    
    vdd = get_vdd();
    
    ptat = frame_data_[224];
    if(ptat > 32767)
    {
        ptat = ptat - 65536;
    }
    
    ptat_art = frame_data_[192];
    if(ptat_art > 32767)
    {
        ptat_art = ptat_art - 65536;
    }
    ptat_art = (ptat / (ptat * calibration_parameters_.alphaPTAT + ptat_art)) * pow(2, (double)18);

    ta = (ptat_art / (1 + calibration_parameters_.KvPTAT * (vdd - 3.3)) - calibration_parameters_.vPTAT25);
    ta = ta / calibration_parameters_.KtPTAT + 25;

    return ta;
}

int MLX90641Sensor::get_sub_page_number() const
{
    return frame_data_[241];
}

void MLX90641Sensor::bad_pixels_correction()
{
    float ap[2];
    uint8_t pix;
    uint8_t line;
    uint8_t column;
    
    pix = 0;
    while(calibration_parameters_.brokenPixels[pix]< 65535)
    {
        line = calibration_parameters_.brokenPixels[pix]>>5;
        column = calibration_parameters_.brokenPixels[pix] - (line<<5);
               
        if(column == 0)
        {
            temps_[calibration_parameters_.brokenPixels[pix]] = temps_[calibration_parameters_.brokenPixels[pix]+1];            
        }
        else if(column == 1 || column == 14)
        {
            temps_[calibration_parameters_.brokenPixels[pix]] = (temps_[calibration_parameters_.brokenPixels[pix]-1]+temps_[calibration_parameters_.brokenPixels[pix]+1])/2.0;                
        } 
        else if(column == 15)
        {
            temps_[calibration_parameters_.brokenPixels[pix]] = temps_[calibration_parameters_.brokenPixels[pix]-1];
        } 
        else
        {            
            ap[0] = temps_[calibration_parameters_.brokenPixels[pix]+1] - temps_[calibration_parameters_.brokenPixels[pix]+2];
            ap[1] = temps_[calibration_parameters_.brokenPixels[pix]-1] - temps_[calibration_parameters_.brokenPixels[pix]-2];
            if(fabs(ap[0]) > fabs(ap[1]))
            {
                temps_[calibration_parameters_.brokenPixels[pix]] = temps_[calibration_parameters_.brokenPixels[pix]-1] + ap[1];                        
            }
            else
            {
                temps_[calibration_parameters_.brokenPixels[pix]] = temps_[calibration_parameters_.brokenPixels[pix]+1] + ap[0];                        
            }
                    
        }                      
     
        pix = pix + 1;    
    }    
}

float MLX90641Sensor::get_emissivity() const
{
    return calibration_parameters_.emissivityEE;
}

int MLX90641Sensor::check_eeprom_valid() const
{
     int device_select;
     device_select = ee_data_[10] & 0x0040;
     if(device_select != 0)
     {
         return 0;
     }
     
     return -7;    
 }

void MLX90641Sensor::log(Logger::Level level, const char* message)
{
    if (logger_) {
        logger_->log(level, message);
    }
}

} // namespace mlx90641