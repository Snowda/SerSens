#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <inttypes.h>
#include "sensor.h"

#include "Arduino.h"

//#include "sha256.h"

uint8_t hmacKey1[]={
  0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b,0x0b
};

sensor oxygen;
central dsk001;
comms_method usb_struct;

uint16_t i;

uint8_t init_parameters(sensor* sens, central* syst, comms_method* comms) {
    sens->name = "Oxygen";
    sens->reading = 0.0;
    sens->sensor_type = 0;
    sens->min_level = 0;
    sens->max_level = 1000000;
    sens->span_factor = 1;
    sens->zero_factor = 0;
    sens->linearisation = false;
    sens->units = "PPM";

    comms->name = "USB";

    //syst->sensor = sens;
    syst->display_method = false;
    //syst->communicate = comms;

    syst->current_time = "0";
    syst->status = "Failure";
    syst->firmware = "v.0.0.01";
    syst->name = "DSK001";
    syst->access = 2;
    syst->compile_time = "";
    syst->compile_date = "";

    return 0;
}

/*
usb_struct = {
    .name = "USB"
}

dsk001 = {
    .sensor1 = oxygen,
    .display_method = false,
    .comms_method = usb_struct,
    .current_time = "0",
    .status = "Failure",
    .firmware = "v.0.0.01",
    .name = "DSK001",
    .access = 2,
    .compile_time = "",
    .compile_date = ""
};
*/

uint8_t save_data (central* configuration) {
    return 0;
}

uint8_t load_configuration (sensor* sens) {
    //configuration->sensor.span_factor = eeprom_read_byte(SPAN_ADDR);
    //configuration->sensor.zero_factor = eeprom_read_byte(ZERO_ADDR);

    //configuration->sensor.span_factor = EEPROM.read(SPAN_ADDR);
    //configuration->sensor.span_factor = EEPROM.read(SPAN_ADDR);
    //configuration->sensor.span_factor = EEPROM.read(SPAN_ADDR);
    //configuration->sensor.span_factor = EEPROM.read(SPAN_ADDR);
    return 0;
}

void power_supply (void) {

}

void simulate_output (void) {

}

uint32_t compareSensors (sensor* sens1, sensor* sens2) {
    if(sens1.sensing_method == sens2.sensing_method){
        return sens1.reading - sens2.reading;
    } else {
        Serial.println("not the same type of sensor");
        return 0;
    }
}

uint8_t usb (central* configuration) {
    Serial.print(*configuration->name); //Machine
    Serial.print("\n");
    Serial.print(*configuration->status); //Status
    Serial.print("\n");
    Serial.print(*configuration->current_time); //Time
    Serial.print("\n");
    return 0;
}

void rs232 (comms_method* comms) {

}

void rs485 (comms_method* comms) {

}

void current_loop (comms_method* comms) {

}

void bluetooth (comms_method* comms) {
    //bluetooth

}

void sd_card (void) {
    
}

void get_time (central* configuration) {
    
}

void log_data (central* configuration) {
    
}

void ethernet (comms_method* comms) {

}

void wifi (comms_method* comms) {

}

void communicate (comms_method* comms) {
    //transmit data
    //data encryption
    switch(comms.type){
        case TYPE_RS232:
            rs232(comms);
        break;
        case TYPE_RS485:
            rs485(comms);
        break;
        case TYPE_ETHERNET:
            ethernet(comms)
        break;
        case TYPE_WIFI:
            wifi(comms);
        break;
        case TYPE_USB:
            usb(comms);
        break;
        case TYPE_LOOP:
            current_loop(comms);
        break;
        case TYPE_BLUETOOTH:
            bluetooth(comms);
        break;
    }

}

void payment_timeout (central* configuration) {
    current_time = get_time(configuration);
    pay_horizon = read_eeprom(PAY_ADDR);
    if(pay_horizon < current_time){
        configuration.payment = false;
    } else {
        configuration.payment = true;
    }
}

void update_payment_date (central* configuration) {
    set_time(PAY_ADDR, payment_time);
}

uint32_t calculation_delay(uint32_t* current_time, uint32_t* start_time) {
    current_time = millis();
    data_ps = current_time - start_time;
    Serial.print(data_ps);
    start_time = millis();
    return start_time;
}

unsigned int count_args (unsigned int Count,  ... ) {
    va_list Numbers;
    va_start(Numbers, Count); 
    va_end(Numbers);
    return (Count);
}

uint8_t adjust_zero (central* configuration) {

    //eeprom_write_byte(ZERO_ADDR, configuration->sensor.zero_factor);
    return 0;
}

uint8_t adjust_span (central* configuration) {

    //eeprom_write_byte(SPAN_ADDR, configuration->sensor.span_factor);
    return 0;
}

float span_sensor (const float sensor_data, const float span_factor) {

    if(0.1 >= span_factor && span_factor <= -0.1){
        return (sensor_data*(span_factor/100));
    }
    printf("Span Factor %.3f too small. Close to div/0.\r\n", (double)sensor_data);
    return 0;
}

float zero_sensor (const float sensor_data, const float zero_factor) {
    if((zero_factor) && (sensor_data <= zero_factor*2)){
        printf("Warning: Zero is large vs sensor reading. Data is unreliable\r\n");
    }
    return sensor_data+zero_factor;
}

float calibrate_sensor (const float sensor_data, 
    const float sensor_min, const float sensor_max){
    //calibrated_sensor = zero_sensor(span_sensor(sensor_data));
    return fmin(fmax(sensor_data, sensor_min), sensor_max);
}

/*
opamp rails measurement circuit
3.3v to 24v to 3.3v regulator
power supply measurement
sensor type and identifier transmitter
send / recieve calibration data

temperature compensation on opamp / sensor
undefined bit level handling
sensor fusion of data

pogo pin based connectors
pogo programmer / tester
power protection, isolated data / power
*/

void adjust_opamp (void) {
    // programmable opamp gain/ATD converter circuit

}

void send_calibration_data (void) {

}

void recieve_calibration_data (void) {

}

void discover_sensor (sensor* sens) {
    // sensor identifier

}

void read_analog_sensor (sensor* sens) {

}

void read_one_wire (void) {

}

void get_temperature (sensor* sens) {
    //i2c protocol goes here

}

void get_lux (sensor* sens) {
    //i2c protocol goes here

}

uint32_t get_oxygen (sensor* sens) {
    //i2c protocol goes here
    return 0;
}

uint32_t transmit_oxygen (sensor* sens) {
    Serial.print("%i %s\r\n", get_oxygen(sens), sens.units);
}

uint32_t get_nitrogen (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_flow (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_hall_effect (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_accelerometer (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_proximity (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_altitude (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_pressure (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_current (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_voltage (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_rotary (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_switch (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

uint32_t get_capacitance (sensor* sens) {
    //i2c protocol goes here

    return 0;
}

void get_sensor (sensor* sens) {
    // finder function

}


void get_sensor_method (sensor* sens) {
    get_sensor_units(sens);

}

void get_pir (sensor* sens) {

    // ambient powered motion detector
}

uint16_t read_sensor(sensor* sens) {
    //measure
    switch(sens.sensor_type){
        case TYPE_OXYGEN:
            reading = get_oxygen(sens);
        break;
        case TYPE_NITROGEN:
            reading = get_nitrogen(sens);
        break;
        case TYPE_LUX:
            reading = get_lux(sens);
        break;
        case TYPE_TEMPERATURE:
            reading = get_temperature(sens);
        break;
        case TYPE_ROTARY:
            reading = get_rotary(sens);
        break;
        case TYPE_CAPACITANCE:
            reading = get_capacitance(sens);
        break;
        case TYPE_VOLTAGE:
            reading = get_voltage(sens);
        break;
        case TYPE_CURRENT:
            reading = get_current(sens);
        break;
        case TYPE_HALL:
            reading = get_hall_effect(sens);
        break;
        case TYPE_PRESSURE:
            reading = get_pressure(sens);
        break;
        case TYPE_FLOW:
            reading = get_flow(sens);
        break;
        case TYPE_SWITCH:
            reading = get_switch(sens);
        break;
        case TYPE_MOTION:
            reading = get_motion(sens);
        break;
        case TYPE_ACCELEROMETER:
            reading = get_accelerometer(sens);
        break;
        case TYPE_SIM:
            reading = get_simulated(sens);
        break;
        case TYPE_PIR:
            reading = get_pir(sens);
        break;
        case TYPE_IR:
            reading = get_infrared(sens);
        break;
    }

    if(sens.linearisation){
        linearized = linearise_sensor(reading);
        limited = max(min(linearized, sens.max_level), sens.min_level);
    } else {
        limited = max(min(reading, sens.max_level), sens.min_level);
    }

    if(reading != limited) {
        printf("linearized reading was out of range!\r\n");
        sens.error = LIN_RANGE;
    }
    calibrated = calibrate_sensor(sens, limited);
    cal_lim = max(min(calibrated, sens.max_level), sens.min_level);
    if(cal_lim != calibrated) {
        printf("calibrated reading was out of range. check calibration values \r\n");
        sens.error = CAL_RANGE;
    }
    sens.reading = cal_lim;

    return sens.reading;
}

void init_sensor (sensor* sens) {
    get_sensor_method(sens);
    get_sensor(sens);
    sensor_thresholds(sens);
    get_saved_calibration(sens);
    init_linearisation(sens);
    read_sensor(sens);
}

void init_linearisation (sensor* sens) {
    sens.linearisation = false;
}

void get_saved_calibration(sensor* sens) {
    //store calibration data 
    if(!eeprom){
        sens.span_factor = 1;
        write_eeprom(SPAN_ADDR, 1);
        sens.zero_factor = 0;
        write_eeprom(ZERO_ADDR, 0);
        sens.bit_factor = 0;
        write_eeprom(BIT_ADDR, 0);
    } else {
        sens.span_factor = read_eeprom(SPAN_ADDR);
        sens.zero_factor = read_eeprom(ZERO_ADDR);
        sens.bit_factor = read_eeprom(BIT_ADDR);
    }
}

void sensor_thresholds (sensor* sens) {
    if(sens.sensing_method == TYPE_OXYGEN){
        sens.min_level = 0;
        if(sens.units == PPM){
            sens.max_level = 1000000;
        } else {
            sens.max_level = 100.0;
        }
    } else if(sens.sensing_method == TYPE_NITROGEN){
        sens.min_level = 0;
        if(sens.units == PPM) {
            sens.max_level = 1000000;
        } else {
            sens.max_level = 100.0;
        }
    }
}

void relays (void) {
    //relay control board
}

void relay_on (uint8_t) {

}

void relay_off (uint8_t) {

}

//store readings
//multicore check

void calibrate_4_20 (void) {

}

void simulate_sensor (sensor* sens) {

}

void get_time (void) {
    //store time 

}

void encrypt_data (uint32_t data) {
    salted_data = salt_data(data, 0xC0044D);
    sha256_hash(salted_data);

}

void sha256_hash (uint32_t data) {
    //Sha256.initHmac(hmacKey1,20);
    //Sha256.print("Hi There");
    //printHash(Sha256.resultHmac());
}

void salt_data (uint32_t data, uint32_t salt) {

}

void verify_password (central* configuration) {

}


void login (central* configuration) {

}

uint8_t payment_timeout (void) {
    //phone home verification
    now = get_time();
    if(now >= 0){
        return true;
    } else {
        return false;
    }
}

uint8_t analyzer_i2c (void) {
    //in-built logic analyser

}

void setup ()
{
    init_parameters(&oxygen, &dsk001, &usb_struct);
    Serial.begin(19200);
    load_configuration(&dsk001);
    load_sensor(&oxygen);

    uint32_t current_time = millis();
    uint32_t start_time = millis();
}

void loop ()
{
    i++;
    Serial.println("Oxygen Sensor:");
    load_configuration(&dsk001);
    usb(&dsk001);
    Serial.print(const_sha[i]);

    calculation_delay(&current_time, &start_time);
}
