#define ZERO_ADDR 100
#define SPAN_ADDR 120
#define EEPROM_SIZE 512

#define TYPE_OXYGEN         0
#define TYPE_NITROGEN       1
#define TYPE_LUX            2
#define TYPE_TEMPERATURE    3
#define TYPE_ROTARY         4
#define TYPE_CAPACITANCE    5
#define TYPE_VOLTAGE        6
#define TYPE_CURRENT        7
#define TYPE_HALL           8
#define TYPE_PRESSURE       9
#define TYPE_FLOW           10
#define TYPE_SWITCH         11
#define TYPE_MOTION         12
#define TYPE_ACCELEROMETER  13
#define TYPE_SIM            14
#define TYPE_PIR            15
#define TYPE_IR             16

#define PPM                 "PPM"
#define DEG_C               "C"
#define DEG_F               "F"
#define PCNT_O2             "0/0 O2"
#define PCNT_N2             "0/0 N2"
#define KELVIN              "K"
#define


#define limit(input, min_val, max_val) max(min(input, max_val), min_val)
#define clamp(reading, accuracy) Serial.println(reading, accuracy) //accuracy | 2 == 0.01


typedef struct {
    char* name;
    volatile float reading;
    volatile uint8_t sensing_method;
    volatile uint8_t min_level;
    volatile uint8_t max_level;
    volatile float span_factor;
    volatile float zero_factor;
    volatile uint8_t bit_factor;
    volatile uint8_t linearisation;
    volatile uint8_t bit_accuracy;

    char* units;
} sensor_type;

typedef struct {
    char* name;
    volatile uint8_t type;
} comms_method;

typedef struct {
    sensor_type sensor1;
    sensor_type sensor2;
    char* name;
    volatile uint8_t display_method;
    comms_method communicate;
    char* current_time;
    char* status;
    const char* firmware;
    volatile uint8_t access;
    const char* compile_time;
    const char* compile_date;
    uint8_t simulation_status;

} central;

uint8_t init_parameters(sensor_type* sens, central* syst, comms_method* comms);
uint8_t save_data(central* configuration);
uint8_t load_configuration(central* configuration);
void current_loop(void);
void bluetooth(void);
void power_supply(void);
void simulate_input(void);
void simulate_output(void);
void compareSensors(int sensor1, int sensor2);
uint8_t usb(central* configuration);
void rs232(void);
void sd_card(void);
void get_time(void);
void log_data(void);
void rs485(void);
void ethernet(void);
void communicate(void);
void read_sensor(void);
void read_analog_sensor(void);
void read_one_wire(void);
void payment_timeout(void);
void clamp(void);
uint8_t limit(uint8_t input, uint8_t min_val, uint8_t max_val);
unsigned int count_args(unsigned int Count,  ... );
void adjust_zero(void);
void adjust_span(void);
float span_sensor(const float sensor_data, const float span_factor);
float zero_sensor(const float sensor_data, const float zero_factor);
float calibrate_sensor(const float sensor_data, 
    const float sensor_min, const float sensor_max);

//measure
//store calibration data 
//transmit data 
//encrypt
//hash
//salt 
//payment_timeout
//4-20 calibration 
//simulate 
//store readings
//store time 
//password
//login 
//phone home verification
//multicore check
//communicate
