
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef FUNCTIONS_H
#define	FUNCTIONS_H

#include <xc.h> // include processor files - each processor file is guarded.  

//macros
#define BUFFER_SIZE (60)
#define STATE_DOLLAR  (1) // we discard everything until a dollar is found
#define STATE_TYPE    (2) // we are reading the type of msg until a comma is found
#define STATE_PAYLOAD (3) // we read the payload until an asterix is found
#define NEW_MESSAGE (1) // new message received and parsed completely
#define NO_MESSAGE (0) // no new messages
#define ALL_OK (1)

// Structures
typedef struct { 
	int state;
	char msg_type[6]; // type is 5 chars + string terminator
	char msg_payload[100];  // assume payload cannot be longer than 100 chars
	int index_type;
	int index_payload;
} parser_state;

typedef struct {
    char buffer[BUFFER_SIZE];
    int readIndex;
    int writeIndex;
} CircularBuffer;


typedef struct {
    int n1; // applied reference signal for the left motor
    int n2; // applied reference signal for the right motor
} velocity_command;


typedef struct {
    int min; // min velocity command
    int max; // max velocity command
} velocity_ref;


typedef struct {
    int n1; // applied reference signal for the left motor
    int n2; // applied reference signal for the right motor
    int state; // 2 if mc is in safe mode, 1 if it is in timeout mode, 0 otherwise.
    float temp; // temp is the temperature
} sensor_data;

// Function prototypes

// Controller function
void choose_prescaler(int ms, int* tckps, int* pr);
void tmr2_wait_ms(int ms);
void tmr1_setup_period(int ms);
void tmr1_wait_period();
void tmr4_wait_period();
void tmr2_setup_period(int ms);
void tmr3_setup_period(int ms);
void tmr4_setup_period(int ms);

void enable_SPI();
void enable_ADC();

void write_temp_screen(double adc);
void wait_buffer();
void clear_screen(int clr);
void write_screen_command(volatile velocity_command* fdata);
void write_screen_ref(volatile velocity_ref* fdata);
void write_screen(int left, int right);

void motor_command(volatile velocity_command* fdata,volatile velocity_ref* rdata);
void timeout_mode(volatile velocity_command* vel_to);
void safe_mode(volatile velocity_command* vel_safe);

//parser functions
void parse_velc(const char* msg,volatile velocity_command* fdata);
void parse_velr(const char* msg,volatile velocity_ref* fdata);
int extract_integer(const char* str);
int next_value(const char* msg, int i);
int parse_byte(parser_state* ps, char byte);

//UART buffer functions
int avl_in_buffer(volatile CircularBuffer* cb);
int read_buffer(volatile CircularBuffer* cb, char* value);
void write_buffer(volatile CircularBuffer* cb, char value);
void msg2PC(double temp,volatile velocity_command* pc_vel, volatile int state);

#endif	/* FUNCTIONS_H */

