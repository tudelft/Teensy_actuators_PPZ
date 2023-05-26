#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"
#include "Definitions.h"

// -------------------------- COMMUNICATION DEFINED VARIABLES-----------------------------

#define COMMUNICATION_SERIAL Serial4
#define COMMUNICATION_SERIAL_BAUD 1500000
#define COMM_REFRESH_TIME 2000 // ~500 Hz message loop 
#define PIN_LED_T4 13

byte START_BYTE_SERIAL_ACT_T4=0x9A;
elapsedMicros last_time_write_to_pixhawk = 0;
int ack_comm_TX_pixhawk = 0; 

struct serial_act_t4_in myserial_act_t4_in;
volatile struct serial_act_t4_out myserial_act_t4_out;
volatile float extra_data_in[255], extra_data_out[255]__attribute__((aligned));

uint8_t rolling_message_out_id_cnt = 0; 

uint16_t serial_act_t4_buf_in_cnt = 0;

uint8_t serial_act_t4_msg_buf_in[ 2*sizeof(struct serial_act_t4_in) ] = {0};
int serial_act_t4_received_packets = 0;
elapsedMillis old_time_frequency_in = 0, time_no_connection_pixhawk = 0;
uint16_t serial_act_t4_message_frequency_in;
int serial_act_t4_missed_packets_in; 

// ----------------------------- ESC DEFINED VARIABLES-----------------------------------
#define ESCPID_NB_ESC             4                 // Number of ESCs
#define ESCPID_MAX_ESC            4                 // Max number of ESCs

#define MIN_DSHOT_CMD 100
#define MAX_DSHOT_CMD 1999
// --------------------------- SERVOS DEFINED VARIABLES-----------------------------------

//PWM servos settings (servo 9 and 10)
#define SERVO9_PWM_PIN 2 
#define SERVO10_PWM_PIN 3    
#define PWM_SERVO_FREQUENCY 300 //[Hz] DO NOT OVERCOME 400 Hz!!!!

#define servo_state_mem_buf_size 100

float PWM_to_pulse_multiplier;
float servo_9_state_memory[servo_state_mem_buf_size], servo_10_state_memory[servo_state_mem_buf_size];
float Servo9_state, Servo9_state_old, Servo10_state, Servo10_state_old;

//END PWM SERVO SETTINGS

//SERIAL BUS servos settings
// The Rx and Tx pins at the Teensy must be tied together and connected via 100 Ohm to the FEETECH data line!

#define BAUDRATE_SERVO  1000000 //Baudrate for the servo communication.
#define SERVO_MAX_COMD 4096 //It tells us the step we use to control the servo angle
#define SERVO_COMM_MARGIN 70 //[uS] Margin of time given to the servo for the response
#define TIME_OF_SERVO_TX 150 //[uS] Margin of time needed to avoid TX conflicts when the servos are killed
elapsedMicros last_time_write_read_servo_cnt = 0;
int ack_write_read = 0; 

//#define VERBOSE_MESSAGE

//Servo 1:
#define SERVO1_serial Serial5
#define SERVO1_serialEnableOpenDrain Serial5EnableOpenDrain
//Servo 2:
#define SERVO2_serial Serial5
#define SERVO2_serialEnableOpenDrain Serial5EnableOpenDrain
//Servo 3:
#define SERVO3_serial Serial5
#define SERVO3_serialEnableOpenDrain Serial5EnableOpenDrain
//Servo 4:
#define SERVO4_serial Serial5
#define SERVO4_serialEnableOpenDrain Serial5EnableOpenDrain

//Servo 5:
#define SERVO5_serial Serial7
#define SERVO5_serialEnableOpenDrain Serial7EnableOpenDrain
//Servo 6:
#define SERVO6_serial Serial7
#define SERVO6_serialEnableOpenDrain Serial7EnableOpenDrain
//Servo 7:
#define SERVO7_serial Serial7
#define SERVO7_serialEnableOpenDrain Serial7EnableOpenDrain
//Servo 8:
#define SERVO8_serial Serial7
#define SERVO8_serialEnableOpenDrain Serial7EnableOpenDrain


//PREPARE HARDWARE TO BE IN TRISTATE MODE
IMXRT_LPUART_t *s_pkuart_1 = &IMXRT_LPUART6;  // underlying hardware for Serial1 
IMXRT_LPUART_t *s_pkuart_2 = &IMXRT_LPUART4;  // underlying hardware for Serial2 
IMXRT_LPUART_t *s_pkuart_3 = &IMXRT_LPUART2;  // underlying hardware for Serial3 
IMXRT_LPUART_t *s_pkuart_4 = &IMXRT_LPUART3;  // underlying hardware for Serial4
IMXRT_LPUART_t *s_pkuart_5 = &IMXRT_LPUART8;  // underlying hardware for Serial5
IMXRT_LPUART_t *s_pkuart_6 = &IMXRT_LPUART1;  // underlying hardware for Serial6
IMXRT_LPUART_t *s_pkuart_7 = &IMXRT_LPUART7;  // underlying hardware for Serial7


// Define time interrupts for the SERVOs:
IntervalTimer SERVO_COMM_WRITE_READ_TIMER;

int iter_counter_SERVO = 0;

int servo_write_read_lock = 0; 

//Reorganized the servos ID to match a previous design setup already on the vehicle. 
int Servo_1_ID = 6; //Azimuth rotor 1
int Servo_2_ID = 2; //Elevator rotor 1

int Servo_3_ID = 8; //Azimuth rotor 4
int Servo_4_ID = 4; //Elevator rotor 4

int Servo_5_ID = 5; //Azimuth rotor 2
int Servo_6_ID = 1; //Elevator rotor 2

int Servo_7_ID = 7; //Azimuth rotor 3
int Servo_8_ID = 3; //Elevator rotor 3

volatile int Target_position_servo_1, Target_position_servo_2, Target_position_servo_3, Target_position_servo_4;
volatile int Target_position_servo_5, Target_position_servo_6, Target_position_servo_7, Target_position_servo_8;

volatile int position_ack_servo_1_lost, position_ack_servo_2_lost, position_ack_servo_3_lost, position_ack_servo_4_lost;
volatile int position_ack_servo_5_lost, position_ack_servo_6_lost, position_ack_servo_7_lost, position_ack_servo_8_lost;

volatile int position_feedback_servo_1_lost, position_feedback_servo_2_lost, position_feedback_servo_3_lost, position_feedback_servo_4_lost; 
volatile int position_feedback_servo_5_lost, position_feedback_servo_6_lost, position_feedback_servo_7_lost, position_feedback_servo_8_lost; 

elapsedMicros pos1_time_uS_counter = 0, pos2_time_uS_counter = 0, pos3_time_uS_counter = 0, pos4_time_uS_counter = 0; 
elapsedMicros pos5_time_uS_counter = 0, pos6_time_uS_counter = 0, pos7_time_uS_counter = 0, pos8_time_uS_counter = 0; 

//Variables for the torque enable feature: 
int servo_status_torque_enable = 1; 
int restart_iter_counter_SERVO_arm = 0, restart_iter_counter_SERVO_disarm = 0;

// ------------------------------ GENERAL VARIABLES---------------------------------------

#define DEBUG_serial Serial
#define DEBUG_serial_baud 115200

elapsedMicros timer_count_servo = 0, timer_count_main = 0, timer_count_esc = 0;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////SETUP///////////////////
void setup(void) 
{
    //////////////////SETUP DEBUGGING USB
    DEBUG_serial.begin(DEBUG_serial_baud);
    
    //////////////////SETUP SERVOS
    InitServos();
    
    //////////////////SETUP MOTORS
    // Initialize the CMD subsystem
    ESCCMD_init( ESCPID_NB_ESC );
  
    // Arming ESCs
    ESCCMD_arm_all( );
    
    // Start periodic loop
    ESCCMD_start_timer( );

    //////////////////SETUP CONNECTION WITH PIXHAWK
    COMMUNICATION_SERIAL.begin(COMMUNICATION_SERIAL_BAUD);

    //LED to monitor the connection with Pixhawk
    pinMode(PIN_LED_T4, OUTPUT);
    analogWriteFrequency(PIN_LED_T4, 500);
    
    //Start the timer for the Communication and Servos 
    SERVO_COMM_WRITE_READ_TIMER.begin(ServosAndCommTic, 10); //Interrupt routine for the tick to servos and communication

    //Give time to the ESC to initialize properly
    while(millis() < 1500) ESCCMD_tic( );
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////LOOP///////////////////
void loop(void) 
{

  TorqueEnableDisableServos();

  writeReadServos();
  
  SendReceivePixhawk();
  
  ServoRoutine();

  EscRoutine();

  //Display statistics
  if(timer_count_main > 5000){ // 200 Hz loop
    //  DisplayEscVoltage();
    //  DisplayEscCurrent();
    //  DisplayEscRpm();
    //  DisplayEscErr();
    //  DisplayEscCmd(); 
    //  DebugServoLostPosPackets();
    //  DebugUpdateTimePosPackets();
    //  DebugServoLostAckPackets();
    //  DebugServoPosition();   
    //  DebugConnection(); 
    //  DebugReceivedCmdMotors();

    timer_count_main = 0;

  }
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////USER DEFINED FCN///////////////////

void DebugReceivedCmdMotors(void){
  DEBUG_serial.print("Motor_1_cmd_dshot:");
  DEBUG_serial.print(myserial_act_t4_in.motor_1_dshot_cmd_int);
  DEBUG_serial.print("Motor_2_cmd_dshot:");
  DEBUG_serial.print(myserial_act_t4_in.motor_2_dshot_cmd_int);
  DEBUG_serial.print("Motor_3_cmd_dshot:");
  DEBUG_serial.print(myserial_act_t4_in.motor_3_dshot_cmd_int);
  DEBUG_serial.print("Motor_4_cmd_dshot:");
  DEBUG_serial.println(myserial_act_t4_in.motor_4_dshot_cmd_int);
}

//Parse the incoming message from Pixhawk and assign them to local variables
void serial_act_t4_parse_msg_in(void){
  
  //Copy received buffer to structure
  memmove(&myserial_act_t4_in,&serial_act_t4_msg_buf_in[1],sizeof(struct serial_act_t4_in)-1);
  extra_data_in[myserial_act_t4_in.rolling_msg_in_id] = myserial_act_t4_in.rolling_msg_in;

  //Apply received message to actuators: 
  if(time_no_connection_pixhawk < 5000 && myserial_act_t4_in.servo_arm_int == 1){
  Target_position_servo_1 = (int) ( constrain( SERVO_MAX_COMD * myserial_act_t4_in.servo_1_cmd_int/36000.0 + (SERVO_MAX_COMD/2.0) , 0, SERVO_MAX_COMD) );
  Target_position_servo_2 = (int) ( constrain( SERVO_MAX_COMD * myserial_act_t4_in.servo_2_cmd_int/36000.0 + (SERVO_MAX_COMD/2.0) , 0, SERVO_MAX_COMD) );
  Target_position_servo_3 = (int) ( constrain( SERVO_MAX_COMD * myserial_act_t4_in.servo_3_cmd_int/36000.0 + (SERVO_MAX_COMD/2.0) , 0, SERVO_MAX_COMD) );
  Target_position_servo_4 = (int) ( constrain( SERVO_MAX_COMD * myserial_act_t4_in.servo_4_cmd_int/36000.0 + (SERVO_MAX_COMD/2.0) , 0, SERVO_MAX_COMD) );
  Target_position_servo_5 = (int) ( constrain( SERVO_MAX_COMD * myserial_act_t4_in.servo_5_cmd_int/36000.0 + (SERVO_MAX_COMD/2.0) , 0, SERVO_MAX_COMD) );
  Target_position_servo_6 = (int) ( constrain( SERVO_MAX_COMD * myserial_act_t4_in.servo_6_cmd_int/36000.0 + (SERVO_MAX_COMD/2.0) , 0, SERVO_MAX_COMD) );
  Target_position_servo_7 = (int) ( constrain( SERVO_MAX_COMD * myserial_act_t4_in.servo_7_cmd_int/36000.0 + (SERVO_MAX_COMD/2.0) , 0, SERVO_MAX_COMD) );
  Target_position_servo_8 = (int) ( constrain( SERVO_MAX_COMD * myserial_act_t4_in.servo_8_cmd_int/36000.0 + (SERVO_MAX_COMD/2.0) , 0, SERVO_MAX_COMD) );
  //For the PWM servo they are handled in the servo_routine() fcn. 
  }

  if(time_no_connection_pixhawk < 5000 && myserial_act_t4_in.motor_arm_int == 1){
  ESCCMD_throttle( 0, constrain(myserial_act_t4_in.motor_1_dshot_cmd_int,MIN_DSHOT_CMD,MAX_DSHOT_CMD) );    
  ESCCMD_throttle( 1, constrain(myserial_act_t4_in.motor_2_dshot_cmd_int,MIN_DSHOT_CMD,MAX_DSHOT_CMD) );  
  ESCCMD_throttle( 2, constrain(myserial_act_t4_in.motor_3_dshot_cmd_int,MIN_DSHOT_CMD,MAX_DSHOT_CMD) );    
  ESCCMD_throttle( 3, constrain(myserial_act_t4_in.motor_4_dshot_cmd_int,MIN_DSHOT_CMD,MAX_DSHOT_CMD) );  
  }

  if(time_no_connection_pixhawk > 5000 || myserial_act_t4_in.motor_arm_int == 0){
  ESCCMD_stop(0);
  ESCCMD_stop(1);
  ESCCMD_stop(2);
  ESCCMD_stop(3);  
  }
  
  if(time_no_connection_pixhawk > 5000 || myserial_act_t4_in.servo_arm_int == 0){
  Target_position_servo_1 = (int) ( SERVO_MAX_COMD/2.0 );
  Target_position_servo_2 = (int) ( SERVO_MAX_COMD/2.0 );
  Target_position_servo_3 = (int) ( SERVO_MAX_COMD/2.0 );
  Target_position_servo_4 = (int) ( SERVO_MAX_COMD/2.0 );
  Target_position_servo_5 = (int) ( SERVO_MAX_COMD/2.0 );
  Target_position_servo_6 = (int) ( SERVO_MAX_COMD/2.0 );
  Target_position_servo_7 = (int) ( SERVO_MAX_COMD/2.0 );
  Target_position_servo_8 = (int) ( SERVO_MAX_COMD/2.0 );     
  }

}

//Collect the telemetry mesage from the ESCs
void CollectEscTelem(void){
  
  //RPM
  int16_t local_rpm1 = 0, local_rpm2 = 0, local_rpm3 = 0, local_rpm4 = 0;
  ESCCMD_read_rpm( 0, &local_rpm1);
  ESCCMD_read_rpm( 1, &local_rpm2);
  ESCCMD_read_rpm( 2, &local_rpm3);
  ESCCMD_read_rpm( 3, &local_rpm4);
  myserial_act_t4_out.motor_1_rpm_int = (int16_t) (abs(local_rpm1*10)); 
  myserial_act_t4_out.motor_2_rpm_int = (int16_t) (abs(local_rpm2*10));  
  myserial_act_t4_out.motor_3_rpm_int = (int16_t) (abs(local_rpm3*10)); 
  myserial_act_t4_out.motor_4_rpm_int = (int16_t) (abs(local_rpm4*10));
  
  //ESC ERROR CODE
  int8_t local_err1 = 0, local_err2 = 0, local_err3 = 0, local_err4 = 0;   
  ESCCMD_read_err( 0, &local_err1 );
  ESCCMD_read_err( 1, &local_err2 );
  ESCCMD_read_err( 2, &local_err3 );
  ESCCMD_read_err( 3, &local_err4 );
  myserial_act_t4_out.motor_1_error_code_int = (int16_t) (local_err1);
  myserial_act_t4_out.motor_2_error_code_int = (int16_t) (local_err2);
  myserial_act_t4_out.motor_3_error_code_int = (int16_t) (local_err3);
  myserial_act_t4_out.motor_4_error_code_int = (int16_t) (local_err4);

  //VOLTAGE 
  uint16_t local_volt1 = 0, local_volt2 = 0, local_volt3 = 0, local_volt4 = 0;   
  ESCCMD_read_volt( 0, &local_volt1 );
  ESCCMD_read_volt( 1, &local_volt2 );
  ESCCMD_read_volt( 2, &local_volt3 );
  ESCCMD_read_volt( 3, &local_volt4 );
  myserial_act_t4_out.motor_1_voltage_int = (int16_t) (local_volt1);
  myserial_act_t4_out.motor_2_voltage_int = (int16_t) (local_volt2);
  myserial_act_t4_out.motor_3_voltage_int = (int16_t) (local_volt3);
  myserial_act_t4_out.motor_4_voltage_int = (int16_t) (local_volt4);
  
  //CURRENT
  uint16_t local_current1 = 0, local_current2 = 0, local_current3 = 0, local_current4 = 0;   
  ESCCMD_read_amp( 0, &local_current1 );  
  ESCCMD_read_amp( 1, &local_current2 ); 
  ESCCMD_read_amp( 2, &local_current3 ); 
  ESCCMD_read_amp( 3, &local_current4 ); 
  myserial_act_t4_out.motor_1_current_int = (int16_t) (local_current1);
  myserial_act_t4_out.motor_2_current_int = (int16_t) (local_current2);
  myserial_act_t4_out.motor_3_current_int = (int16_t) (local_current3);
  myserial_act_t4_out.motor_4_current_int = (int16_t) (local_current4);

  uint16_t local_consumption1 = 0, local_consumption2 = 0, local_consumption3 = 0, local_consumption4 = 0;
  ESCCMD_read_mah( 0, &local_consumption1 ); 
  ESCCMD_read_mah( 1, &local_consumption2 ); 
  ESCCMD_read_mah( 2, &local_consumption3 ); 
  ESCCMD_read_mah( 3, &local_consumption4 ); 
  extra_data_out[0] = (float) (local_consumption1);
  extra_data_out[1] = (float) (local_consumption2);
  extra_data_out[2] = (float) (local_consumption3);
  extra_data_out[3] = (float) (local_consumption4);
  
}

//Receive and send message from Pixhawk 
void SendReceivePixhawk(void){
  
  //SENDING PACKET
  if(ack_comm_TX_pixhawk == 0){
  //Collect the ESC telemetry and update the struct:
  CollectEscTelem();
  //Assign rolling message: 
  myserial_act_t4_out.rolling_msg_out = extra_data_out[rolling_message_out_id_cnt];
  myserial_act_t4_out.rolling_msg_out_id = rolling_message_out_id_cnt;
  rolling_message_out_id_cnt++;

  //Calculate checksum for outbound packet: 
  uint8_t *buf_send = (uint8_t *)&myserial_act_t4_out;
  myserial_act_t4_out.checksum_out = 0;
  for(uint16_t i = 0; i < sizeof(struct serial_act_t4_out) - 1; i++){
    myserial_act_t4_out.checksum_out += buf_send [i];
  }
  
  //Send out packet to buffer:
  COMMUNICATION_SERIAL.write(START_BYTE_SERIAL_ACT_T4);
  COMMUNICATION_SERIAL.write(buf_send,sizeof(struct serial_act_t4_out));
  
  ack_comm_TX_pixhawk = 1;
  last_time_write_to_pixhawk = 0;
  
  }

  //RECEIVING PACKET
  //Reset received packets to zero every 5 second to update the statistics
  if(old_time_frequency_in > 5000){ 
    serial_act_t4_received_packets = 0;
    old_time_frequency_in = 0;
  }
  //Collect packets on the buffer if available:
  while(COMMUNICATION_SERIAL.available()) {
      uint8_t serial_act_t4_byte_in;
      serial_act_t4_byte_in = COMMUNICATION_SERIAL.read();
      if ((serial_act_t4_byte_in == START_BYTE_SERIAL_ACT_T4) || (serial_act_t4_buf_in_cnt > 0)) {
          serial_act_t4_msg_buf_in[serial_act_t4_buf_in_cnt] = serial_act_t4_byte_in;
          serial_act_t4_buf_in_cnt++;
      }
      if (serial_act_t4_buf_in_cnt > sizeof(struct serial_act_t4_in)  ) {
          serial_act_t4_buf_in_cnt = 0;
          uint8_t checksum_in_local = 0;
          for(uint16_t i = 1; i < sizeof(struct serial_act_t4_in) ; i++){
              checksum_in_local += serial_act_t4_msg_buf_in[i];
          }
          if(checksum_in_local == serial_act_t4_msg_buf_in[sizeof(struct serial_act_t4_in)]){
              serial_act_t4_parse_msg_in();
              serial_act_t4_received_packets++;
              time_no_connection_pixhawk = 0;
          }
          else {
              serial_act_t4_missed_packets_in++;              
          }
      }
  }
  serial_act_t4_message_frequency_in = (uint16_t) ( (1.0*serial_act_t4_received_packets/(old_time_frequency_in) )*1000.0);
  //Write the frequency to the status led:
  analogWrite(PIN_LED_T4,serial_act_t4_message_frequency_in*3);
}

//Servo routine to be run inside the main loop
void ServoRoutine(void){
    
    if(timer_count_servo >= 2000 ){ //Run at ~500 Hz 
      timer_count_servo = 0;
      writeEstimatePwmServos();
    }
}

//Write the PWM servos and estimate the dynamics to be sent back to the pixhawk
void writeEstimatePwmServos(void){
  float SERVO_9_FIRST_ORD_DYN_DEN = extra_data_in[0];
  float SERVO_9_FIRST_ORD_DYN_NUM = extra_data_in[1];  
  float SERVO_9_MAX_PWM = extra_data_in[2];
  float SERVO_9_MIN_PWM = extra_data_in[3];
  float SERVO_9_ZERO_PWM = extra_data_in[4];
  float SERVO_9_MIN_ANGLE_DEG = extra_data_in[5];
  float SERVO_9_MAX_ANGLE_DEG = extra_data_in[6];
  int SERVO_9_DELAY_TS = (int) extra_data_in[7];
  
  float SERVO_10_FIRST_ORD_DYN_DEN = extra_data_in[8];
  float SERVO_10_FIRST_ORD_DYN_NUM = extra_data_in[9];  
  float SERVO_10_MAX_PWM = extra_data_in[10];
  float SERVO_10_MIN_PWM = extra_data_in[11];
  float SERVO_10_ZERO_PWM = extra_data_in[12];
  float SERVO_10_MIN_ANGLE_DEG = extra_data_in[13];
  float SERVO_10_MAX_ANGLE_DEG = extra_data_in[14];
  int SERVO_10_DELAY_TS = (int) extra_data_in[15];
  
  //Protection to max and min values, in case the Pixhawk is not yet connected: 
  if(SERVO_9_MAX_PWM == 0 || SERVO_9_MIN_PWM == 0 || SERVO_9_ZERO_PWM == 0){
    SERVO_9_MAX_PWM = 1500; 
    SERVO_9_MIN_PWM = 1500; 
    SERVO_9_ZERO_PWM = 1500; 
  }
  if(SERVO_10_MAX_PWM == 0 || SERVO_10_MIN_PWM == 0 || SERVO_10_ZERO_PWM == 0){
    SERVO_10_MAX_PWM = 1500; 
    SERVO_10_MIN_PWM = 1500; 
    SERVO_10_ZERO_PWM = 1500; 
  }
  
  //Apply the pwm values to the servos: 
  int servo9_PWM_value = SERVO_9_ZERO_PWM; 
  if( myserial_act_t4_in.servo_9_cmd_int >= 0){
    servo9_PWM_value += (int) ( ((myserial_act_t4_in.servo_9_cmd_int/100.0)/SERVO_9_MAX_ANGLE_DEG) * (SERVO_9_MAX_PWM - SERVO_9_ZERO_PWM) );
  }
  else{
    servo9_PWM_value += (int) ( ((myserial_act_t4_in.servo_9_cmd_int/100.0)/SERVO_9_MIN_ANGLE_DEG) * (SERVO_9_MIN_PWM - SERVO_9_ZERO_PWM) );
  }

  int servo10_PWM_value = SERVO_10_ZERO_PWM; 
  if( myserial_act_t4_in.servo_10_cmd_int >= 0){
    servo10_PWM_value += (int) ( ((myserial_act_t4_in.servo_10_cmd_int/100.0)/SERVO_10_MAX_ANGLE_DEG) * (SERVO_10_MAX_PWM - SERVO_10_ZERO_PWM) );
  }
  else{
    servo10_PWM_value += (int) ( ((myserial_act_t4_in.servo_10_cmd_int/100.0)/SERVO_10_MIN_ANGLE_DEG) * (SERVO_10_MIN_PWM - SERVO_10_ZERO_PWM) );
  }      
  //Bound the PWM output: 
  servo9_PWM_value = constrain(servo9_PWM_value, min(SERVO_9_MIN_PWM,SERVO_9_MAX_PWM), max(SERVO_9_MIN_PWM,SERVO_9_MAX_PWM) );
  servo10_PWM_value = constrain(servo10_PWM_value, min(SERVO_10_MIN_PWM,SERVO_10_MAX_PWM), max(SERVO_10_MIN_PWM,SERVO_10_MAX_PWM) );
  
  //WRITE TO PIN: 
  analogWrite(SERVO9_PWM_PIN,PWM_to_pulse_multiplier*servo9_PWM_value);
  analogWrite(SERVO10_PWM_PIN,PWM_to_pulse_multiplier*servo10_PWM_value);
  
  //Servo 9 estimation      
  Servo9_state = - SERVO_9_FIRST_ORD_DYN_DEN * Servo9_state_old +
              SERVO_9_FIRST_ORD_DYN_NUM * servo_9_state_memory[servo_state_mem_buf_size - SERVO_9_DELAY_TS - 1];
  Servo9_state = constrain(Servo9_state,SERVO_9_MIN_ANGLE_DEG,SERVO_9_MAX_ANGLE_DEG);
  //Assign servo 9 to telemetry back for the estimation: 
  myserial_act_t4_out.servo_9_angle_int = (int16_t) (Servo9_state*100); 

  //Servo 10 estimation      
  Servo10_state = - SERVO_10_FIRST_ORD_DYN_DEN * Servo10_state_old +
              SERVO_10_FIRST_ORD_DYN_NUM * servo_10_state_memory[servo_state_mem_buf_size - SERVO_10_DELAY_TS - 1];
  Servo10_state = constrain(Servo10_state,SERVO_10_MIN_ANGLE_DEG,SERVO_10_MAX_ANGLE_DEG);
  myserial_act_t4_out.servo_10_angle_int = (int16_t) (Servo10_state*100); 
  
  //Assign the memory variables:
  Servo9_state_old = Servo9_state;
  Servo10_state_old = Servo10_state;
  for (int j = 1; j < servo_state_mem_buf_size ; j++){
    servo_9_state_memory[j-1] = servo_9_state_memory[j];
    servo_10_state_memory[j-1] = servo_10_state_memory[j];
  }
  servo_9_state_memory[servo_state_mem_buf_size-1] = myserial_act_t4_in.servo_9_cmd_int/100.0;   
  servo_10_state_memory[servo_state_mem_buf_size-1] = myserial_act_t4_in.servo_10_cmd_int/100.0;   
}

//Esc routine to be run inside the main loop 
void EscRoutine(void){
  static int ret;

  // Keep timing awake
  ret = ESCCMD_tic( );
  if(ret == ESCCMD_TIC_OCCURED){ //run at tick time, around 500 Hz
    
  }
  
  if(timer_count_esc >= 2000 ){ //Run at ~500 Hz 
    timer_count_esc = 0;

  }     
  
}

//Debug the Ack received by the servos after we set a desired angle
void DebugServoPosition(void){
      DEBUG_serial.print("Servo_1_position_deg:");
      DEBUG_serial.print(myserial_act_t4_out.servo_1_angle_int/100.0);  
      DEBUG_serial.print(",Servo_2_position_deg:");
      DEBUG_serial.print(myserial_act_t4_out.servo_2_angle_int/100.0);  
      DEBUG_serial.print(",Servo_3_position_deg:");
      DEBUG_serial.print(myserial_act_t4_out.servo_3_angle_int/100.0);  
      DEBUG_serial.print(",Servo_4_position_deg:");
      DEBUG_serial.print(myserial_act_t4_out.servo_4_angle_int/100.0);  
      DEBUG_serial.print(",Servo_5_position_deg:");
      DEBUG_serial.print(myserial_act_t4_out.servo_5_angle_int/100.0);  
      DEBUG_serial.print(",Servo_6_position_deg:");
      DEBUG_serial.print(myserial_act_t4_out.servo_6_angle_int/100.0);  
      DEBUG_serial.print(",Servo_7_position_deg:");
      DEBUG_serial.print(myserial_act_t4_out.servo_7_angle_int/100.0);        
      DEBUG_serial.print(",Servo_8_position_deg:");
      DEBUG_serial.println(myserial_act_t4_out.servo_8_angle_int/100.0);                                        
}

//Debug the Ack received by the servos after we set a desired angle
void DebugServoLostAckPackets(void){
      DEBUG_serial.print("Ack_S1_lost:");
      DEBUG_serial.print(position_ack_servo_1_lost);  
      DEBUG_serial.print(",Ack_S2_lost:");
      DEBUG_serial.print(position_ack_servo_2_lost);
      DEBUG_serial.print(",Ack_S3_lost:");
      DEBUG_serial.print(position_ack_servo_3_lost);
      DEBUG_serial.print(",Ack_S4_lost:");
      DEBUG_serial.print(position_ack_servo_4_lost);
      DEBUG_serial.print(",Ack_S5_lost:");
      DEBUG_serial.print(position_ack_servo_5_lost);
      DEBUG_serial.print(",Ack_S6_lost:");
      DEBUG_serial.print(position_ack_servo_6_lost);
      DEBUG_serial.print(",Ack_S7_lost:");
      DEBUG_serial.print(position_ack_servo_7_lost);
      DEBUG_serial.print(",Ack_S8_lost:");
      DEBUG_serial.println(position_ack_servo_8_lost);                                         
}

//Displays the amount of lost position feedback packets 
void DebugServoLostPosPackets(void){
      DEBUG_serial.print("Pos_S1_lost:");
      DEBUG_serial.print(position_feedback_servo_1_lost);      
      DEBUG_serial.print(",Pos_S2_lost:");
      DEBUG_serial.print(position_feedback_servo_2_lost);  
      DEBUG_serial.print(",Pos_S3_lost:");
      DEBUG_serial.print(position_feedback_servo_3_lost);  
      DEBUG_serial.print(",Pos_S4_lost:");
      DEBUG_serial.print(position_feedback_servo_4_lost);   
      DEBUG_serial.print(",Pos_S5_lost:");
      DEBUG_serial.print(position_feedback_servo_5_lost);  
      DEBUG_serial.print(",Pos_S6_lost:");
      DEBUG_serial.print(position_feedback_servo_6_lost);
      DEBUG_serial.print(",Pos_S7_lost:");
      DEBUG_serial.print(position_feedback_servo_7_lost);  
      DEBUG_serial.print(",Pos_S8_lost:");
      DEBUG_serial.println(position_feedback_servo_8_lost);                                           
}

//Displays the refresh rate of each servo (not in PWM)
void DebugUpdateTimePosPackets(void){
      DEBUG_serial.print("Pos_S1_time_update:");
      DEBUG_serial.print(myserial_act_t4_out.servo_1_update_time_us);      
      DEBUG_serial.print(",Pos_S2_time_update:");
      DEBUG_serial.print(myserial_act_t4_out.servo_2_update_time_us); 
      DEBUG_serial.print(",Pos_S3_time_update:");
      DEBUG_serial.print(myserial_act_t4_out.servo_3_update_time_us); 
      DEBUG_serial.print(",Pos_S4_time_update:");
      DEBUG_serial.print(myserial_act_t4_out.servo_4_update_time_us); 
      DEBUG_serial.print(",Pos_S5_time_update:");
      DEBUG_serial.print(myserial_act_t4_out.servo_5_update_time_us); 
      DEBUG_serial.print(",Pos_S6_time_update:");
      DEBUG_serial.print(myserial_act_t4_out.servo_6_update_time_us);  
      DEBUG_serial.print(",Pos_S7_time_update:");
      DEBUG_serial.print(myserial_act_t4_out.servo_7_update_time_us);   
      DEBUG_serial.print(",Pos_S8_time_update:");
      DEBUG_serial.println(myserial_act_t4_out.servo_8_update_time_us);                                        
}

//Displays some info about the connection with the pixhawk
void DebugConnection(void){
  DEBUG_serial.print("Missed_packet:");
  DEBUG_serial.print(serial_act_t4_missed_packets_in);
  DEBUG_serial.print(",Frequency_in:");
  DEBUG_serial.println(serial_act_t4_message_frequency_in);  
}

// Initialize all the serials of the servos including the tristate mode and PWM servos
void InitServos(void){

  SERVO1_serial.begin(BAUDRATE_SERVO); //1,2,3,4 on the same serial, not needed to recall them 
  SERVO1_serialEnableOpenDrain(true); 
  
  SERVO5_serial.begin(BAUDRATE_SERVO); //5,6,7,8 on the same serial, not needed to recall them 
  SERVO5_serialEnableOpenDrain(true);     
    
  // Serial 1 half duplex mode:
  // s_pkuart_1->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;
  
  // Serial 2 half duplex mode:
  // s_pkuart_2->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;
  
  // Serial 3 half duplex mode:
  // s_pkuart_3->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;
  
  // Serial 4 half duplex mode:
  // s_pkuart_4->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;
    
  // Serial 5 half duplex mode:
  s_pkuart_5->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC; //Servos 1,2,3,4 on the same serial, not needed to recall it 
    
  // Serial 6 half duplex mode:
  // s_pkuart_6->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;    
      
  //Serial 7 half duplex mode:
  s_pkuart_7->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC; //Servos 5,6,7,8 on the same serial, not needed to recall it 

  //INIT PWM SERVOS: 
  pinMode(SERVO9_PWM_PIN, OUTPUT);
  pinMode(SERVO10_PWM_PIN, OUTPUT);
  analogWriteFrequency(SERVO9_PWM_PIN, PWM_SERVO_FREQUENCY); // Servo frequency
  analogWriteFrequency(SERVO10_PWM_PIN, PWM_SERVO_FREQUENCY); // Servo frequency
  analogWriteResolution(12); //4096 values for duty cycle 
  PWM_to_pulse_multiplier = (1e-6 * 4096.0 * PWM_SERVO_FREQUENCY);
    
}

// Send instructions to servo 1 
void SendInstructionServo1(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO1_serial.clear();
    SERVO1_serialEnableOpenDrain(false);
    SERVO1_serial.write(buffer_tx, buffer_tx_idx);
    SERVO1_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 1: ", buffer_tx, buffer_tx_idx);
    #endif
}

// Send instructions to servo 2 
void SendInstructionServo2(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO2_serial.clear();
    SERVO2_serialEnableOpenDrain(false);
    SERVO2_serial.write(buffer_tx, buffer_tx_idx);
    SERVO2_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 2: ", buffer_tx, buffer_tx_idx);
    #endif
}

// Send instructions to servo 3 
void SendInstructionServo3(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO3_serial.clear();
    SERVO3_serialEnableOpenDrain(false);
    SERVO3_serial.write(buffer_tx, buffer_tx_idx);
    SERVO3_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 3: ", buffer_tx, buffer_tx_idx);
    #endif
}

// Send instructions to servo 4
void SendInstructionServo4(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO4_serial.clear();
    SERVO4_serialEnableOpenDrain(false);
    SERVO4_serial.write(buffer_tx, buffer_tx_idx);
    SERVO4_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 4: ", buffer_tx, buffer_tx_idx);
    #endif
}

// Send instructions to servo 5 
void SendInstructionServo5(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO5_serial.clear();
    SERVO5_serialEnableOpenDrain(false);
    SERVO5_serial.write(buffer_tx, buffer_tx_idx);
    SERVO5_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 5: ", buffer_tx, buffer_tx_idx);
    #endif
}

// Send instructions to servo 6 
void SendInstructionServo6(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO6_serial.clear();
    SERVO6_serialEnableOpenDrain(false);
    SERVO6_serial.write(buffer_tx, buffer_tx_idx);
    SERVO6_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 6: ", buffer_tx, buffer_tx_idx);
    #endif
}

// Send instructions to servo 7 
void SendInstructionServo7(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO7_serial.clear();
    SERVO7_serialEnableOpenDrain(false);
    SERVO7_serial.write(buffer_tx, buffer_tx_idx);
    SERVO7_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 7: ", buffer_tx, buffer_tx_idx);
    #endif
}

// Send instructions to servo 8 
void SendInstructionServo8(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount){
    // -------- SEND INSTRUCTION ------------   
    int buffer_tx_idx = 0; 
    byte buffer_tx[50];
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = u8_ServoID;
    buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
    buffer_tx[buffer_tx_idx++] = u8_Instruction;
    
    for (int i=0; i<s32_ParamCount; i++)
    {
        buffer_tx[buffer_tx_idx++] = u8_Params[i];
    }

    byte u8_Checksum = 0;
    for (int i=2; i<buffer_tx_idx; i++)
    {
        u8_Checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

    //Send the instruction to servo
    SERVO8_serial.clear();
    SERVO8_serialEnableOpenDrain(false);
    SERVO8_serial.write(buffer_tx, buffer_tx_idx);
    SERVO8_serialEnableOpenDrain(true);
    
    #ifdef VERBOSE_MESSAGE
      PrintHex("Instruction sent servo 8: ", buffer_tx, buffer_tx_idx);
    #endif
}

// Print a desired buffer in HEX mode
void PrintHex(const char* s8_Text, const byte* data, int count){
    DEBUG_serial.print(s8_Text);
    
    for (int i=0; i < count; i++)
    {
        if (i > 0)
            DEBUG_serial.print(" ");
        
        if (data[i] <= 0xF)
            DEBUG_serial.print("0");
          
        DEBUG_serial.print(data[i], HEX);
    }
    DEBUG_serial.println();
}

// 1 16-bit split into 2 8 digits
void SplitByte(uint8_t *DataL, uint8_t* DataH, uint16_t Data){
  *DataH = (Data>>8);
  *DataL = (Data&0xff);
}

// 2 8-digit combinations for 1 16-digit number
uint16_t CompactBytes(uint8_t DataL, uint8_t DataH){
  uint16_t Data;
  Data = DataL;
  Data<<=8;
  Data |= DataH;
  return Data;
}

//Keep track of the tick to Servos and pixhawk communication
void ServosAndCommTic(void){
  //Servo tic
  if(ack_write_read && last_time_write_read_servo_cnt >= SERVO_COMM_MARGIN){
    iter_counter_SERVO++;
    ack_write_read = 0;
    if(iter_counter_SERVO > 16){
      iter_counter_SERVO = 1;
    }
  }
  //Communication tic
  if(ack_comm_TX_pixhawk && last_time_write_to_pixhawk >= COMM_REFRESH_TIME){
    ack_comm_TX_pixhawk = 0;
  }
}

//Enable or disable the torque of the servos and the possibility for them to move. 
void TorqueEnableDisableServos(void){

  byte u8_Data_send[2] = { 0 }; //write SDRAM

  //Arm all servos if there is a mismatch between the current servo state and the required one:
  if(servo_status_torque_enable == 0 && myserial_act_t4_in.servo_arm_int == 1){ 

    //To be run only once!! --> prevent main loop to write to servo and restart iter_counter_SERVO variable  
    if(restart_iter_counter_SERVO_arm == 0){
      //Reset the iter_counter_SERVO variable giving at least two iterations of margin to avoid any conflicts with the pending messages:
      iter_counter_SERVO = -1; 
      //Prevent the writeReadServos loop to write to servos while we are doing so 
      servo_write_read_lock = 1;
      restart_iter_counter_SERVO_arm = 1;
    }

    if(ack_write_read == 0 && servo_write_read_lock == 1){ //Write only if the other loop is not writing

      if(iter_counter_SERVO == 1 || iter_counter_SERVO == 9){ //Request to enable torque hold to servo 1
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 1; 
        SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }

      if(iter_counter_SERVO == 2 || iter_counter_SERVO == 10){ //Request to enable torque hold to servo 5
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 1; 
        SendInstructionServo5(Servo_5_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }   

      if(iter_counter_SERVO == 3 || iter_counter_SERVO == 11){ //Request to enable torque hold to servo 2
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 1; 
        SendInstructionServo2(Servo_2_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }

      if(iter_counter_SERVO == 4 || iter_counter_SERVO == 12){ //Request to enable torque hold to servo 6
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 1; 
        SendInstructionServo6(Servo_6_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }   

      if(iter_counter_SERVO == 5 || iter_counter_SERVO == 13){ //Request to enable torque hold to servo 3
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 1; 
        SendInstructionServo3(Servo_3_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }

      if(iter_counter_SERVO == 6 || iter_counter_SERVO == 14){ //Request to enable torque hold to servo 7
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 1; 
        SendInstructionServo7(Servo_7_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }   

      if(iter_counter_SERVO == 7 || iter_counter_SERVO == 15){ //Request to enable torque hold to servo 4
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 1; 
        SendInstructionServo4(Servo_4_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }

      if(iter_counter_SERVO == 8 || iter_counter_SERVO == 16){ //Request to enable torque hold to servo 8
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 1; 
        SendInstructionServo8(Servo_8_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }   

      // Re-enable the main loop to write to servo, restart the iter_counter_SERVO and restart_iter_counter_SERVO variables and register the new servo states
      if(iter_counter_SERVO == 16){
        //Reset the iter_counter_SERVO variable giving at least two iterations of margin to avoid any conflicts with the pending messages:
        iter_counter_SERVO = -1; 
        //Enable the writeReadServos loop to write to servos
        servo_write_read_lock = 0;
        //Restart the restart_iter_counter_SERVO variable for the next time we want to enable the torque again
        restart_iter_counter_SERVO_arm = 0;
        //Register the new servo state
        servo_status_torque_enable = 1;           
      }   

      //Say to the tick that it can now increase the counter reporting also the time information
      ack_write_read = 1; 
      last_time_write_read_servo_cnt = 0;
    }
  }
  else{
    //Make sure the variables are unlocked to prevent others to write to servos.
    //This can happen if we get out from the sending torque enable loop before the end. In that case restart_iter_counter_SERVO_arm will be equal to one.
    if(restart_iter_counter_SERVO_arm == 1){
      //Again, restart the counter to avoid transmitting conflicts
      iter_counter_SERVO = -1;
      servo_write_read_lock = 0;
      restart_iter_counter_SERVO_arm = 0;
      //Since we don't know how far the enable torque loop went, we flip the servo_status_torque_enable status to false
      servo_status_torque_enable = 0;
    }    
  }

  //Disarm all servos if there is a mismatch between the current servo state and the required one:
  if(servo_status_torque_enable == 1 && myserial_act_t4_in.servo_arm_int == 0){ 

    //To be run only once!! --> prevent main loop to write to servo and restart iter_counter_SERVO variable  
    if(restart_iter_counter_SERVO_disarm == 0){
      //Reset the iter_counter_SERVO variable giving at least two iterations of margin to avoid any conflicts with the pending messages:
      iter_counter_SERVO = -1; 
      //Prevent the writeReadServos loop to write to servos while we are doing so 
      servo_write_read_lock = 1;
      restart_iter_counter_SERVO_disarm = 1;
    }

    if(ack_write_read == 0 && servo_write_read_lock == 1){ //Write only if the other loop is not writing

      if(iter_counter_SERVO == 1 || iter_counter_SERVO == 9){ //Request to disable torque hold to servo 1
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 0; 
        SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }

      if(iter_counter_SERVO == 2 || iter_counter_SERVO == 10){ //Request to disable torque hold to servo 5
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 0; 
        SendInstructionServo5(Servo_5_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }   

      if(iter_counter_SERVO == 3 || iter_counter_SERVO == 11){ //Request to disable torque hold to servo 2
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 0; 
        SendInstructionServo2(Servo_2_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }

      if(iter_counter_SERVO == 4 || iter_counter_SERVO == 12){ //Request to disable torque hold to servo 6
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 0; 
        SendInstructionServo6(Servo_6_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }   

      if(iter_counter_SERVO == 5 || iter_counter_SERVO == 13){ //Request to disable torque hold to servo 3
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 0; 
        SendInstructionServo3(Servo_3_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }

      if(iter_counter_SERVO == 6 || iter_counter_SERVO == 14){ //Request to disable torque hold to servo 7
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 0; 
        SendInstructionServo7(Servo_7_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }   

      if(iter_counter_SERVO == 7 || iter_counter_SERVO == 15){ //Request to disable torque hold to servo 4
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 0; 
        SendInstructionServo4(Servo_4_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }

      if(iter_counter_SERVO == 8 || iter_counter_SERVO == 16){ //Request to disable torque hold to servo 8
        u8_Data_send[0] = SBS_TORQUE_ENABLE;
        u8_Data_send[1] = 0; 
        SendInstructionServo8(Servo_8_ID, INST_WRITE, u8_Data_send, sizeof(u8_Data_send));   
      }   

      // Re-enable the main loop to write to servo, restart the iter_counter_SERVO and restart_iter_counter_SERVO variables and register the new servo states
      if(iter_counter_SERVO == 16){
        //Reset the iter_counter_SERVO variable giving at least two iterations of margin to avoid any conflicts with the pending messages:
        iter_counter_SERVO = -1; 
        //Enable the writeReadServos loop to write to servos
        servo_write_read_lock = 0;
        //Restart the restart_iter_counter_SERVO_disarm variable for the next time we want to enable the torque again
        restart_iter_counter_SERVO_disarm = 0;
        //Register the new servo state
        servo_status_torque_enable = 0; 
      }   

      //Say to the tick that it can now increase the counter reporting also the time information
      ack_write_read = 1; 
      last_time_write_read_servo_cnt = 0;
    }
  }
  else{
    //Make sure the variables are unlocked to prevent others to write to servos.
    //This can happen if we get out from the sending torque disable loop before the end. In that case restart_iter_counter_SERVO_disarm will be equal to one.
    if(restart_iter_counter_SERVO_disarm == 1){
      //Again, restart the counter to avoid transmitting conflicts
      iter_counter_SERVO = -1;
      servo_write_read_lock = 0;
      restart_iter_counter_SERVO_disarm = 0;
      //Since we don't know how far the disable torque loop went, we flip the servo_status_torque_enable status to true
      servo_status_torque_enable = 1;
    }      
  }

} 

//Interrupt routine to read and write to servos
void writeReadServos(void){
  
  byte buffer_servo[50] = { 0 };
  int buffer_servo_idx = 0;
  byte u8_Data_1[7] = { 0 }; //Send position frame
  byte u8_Data_2[2] = { 0 }; //request data frame

  if(ack_write_read == 0 && servo_write_read_lock == 0){

    if(iter_counter_SERVO == 1){ //Receive position feedback from servo 4
      while(SERVO4_serial.available()){
        buffer_servo[buffer_servo_idx] = SERVO4_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 7; i++){ //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 4: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_out.servo_4_update_time_us = (int16_t) pos4_time_uS_counter;
      if(255 - bitsum_servo == buffer_servo[7] && buffer_servo[2] == Servo_4_ID){
        myserial_act_t4_out.servo_4_angle_int = (int16_t) ( ( CompactBytes(buffer_servo[6],buffer_servo[5]) - SERVO_MAX_COMD/2 ) *36000/SERVO_MAX_COMD );
        pos4_time_uS_counter = 0;
      }
      else{
        position_feedback_servo_4_lost++;
      }    
      SERVO4_serial.flush();
    }

    if(iter_counter_SERVO == 2){ //Receive position feedback from servo 8
      while(SERVO8_serial.available()){
        buffer_servo[buffer_servo_idx] = SERVO8_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 7; i++){ //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 8: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_out.servo_8_update_time_us = (int16_t) pos8_time_uS_counter;
      if(255 - bitsum_servo == buffer_servo[7] && buffer_servo[2] == Servo_8_ID){
        myserial_act_t4_out.servo_8_angle_int = (int16_t) ( ( CompactBytes(buffer_servo[6],buffer_servo[5]) - SERVO_MAX_COMD/2 ) *36000/SERVO_MAX_COMD );
        pos8_time_uS_counter = 0;
      }
      else{
        position_feedback_servo_8_lost++;
      }    
      SERVO8_serial.flush();
    }

    if(iter_counter_SERVO == 1){ //Send position target to servo 1 or wait in case of servo KILL       
      if(myserial_act_t4_in.servo_arm_int){
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1],&u8_Data_1[2],Target_position_servo_1);
        SplitByte(&u8_Data_1[4],&u8_Data_1[3],0);
        SplitByte(&u8_Data_1[6],&u8_Data_1[5],0);
        SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); //Servo 1 
      }
      else{
        delayMicroseconds(TIME_OF_SERVO_TX); 
      }
    }

    if(iter_counter_SERVO == 2){ //Send position target to servo 5 or wait in case of servo KILL   
      if(myserial_act_t4_in.servo_arm_int){
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1],&u8_Data_1[2],Target_position_servo_5);
        SplitByte(&u8_Data_1[4],&u8_Data_1[3],0);
        SplitByte(&u8_Data_1[6],&u8_Data_1[5],0);
        SendInstructionServo5(Servo_5_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); //Servo 5 
      }
      else{
        delayMicroseconds(TIME_OF_SERVO_TX); 
      }
    }
  
    if(iter_counter_SERVO == 3){ //Receive Ack from servo 1
      //Collect Ack response from servo 1
      while(SERVO1_serial.available()){ 
        buffer_servo[buffer_servo_idx] = SERVO1_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 5;i++){ //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 1: ", &buffer_servo[0], 6);
      #endif
      if((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_1_ID)==0){ //If checksum is incorrect, increase counter.
      position_ack_servo_1_lost++;  
      }
      SERVO1_serial.flush();
    }

    if(iter_counter_SERVO == 4){ //Receive Ack from servo 5
      //Collect Ack response from servo 5
      while(SERVO5_serial.available()){ 
        buffer_servo[buffer_servo_idx] = SERVO5_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 5;i++){ //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 5: ", &buffer_servo[0], 6);
      #endif
      if((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_5_ID)==0){ //If checksum is incorrect, increase counter.
      position_ack_servo_5_lost++;  
      }
      SERVO5_serial.flush();
    }

    if(iter_counter_SERVO == 3){ //Send position target to servo 2 or wait in case of servo KILL   
      if(myserial_act_t4_in.servo_arm_int){
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1],&u8_Data_1[2],Target_position_servo_2);
        SplitByte(&u8_Data_1[4],&u8_Data_1[3],0);
        SplitByte(&u8_Data_1[6],&u8_Data_1[5],0);
        SendInstructionServo2(Servo_2_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); //Servo 2 
      }
      else{
        delayMicroseconds(TIME_OF_SERVO_TX); 
      }
    }

    if(iter_counter_SERVO == 4){ //Send position target to servo 6 or wait in case of servo KILL   
      if(myserial_act_t4_in.servo_arm_int){
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1],&u8_Data_1[2],Target_position_servo_6);
        SplitByte(&u8_Data_1[4],&u8_Data_1[3],0);
        SplitByte(&u8_Data_1[6],&u8_Data_1[5],0);
        SendInstructionServo6(Servo_6_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); //Servo 6
      }
      else{
        delayMicroseconds(TIME_OF_SERVO_TX); 
      }
    }
  
    if(iter_counter_SERVO == 5){ //Receive Ack from servo 2
      //Collect Ack response from servo 2
      while(SERVO2_serial.available()){ 
        buffer_servo[buffer_servo_idx] = SERVO2_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 5;i++){ //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 2: ", &buffer_servo[0], 6);
      #endif
      if((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_2_ID)==0){ //If checksum is incorrect, increase counter.
      position_ack_servo_2_lost++;  
      }
      SERVO2_serial.flush();
    }

    if(iter_counter_SERVO == 6){ //Receive Ack from servo 6
      //Collect Ack response from servo 6
      while(SERVO6_serial.available()){ 
        buffer_servo[buffer_servo_idx] = SERVO6_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 5;i++){ //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 6: ", &buffer_servo[0], 6);
      #endif
      if((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_6_ID)==0){ //If checksum is incorrect, increase counter.
      position_ack_servo_6_lost++;  
      }
      SERVO6_serial.flush();
    }

    if(iter_counter_SERVO == 5){ //Send position target to servo 3 or wait in case of servo KILL   
      if(myserial_act_t4_in.servo_arm_int){  
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1],&u8_Data_1[2],Target_position_servo_3);
        SplitByte(&u8_Data_1[4],&u8_Data_1[3],0);
        SplitByte(&u8_Data_1[6],&u8_Data_1[5],0);
        SendInstructionServo3(Servo_3_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); //Servo 3 
      }
      else{
        delayMicroseconds(TIME_OF_SERVO_TX); 
      }    
    }

    if(iter_counter_SERVO == 6){ //Send position target to servo 7 or wait in case of servo KILL   
      if(myserial_act_t4_in.servo_arm_int){  
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1],&u8_Data_1[2],Target_position_servo_7);
        SplitByte(&u8_Data_1[4],&u8_Data_1[3],0);
        SplitByte(&u8_Data_1[6],&u8_Data_1[5],0);
        SendInstructionServo7(Servo_7_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); //Servo 7 
      }
      else{
        delayMicroseconds(TIME_OF_SERVO_TX); 
      }
    }
  
    if(iter_counter_SERVO == 7){ //Receive Ack from servo 3
      //Collect Ack response from servo 3
      while(SERVO3_serial.available()){ 
        buffer_servo[buffer_servo_idx] = SERVO3_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 5;i++){ //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 3: ", &buffer_servo[0], 6);
      #endif
      if((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_3_ID)==0){ //If checksum is incorrect, increase counter.
      position_ack_servo_3_lost++;  
      }
      SERVO3_serial.flush();
    }

    if(iter_counter_SERVO == 8){ //Receive Ack from servo 7
      //Collect Ack response from servo 7
      while(SERVO7_serial.available()){ 
        buffer_servo[buffer_servo_idx] = SERVO7_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 5;i++){ //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 7: ", &buffer_servo[0], 6);
      #endif
      if((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_7_ID)==0){ //If checksum is incorrect, increase counter.
      position_ack_servo_7_lost++;  
      }
      SERVO7_serial.flush();
    }

    if(iter_counter_SERVO == 7){ //Send position target to servo 4 or wait in case of servo KILL      
      if(myserial_act_t4_in.servo_arm_int){ 
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1],&u8_Data_1[2],Target_position_servo_4);
        SplitByte(&u8_Data_1[4],&u8_Data_1[3],0);
        SplitByte(&u8_Data_1[6],&u8_Data_1[5],0);
        SendInstructionServo4(Servo_4_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); //Servo 4 
      }
      else{
        delayMicroseconds(TIME_OF_SERVO_TX); 
      }    
    }

    if(iter_counter_SERVO == 8){ //Send position target to servo 8 or wait in case of servo KILL   
      if(myserial_act_t4_in.servo_arm_int){ 
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1],&u8_Data_1[2],Target_position_servo_8);
        SplitByte(&u8_Data_1[4],&u8_Data_1[3],0);
        SplitByte(&u8_Data_1[6],&u8_Data_1[5],0);
        SendInstructionServo8(Servo_8_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); //Servo 8 
      }
      else{
        delayMicroseconds(TIME_OF_SERVO_TX); 
      }      
    }
  
    if(iter_counter_SERVO == 9){ //Receive Ack from servo 4
      //Collect Ack response from servo 4
      while(SERVO4_serial.available()){ 
        buffer_servo[buffer_servo_idx] = SERVO4_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 5;i++){ //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 4: ", &buffer_servo[0], 6);
      #endif
      if((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_4_ID)==0){ //If checksum is incorrect, increase counter.
      position_ack_servo_4_lost++;  
      }
      SERVO4_serial.flush();
    }

    if(iter_counter_SERVO == 10){ //Receive Ack from servo 8
      //Collect Ack response from servo 8
      while(SERVO8_serial.available()){ 
        buffer_servo[buffer_servo_idx] = SERVO8_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 5;i++){ //Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 8: ", &buffer_servo[0], 6);
      #endif
      if((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_8_ID)==0){ //If checksum is incorrect, increase counter.
      position_ack_servo_8_lost++;  
      }
      SERVO8_serial.flush();
    }

    if(iter_counter_SERVO == 9){ //Request position feedback to servo 1
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 2; //We want full 16 bit register output
      SendInstructionServo1(Servo_1_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));   
    }

    if(iter_counter_SERVO == 10){ //Request position feedback to servo 5
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 2; //We want full 16 bit register output
      SendInstructionServo5(Servo_5_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));   
    }

    if(iter_counter_SERVO == 11){ //Receive position feedback from servo 1
      while(SERVO1_serial.available()){
        buffer_servo[buffer_servo_idx] = SERVO1_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 7; i++){ //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 1: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_out.servo_1_update_time_us = (int16_t) pos1_time_uS_counter;
      if(255 - bitsum_servo == buffer_servo[7] && buffer_servo[2] == Servo_1_ID){
        myserial_act_t4_out.servo_1_angle_int = (int16_t) ( ( CompactBytes(buffer_servo[6],buffer_servo[5]) - SERVO_MAX_COMD/2 ) *36000/SERVO_MAX_COMD );
        pos1_time_uS_counter = 0;
      }
      else{
        position_feedback_servo_1_lost++;
      }    
      SERVO1_serial.flush();
    }

    if(iter_counter_SERVO == 12){ //Receive position feedback from servo 5
      while(SERVO5_serial.available()){
        buffer_servo[buffer_servo_idx] = SERVO5_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 7; i++){ //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 5: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_out.servo_5_update_time_us = (int16_t) pos5_time_uS_counter;
      if(255 - bitsum_servo == buffer_servo[7] && buffer_servo[2] == Servo_5_ID){
        myserial_act_t4_out.servo_5_angle_int = (int16_t) ( ( CompactBytes(buffer_servo[6],buffer_servo[5]) - SERVO_MAX_COMD/2 ) *36000/SERVO_MAX_COMD );
        pos5_time_uS_counter = 0;
      }
      else{
        position_feedback_servo_5_lost++;
      }    
      SERVO5_serial.flush();
    }

    if(iter_counter_SERVO == 11){ //Request position feedback to servo 2
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 2; //We want full 16 bit register output
      SendInstructionServo2(Servo_2_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));   
    }

    if(iter_counter_SERVO == 12){ //Request position feedback to servo 6
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 2; //We want full 16 bit register output
      SendInstructionServo6(Servo_6_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));   
    }

    if(iter_counter_SERVO == 13){ //Receive position feedback from servo 2
      while(SERVO2_serial.available()){
        buffer_servo[buffer_servo_idx] = SERVO2_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 7; i++){ //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 2: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_out.servo_2_update_time_us = (int16_t) pos2_time_uS_counter;
      if(255 - bitsum_servo == buffer_servo[7] && buffer_servo[2] == Servo_2_ID){
        myserial_act_t4_out.servo_2_angle_int = (int16_t) ( ( CompactBytes(buffer_servo[6],buffer_servo[5]) - SERVO_MAX_COMD/2 ) *36000/SERVO_MAX_COMD );
        pos2_time_uS_counter = 0;
      }
      else{
        position_feedback_servo_2_lost++;
      }    
      SERVO2_serial.flush();
    }

    if(iter_counter_SERVO == 14){ //Receive position feedback from servo 6
      while(SERVO6_serial.available()){
        buffer_servo[buffer_servo_idx] = SERVO6_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 7; i++){ //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 6: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_out.servo_6_update_time_us = (int16_t) pos6_time_uS_counter;
      if(255 - bitsum_servo == buffer_servo[7] && buffer_servo[2] == Servo_6_ID){
        myserial_act_t4_out.servo_6_angle_int = (int16_t) ( ( CompactBytes(buffer_servo[6],buffer_servo[5]) - SERVO_MAX_COMD/2 ) *36000/SERVO_MAX_COMD );
        pos6_time_uS_counter = 0;
      }
      else{
        position_feedback_servo_6_lost++;
      }    
      SERVO6_serial.flush();
    }

    if(iter_counter_SERVO == 13){ //Request position feedback to servo 3
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 2; //We want full 16 bit register output
      SendInstructionServo3(Servo_3_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));   
    }

    if(iter_counter_SERVO == 14){ //Request position feedback to servo 7
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 2; //We want full 16 bit register output
      SendInstructionServo7(Servo_7_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));   
    }

    if(iter_counter_SERVO == 15){ //Receive position feedback from servo 3
      while(SERVO3_serial.available()){
        buffer_servo[buffer_servo_idx] = SERVO3_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 7; i++){ //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 3: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_out.servo_3_update_time_us = (int16_t) pos3_time_uS_counter;
      if(255 - bitsum_servo == buffer_servo[7] && buffer_servo[2] == Servo_3_ID){
        myserial_act_t4_out.servo_3_angle_int = (int16_t) ( ( CompactBytes(buffer_servo[6],buffer_servo[5]) - SERVO_MAX_COMD/2 ) *36000/SERVO_MAX_COMD );
        pos3_time_uS_counter = 0;
      }
      else{
        position_feedback_servo_3_lost++;
      }    
      SERVO3_serial.flush();
    }

    if(iter_counter_SERVO == 16){ //Receive position feedback from servo 7
      while(SERVO7_serial.available()){
        buffer_servo[buffer_servo_idx] = SERVO7_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for(int i = 2; i < 7; i++){ //Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
      #ifdef VERBOSE_MESSAGE
        PrintHex("Instruction received from servo 7: ", &buffer_servo[0], buffer_servo_idx);
      #endif
      myserial_act_t4_out.servo_7_update_time_us = (int16_t) pos7_time_uS_counter;
      if(255 - bitsum_servo == buffer_servo[7] && buffer_servo[2] == Servo_7_ID){
        myserial_act_t4_out.servo_7_angle_int = (int16_t) ( ( CompactBytes(buffer_servo[6],buffer_servo[5]) - SERVO_MAX_COMD/2 ) *36000/SERVO_MAX_COMD );
        pos7_time_uS_counter = 0;
      }
      else{
        position_feedback_servo_7_lost++;
      }    
      SERVO7_serial.flush();
    }

    if(iter_counter_SERVO == 15){ //Request position feedback to servo 4
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 2; //We want full 16 bit register output
      SendInstructionServo4(Servo_4_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));   
    }

    if(iter_counter_SERVO == 16){ //Request position feedback to servo 8
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 2; //We want full 16 bit register output
      SendInstructionServo8(Servo_8_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));  
    }

    //Say to the tick that it can now increase the counter reporting also the time information
    ack_write_read = 1; 
    last_time_write_read_servo_cnt = 0;

  }

}

//Active or deactivate Serial 1 tristate mode for half duplex com.
void Serial1EnableOpenDrain(bool bEnable){
    if (bEnable){
      Serial1.flush();  // Make sure we output everything first before changing state.
      s_pkuart_1->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
    }
    else{
      s_pkuart_1->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
    }
}

//Active or deactivate Serial 2 tristate mode for half duplex com.
void Serial2EnableOpenDrain(bool bEnable){
    if (bEnable){
      Serial2.flush();  // Make sure we output everything first before changing state.
      s_pkuart_2->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
    }
    else{
      s_pkuart_2->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
    }
}

//Active or deactivate Serial 3 tristate mode for half duplex com.
void Serial3EnableOpenDrain(bool bEnable){
    if (bEnable){
      Serial3.flush();  // Make sure we output everything first before changing state.
      s_pkuart_3->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
    }
    else{
      s_pkuart_3->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
    }
}

//Active or deactivate Serial 4 tristate mode for half duplex com.
void Serial4EnableOpenDrain(bool bEnable){
    if (bEnable){
      Serial4.flush();  // Make sure we output everything first before changing state.
      s_pkuart_4->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
    }
    else{
      s_pkuart_4->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
    }
}

//Active or deactivate Serial 5 tristate mode for half duplex com.
void Serial5EnableOpenDrain(bool bEnable){
    if (bEnable){
      Serial5.flush();  // Make sure we output everything first before changing state.
      s_pkuart_5->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
    }
    else{
      s_pkuart_5->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
    }
}

//Active or deactivate Serial 6 tristate mode for half duplex com.
void Serial6EnableOpenDrain(bool bEnable){
    if (bEnable){
      Serial6.flush();  // Make sure we output everything first before changing state.
      s_pkuart_6->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
    }
    else{
      s_pkuart_6->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
    }
}

//Active or deactivate Serial 7 tristate mode for half duplex com.
void Serial7EnableOpenDrain(bool bEnable){
    if (bEnable){
      Serial7.flush();  // Make sure we output everything first before changing state.
      s_pkuart_7->CTRL &= ~LPUART_CTRL_TXDIR;  // Set in to RX Mode...
    }
    else{
      s_pkuart_7->CTRL |= LPUART_CTRL_TXDIR;  // Set in to TX Mode...
    }
}

void DisplayEscVoltage(void){
    uint16_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;   
    ESCCMD_read_volt( 0, &local_var1 );
    DEBUG_serial.print("Voltage_ESC_1:");
    DEBUG_serial.print(local_var1/100.0);
    DEBUG_serial.print(",");
    ESCCMD_read_volt( 1, &local_var2 );
    DEBUG_serial.print("Voltage_ESC_2:");
    DEBUG_serial.print(local_var2/100.0);  
    DEBUG_serial.print(",");  
    ESCCMD_read_volt( 2, &local_var3 );
    DEBUG_serial.print("Voltage_ESC_3:");
    DEBUG_serial.print(local_var3/100.0);    
    DEBUG_serial.print(",");  
    ESCCMD_read_volt( 3, &local_var4 );
    DEBUG_serial.print("Voltage_ESC_4:");
    DEBUG_serial.println(local_var4/100.0);      
}

void DisplayEscCurrent(void){
    uint16_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;   
    ESCCMD_read_amp( 0, &local_var1 );
    DEBUG_serial.print("Current_ESC_1:");
    DEBUG_serial.print(local_var1/100.0);
    DEBUG_serial.print(",");
    ESCCMD_read_amp( 1, &local_var2 );
    DEBUG_serial.print("Current_ESC_2:");
    DEBUG_serial.print(local_var2/100.0);  
    DEBUG_serial.print(",");  
    ESCCMD_read_amp( 2, &local_var3 );
    DEBUG_serial.print("Current_ESC_3:");
    DEBUG_serial.print(local_var3/100.0);    
    DEBUG_serial.print(",");  
    ESCCMD_read_amp( 3, &local_var4 );
    DEBUG_serial.print("Current_ESC_4:");
    DEBUG_serial.println(local_var4/100.0);      
}

void DisplayEscRpm(void){
    int16_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;   
    ESCCMD_read_rpm( 0, &local_var1 );
    DEBUG_serial.print("Rpm_ESC_1:");
    DEBUG_serial.print(local_var1);
    DEBUG_serial.print(",");
    ESCCMD_read_rpm( 1, &local_var2 );
    DEBUG_serial.print("Rpm_ESC_2:");
    DEBUG_serial.print(local_var2);  
    DEBUG_serial.print(",");  
    ESCCMD_read_rpm( 2, &local_var3 );
    DEBUG_serial.print("Rpm_ESC_3:");
    DEBUG_serial.print(local_var3);    
    DEBUG_serial.print(",");  
    ESCCMD_read_rpm( 3, &local_var4 );
    DEBUG_serial.print("Rpm_ESC_4:");
    DEBUG_serial.println(local_var4);      
}

void DisplayEscTemp(void){
    uint8_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;   
    ESCCMD_read_deg( 0, &local_var1 );
    DEBUG_serial.print("Temperature_ESC_1:");
    DEBUG_serial.print(local_var1);
    DEBUG_serial.print(",");
    ESCCMD_read_deg( 1, &local_var2 );
    DEBUG_serial.print("Temperature_ESC_2:");
    DEBUG_serial.print(local_var2);  
    DEBUG_serial.print(",");  
    ESCCMD_read_deg( 2, &local_var3 );
    DEBUG_serial.print("Temperature_ESC_3:");
    DEBUG_serial.print(local_var3);    
    DEBUG_serial.print(",");  
    ESCCMD_read_deg( 3, &local_var4 );
    DEBUG_serial.print("Temperature_ESC_4:");
    DEBUG_serial.println(local_var4);      
}

void DisplayEscErr(void){
    int8_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;   
    ESCCMD_read_err( 0, &local_var1 );
    DEBUG_serial.print("Error_ESC_1:");
    DEBUG_serial.print(local_var1);
    DEBUG_serial.print(",");
    ESCCMD_read_err( 1, &local_var2 );
    DEBUG_serial.print("Error_ESC_2:");
    DEBUG_serial.print(local_var2);  
    DEBUG_serial.print(",");  
    ESCCMD_read_err( 2, &local_var3 );
    DEBUG_serial.print("Error_ESC_3:");
    DEBUG_serial.print(local_var3);    
    DEBUG_serial.print(",");  
    ESCCMD_read_err( 3, &local_var4 );
    DEBUG_serial.print("Error_ESC_4:");
    DEBUG_serial.println(local_var4);      
}

void DisplayEscCmd(void){
    uint16_t local_var1 = 0, local_var2 = 0, local_var3 = 0, local_var4 = 0;   
    ESCCMD_read_cmd( 0, &local_var1 );
    DEBUG_serial.print("Cmd_ESC_1:");
    DEBUG_serial.print(local_var1);
    DEBUG_serial.print(",");
    ESCCMD_read_cmd( 1, &local_var2 );
    DEBUG_serial.print("Cmd_ESC_2:");
    DEBUG_serial.print(local_var2);  
    DEBUG_serial.print(",");  
    ESCCMD_read_cmd( 2, &local_var3 );
    DEBUG_serial.print("Cmd_ESC_3:");
    DEBUG_serial.print(local_var3);    
    DEBUG_serial.print(",");  
    ESCCMD_read_cmd( 3, &local_var4 );
    DEBUG_serial.print("Cmd_ESC_4:");
    DEBUG_serial.println(local_var4);      
}
