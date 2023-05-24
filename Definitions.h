    

struct __attribute__((__packed__)) serial_act_t4_out {
    //ESC telemetry & error code
    int16_t motor_1_rpm_int; //RPM
    int16_t motor_2_rpm_int; //RPM
    int16_t motor_3_rpm_int; //RPM
    int16_t motor_4_rpm_int; //RPM
    int16_t motor_1_error_code_int; //ESC 1 error code 
    int16_t motor_2_error_code_int; //ESC 2 error code 
    int16_t motor_3_error_code_int; //ESC 3 error code 
    int16_t motor_4_error_code_int; //ESC 4 error code 
    int16_t motor_1_current_int; //ESC 1 current mA
    int16_t motor_2_current_int; //ESC 2 current mA
    int16_t motor_3_current_int; //ESC 3 current mA
    int16_t motor_4_current_int; //ESC 4 current mA   
    int16_t motor_1_voltage_int; //ESC 1 voltage mV
    int16_t motor_2_voltage_int; //ESC 2 voltage mV
    int16_t motor_3_voltage_int; //ESC 3 voltage mV
    int16_t motor_4_voltage_int; //ESC 4 voltage mV     
    //SERVOS telemetry & update rate 
    int16_t servo_1_angle_int; //Degrees * 100 
    int16_t servo_2_angle_int; //Degrees * 100 
    int16_t servo_3_angle_int; //Degrees * 100 
    int16_t servo_4_angle_int; //Degrees * 100 
    int16_t servo_5_angle_int; //Degrees * 100 
    int16_t servo_6_angle_int; //Degrees * 100 
    int16_t servo_7_angle_int; //Degrees * 100 
    int16_t servo_8_angle_int; //Degrees * 100 
    int16_t servo_9_angle_int; //Degrees * 100 
    int16_t servo_10_angle_int; //Degrees * 100 
    int16_t servo_1_update_time_us; //MicroSeconds
    int16_t servo_2_update_time_us; //MicroSeconds
    int16_t servo_3_update_time_us; //MicroSeconds
    int16_t servo_4_update_time_us; //MicroSeconds
    int16_t servo_5_update_time_us; //MicroSeconds
    int16_t servo_6_update_time_us; //MicroSeconds 
    int16_t servo_7_update_time_us; //MicroSeconds 
    int16_t servo_8_update_time_us; //MicroSeconds 
    int16_t servo_9_update_time_us; //MicroSeconds 
    int16_t servo_10_update_time_us; //MicroSeconds    
    //Rolling message in 
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;   
    //CHECKSUM
    uint8_t checksum_out;
};


struct __attribute__((__packed__)) serial_act_t4_in {
    //ARM cmd
    int8_t motor_arm_int; //Arm motor boolean
    int8_t servo_arm_int; //Arm servo boolean
    //ESC cmd
    int16_t motor_1_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_2_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_3_dshot_cmd_int; //Motor cmd 0 - 1999
    int16_t motor_4_dshot_cmd_int; //Motor cmd 0 - 1999
    //Servo cmd
    int16_t servo_1_cmd_int; //Degrees * 100 
    int16_t servo_2_cmd_int; //Degrees * 100 
    int16_t servo_3_cmd_int; //Degrees * 100 
    int16_t servo_4_cmd_int; //Degrees * 100 
    int16_t servo_5_cmd_int; //Degrees * 100 
    int16_t servo_6_cmd_int; //Degrees * 100 
    int16_t servo_7_cmd_int; //Degrees * 100 
    int16_t servo_8_cmd_int; //Degrees * 100 
    int16_t servo_9_cmd_int; //Degrees * 100 
    int16_t servo_10_cmd_int; //Degrees * 100   
    //Rolling message out
    float rolling_msg_in;
    uint8_t rolling_msg_in_id;
    //CHECKSUM
    uint8_t checksum_in;
};


// Instruction set
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_WRITE 0x83

 // Memory Address
//-------EPROM(Read only)--------
#define SBS_VERSION_L 3
#define SBS_VERSION_H 4

//-------EPROM(Read And write)--------
#define SBS_ID 5
#define SBS_BAUD_RATE 6
#define SBS_MIN_ANGLE_LIMIT_L 9
#define SBS_MIN_ANGLE_LIMIT_H 10
#define SBS_MAX_ANGLE_LIMIT_L 11
#define SBS_MAX_ANGLE_LIMIT_H 12
#define SBS_CW_DEAD 26
#define SBS_CCW_DEAD 27

//-------SRAM(Read & Write)--------
#define SBS_TORQUE_ENABLE 40
#define SBS_GOAL_POSITION_L 42
#define SBS_GOAL_POSITION_H 43
#define SBS_GOAL_TIME_L 44
#define SBS_GOAL_TIME_H 45
#define SBS_GOAL_SPEED_L 46
#define SBS_GOAL_SPEED_H 47
#define SBS_LOCK 48

//-------SRAM(Read Only)--------
#define SBS_PRESENT_POSITION_L 56
#define SBS_PRESENT_POSITION_H 57
#define SBS_PRESENT_SPEED_L 58
#define SBS_PRESENT_SPEED_H 59
#define SBS_PRESENT_LOAD_L 60
#define SBS_PRESENT_LOAD_H 61
#define SBS_PRESENT_VOLTAGE 62
#define SBS_PRESENT_TEMPERATURE 63
#define SBS_MOVING 66
#define SBS_PRESENT_CURRENT_L 69
#define SBS_PRESENT_CURRENT_H 70
