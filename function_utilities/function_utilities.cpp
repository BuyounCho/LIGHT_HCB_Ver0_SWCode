#include "setting.h"
#include "SPI_EEP_ENC.h"
#include "function_utilities.h"
#include "function_CAN.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"

/*******************************************************************************
 * VARIABLE
 ******************************************************************************/

// Board Information
uint8_t BNO = 0;
uint8_t CONTROL_MODE = 0;
uint8_t OPERATING_MODE = 0; // (00 : Moog & Rot, 01 : Moog & Lin, 10 : KNR & Rot, 11 : KNR & Lin, 101 : SW & Lin)
uint8_t SENSING_MODE = 0; // (0 : torque, 1: pressure)
uint8_t SUPPLY_PRESSURE_UPDATE = 0; // (0 : Update Off (constant Ps) , 1 : Update On (variable Ps))

uint8_t CONTROL_UTILITY_MODE = 0;
uint8_t CURRENT_CONTROL_MODE = 0; // (0 : pwm, 1 : current control)
uint8_t FLAG_VALVE_DEADZONE = 0;
uint8_t REFERENCE_MODE = 1;
int16_t CAN_FREQ = 500;
int16_t DIR_JOINT_ENC = 0;
int16_t DIR_VALVE = 0;
int16_t DIR_VALVE_ENC = 0;

float SUPPLY_VOLTAGE = 12.0f;
float VALVE_VOLTAGE_LIMIT = 12.0f;  //v

float P_GAIN_VALVE_POSITION = 0.0f;
float I_GAIN_VALVE_POSITION= 0.0f;
float D_GAIN_VALVE_POSITION= 0.0f;
float P_GAIN_JOINT_POSITION = 0.0f;
float I_GAIN_JOINT_POSITION = 0.0f;
float D_GAIN_JOINT_POSITION = 0.0f;
float P_GAIN_JOINT_TORQUE = 0.0f;
float I_GAIN_JOINT_TORQUE = 0.0f;
float D_GAIN_JOINT_TORQUE = 0.0f;
float P_GAIN_JOINT_TORQUE_FF = 0.0f;
float I_GAIN_JOINT_TORQUE_FF = 0.0f;
float D_GAIN_JOINT_TORQUE_FF = 0.0f;

int16_t K_SPRING = 0.0;
int16_t D_DAMPER = 0.0;

int16_t flag_delay_test = 0;

//float P_GAIN_VALVE_POSITION_OPP = 0.0f;
//float I_GAIN_VALVE_POSITION_OPP= 0.0f;
//float D_GAIN_VALVE_POSITION_OPP= 0.0f;
//float P_GAIN_JOINT_POSITION_OPP = 0.0f;
//float I_GAIN_JOINT_POSITION_OPP = 0.0f;
//float D_GAIN_JOINT_POSITION_OPP = 0.0f;
//float P_GAIN_JOINT_TORQUE_OPP = 0.0f;
//float I_GAIN_JOINT_TORQUE_OPP = 0.0;
//float D_GAIN_JOINT_TORQUE_OPP = 0.0;

int16_t VALVE_DEADZONE_PLUS;
int16_t VALVE_DEADZONE_MINUS;

int16_t VELOCITY_COMP_GAIN;
int16_t COMPLIANCE_GAIN;

int16_t VALVE_CENTER;

int16_t VALVE_FF;

int16_t BULK_MODULUS;

int16_t CHAMBER_VOLUME_A;
int16_t CHAMBER_VOLUME_B;

int16_t PISTON_AREA_A;
int16_t PISTON_AREA_B;
float PISTON_AREA_alpha;
float alpha3 = 1.0f;

float PRES_SUPPLY_NOM = 70.0f;
float PRES_SUPPLY = 70.0f;

int16_t ENC_LIMIT_PLUS;
int16_t ENC_LIMIT_MINUS;

int16_t STROKE;

float Amm = 236.4f;
float beta = 1300000000.0f;
float Ps = 10000000.0f; //100bar = 100*10^5 Pa
float Pt = 0.0f;    // 0bar = 0Pa
//float Kv = 0.00000002635f;          // Q = Kv*xv*sqrt(Ps-Pa)    => 100bar full opening 5LPM    (full opening : xv = 1)  [unit] m^3.5/kg^0.5
float gamma_hat = 1075.0f; // Kv*beta*A/(sqrt(2)*V)   0.00000002635f * 1300000000.0f *  / (sqrt(2.0f)*(1256.6f + 236.4f * 39.75f) * 0.000000001f / 2)     [unit] m^3.5/kg^0.5
float a_hat = -13707631.7f;
float V_adapt = 0.0000053f;           // (1256.6f + 236.4f * 39.75f) * 0.000000001f / 2
float x_4_des_old = 0.0f;

//int16_t VALVE_LIMIT_PLUS;
//int16_t VALVE_LIMIT_MINUS;

float ENC_PULSE_PER_POSITION = 1.0f;
float TORQUE_SENSOR_PULSE_PER_TORQUE = 1.0f;
float PRES_SENSOR_A_PULSE_PER_BAR = 4096.0f / 200.0f;
float PRES_SENSOR_B_PULSE_PER_BAR = 4096.0f / 200.0f;

int16_t HOMEPOS_OFFSET;
int HOMEPOS_VALVE_OPENING;

float FRICTION;
float REF_PERIOD;
float REF_MAG;
int REF_NUM;


float DAC_REF;
float DAC_RESOL;

float REF_FORCE = 0.0;
float REF_TORQUE = 0.0;
float REF_POSITION = 0.0;
float REF_VELOCITY = 0.0;

float REF_POSITION_FINDHOME = 0.0;

int16_t REF_PWM;
int16_t REF_VALVE_POSITION;
int16_t REF_CURRENT;

int REF_MOVE_TIME_5k;
int INIT_REF_PWM;
int INIT_REF_VALVE_POS;
int INIT_REF_VEL;
int INIT_REF_TORQUE;
int INIT_REF_PRES_DIFF;
int INIT_REF_CURRENT;

unsigned int    TMR2_COUNT_LED1;
unsigned int    TMR2_COUNT_LED2;
unsigned int    TMR2_COUNT_CAN_TX = 0;
unsigned int    TMR3_COUNT_TEST = 0;

int num_err;
int flag_err[8];
int flag_err_old[8];
int flag_err_rt;

int flag_ref_enable;

int flag_data_request[5];

int MODE_POS_FT_TRANS = 0;
int NN_Control_Flag = 0;

int cnt_buffer = 0;

float CUR_CURRENT_mA = 0.0f;
float CUR_TORQUE_NM = 0.0f;
float CUR_TORQUE_NM_PRESS = 0.0f;

float FORCE_VREF = 0.0f;
float FORCE_VREF2 = 0.0f;
float PRES_A_VREF = 0.0f;
float PRES_B_VREF = 0.0f;

float VALVE_PWM_RAW_FB = 0.0f;
float VALVE_PWM_RAW_FF = 0.0f;
float VALVE_PWM_RAW = 0.0f;
int VALVE_PWM_VALVE_DZ = 0;

float VALVE_GAIN_LPM_PER_V[10];
float VALVE_POS_VS_PWM[25];
long JOINT_VEL[100];

int VALVE_MAX_POS;
int VALVE_MIN_POS;
int VALVE_POS_NUM;
float VALVE_CENTER_OFFSET;
float VALVE_DZ_MINUS_OFFSET;
float VALVE_DZ_PLUS_OFFSET;

int TMR3_COUNT_FINDHOME = 0;
int TMR3_COUNT_FLOWRATE = 0;
int TMR3_COUNT_DEADZONE = 0;
int TMR3_COUNT_PRES_NULL = 0;
int TMR3_COUNT_TORQUE_NULL = 0;
int TMR3_COUNT_PRES_CALIB = 0;
int TMR3_COUNT_REFERENCE = 0;
int TMR3_COUNT_JOINT = 0;
int TMR3_COUNT_ROTARY_FRIC_TUNE = 0;

float TUNING_TIME = 600.0f;  // sec

float REFERENCE_FREQ = 1.0f;
float REFERENCE_MAG = 0.0f;

bool FLAG_FIND_HOME;

int MODE_JUMP_STATUS;
enum _JUMP_STATUS {
    JUMP_NO_ACT = 0,                                //0
    JUMP_START,                                //1
    JUMP_TAKEOFF,                                  //2
    JUMP_FLYING,                                 //3
    JUMP_LANDING,                                  //4
};


float PRES_A_NULL_pulse = 300.0f;
float PRES_B_NULL_pulse = 300.0f;
float FORCE_NULL_pulse = 3900.0f;

float Ref_Valve_Pos_Old = 0.0f;

int VALVE_ID_timer = 0;
int VALVE_DZ_timer = 0;
int VALVE_FR_timer = 0;
//int VALVE_HPL_timer = 0;
int VALVE_POS_TMP = 0;
int JOINT_VEL_TMP = 0;
int DDV_POS_AVG = 0;
int VALVE_POS_AVG[50] = {0};
int VALVE_POS_AVG_OLD = 0;
int data_num = 0;
int ID_index = 0;
int DZ_index = 1;
int ID_index_array[50] = {0};
int first_check = 0;
float init_time = 0.0f;
int DZ_case = 0;
int START_POS = 0;
int FINAL_POS = 0;
int DZ_DIRECTION = 0;
int FIRST_DZ = 0;
int SECOND_DZ = 0;
int DZ_NUM = 0;
int one_period_end = 0;
float Ref_Vel_Test = 0.0f;
long TMR2_FOR_SLOW_LOGGING = 0;
char max_check = 0;
char min_check = 0;

float valve_pos_err = 0.0f, valve_pos_err_old = 0.0f, valve_pos_err_diff = 0.0f, valve_pos_err_sum = 0.0f;
float joint_pos_err = 0.0f, joint_pos_err_old = 0.0f, joint_pos_err_diff = 0.0f, joint_pos_err_diff_fil = 0.0f, joint_pos_err_sum = 0.0f;
float joint_torq_err = 0.0f, joint_torq_err_old = 0.0f, joint_torq_err_diff = 0.0f, joint_torq_err_sum = 0.0f;
float VALVE_PWM_RAW_POS = 0.0f, VALVE_PWM_RAW_TORQ = 0.0f;

float CUR_FLOWRATE = 0.0f;
float VALVE_FF_VOLTAGE = 0.0f;

int pos_plus_end = 0;
int pos_minus_end = 0;

bool need_enc_init = false;

int temp_time = 0;

float CUR_VELOCITY_sum = 0.0f;
float temp_vel_sum = 0.0f;

int DZ_dir = 0;
int DZ_temp_cnt = 0;
int DZ_temp_cnt2 = 0;
int DZ_end = 2;
int flag_flowrate = 0;
int fl_temp_cnt = 0;
int fl_temp_cnt2 = 0;
int cur_vel_sum = 0;

int cnt_finddz = 0;
int cnt_vel_finddz = 0;
int flag_finddz = 0;
int FINDDZ_VELOCITY = 0;
int FINDDZ_VELOCITY_OLD = 0;
int FINDDZ_POSITION = 0;
int FINDDZ_POSITION_OLD = 0;

double temp_VALVE_DEADZONE_PLUS = 0.0f;
double temp_VALVE_DEADZONE_MINUS = 0.0f;
float temp_pos_ref = 0.0f;
float temp_pos_ref_offset = 0.0f;


// valve gain
int check_vel_pos_init = 0;
int check_vel_pos_fin = 0;
int check_vel_pos_interv = 0;
int valve_gain_repeat_cnt = 0;
float VALVE_VOLTAGE = 0.0f;

float freq_fric_tune = 1.0f;

uint32_t TMR3_COUNT_CAN_TX = 0;

// Current Control Variables
double I_REF = 0.0f;
double I_REF_fil = 0.0f;
double I_REF_fil_DZ = 0.0f;
double I_ERR = 0.0f;
double I_ERR_INT = 0.0f;
double I_REF_fil_old = 0.0f;
double I_REF_fil_diff = 0.0f;

// system id
int cnt_sysid = 0;
double freq_sysid_Iref = 0.0f;

int cnt_freq_test = 0;
int cnt_step_test = 0;
int buffer_data_size = 0;
int cnt_send_buffer = 0;
float freq_test_valve_ref = 1.0f;
float ref_array[10000];
int pos_array[10000];
int flag_every_reference = 0;

int TMR3_COUNT_IREF = 0;
float CUR_CURRENT = 0.0f;
float u_CUR[3] = {0.0f,0.0f,0.0f};
int FINDHOME_STAGE = 0;
int FINDHOME_INIT = 0;
int FINDHOME_GOTOLIMIT = 1;
int FINDHOME_ZEROPOSE = 2;

int FINDDZ_STAGE = 0;
int FINDDZ_INIT = 0;
int FINDDZ_START1 = 1;
int FINDDZ_START2 = 2;
int FINDDZ_STOP = 3;

float alpha_trans = 0.0f;

float V_out=0.0f;
float V_rem=0.0f; // for anti-windup
float V_MAX = 12000.0f; // Maximum Voltage : 12V = 12000mV

float PWM_out=0.0f;

double K_v = 1.0f; // valve flowrate gain 1
double C_d = 0.16f; // valve flowrate gain 2

double mV_PER_mA = 600.0f; // current >> voltage
double mV_PER_pulse = 0.6f; // pulse >> voltage
double mA_PER_pulse = 0.001f; // pulse >> current

int timer_while = 0;
int while_index = 0;
int RL_timer = 0;

float K_LPF = 0.0f;
float D_LPF = 0.0f;

float torq_sen_past = 0.0f;
float torq_ref_past = 0.0f;
float output_normalized = 0.0f;


/*******************************************************************************
 * General math functions
 ******************************************************************************/


float dabs(float tx)
{
    if (tx >= 0.0f)
        return tx;
    else
        return -tx;
}

float change_int_to_efloat(int input)
{
    int i = 0;

    float output = 0;
    int vn = (int) ((float) input / 10.0f);
    int en = input % 10;

    float temp = 1.;
    for (i = 0; i < en; i++)
        temp *= 0.1f;

    output = (float) vn*temp;
    return output;
}

void make_delay(void)
{
    int i = 0;

    for (i = 0; i < 1000000; i++) {
        ;
    }
}


/*******************************************************************************
 * ROM functions
 ******************************************************************************/

void ROM_CALL_DATA(void)
{
    BNO = spi_eeprom_read(RID_BNO);
//    BNO = 11;
    OPERATING_MODE = spi_eeprom_read(RID_OPERATING_MODE);
    SENSING_MODE = spi_eeprom_read(RID_SENSING_MODE);
    CURRENT_CONTROL_MODE = spi_eeprom_read(RID_CURRENT_CONTROL_MODE);
    FLAG_VALVE_DEADZONE = spi_eeprom_read(RID_FLAG_VALVE_DEADZONE);
    CAN_FREQ = spi_eeprom_read(RID_CAN_FREQ);
    DIR_JOINT_ENC = spi_eeprom_read(RID_JOINT_ENC_DIR);
    DIR_VALVE = spi_eeprom_read(RID_VALVE_DIR);
    DIR_VALVE_ENC = spi_eeprom_read(RID_VALVE_ENC_DIR);
    SUPPLY_VOLTAGE = (float) (spi_eeprom_read(RID_VOLATGE_SUPPLY)) *0.1f;
    VALVE_VOLTAGE_LIMIT = (float) (spi_eeprom_read(RID_VOLTAGE_VALVE)) * 0.1f;
    P_GAIN_VALVE_POSITION = spi_eeprom_read(RID_P_GAIN_VALVE_POSITION);
    I_GAIN_VALVE_POSITION = spi_eeprom_read(RID_I_GAIN_VALVE_POSITION);
    D_GAIN_VALVE_POSITION = spi_eeprom_read(RID_D_GAIN_VALVE_POSITION);
    P_GAIN_JOINT_POSITION = spi_eeprom_read(RID_P_GAIN_JOINT_POSITION);
    I_GAIN_JOINT_POSITION = spi_eeprom_read(RID_I_GAIN_JOINT_POSITION);
    D_GAIN_JOINT_POSITION = spi_eeprom_read(RID_D_GAIN_JOINT_POSITION);
    P_GAIN_JOINT_TORQUE = spi_eeprom_read(RID_P_GAIN_JOINT_TORQUE);
    I_GAIN_JOINT_TORQUE = spi_eeprom_read( RID_I_GAIN_JOINT_TORQUE);
    D_GAIN_JOINT_TORQUE = spi_eeprom_read(RID_D_GAIN_JOINT_TORQUE);
    VALVE_DEADZONE_PLUS = (spi_eeprom_read(RID_VALVE_DEADZONE_PLUS));
    VALVE_DEADZONE_MINUS = (spi_eeprom_read(RID_VALVE_DEADZONE_MINUS));
    VELOCITY_COMP_GAIN = spi_eeprom_read(RID_VELOCITY_COMP_GAIN);
    COMPLIANCE_GAIN = spi_eeprom_read(RID_COMPLIANCE_GAIN);
    VALVE_CENTER = spi_eeprom_read(RID_VALVE_CNETER);
    VALVE_FF = spi_eeprom_read(RID_VALVE_FF);
    BULK_MODULUS = spi_eeprom_read(RID_BULK_MODULUS);
    CHAMBER_VOLUME_A = spi_eeprom_read(RID_CHAMBER_VOLUME_A);
    CHAMBER_VOLUME_B = spi_eeprom_read(RID_CHAMBER_VOLUME_B);
    PISTON_AREA_A = spi_eeprom_read(RID_PISTON_AREA_A);
    PISTON_AREA_B = spi_eeprom_read(RID_PISTON_AREA_B);
    PISTON_AREA_alpha = (float)PISTON_AREA_A/(float)PISTON_AREA_B;
    alpha3 = PISTON_AREA_alpha * PISTON_AREA_alpha*PISTON_AREA_alpha;
    PRES_SUPPLY_NOM = spi_eeprom_read(RID_PRES_SUPPLY);
    PRES_SUPPLY = PRES_SUPPLY_NOM;
    ENC_LIMIT_MINUS = spi_eeprom_read(RID_ENC_LIMIT_MINUS);
    ENC_LIMIT_PLUS = spi_eeprom_read(RID_ENC_LIMIT_PLUS);
    STROKE = spi_eeprom_read(RID_STROKE);
    ENC_PULSE_PER_POSITION = (float) (spi_eeprom_read(RID_ENC_PULSE_PER_POSITION));
    TORQUE_SENSOR_PULSE_PER_TORQUE = (float) (spi_eeprom_read(RID_TORQUE_SENSOR_PULSE_PER_TORQUE)) * 0.001f;
    PRES_SENSOR_A_PULSE_PER_BAR = (float) (spi_eeprom_read(RID_PRES_SENSOR_A_PULSE_PER_BAR)) * 0.01f;
//    PRES_SENSOR_A_PULSE_PER_BAR = 4096.0f * 946.0f / 3.3f / 300.0f / 210.0f;
    PRES_SENSOR_B_PULSE_PER_BAR = (float) (spi_eeprom_read(RID_PRES_SENSOR_B_PULSE_PER_BAR)) * 0.01f;
//    PRES_SENSOR_B_PULSE_PER_BAR = 4096.0f * 946.0f / 3.3f / 300.0f / 210.0f;
    FRICTION = (float) (spi_eeprom_read(RID_FRICTION)) * 0.1f;
    HOMEPOS_OFFSET = spi_eeprom_read(RID_HOMEPOS_OFFSET);
    HOMEPOS_VALVE_OPENING = spi_eeprom_read(RID_HOMEPOS_VALVE_OPENING);
    FORCE_VREF = (float) (spi_eeprom_read(RID_FORCE_SENSOR_VREF)) *0.001f;
    FORCE_VREF2 = (float) (spi_eeprom_read(RID_FORCE_SENSOR_VREF2)) *0.001f;
    PRES_A_VREF = (float) spi_eeprom_read(RID_PRES_A_SENSOR_VREF) * 0.001f;
    PRES_B_VREF = (float) spi_eeprom_read(RID_PRES_B_SENSOR_VREF) * 0.001f;
    VALVE_GAIN_LPM_PER_V[0] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_1)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[2] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_2)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[4] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_3)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[6] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_4)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[8] = (float) (spi_eeprom_read(RID_VALVE_GAIN_PLUS_5)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[1] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_1)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[3] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_2)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[5] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_3)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[7] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_4)) * 0.01f;
    VALVE_GAIN_LPM_PER_V[9] = (float) (spi_eeprom_read(RID_VALVE_GAIN_MINUS_5)) * 0.01f;
    for(int i=0; i<25; i++) {
        VALVE_POS_VS_PWM[i] = (float) (spi_eeprom_read(RID_VALVE_POS_VS_PWM_0 + i));
    }
    for(int i=0; i<100; i++) {
        JOINT_VEL[i] = ( ((spi_eeprom_read( RID_VALVE_POS_VS_FLOWRATE_0 + i)) & 0xFFFF) | ((spi_eeprom_read(RID_VALVE_POS_VS_FLOWRATE_0_1 + i)) & 0xFFFF) << 16 ) ;
    }
    VALVE_MAX_POS = spi_eeprom_read(RID_VALVE_MAX_POS);
    VALVE_MIN_POS = spi_eeprom_read(RID_VALVE_MIN_POS);
    VALVE_POS_NUM = spi_eeprom_read(RID_VALVE_POS_NUM);

//    K_SPRING = spi_eeprom_read(RID_K_SPRING);
//    D_DAMPER = spi_eeprom_read(RID_D_DAMPER);

}

/*******************************************************************************
 * ENCODER functions

 ******************************************************************************/
long ENC_pulse = 0, ENC_pulse_old = 0, ENC_pulse_diff = 0;
long ENC_pulse_offset = 0;

void ENC_UPDATE(void)
{
    ENC_pulse = spi_enc_read(); // Unit : pulse
    ENC_pulse_diff = ENC_pulse - ENC_pulse_old;

    pos.UpdateSen((float)((long)DIR_JOINT_ENC * ENC_pulse + ENC_pulse_offset)/ENC_PULSE_PER_POSITION, FREQ_10k, 100.0f); // Unit : deg or mm
    vel.UpdateSen((float)((long)DIR_JOINT_ENC * ENC_pulse_diff * (long)FREQ_10k)/ENC_PULSE_PER_POSITION, FREQ_10k, 100.0f); // Unit : deg/s or mm/s

    ENC_pulse_old = ENC_pulse;
}

void ENC_SET_ZERO(void)
{
    spi_enc_set_clear();
    pos.Reset();
    ENC_pulse_offset = 0;
    ENC_pulse = ENC_pulse_old = ENC_pulse_diff = 0;
}

void ENC_SET(long value_e)
{
    spi_enc_set_clear();
    ENC_pulse_offset = value_e;
    pos.Reset();
    pos.sen = value_e/ENC_PULSE_PER_POSITION;
    ENC_pulse = ENC_pulse_old = value_e;
    ENC_pulse_diff = 0;
}




