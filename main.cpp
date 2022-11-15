//Hydraulic Control Board
//distributed by Sungwoo Kim
//       2020/12/28
//revised by Buyoun Cho
//       2021/04/20

// 유의사항
// 소수 적을때 뒤에 f 꼭 붙이기
// CAN 선은 ground까지 있는 3상 선으로 써야함.
// 전원은 12~24V 인가.

#include "mbed.h"
#include "FastPWM.h"
#include "INIT_HW.h"
#include "function_CAN.h"
#include "SPI_EEP_ENC.h"
#include "I2C_AS5510.h"
#include "setting.h"
#include "function_utilities.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"
#include <string>
#include <iostream>
#include <cmath>

using namespace std;
Timer t;

// dac & check ///////////////////////////////////////////
DigitalOut check(PC_2);
DigitalOut check_2(PC_3);
AnalogOut dac_1(PA_4); // 0.0f ~ 1.0f
AnalogOut dac_2(PA_5); // 0.0f ~ 1.0f
AnalogIn adc1(PC_4); //pressure_1
AnalogIn adc2(PB_0); //pressure_2
AnalogIn adc3(PC_1); //current

// PWM ///////////////////////////////////////////
float dtc_v=0.0f;
float dtc_w=0.0f;

// I2C ///////////////////////////////////////////
I2C i2c(PC_9,PA_8); // SDA, SCL (for K22F)
const int i2c_slave_addr1 =  0x56;  // AS5510 address
unsigned int value; // 10bit output of reading sensor AS5510

// SPI ///////////////////////////////////////////
SPI eeprom(PB_15, PB_14, PB_13); // EEPROM //(SPI_MOSI, SPI_MISO, SPI_SCK);
DigitalOut eeprom_cs(PB_12);
SPI enc(PC_12,PC_11,PC_10);
DigitalOut enc_cs(PD_2);
DigitalOut LED(PA_15);

// UART ///////////////////////////////////////////
Serial pc(PA_9,PA_10); //  _ UART

// CAN ///////////////////////////////////////////
CAN can(PB_8, PB_9, 1000000);
CANMessage msg;
void onMsgReceived()
{
    CAN_RX_HANDLER();
}

// Variables ///////////////////////////////////////////
State pos;
State vel;
State Vout;
State force;
State torq;         // unit : N
State torq_dot;
State pres_A;       // unit : bar
State pres_B;
State cur;          // unit : mA
State valve_pos;

State INIT_Vout;
State INIT_Valve_Pos;
State INIT_Pos;
State INIT_torq;

extern int CID_RX_CMD;
extern int CID_RX_REF_POSITION;
extern int CID_RX_REF_OPENLOOP;
extern int CID_RX_REF_PWM;

extern int CID_TX_INFO;
extern int CID_TX_POS_VEL_TORQ;
extern int CID_TX_PWM;
extern int CID_TX_CURRENT;
extern int CID_TX_VOUT;
extern int CID_TX_VALVE_POSITION;
extern int CID_TX_SOMETHING;

float temp_P_GAIN = 0.0f;
float temp_I_GAIN = 0.0f;
int temp_VELOCITY_COMP_GAIN = 0;
int logging = 0;

inline float tanh_inv(float y)
{
    if(y >= 1.0f - 0.000001f) y = 1.0f - 0.000001f;
    if(y <= -1.0f + 0.000001f) y = -1.0f + 0.000001f;
    return log(sqrt((1.0f+y)/(1.0f-y)));
}


/*******************************************************************************
 *  REFERENCE MODE
 ******************************************************************************/
enum _REFERENCE_MODE {
    MODE_REF_NO_ACT = 0,
    MODE_REF_DIRECT,
    MODE_REF_FINDHOME
};

/*******************************************************************************
 *  CONTROL MODE
 ******************************************************************************/
enum _CONTROL_MODE {
    //control mode
    MODE_NO_ACT = 0,                                    //0
    MODE_VALVE_POSITION_CONTROL,                        //1
    MODE_JOINT_CONTROL,                                 //2

    MODE_VALVE_OPEN_LOOP,                               //3
    MODE_JOINT_ADAPTIVE_BACKSTEPPING,                   //4
    MODE_RL,                                            //5

    MODE_JOINT_POSITION_PRES_CONTROL_PWM,               //6
    MODE_JOINT_POSITION_PRES_CONTROL_VALVE_POSITION,    //7
    MODE_VALVE_POSITION_PRES_CONTROL_LEARNING,          //8

    MODE_TEST_CURRENT_CONTROL,                          //9
    MODE_TEST_PWM_CONTROL,                              //10

    MODE_CURRENT_CONTROL,                               //11
    MODE_JOINT_POSITION_TORQUE_CONTROL_CURRENT,         //12
    MODE_JOINT_POSITION_PRES_CONTROL_CURRENT,           //13
    MODE_VALVE_POSITION_TORQUE_CONTROL_LEARNING,        //14

    //utility
    MODE_TORQUE_SENSOR_NULLING = 20,                    //20
    MODE_VALVE_NULLING_AND_DEADZONE_SETTING,            //21
    MODE_FIND_HOME,                                     //22
    MODE_VALVE_GAIN_SETTING,                            //23
    MODE_PRESSURE_SENSOR_NULLING,                       //24
    MODE_PRESSURE_SENSOR_CALIB,                         //25
    MODE_ROTARY_FRICTION_TUNING,                        //26

    MODE_DDV_POS_VS_PWM_ID = 30,                        //30
    MODE_DDV_DEADZONE_AND_CENTER,                       //31
    MODE_DDV_POS_VS_FLOWRATE,                           //32
    MODE_SYSTEM_ID,                                     //33
    MODE_FREQ_TEST,                                     //34
    MODE_SEND_BUFFER,                                   //35
    MODE_SEND_OVER,                                     //36
    MODE_STEP_TEST,                                     //37
};

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;//8
    RCC_OscInitStruct.PLL.PLLN = 180; //180
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        //Error_Handler();
    }
    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        //Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        //Error_Handler();
    }
}


int main()
{
    /*********************************
    ***     Initialization
    *********************************/

    HAL_Init();
    SystemClock_Config();

    LED = 0;
    pc.baud(9600);

    // i2c init
//    i2c.frequency(400 * 1000);          // 0.4 mHz
//    wait_ms(2);                         // Power Up wait
//    look_for_hardware_i2c();            // Hardware present
//    init_as5510(i2c_slave_addr1);
//    make_delay();

    // spi init
    eeprom_cs = 1;
    eeprom.format(8,3);
    eeprom.frequency(5000000); //5M
    eeprom_cs = 0;
    make_delay();

    enc_cs = 1;     //sw add
    enc.format(8,0);
    enc.frequency(5000000); //10M
    enc_cs = 0;     //sw add

    make_delay();

    // spi _ enc
    spi_enc_set_init();
    make_delay();

    ////// bno rom
    spi_eeprom_write(RID_BNO, (int16_t) 8);
    make_delay();
    ////////

    // rom
    ROM_CALL_DATA();
    make_delay();

    // ADC init
    Init_ADC();
    make_delay();

    // Pwm init
    Init_PWM();
    TIM4->CR1 ^= TIM_CR1_UDIS;
    make_delay();

    // CAN
    can.attach(&CAN_RX_HANDLER);
    CAN_ID_INIT();
    make_delay();

    //can.reset();
    can.filter(msg.id, 0xFFFFF000, CANStandard);

    // TMR3 init
    Init_TMR3();
    TIM3->CR1 ^= TIM_CR1_UDIS;
    make_delay();

    //Timer priority
    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_SetPriority(TIM4_IRQn, 3);


    //DAC init
    if (SENSING_MODE == 0) {
        dac_1 = FORCE_VREF / 3.3f;
        dac_2 = 0.0f;
    } else if (SENSING_MODE == 1) {
        if (DIR_VALVE_ENC > 0) {
            dac_1 = PRES_A_VREF / 3.3f;
            dac_2 = PRES_B_VREF / 3.3f;
        } else {
            dac_1 = PRES_B_VREF / 3.3f;
            dac_2 = PRES_A_VREF / 3.3f;
        }
    } else if (SENSING_MODE == 2) {
        dac_1 = 0.0f;
        dac_2 = FORCE_VREF2 / 3.3f;
    }
    make_delay();

    for (int i=0; i<50; i++) {
        if(i%2==0)
            ID_index_array[i] = - i * 0.5f;
        else
            ID_index_array[i] =  (i+1) * 0.5f;
    }

    /************************************
    ***     Program is operating!
    *************************************/
    while(1) {

        // UART example
//        if(timer_while==100000) {
//            timer_while = 0;
//            pc.printf("%f\n", value);
//        }
//        timer_while ++;

        //i2c for SW valve
        //if(OPERATING_MODE == 5) {
//            read_field(i2c_slave_addr1);
//            if(DIR_VALVE_ENC < 0) value = 1023 - value;
//        }
    }
}


// Velocity feedforward for SW valve
float DDV_JOINT_POS_FF(float REF_JOINT_VEL)
{
    int i = 0;
    float Ref_Valve_Pos_FF = 0.0f;
    for(i=0; i<VALVE_POS_NUM; i++) {
        if(REF_JOINT_VEL >= min(JOINT_VEL[i],JOINT_VEL[i+1]) && REF_JOINT_VEL <=  max(JOINT_VEL[i],JOINT_VEL[i+1])) {
            if(i==0) {
                if(JOINT_VEL[i+1] == JOINT_VEL[i]) {
                    Ref_Valve_Pos_FF = (float) VALVE_CENTER;
                } else {
                    Ref_Valve_Pos_FF = ((float) 10/(JOINT_VEL[i+1] - JOINT_VEL[i]) * (REF_JOINT_VEL - JOINT_VEL[i])) + (float) VALVE_CENTER;
                }
            } else {
                if(JOINT_VEL[i+1] == JOINT_VEL[i-1]) {
                    Ref_Valve_Pos_FF = (float) VALVE_CENTER;
                } else {
                    Ref_Valve_Pos_FF = ((float) 10*(ID_index_array[i+1] - ID_index_array[i-1])/(JOINT_VEL[i+1] - JOINT_VEL[i-1]) * (REF_JOINT_VEL - JOINT_VEL[i-1])) + (float) VALVE_CENTER + (float) (10*ID_index_array[i-1]);
                }
            }
            break;
        }
    }
    if(REF_JOINT_VEL > max(JOINT_VEL[VALVE_POS_NUM-1], JOINT_VEL[VALVE_POS_NUM-2])) {
        Ref_Valve_Pos_FF = (float) VALVE_MAX_POS;
    } else if(REF_JOINT_VEL < min(JOINT_VEL[VALVE_POS_NUM-1], JOINT_VEL[VALVE_POS_NUM-2])) {
        Ref_Valve_Pos_FF = (float) VALVE_MIN_POS;
    }

    Ref_Valve_Pos_FF = (float) VELOCITY_COMP_GAIN * 0.01f * (float) (Ref_Valve_Pos_FF - (float) VALVE_CENTER);  //VELOCITY_COMP_GAIN : 0~100
    return Ref_Valve_Pos_FF;
}

// Valve feedforward for SW valve
void VALVE_POS_CONTROL(float REF_VALVE_POS)
{
    int i = 0;

    if(REF_VALVE_POS > VALVE_MAX_POS) {
        REF_VALVE_POS = VALVE_MAX_POS;
    } else if(REF_VALVE_POS < VALVE_MIN_POS) {
        REF_VALVE_POS = VALVE_MIN_POS;
    }
    valve_pos_err = (float) (REF_VALVE_POS - value);
    valve_pos_err_diff = valve_pos_err - valve_pos_err_old;
    valve_pos_err_old = valve_pos_err;
    valve_pos_err_sum += valve_pos_err;
    if (valve_pos_err_sum > 1000.0f) valve_pos_err_sum = 1000.0f;
    if (valve_pos_err_sum<-1000.0f) valve_pos_err_sum = -1000.0f;

    VALVE_PWM_RAW_FB = P_GAIN_VALVE_POSITION * valve_pos_err + I_GAIN_VALVE_POSITION * valve_pos_err_sum + D_GAIN_VALVE_POSITION * valve_pos_err_diff;

    for(i=0; i<24; i++) {
        if(REF_VALVE_POS >= min(VALVE_POS_VS_PWM[i],VALVE_POS_VS_PWM[i+1]) && (float) REF_VALVE_POS <=  max(VALVE_POS_VS_PWM[i],VALVE_POS_VS_PWM[i+1])) {
            if(i==0) {
                VALVE_PWM_RAW_FF = (float) 1000.0f / (float) (VALVE_POS_VS_PWM[i+1] - VALVE_POS_VS_PWM[i]) * ((float) REF_VALVE_POS - VALVE_POS_VS_PWM[i]);
            } else {
                VALVE_PWM_RAW_FF = (float) 1000.0f* (float) (ID_index_array[i+1] - ID_index_array[i-1])/(VALVE_POS_VS_PWM[i+1] - VALVE_POS_VS_PWM[i-1]) * ((float) REF_VALVE_POS - VALVE_POS_VS_PWM[i-1]) + 1000.0f * (float) ID_index_array[i-1];
            }
            break;
        }
    }
    Vout.ref = VALVE_PWM_RAW_FF + VALVE_PWM_RAW_FB;
}

// PWM duty vs. voltage output of L6205 in STM board
#define LT_MAX_IDX  57
float LT_PWM_duty[LT_MAX_IDX] = {-100.0f, -80.0f, -60.0f, -50.0f, -40.0f, -35.0f, -30.0f, -25.0f, -20.0f,
                                 -19.0f, -18.0f, -17.0f, -16.0f, -15.0f, -14.0f, -13.0f, -12.0f, -11.0f, -10.0f,
                                 -9.0f, -8.0f, -7.0f, -6.0f, -5.0f, -4.0f, -3.0f, -2.0f, -1.0f, 0.0f,
                                 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f,
                                 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 19.0f, 20.0f,
                                 25.0f, 30.0f, 35.0f, 40.0f, 50.0f, 60.0f, 80.0f, 100.0f
                                };  // duty
float LT_Voltage_Output[LT_MAX_IDX] = {-230.0f, -215.0f, -192.5f, -185.0f, -177.5f, -170.0f, -164.0f, -160.0f, -150.0f,
                                       -150.0f, -145.0f, -145.0f, -145.0f, -135.0f, -135.0f, -135.0f, -127.5f, -127.5f, -115.0f,
                                       -115.0f, -115.0F, -100.0f, -100.0f, -100.0f, -60.0f, -60.0f, -10.0f, -5.0f, 0.0f,
                                       7.5f, 14.0f, 14.0f, 14.0f, 42.5f, 42.5f, 42.5f, 80.0f, 80.0f, 105.0f,
                                       105.0f, 105.0f, 120.0f, 120.0f, 120.0f, 131.0f, 131.0f, 140.0f, 140.0f, 140.0f,
                                       155.0f, 160.0f, 170.0f, 174.0f, 182.0f, 191.0f, 212.0f, 230.0f
                                      }; // mV

float PWM_duty_byLT(float Ref_V)
{
    float PWM_duty = 0.0f;
    if(Ref_V<LT_Voltage_Output[0]) {
        PWM_duty = (Ref_V-LT_Voltage_Output[0])/1.5f+LT_PWM_duty[0];
    } else if (Ref_V>=LT_Voltage_Output[LT_MAX_IDX-1]) {
        PWM_duty = (Ref_V-LT_Voltage_Output[LT_MAX_IDX-1])/1.5f+LT_PWM_duty[LT_MAX_IDX-1];
    } else {
        int idx = 0;
        for(idx=0; idx<LT_MAX_IDX-1; idx++) {
            float ini_x = LT_Voltage_Output[idx];
            float fin_x = LT_Voltage_Output[idx+1];
            float ini_y = LT_PWM_duty[idx];
            float fin_y = LT_PWM_duty[idx+1];
            if(Ref_V>=ini_x && Ref_V<fin_x) {
                PWM_duty = (fin_y-ini_y)/(fin_x-ini_x)*(Ref_V-ini_x) + ini_y;
                break;
            }
        }
    }

    return PWM_duty;
}



/*******************************************************************************
                            TIMER INTERRUPT
*******************************************************************************/

//------------------------------------------------
//     TMR4 : Sensor Read & Data Handling
//-----------------------------------------------
float FREQ_TMR4 = (float)FREQ_20k;
float DT_TMR4 = (float)DT_20k;
long  CNT_TMR4 = 0;
int   TMR4_FREQ_10k = (int)FREQ_10k;
extern "C" void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_UIF ) {

        // Current ===================================================
        //ADC3->CR2  |= 0x40000000;                        // adc _ 12bit

        cur.UpdateSen(((float)ADC3->DR-2047.5f)/2047.5f*10.0f, FREQ_TMR4, 500.0f); // unit : mA

        // Encoder ===================================================
        if (CNT_TMR4 % (int) ((int) FREQ_TMR4/TMR4_FREQ_10k) == 0) {
            ENC_UPDATE();
        }

        // Force or Pressure Transducer =============================================
        ADC1->CR2  |= 0x40000000;
        if (SENSING_MODE == 0) {  // Force sensing
            force.UpdateSen((((float)ADC1->DR) - 2047.5f)/TORQUE_SENSOR_PULSE_PER_TORQUE, FREQ_TMR4, 100.0f); // unit : N
        } else if (SENSING_MODE == 1) { // Pressure sensing
            float pres_A_new, pres_B_new;
            if (DIR_VALVE_ENC > 0) {
                pres_A_new = (((float)ADC1->DR) - PRES_A_NULL_pulse)/ PRES_SENSOR_A_PULSE_PER_BAR; // unit : bar
                pres_B_new = (((float)ADC2->DR) - PRES_B_NULL_pulse)/ PRES_SENSOR_B_PULSE_PER_BAR;
            } else {
                pres_A_new = (((float)ADC2->DR) - PRES_A_NULL_pulse)/ PRES_SENSOR_A_PULSE_PER_BAR; // unit : bar
                pres_B_new = (((float)ADC1->DR) - PRES_B_NULL_pulse)/ PRES_SENSOR_B_PULSE_PER_BAR;
            }
            pres_A.UpdateSen(pres_A_new,FREQ_TMR4,200.0f);
            pres_B.UpdateSen(pres_B_new,FREQ_TMR4,200.0f);

            if ((OPERATING_MODE & 0b01) == 0) { // Rotary Actuator
                float torq_new = (PISTON_AREA_A * pres_A.sen - PISTON_AREA_B * pres_B.sen) * 0.0001f; // mm^3*bar >> Nm
                torq.UpdateSen(torq_new,FREQ_TMR4,20.0f);  // unit : Nm
            } else if ((OPERATING_MODE & 0b01) == 1) { // Linear Actuator
                float force_new = (PISTON_AREA_A * pres_A.sen - PISTON_AREA_B * pres_B.sen) * 0.1f; // mm^2*bar >> N
                force.UpdateSen(force_new,FREQ_TMR4,20.0f);  // unit : N
            }
        } else if (SENSING_MODE == 2) {
            force.UpdateSen((((float)ADC2->DR) - 2047.5f)/TORQUE_SENSOR_PULSE_PER_TORQUE, FREQ_TMR4, 100.0f); // unit : N
        }

        CNT_TMR4++;
    }
    TIM4->SR = 0x0;  // reset the status register
}


int j =0;
float FREQ_TMR3 = (float)FREQ_5k;
float DT_TMR3 = (float)DT_5k;
int cnt_trans = 0;
double VALVE_POS_RAW_FORCE_FB_LOGGING = 0.0f;
int can_rest =0;
float force_ref_act_can = 0.0f;

extern "C" void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF ) {

        if(MODE_POS_FT_TRANS == 1) {
            if (alpha_trans == 1.0f) MODE_POS_FT_TRANS = 2;
            alpha_trans = (float)(1.0f - cos(3.141592f * (float)cnt_trans * DT_TMR3 /3.0f))/2.0f;
            cnt_trans++;
            torq.err_int = 0.0f;
            force.err_int = 0.0f;
            if((float)cnt_trans * DT_TMR3 > 3.0f)
                MODE_POS_FT_TRANS = 2;
        } else if(MODE_POS_FT_TRANS == 3) {
            if (alpha_trans == 0.0f) MODE_POS_FT_TRANS = 0;
            alpha_trans = (float)(1.0f + cos(3.141592f * (float)cnt_trans * DT_TMR3 /3.0f))/2.0f;
            cnt_trans++;
            torq.err_int = 0.0f;
            force.err_int = 0.0f;
            if((float) cnt_trans * DT_TMR3 > 3.0f )
                MODE_POS_FT_TRANS = 0;
        } else if(MODE_POS_FT_TRANS == 2) {
            alpha_trans = 1.0f;
            cnt_trans = 0;
        } else {
            alpha_trans = 0.0f;
            cnt_trans = 0;
        }


        // Reference Update ==========================================================
        switch (REFERENCE_MODE) {
            case MODE_REF_NO_ACT: {
                break;
            }
            case MODE_REF_DIRECT: {
                pos.ref = REF_POSITION;
                vel.ref = REF_VELOCITY;
                torq.ref = REF_TORQUE;
                force.ref = REF_FORCE;
                break;
            }
            case MODE_REF_FINDHOME: {
                pos.ref = REF_POSITION_FINDHOME;
                vel.ref = 0.0f;
                torq.ref = 0.0f;
                force.ref = 0.0f;
                break;
            }
            default:
                break;
        }

        if (((OPERATING_MODE&0b010)>>1) == 0) {
            K_v = 1.03f; // Q = K_v*sqrt(deltaP)*tanh(C_d*Xv);
            C_d = 0.16f;
            mV_PER_mA = 500.0f; // 5000mV/10mA
            mV_PER_pulse = 0.5f; // 5000mV/10000pulse
            mA_PER_pulse = 0.001f; // 10mA/10000pulse
        } else if (((OPERATING_MODE&0b010)>>1) == 1) {
            K_v = 0.5f; // KNR (LPM >> mA) , 100bar
            mV_PER_mA = 166.6666f; // 5000mV/30mA
            mV_PER_pulse = 0.5f; // 5000mV/10000pulse
            mA_PER_pulse = 0.003f; // 30mA/10000pulse
        }

        // =====================================================================
        // CONTROL LOOP --------------------------------------------------------
        // =====================================================================
        int UTILITY_MODE = 0;
        int CONTROL_MODE = 0;

        if (CONTROL_UTILITY_MODE >= 20 || CONTROL_UTILITY_MODE == 0) {
            UTILITY_MODE = CONTROL_UTILITY_MODE;
            CONTROL_MODE = MODE_NO_ACT;
        } else {
            CONTROL_MODE = CONTROL_UTILITY_MODE;
            UTILITY_MODE = MODE_NO_ACT;
        }
        // UTILITY MODE ------------------------------------------------------------
        switch (UTILITY_MODE) {
            case MODE_NO_ACT: {
                break;
            }

            case MODE_TORQUE_SENSOR_NULLING: {
                static float FORCE_pulse_sum = 0.0;
                static float PresA_pulse_sum = 0.0;
                static float PresB_pulse_sum = 0.0;

                // DAC Voltage reference set
                float VREF_TuningGain = -0.000003f;
                if (TMR3_COUNT_TORQUE_NULL < TMR_FREQ_5k * 5) {
                    LED = 1;
                    if(SENSING_MODE == 0) { // Force Sensor (Loadcell)
                        FORCE_pulse_sum = FORCE_pulse_sum + force.sen*TORQUE_SENSOR_PULSE_PER_TORQUE;
                        if (TMR3_COUNT_TORQUE_NULL % 10 == 0) {
                            float FORCE_pluse_mean = FORCE_pulse_sum / 10.0f;
                            FORCE_pulse_sum = 0.0f;

                            FORCE_VREF += VREF_TuningGain * (0.0f - FORCE_pluse_mean);
                            if (FORCE_VREF > 3.3f) FORCE_VREF = 3.3f;
                            if (FORCE_VREF < 0.0f) FORCE_VREF = 0.0f;
                            dac_1 = FORCE_VREF / 3.3f;
                        }
                    } else if (SENSING_MODE == 1) { // Pressure Sensor
                        PresA_pulse_sum += pres_A.sen*PRES_SENSOR_A_PULSE_PER_BAR;
                        PresB_pulse_sum += pres_B.sen*PRES_SENSOR_B_PULSE_PER_BAR;
                        if (TMR3_COUNT_TORQUE_NULL % 10 == 0) {
                            float PresA_pluse_mean = PresA_pulse_sum / 10.0f;
                            float PresB_pluse_mean = PresB_pulse_sum / 10.0f;
                            PresA_pulse_sum = 0.0f;
                            PresB_pulse_sum = 0.0f;

                            PRES_A_VREF += VREF_TuningGain * (0.0f - PresA_pluse_mean);
                            if (PRES_A_VREF > 3.3f) PRES_A_VREF = 3.3f;
                            if (PRES_A_VREF < 0.0f) PRES_A_VREF = 0.0f;
                            dac_1 = PRES_A_VREF / 3.3f;
                            PRES_B_VREF += VREF_TuningGain * (0.0f - PresB_pluse_mean);
                            if (PRES_B_VREF > 3.3f) PRES_B_VREF = 3.3f;
                            if (PRES_B_VREF < 0.0f) PRES_B_VREF = 0.0f;
                            dac_2 = PRES_B_VREF / 3.3f;
                        }
                    } else if (SENSING_MODE == 2) {
                        FORCE_pulse_sum = FORCE_pulse_sum + force.sen*TORQUE_SENSOR_PULSE_PER_TORQUE;
                        if (TMR3_COUNT_TORQUE_NULL % 10 == 0) {
                            float FORCE_pluse_mean = FORCE_pulse_sum / 10.0f;
                            FORCE_pulse_sum = 0.0f;

                            FORCE_VREF2 += VREF_TuningGain * (0.0f - FORCE_pluse_mean);
                            if (FORCE_VREF2 > 3.3f) FORCE_VREF2 = 3.3f;
                            if (FORCE_VREF2 < 0.0f) FORCE_VREF2 = 0.0f;
                            dac_2 = FORCE_VREF2 / 3.3f;
                        }
                    }
                    TMR3_COUNT_TORQUE_NULL++;
                } else {
                    if(SENSING_MODE == 0 ) { // Force Sensor (Loadcell)
                        FORCE_pulse_sum = 0.0f;
                        dac_1 = FORCE_VREF / 3.3f;
                        spi_eeprom_write(RID_FORCE_SENSOR_VREF, (int16_t)(FORCE_VREF * 1000.0f));
                    } else if (SENSING_MODE == 1) {
                        PresA_pulse_sum = 0.0f;
                        PresB_pulse_sum = 0.0f;
                        dac_1 = PRES_A_VREF / 3.3f;
                        dac_2 = PRES_B_VREF / 3.3f;
                        spi_eeprom_write(RID_PRES_A_SENSOR_VREF, (int16_t)(PRES_A_VREF * 1000.0f));
                        spi_eeprom_write(RID_PRES_B_SENSOR_VREF, (int16_t)(PRES_B_VREF * 1000.0f));
                    } else if (SENSING_MODE == 2) {
                        FORCE_pulse_sum = 0.0f;
                        dac_2 = FORCE_VREF2 / 3.3f;
                        spi_eeprom_write(RID_FORCE_SENSOR_VREF2, (int16_t)(FORCE_VREF2 * 1000.0f));
                    }
                    CONTROL_UTILITY_MODE = MODE_NO_ACT;
                    TMR3_COUNT_TORQUE_NULL = 0;
                }
                break;
            }

            case MODE_FIND_HOME: {
                static int cnt_findhome = 0;
                static int cnt_terminate_findhome = 0;
                static float FINDHOME_POSITION_pulse = 0.0f;
                static float FINDHOME_POSITION_pulse_OLD = 0.0f;
                static float FINDHOME_VELOCITY_pulse = 0.0f;
                static float REF_POSITION_FINDHOME_INIT = 0.0f;

                if (FINDHOME_STAGE == FINDHOME_INIT) {
                    REFERENCE_MODE = MODE_REF_FINDHOME;
                    cnt_findhome = 0;
                    cnt_terminate_findhome = 0;
                    pos.ref = pos.sen;
                    vel.ref = 0.0f;
                    REF_POSITION_FINDHOME = pos.ref;
                    FINDHOME_STAGE = FINDHOME_GOTOLIMIT;
                } else if (FINDHOME_STAGE == FINDHOME_GOTOLIMIT) {
                    int cnt_check_enc = (TMR_FREQ_5k/20); // 5000/20 = 250tic = 50msec
                    if(cnt_findhome%cnt_check_enc == 0) {
                        FINDHOME_POSITION_pulse = pos.sen*ENC_PULSE_PER_POSITION;
                        FINDHOME_VELOCITY_pulse = FINDHOME_POSITION_pulse - FINDHOME_POSITION_pulse_OLD;
                        FINDHOME_POSITION_pulse_OLD = FINDHOME_POSITION_pulse;
                    }
                    cnt_findhome++;

                    if (fabs(FINDHOME_VELOCITY_pulse) <= 1) {
                        cnt_terminate_findhome = cnt_terminate_findhome + 1;
                    } else {
                        cnt_terminate_findhome = 0;
                    }

                    if ((cnt_terminate_findhome < 3*TMR_FREQ_5k) &&  cnt_findhome < 10*TMR_FREQ_5k) { // wait for 3sec
                        double GOTOHOME_SPEED = 10.0f; // 20mm/s or 20deg/s
                        if (HOMEPOS_OFFSET > 0) {
                            REF_POSITION_FINDHOME = REF_POSITION_FINDHOME + GOTOHOME_SPEED*DT_5k;
                        } else {
                            REF_POSITION_FINDHOME = REF_POSITION_FINDHOME - GOTOHOME_SPEED*DT_5k;
                        }
                        CONTROL_MODE = MODE_JOINT_CONTROL;
                        alpha_trans = 0.0f;
                    } else {
                        ENC_SET((long)((long)HOMEPOS_OFFSET*10));
                        REF_POSITION_FINDHOME_INIT = (float)((long)HOMEPOS_OFFSET*10);
                        FINDHOME_POSITION_pulse = 0;
                        FINDHOME_POSITION_pulse_OLD = 0;
                        FINDHOME_VELOCITY_pulse = 0;

                        cnt_findhome = 0;
                        cnt_terminate_findhome = 0;
                        pos.ref = 0.0f;
                        FINDHOME_STAGE = FINDHOME_ZEROPOSE;
                    }
                } else if (FINDHOME_STAGE == FINDHOME_ZEROPOSE) {

//                    int T_move = 2*TMR_FREQ_5k;
                    int T_move = 10000;
                    REF_POSITION_FINDHOME = ((0.0f - REF_POSITION_FINDHOME_INIT)*0.5f*(1.0f - cos(3.14159f * (float)cnt_findhome / (float)T_move)) + (float)REF_POSITION_FINDHOME_INIT)/ENC_PULSE_PER_POSITION;

                    cnt_findhome++;

                    REFERENCE_MODE = MODE_REF_FINDHOME;
                    CONTROL_MODE = MODE_JOINT_CONTROL;
                    alpha_trans = 0.0f;

                    if (cnt_findhome >= T_move) {
                        cnt_findhome = 0;
                        pos.ref = 0.0f;
                        FINDHOME_STAGE = FINDHOME_INIT;
                        CONTROL_UTILITY_MODE = MODE_JOINT_CONTROL;
                        REFERENCE_MODE = MODE_REF_DIRECT;
                    }
                }
                break;
            }
            default:
                break;
        }

        // CONTROL MODE ------------------------------------------------------------
        switch (CONTROL_MODE) {
            case MODE_NO_ACT: {
                V_out = 0.0f;
                break;
            }

            case MODE_VALVE_POSITION_CONTROL: {
                if (OPERATING_MODE == 5) { //SW Valve
                    VALVE_POS_CONTROL(valve_pos.ref);
                    V_out = Vout.ref;
                } else if (CURRENT_CONTROL_MODE == 0) { //PWM
                    V_out = valve_pos.ref;
                } else {
                    I_REF = valve_pos.ref * 0.001f; // Unit : pulse >> mA
                    float I_MAX = 10.0f; // Max : 10mA
                    if (I_REF > I_MAX) {
                        I_REF = I_MAX;
                    } else if (I_REF < -I_MAX) {
                        I_REF = -I_MAX;
                    }
                }
                break;
            }

            case MODE_JOINT_CONTROL: {

                float temp_vel_pos = 0.0f; // desired velocity for position control
                float temp_vel_FT = 0.0f; // desired velocity for force/torque control
                float temp_vel_ff = 0.0f; // desired velocity for feedforward control
                float temp_vel = 0.0f;

                float wn_Pos = 2.0f * PI * 5.0f; // f_cut : 5Hz Position Control

                pos.err = pos.ref - pos.sen; // Unit : mm or deg
                vel.err = vel.ref - vel.sen; // Unit : mm/s or deg/s

                // position control command ===============================================================================================================================================
                if ((OPERATING_MODE & 0b01) == 0) { // Rotary Mode
                    temp_vel_pos = 0.1f * (P_GAIN_JOINT_POSITION * wn_Pos * pos.err) * PI / 180.0f; // rad/s
                    //                            L when P-gain = 100, f_cut = 10Hz
                } else {
                    temp_vel_pos = 0.1f * (P_GAIN_JOINT_POSITION * wn_Pos * pos.err); // mm/s
                    //                            L when P-gain = 100, f_cut = 10Hz
                }

                // torque control command ===============================================================================================================================================
                float alpha_SpringDamper = 1.0f/(1.0f+TMR_FREQ_5k/(2.0f*PI*30.0f));
                K_LPF = (1.0f-alpha_SpringDamper) * K_LPF + alpha_SpringDamper * K_SPRING;
                D_LPF = (1.0f-alpha_SpringDamper) * D_LPF + alpha_SpringDamper * D_DAMPER;

                if ((OPERATING_MODE & 0b01) == 0) { // Rotary Mode
                    float torq_ref_act = torq.ref + K_SPRING * pos.err + D_DAMPER * vel.err; // unit : Nm
                    torq.err = torq_ref_act - torq.sen;
                    torq.err_int += torq.err/((float)TMR_FREQ_5k);
                    temp_vel_FT = 0.001f * (P_GAIN_JOINT_TORQUE * torq.err + I_GAIN_JOINT_TORQUE * torq.err_int); // Nm >> rad/s
                } else {
                    float force_ref_act = force.ref + K_SPRING * pos.err + D_DAMPER * vel.err; // unit : N
//                    force_ref_act_can = force_ref_act;
                    force.err = force_ref_act - force.sen;
                    float alpha_int = 1.0f / (1.0f + TMR_FREQ_5k / (2.0f * PI * 0.1f));
                    force.err_int = (1.0f-alpha_int)*force.err_int;
                    //                    force.err_int += force.err/((float)TMR_FREQ_5k);
                    if (force.err < 20.0f && force.err > -20.0f) {
                        force.err = 0.0f;
                    } else {
                        force.err_int += force.err / ((float)TMR_FREQ_5k);
                    }
                    temp_vel_FT = 0.001f * (P_GAIN_JOINT_TORQUE * force.err + I_GAIN_JOINT_TORQUE * force.err_int); // N >> mm/s
                }


                // velocity feedforward command ========================================================================================================================================
                if ((OPERATING_MODE & 0b01) == 0) { // Rotary Mode
                    temp_vel_ff = 0.01f * (float)VELOCITY_COMP_GAIN * vel.ref * PI / 180.0f; // rad/s
                } else {
                    temp_vel_ff = 0.01f * (float)VELOCITY_COMP_GAIN * vel.ref; // mm/s
                }

                // command integration =================================================================================================================================================
                temp_vel = (1.0f - alpha_trans) * temp_vel_pos  + alpha_trans * temp_vel_FT + temp_vel_ff; // Position Control + Torque Control + Velocity Feedforward
                
                float Qact = 0.0f; // required flow rate
                if( temp_vel > 0.0f ) {
                    Qact = temp_vel * ((float)PISTON_AREA_A * 0.00006f); // mm^3/sec >> LPM
                    I_REF = tanh_inv(Qact/(K_v * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f))))/C_d;
                } else {
                    Qact = temp_vel * ((float)PISTON_AREA_B * 0.00006f); // mm^3/sec >> LPM
                    I_REF = tanh_inv(Qact/(K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f))))/C_d;
                }
                
                float I_MAX = 10.0f; // Maximum Current : 10mA
                
                // Anti-windup for FT
//                if (I_GAIN_JOINT_TORQUE != 0.0f) {
                if (I_GAIN_JOINT_TORQUE > 0.001f) {
                    float Ka = 2.0f;
                    if (I_REF > I_MAX) {
                        float I_rem = I_REF - I_MAX;
                        I_REF = I_MAX;
                        float temp_vel_rem = K_v * sqrt(PRES_SUPPLY * alpha3 / (alpha3 + 1.0f)) * tanh(C_d*I_rem) / ((double) PISTON_AREA_A * 0.00006f); // Unit : mm/s [linear] / rad/s [rotary]
                        torq.err_int = torq.err_int - Ka * temp_vel_rem * (10000.0f/I_GAIN_JOINT_TORQUE);
                    } else if (I_REF < -I_MAX) {
                        double I_rem = I_REF - (-I_MAX);
                        I_REF = -I_MAX;
                        float temp_vel_rem = K_v * sqrt(PRES_SUPPLY / (alpha3 + 1.0f)) * tanh(C_d*I_rem) / ((double) PISTON_AREA_B * 0.00006f); // Unit : mm/s [linear] / rad/s [rotary]
                        torq.err_int = torq.err_int - Ka * temp_vel_rem * (10000.0f/I_GAIN_JOINT_TORQUE);
                    }
                } else {
                    if(I_REF > I_MAX) {
                        I_REF = I_MAX;
                    } else if (I_REF < -I_MAX) {
                        I_REF = -I_MAX;  
                    }
                }
                break;
            }

            case MODE_VALVE_OPEN_LOOP: {
                V_out = (float) Vout.ref;
                break;
            }

//            case MODE_JOINT_ADAPTIVE_BACKSTEPPING: {
//
//                float Va = (1256.6f + Amm * pos.sen/(float)(ENC_PULSE_PER_POSITION)) * 0.000000001f; // 4mm pipe * 100mm + (25mm Cylinder 18mm Rod) * x,      unit : m^3
//                float Vb = (1256.6f + Amm  * (79.0f - pos.sen/(float)(ENC_PULSE_PER_POSITION))) * 0.000000001f; // 4mm pipe * 100mm + (25mm Cylinder 18mm Rod) * (79.0mm-x),      unit : m^3
//
//                V_adapt = 1.0f / (1.0f/Va + 1.0f/Vb); //initial 0.0000053f
//
//                //float f3 = -Amm*Amm*beta*0.000001f*0.000001f/V_adapt * vel.sen/(float)(ENC_PULSE_PER_POSITION)*0.001f; // unit : N/s    //xdot=10mm/s일때 -137076
//                float f3_hat = -a_hat * vel.sen/(float)(ENC_PULSE_PER_POSITION)*0.001f; // unit : N/s    //xdot=10mm/s일때 -137076
//
//                float g3_prime = 0.0f;
//                if (torq.sen > Amm*(Ps-Pt)*0.000001f) {
//                    g3_prime = 1.0f;
//                } else if (torq.sen < -Amm*(Ps-Pt)*0.000001f) {
//                    g3_prime = -1.0f;
//                } else {
//                    if ((value-VALVE_CENTER) > 0) {
//                        g3_prime = sqrt(Ps-Pt-torq.sen/Amm*1000000.0f);
////                        g3_prime = sqrt(Ps-Pt);
//                    } else {
//                        g3_prime = sqrt(Ps-Pt+torq.sen/Amm*1000000.0f);
////                        g3_prime = sqrt(Ps-Pt);
//                    }
//                }
//                float tau = 0.01f;
//                float K_valve = 0.0004f;
//
//                float x_v = 0.0f;   //x_v : -1~1
//                if(value>=VALVE_CENTER) {
//                    x_v = 1.0f*((double)value - (double)VALVE_CENTER)/((double)VALVE_MAX_POS - (double)VALVE_CENTER);
//                } else {
//                    x_v = -1.0f*((double)value - (double)VALVE_CENTER)/((double)VALVE_MIN_POS - (double)VALVE_CENTER);
//                }
//                float f4 = -x_v/tau;
//                float g4 = K_valve/tau;
//
//                float torq_ref_dot = torq.ref_diff * 500.0f;
//
//                pos.err = (pos.ref - pos.sen)/(float)(ENC_PULSE_PER_POSITION); //[mm]
//                vel.err = (0.0f - vel.sen)/(float)(ENC_PULSE_PER_POSITION); //[mm/s]
//                pos.err_sum += pos.err/(float) TMR_FREQ_5k; //[mm]
//
//                torq.err = torq.ref - torq.sen; //[N]
//                torq.err_sum += torq.err/(float) TMR_FREQ_5k; //[N]
//
//                float k3 = 2000.0f; //2000  //20000
//                float k4 = 10.0f;
//                float rho3 = 3.2f;
//                float rho4 = 10000000.0f;  //25000000.0f;
//                float x_4_des = (-f3_hat + torq_ref_dot - k3*(-torq.err))/(gamma_hat*g3_prime);
//                if (x_4_des > 1) x_4_des = 1;
//                else if (x_4_des < -1) x_4_des = -1;
//
//                if (x_4_des > 0) {
//                    valve_pos.ref = x_4_des * (float)(VALVE_MAX_POS - VALVE_CENTER) + (float) VALVE_CENTER;
//                } else {
//                    valve_pos.ref = x_4_des * (float)(VALVE_CENTER - VALVE_MIN_POS) + (float) VALVE_CENTER;
//                }
//
//                float x_4_des_dot = (x_4_des - x_4_des_old)*(float) TMR_FREQ_5k;
//                x_4_des_old = x_4_des;
//                V_out = (-f4 + x_4_des_dot - k4*(x_v-x_4_des)- rho3/rho4*gamma_hat*g3_prime*(-torq.err))/g4;
//
//                float rho_a = 0.00001f;
//                float a_hat_dot = -rho3/rho_a*vel.sen/(float)(ENC_PULSE_PER_POSITION)*0.001f*(-torq.err);
//                a_hat = a_hat + a_hat_dot / (float) TMR_FREQ_5k;
//
//                if(a_hat > -3000000.0f) a_hat = -3000000.0f;
//                else if(a_hat < -30000000.0f) a_hat = -30000000.0f;
//
//                break;
//            }

            default:
                break;
        }


        if (((OPERATING_MODE&0b110)>>1) == 0 || ((OPERATING_MODE&0b110)>>1) == 1) { //Moog Valve or KNR Valve

            ////////////////////////////////////////////////////////////////////////////
            ////////////////////////////  CURRENT CONTROL //////////////////////////////
            ////////////////////////////////////////////////////////////////////////////
            if (CURRENT_CONTROL_MODE) {
                double alpha_update_Iref = 1.0f / (1.0f + 5000.0f / (2.0f * 3.14f * 300.0f)); // f_cutoff : 500Hz
                I_REF_fil = (1.0f - alpha_update_Iref) * I_REF_fil + alpha_update_Iref*I_REF;

                if (I_REF_fil > 0.0f) I_REF_fil_DZ = I_REF_fil + (double)VALVE_DEADZONE_PLUS*mA_PER_pulse; // unit: mA
                else if (I_REF_fil < 0.0f) I_REF_fil_DZ = I_REF_fil + (double)VALVE_DEADZONE_MINUS*mA_PER_pulse; // unit: mA
                else I_REF_fil_DZ = I_REF_fil + (double)(VALVE_DEADZONE_PLUS+VALVE_DEADZONE_MINUS)/2.0f*mA_PER_pulse; // unit: mA

                I_ERR = I_REF_fil_DZ - (double)cur.sen;
                I_ERR_INT = I_ERR_INT + (I_ERR) * 0.0002f;


                // Moog Valve Current Control Gain
                double R_model = 500.0f; // ohm
                double L_model = 1.2f;
                double w0 = 2.0f * 3.14f * 50.0f;
                double KP_I = 0.1f * L_model*w0;
                double KI_I = 0.1f * R_model*w0;

                // KNR Valve Current Control Gain
                if (((OPERATING_MODE & 0b110)>>1) == 1) { // KNR Valve
                    R_model = 163.0f; // ohm
                    L_model = 1.0f;
                    w0 = 2.0f * 3.14f * 80.0f;
                    KP_I = 1.0f * L_model*w0;
                    KI_I = 0.08f * R_model*w0;
                }

                double FF_gain = 1.0f;

                VALVE_PWM_RAW = KP_I * 2.0f * I_ERR + KI_I * 2.0f* I_ERR_INT;
                I_REF_fil_diff = I_REF_fil_DZ - I_REF_fil_old;
                I_REF_fil_old = I_REF_fil_DZ;
//                VALVE_PWM_RAW = VALVE_PWM_RAW + FF_gain * (R_model * I_REF_fil + L_model * I_REF_fil_diff * 5000.0f); // Unit : mV
                VALVE_PWM_RAW = VALVE_PWM_RAW + FF_gain * (R_model * I_REF_fil_DZ); // Unit : mV
                double V_MAX = 12000.0f; // Maximum Voltage : 12V = 12000mV

                double Ka = 3.0f / KP_I;
                if (VALVE_PWM_RAW > V_MAX) {
                    V_rem = VALVE_PWM_RAW - V_MAX;
                    V_rem = Ka*V_rem;
                    VALVE_PWM_RAW = V_MAX;
//                    I_ERR_INT = I_ERR_INT - V_rem * 0.0002f;
                    I_ERR_INT = I_ERR_INT - V_rem;
                } else if (VALVE_PWM_RAW < -V_MAX) {
                    V_rem = VALVE_PWM_RAW - (-V_MAX);
                    V_rem = Ka*V_rem;
                    VALVE_PWM_RAW = -V_MAX;
//                    I_ERR_INT = I_ERR_INT - V_rem * 0.0002f;
                    I_ERR_INT = I_ERR_INT - V_rem;
                }
            } else {
                VALVE_PWM_RAW = I_REF * mV_PER_mA;
            }

            ////////////////////////////////////////////////////////////////////////////
            /////////////////  Dead Zone Cancellation & Linearization //////////////////
            ////////////////////////////////////////////////////////////////////////////

            // Output Voltage Linearization
            double CUR_PWM_nonlin = (double)VALVE_PWM_RAW; // Unit : mV
            double CUR_PWM_lin = PWM_duty_byLT(CUR_PWM_nonlin);  // -8000~8000

            // Dead Zone Cancellation (Electrical dead-zone)
            if (CUR_PWM_lin > 0) V_out = (float) (CUR_PWM_lin + 169.0f);
            else if (CUR_PWM_lin < 0) V_out = (float) (CUR_PWM_lin - 174.0f);
            else V_out = (float) (CUR_PWM_lin);

        } else {            //////////////////////////sw valve
            // Output Voltage Linearization & Dead Zone Cancellation (Electrical dead-zone) by SW
            if (V_out > 0 ) V_out = (V_out + 180.0f)/0.8588f;
            else if (V_out < 0) V_out = (V_out - 200.0f)/0.8651f;
            else V_out = 0.0f;
        }

        ////////////////////////////////////////////////////////////////////
        ///////////////////  PWM Command ///////////////////////////////////
        ////////////////////////////////////////////////////////////////////
        if(DIR_VALVE<0) {
            V_out = -V_out;
        }

        if (V_out >= VALVE_VOLTAGE_LIMIT*1000.0f) {
            V_out = VALVE_VOLTAGE_LIMIT*1000.0f;
        } else if(V_out<=-VALVE_VOLTAGE_LIMIT*1000.0f) {
            V_out = -VALVE_VOLTAGE_LIMIT*1000.0f;
        }
        PWM_out= V_out/(SUPPLY_VOLTAGE*1000.0f);

        // Saturation of output voltage
        if(PWM_out > 1.0f) PWM_out=1.0f;
        else if (PWM_out < -1.0f) PWM_out=-1.0f;

        if (PWM_out>0.0f) {
            dtc_v=0.0f;
            dtc_w=PWM_out;
        } else {
            dtc_v=-PWM_out;
            dtc_w=0.0f;
        }

        //pwm
        TIM4->CCR2 = (PWM_ARR)*(1.0f-dtc_v);
        TIM4->CCR1 = (PWM_ARR)*(1.0f-dtc_w);

        ////////////////////////////////////////////////////////////////////////////
        //////////////////////  Data transmission through CAN //////////////////////
        ////////////////////////////////////////////////////////////////////////////

        if (TMR2_COUNT_CAN_TX % (int) ((int) TMR_FREQ_5k/CAN_FREQ) == 0) {

            // Position, Velocity, and Torque (ID:1200)
            if (flag_data_request[0] == HIGH) {
                if ((OPERATING_MODE & 0b01) == 0) { // Rotary Actuator
                    CAN_TX_POSITION_FT((int16_t) (pos.sen*200.0f), (int16_t) (vel.sen*20.0f), (int16_t) (torq.sen*TORQUE_SENSOR_PULSE_PER_TORQUE*10.0f));
//                    CAN_TX_POSITION_FT((int16_t) (PRES_B_VREF*10.0f*200.0f), (int16_t) (vel.sen*20.0f), (int16_t) (pres_B.sen*TORQUE_SENSOR_PULSE_PER_TORQUE*10.0f));

                } else if ((OPERATING_MODE & 0b01) == 1) { // Linear Actuator
                    CAN_TX_POSITION_FT((int16_t) (pos.sen*200.0f), (int16_t) (vel.sen*20.0f), (int16_t) (force.sen*TORQUE_SENSOR_PULSE_PER_TORQUE*10.0f));
//                    CAN_TX_POSITION_FT((int16_t) (PRES_B_VREF*10.0f*200.0f), (int16_t) (vel.sen*20.0f), (int16_t) (pres_B.sen*TORQUE_SENSOR_PULSE_PER_TORQUE*10.0f));
//                    CAN_TX_POSITION_FT((int16_t) (logging*200.0f), (int16_t) (vel.sen*20.0f), (int16_t) (force.sen*TORQUE_SENSOR_PULSE_PER_TORQUE*10.0f));
                }
            }

            // Valve Position (ID:1300)
            if (flag_data_request[1] == HIGH) {
                CAN_TX_PWM((int16_t)(cur.sen/mA_PER_pulse));
            }

            // Others : Pressure A, B, Supply Pressure, etc. (for Debugging)  (ID:1400)
            if (flag_data_request[2] == HIGH) {
                CAN_TX_SOMETHING((int16_t)(pres_A.sen*100.0f), (int16_t)(pres_B.sen*100.0f), (int16_t) (K_SPRING), (int16_t) (D_DAMPER));
            }

            TMR2_COUNT_CAN_TX = 0;
        }
        TMR2_COUNT_CAN_TX++;

    }
    TIM3->SR = 0x0;  // reset the status register

}