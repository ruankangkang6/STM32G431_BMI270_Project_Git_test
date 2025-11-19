#include "idcu_api.h"
#include "fdcan.h"

#include "can_api.h" 
#include "usart_driver.h"
#include "angle_calc.h"

#include "idcu_hill_start_assist.h"
#include "idcu_vehicle_start_opt.h"
#include "idcu_hill_descent_assist.h"
#include "idcu_falling_security_protection.h"


bool iDCU_HSA_Enable = true;  // Hill Start Assist enable flag
bool iDCU_HDA_Enable = true;  // Hill Descent Assist enable flag  
bool iDCU_VSO_Enable = true;  // Vehicle Start Optimization enable flag
bool iDCU_FSP_Enable = true;    


extern  uint8_t tx_data[8];
extern  uint8_t data_len;          
extern uint32_t can_id;
extern CAN_Comm_Struct_t fdcan_comm;
extern UART_Comm_Struct_t idcu_uart_comm;
// extern iDCU_SystemStatus_s iDCU_SystemStatus;

extern void idcu_hsa_interface(void);

//Define the attitude Angle variable structure
static Attitude_t current_attitude = {0.0f, 0.0f, 0.0f};
static uint32_t last_update_time = 0;
//Define the Angle variable

int idcu_main(void)
{
    idcu_init();

    while (1)
    {
        // Task_1 get the rawdata for filtering (5ms)
        if(read_gsensor_flag == 1) 
        {
            read_gsensor_flag = 0;
            // 1 Read the rawdata and filtering
            get_acc_gyr_data(acc_gyro_buffer, 6);

            // 2 Check the condition of the scooter
            iDCU_FSP_MainFunction();
        }

        // Task_2 CAN_Data receive was successful. (100ms)
        if (fdcan_comm.rx_complete == true)
        {
            // 1> Obtain the speed based on the Gsensor status

            // 2> Send uart data to the controller

            // 3> Sent uart data completed. Clear the mark
            fdcan_comm.rx_complete = false;
            controller_update_message_process();
			
        }

        // Task_3 Uart_Receiv data receive. Send CAN data to the screen
        if (idcu_uart_comm.rx_complete)
        {
            // sent uart data to the screen.
            // can_send_message_to_hmi();
            hmi_update_message_process();
            idcu_uart_comm.rx_complete = false;
        }
    }

}


int controller_update_message_process()
{
    uint16_t vehicle_can_speed = 0;
    uint16_t speed_tmp = 0;
    uint8_t ABS_BrakeLevel = 0;

    float hall_speedkph;
    uint16_t hall_time;

    // Prepare uart tx buff
    preparer_data_uart_Tx((uint8_t *)fdcan_comm.data_buffer, sizeof(fdcan_comm.data_buffer));

    // Get the original throttle signal
    speed_tmp = ((fdcan_comm.data_buffer[7] << 8) | fdcan_comm.data_buffer[6]);
    hall_time   =  ((idcu_uart_comm.rx_buff[11] << 8) | idcu_uart_comm.rx_buff[12]);
 
    hall_speedkph = (WHEEL_DIAMETER_INCH * WHEEL_CONST) / (float)hall_time;

    iDCU_SystemStatus.throttle_speed = speed_tmp;
    iDCU_SystemStatus.wheel_speed = hall_speedkph;

    iDCU_SystemStatus.current_gear = fdcan_comm.data_buffer[1];


    //====== Task: IDCU Vehicle Start output ==========
    vehicle_can_speed = iDCU_VSO_GetOutputThrottle();
    vehicle_start_speed_output(vehicle_can_speed);

    //====== Task: Hill Descent Assist ==========
    iDCU_HDA_MainFunction();
    HDA_ABS_output(enHDA_BrakeLevel);

    //========= Task: Hill Start Assist =========
    check_HSA_status();
    Hill_Start_Assist_speed_output(speed_tmp);

    //======== Task: falling security protection ==========
    iDCU_SkidControl_Handler();

    //========= UART transmits data to the controller ===============
    idcu_uart_comm.tx_buff[19] = calc_xor_checksum(idcu_uart_comm.tx_buff, 19);
    HAL_UART_Transmit_DMA(&huart1,(uint8_t *)idcu_uart_comm.tx_buff, sizeof(idcu_uart_comm.tx_buff));

    return 0;
}

// Final throttle speed output.  Hill_Start_Assist
void Hill_Start_Assist_speed_output(uint16_t speed)
{
    uint16_t final_throttle = 0;

    // If the HHC function is enabled
    if (iDCU_HSA_IsActive())
    {
        if (speed < HSA_CAN_THROTTLE_ENTRY_THRESHOLD)
        {
            final_throttle = iDCU_HSA_GetOverrideThrottle_0_1000(); 

            // Update throttle signal.
            idcu_uart_comm.tx_buff[16] = (uint8_t)((final_throttle >> 8) & 0xFF);          //High byte speed
            idcu_uart_comm.tx_buff[17] = (uint8_t)(final_throttle & 0xFF);   //Low byte speed
        }
    }
}


// Vehicle start speed output.
void vehicle_start_speed_output(const uint16_t start_speed)
{
    // Update throttle signal.
    idcu_uart_comm.tx_buff[16] = (uint8_t)((start_speed >> 8) & 0xFF);          //High byte speed
    idcu_uart_comm.tx_buff[17] = (uint8_t)(start_speed & 0xFF);   //Low byte speed
}

// Hill Descent Assist ABS brake updata.
int HDA_ABS_output(const uint16_t ABS_BrakeLevel)
{
    uint32_t ret = 0;
    uint8_t temp, high_bits;

    // // Update throttle signal.
    // idcu_uart_comm.tx_buff[16] = (uint8_t)(start_speed & 0xFF);          //High byte speed
    // idcu_uart_comm.tx_buff[17] = (uint8_t)((start_speed >> 8) & 0xFF);   //Low byte speed\

    // if (start_speed >= ABS_LEVEL_CLOSE)
    // {
    //     ret = 1;
    //     return ret;
    // }

    if (ABS_BrakeLevel >= ABS_LEVEL_CLOSE)
    {
        idcu_uart_comm.tx_buff[10] = 0x00;
        idcu_uart_comm.tx_buff[18] &= ~0x20;
        ret = 1;
 
        return ret;
    }
    else		// else (ABS_BrakeLevel < ABS_LEVEL_CLOSE)
    {
        temp = (idcu_uart_comm.tx_buff[10] & 0x0F);
        switch(ABS_BrakeLevel) 
        {
            case 0: high_bits = 0x00;   break;  // 0000
            case 1: high_bits = 0x10;   break;  // 0001
            case 2: high_bits = 0x20;   break;  // 0010
            case 3: high_bits = 0x30;   break;  // 0011
        }

        idcu_uart_comm.tx_buff[10] = temp | high_bits;
        idcu_uart_comm.tx_buff[18] |= 0x20;
    }


    return ret;    
}

void iDCU_SkidControl_Handler(void)
{
    // Handle FSP request
    if (g_FSP_Request_PowerCut) 
    {
        // Cut off the power
        idcu_uart_comm.tx_buff[16] = 0x00;   //Speed High
        idcu_uart_comm.tx_buff[17] = 0x00;   //Speed Low 
    }

    if (g_FSP_Alarm_Active) 
    {
        // Trigger the buzzer and illuminate the fault indicator light
        // Todo!!!
    }
}


void idcu_init(void)
{
    // Hill Start Assist init.
    iDCU_HSA_Init();

    // Vehicle Start output init.
    iDCU_VSO_Init();

    // Hill Descent Assist init.
    iDCU_HDA_Init();

    // iDCU falling security protection
    iDCU_FSP_Init();

    // UART send buff init.
    controller_buff_init();
}


