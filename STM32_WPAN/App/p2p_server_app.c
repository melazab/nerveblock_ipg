/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    p2p_server_app.c
 * @author  MCD Application Team
 * @brief   Peer to peer Server Application
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2019-2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "p2p_server_app.h"
#include "CmdRespProtocol.h"
#include "app_ble.h"
#include "app_common.h"
#include "ble.h"
#include "dbg_trace.h"
#include "main.h"
#include "tx_api.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Queue definitions
#define QUEUE_SIZE       10     // Adjust based on your buffering needs
#define MAX_MESSAGE_SIZE 137    // Your specified maximum data size

                                /* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint8_t Device_Led_Selection;
    uint8_t Led1;
} P2P_LedCharValue_t;

typedef struct
{
    uint8_t Device_Button_Selection;
    uint8_t ButtonStatus;
} P2P_ButtonCharValue_t;

typedef struct
{
    uint8_t Notification_Status; /* used to check if P2P Server is enabled to Notify */
    P2P_LedCharValue_t LedControl;
    P2P_ButtonCharValue_t ButtonControl;
    uint16_t ConnectionHandle;
} P2P_Server_App_Context_t;

// Message structure for the queue
typedef struct
{
    uint8_t message_type;    // 1 byte
    uint8_t data_length;     // 1 byte
    uint8_t* data_ptr;       // 4 bytes - pointer to data
} NOTIFICATION_MESSAGE_T;    // Total: 8 bytes (2 ULONGs)

#define MESSAGE_SIZE_ULONG                                                                         \
    ( ( sizeof( NOTIFICATION_MESSAGE_T ) + sizeof( ULONG ) - 1 ) / sizeof( ULONG ) )

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/**
 * START of Section BLE_APP_CONTEXT
 */

static P2P_Server_App_Context_t P2P_Server_App_Context;
static SERIAL_DATA_OUT_BUF_T notification_data;
/**
 * END of Section BLE_APP_CONTEXT
 */
TX_EVENT_FLAGS_GROUP event_flags;
#define BUTTON_NOTIFICATION  0x01
#define DATA_TX_NOTIFICATION 0x02

// static TX_SEMAPHORE sem_SendNotificationSignal;
TX_THREAD* thread_SendNotificationProcess;
TX_QUEUE* notification_queue;
TX_BYTE_POOL* msg_byte_pool;
// UCHAR queue_memory[ QUEUE_SIZE * MESSAGE_SIZE_ULONG * sizeof( ULONG ) ];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void P2PS_Send_Notification( void );
static void P2PS_Send_Notification_Data( SERIAL_DATA_OUT_BUF_T* p_data );
static void P2PS_APP_LED_BUTTON_context_Init( void );
static void thread_SendNotification_entry( ULONG argument );

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification( P2PS_STM_App_Notification_evt_t* pNotification )
{
    /* USER CODE BEGIN P2PS_STM_App_Notification_1 */

    /* USER CODE END P2PS_STM_App_Notification_1 */
    switch ( pNotification->P2P_Evt_Opcode )
    {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */
#if ( BLE_CFG_OTA_REBOOT_CHAR != 0 )
        case P2PS_STM_BOOT_REQUEST_EVT:
            APP_DBG_MSG( "-- P2P APPLICATION SERVER : BOOT REQUESTED\n" );
            APP_DBG_MSG( " \n\r" );

            *(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
            NVIC_SystemReset();
            break;
#endif
            /* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

        case P2PS_STM__NOTIFY_ENABLED_EVT:
            /* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
            P2P_Server_App_Context.Notification_Status = 1;
            APP_DBG_MSG( "-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n" );
            APP_DBG_MSG( " \n\r" );
            /* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
            break;

        case P2PS_STM_NOTIFY_DISABLED_EVT:
            /* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
            P2P_Server_App_Context.Notification_Status = 0;
            APP_DBG_MSG( "-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n" );
            APP_DBG_MSG( " \n\r" );
            /* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
            break;

        case P2PS_STM_WRITE_EVT:
            /* USER CODE BEGIN P2PS_STM_WRITE_EVT */
            int i = 0;
            while ( i < pNotification->DataTransfered.Length )
            {
                crProtQueueByte( pNotification->DataTransfered.pPayload[ i ] );
                i++;
            }

            if ( pNotification->DataTransfered.pPayload[ 0 ] == 0x00 )
            { /* ALL Deviceselected - may be necessary as LB Routeur informs all connection */
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x01 )
                {
                    BSP_LED_On( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER  : LED1 ON\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x01; /* LED1 ON */
                }
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x00 )
                {
                    BSP_LED_Off( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER  : LED1 OFF\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x00; /* LED1 OFF */
                }
            }

#if ( P2P_SERVER1 != 0 )
            if ( pNotification->DataTransfered.pPayload[ 0 ] == 0x01 )
            { /* end device 1 selected - may be necessary as LB Routeur informs all connection */
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x01 )
                {
                    BSP_LED_On( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 1 : LED1 ON\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x01; /* LED1 ON */
                }
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x00 )
                {
                    BSP_LED_Off( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 1 : LED1 OFF\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x00; /* LED1 OFF */
                }
            }
#endif
#if ( P2P_SERVER2 != 0 )
            if ( pNotification->DataTransfered.pPayload[ 0 ] == 0x02 )
            { /* end device 2 selected */
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x01 )
                {
                    BSP_LED_On( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 2 : LED1 ON\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x01; /* LED1 ON */
                }
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x00 )
                {
                    BSP_LED_Off( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 2 : LED1 OFF\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x00; /* LED1 OFF */
                }
            }
#endif
#if ( P2P_SERVER3 != 0 )
            if ( pNotification->DataTransfered.pPayload[ 0 ] == 0x03 )
            { /* end device 3 selected - may be necessary as LB Routeur informs all connection */
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x01 )
                {
                    BSP_LED_On( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 3 : LED1 ON\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x01; /* LED1 ON */
                }
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x00 )
                {
                    BSP_LED_Off( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 3 : LED1 OFF\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x00; /* LED1 OFF */
                }
            }
#endif
#if ( P2P_SERVER4 != 0 )
            if ( pNotification->DataTransfered.pPayload[ 0 ] == 0x04 )
            { /* end device 4 selected */
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x01 )
                {
                    BSP_LED_On( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 2 : LED1 ON\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x01; /* LED1 ON */
                }
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x00 )
                {
                    BSP_LED_Off( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 2 : LED1 OFF\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x00; /* LED1 OFF */
                }
            }
#endif
#if ( P2P_SERVER5 != 0 )
            if ( pNotification->DataTransfered.pPayload[ 0 ] == 0x05 )
            { /* end device 5 selected - may be necessary as LB Routeur informs all connection */
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x01 )
                {
                    BSP_LED_On( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 5 : LED1 ON\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x01; /* LED1 ON */
                }
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x00 )
                {
                    BSP_LED_Off( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 5 : LED1 OFF\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x00; /* LED1 OFF */
                }
            }
#endif
#if ( P2P_SERVER6 != 0 )
            if ( pNotification->DataTransfered.pPayload[ 0 ] == 0x06 )
            { /* end device 6 selected */
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x01 )
                {
                    BSP_LED_On( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 6 : LED1 ON\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x01; /* LED1 ON */
                }
                if ( pNotification->DataTransfered.pPayload[ 1 ] == 0x00 )
                {
                    BSP_LED_Off( LED_BLUE );
                    APP_DBG_MSG( "-- P2P APPLICATION SERVER 6 : LED1 OFF\n" );
                    APP_DBG_MSG( " \n\r" );
                    P2P_Server_App_Context.LedControl.Led1 = 0x00; /* LED1 OFF */
                }
            }
#endif
            /* USER CODE END P2PS_STM_WRITE_EVT */
            break;

        default:
            /* USER CODE BEGIN P2PS_STM_App_Notification_default */

            /* USER CODE END P2PS_STM_App_Notification_default */
            break;
    }
    /* USER CODE BEGIN P2PS_STM_App_Notification_2 */

    /* USER CODE END P2PS_STM_App_Notification_2 */
    return;
}

void P2PS_APP_Notification( P2PS_APP_ConnHandle_Not_evt_t* pNotification )
{
    /* USER CODE BEGIN P2PS_APP_Notification_1 */

    /* USER CODE END P2PS_APP_Notification_1 */
    switch ( pNotification->P2P_Evt_Opcode )
    {
            /* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

            /* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
        case PEER_CONN_HANDLE_EVT:
            /* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
            APP_BLE_Key_Button2_Action();
            /* USER CODE END PEER_CONN_HANDLE_EVT */
            break;

        case PEER_DISCON_HANDLE_EVT:
            /* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
            P2PS_APP_LED_BUTTON_context_Init();
            /* USER CODE END PEER_DISCON_HANDLE_EVT */
            break;

        default:
            /* USER CODE BEGIN P2PS_APP_Notification_default */

            /* USER CODE END P2PS_APP_Notification_default */
            break;
    }
    /* USER CODE BEGIN P2PS_APP_Notification_2 */

    /* USER CODE END P2PS_APP_Notification_2 */
    return;
}

void P2PS_APP_Init( TX_BYTE_POOL* p_byte_pool )
{
    /* USER CODE BEGIN P2PS_APP_Init */
    UINT status;
    VOID* tx_thread_memory;
    VOID* tx_thread_stack_memory;
    VOID* queue_memory;
    VOID* queue_stack_memory;

    /* Allocate memory for queue control block */
    status = tx_byte_allocate( p_byte_pool, &queue_memory, sizeof( TX_QUEUE ), TX_NO_WAIT );
    if ( status != TX_SUCCESS )
    {
        Error_Handler();
    }

    notification_queue = (TX_QUEUE*)queue_memory;

    // Allocate memory for queue messages (8-byte messages, not data payload)
    status = tx_byte_allocate( p_byte_pool,
                               &queue_stack_memory,
                               QUEUE_SIZE * MESSAGE_SIZE_ULONG * sizeof( ULONG ),
                               TX_NO_WAIT );

    /* Create message queue */
    status = tx_queue_create( (TX_QUEUE*)queue_memory,
                              "Notification Queue",
                              MESSAGE_SIZE_ULONG,
                              queue_stack_memory,
                              QUEUE_SIZE * MESSAGE_SIZE_ULONG * sizeof(ULONG));

    if ( status != TX_SUCCESS )
    {
        Error_Handler();
    }

    /* Allocate memory for BLE TX thread control block */
    status = tx_byte_allocate( p_byte_pool, &tx_thread_memory, sizeof( TX_THREAD ), TX_NO_WAIT );
    if ( status != TX_SUCCESS )
    {
        Error_Handler();
    }

    thread_SendNotificationProcess = (TX_THREAD*)tx_thread_memory;

    /* Allocate memory for BLE TX thread stack */
    status = tx_byte_allocate( p_byte_pool, &tx_thread_stack_memory, 1024, TX_NO_WAIT );
    if ( status != TX_SUCCESS )
    {
        Error_Handler();
    }

    /* Create BLE transmission thread */
    status = tx_thread_create( (TX_THREAD*)tx_thread_memory,
                               "thread_SendNotificationProcess",
                               thread_SendNotification_entry,
                               0,
                               tx_thread_stack_memory,
                               1024,
                               3,
                               3,
                               TX_NO_TIME_SLICE,
                               TX_AUTO_START );

    if ( status != TX_SUCCESS )
    {
        Error_Handler();
    }

    msg_byte_pool = (TX_BYTE_POOL*)p_byte_pool;
    /**
     * Initialize LedButton Service
     */
    P2P_Server_App_Context.Notification_Status = 0;
    P2PS_APP_LED_BUTTON_context_Init();
    /* USER CODE END P2PS_APP_Init */
    return;
}

/* USER CODE BEGIN FD */
void P2PS_APP_LED_BUTTON_context_Init( void )
{
    BSP_LED_Off( LED_BLUE );

#if ( P2P_SERVER1 != 0 )
    P2P_Server_App_Context.LedControl.Device_Led_Selection       = 0x01; /* Device1 */
    P2P_Server_App_Context.LedControl.Led1                       = 0x00; /* led OFF */
    P2P_Server_App_Context.ButtonControl.Device_Button_Selection = 0x01; /* Device1 */
    P2P_Server_App_Context.ButtonControl.ButtonStatus            = 0x00;
#endif
#if ( P2P_SERVER2 != 0 )
    P2P_Server_App_Context.LedControl.Device_Led_Selection       = 0x02; /* Device2 */
    P2P_Server_App_Context.LedControl.Led1                       = 0x00; /* led OFF */
    P2P_Server_App_Context.ButtonControl.Device_Button_Selection = 0x02; /* Device2 */
    P2P_Server_App_Context.ButtonControl.ButtonStatus            = 0x00;
#endif
#if ( P2P_SERVER3 != 0 )
    P2P_Server_App_Context.LedControl.Device_Led_Selection       = 0x03; /* Device3 */
    P2P_Server_App_Context.LedControl.Led1                       = 0x00; /* led OFF */
    P2P_Server_App_Context.ButtonControl.Device_Button_Selection = 0x03; /* Device3 */
    P2P_Server_App_Context.ButtonControl.ButtonStatus            = 0x00;
#endif
#if ( P2P_SERVER4 != 0 )
    P2P_Server_App_Context.LedControl.Device_Led_Selection       = 0x04; /* Device4 */
    P2P_Server_App_Context.LedControl.Led1                       = 0x00; /* led OFF */
    P2P_Server_App_Context.ButtonControl.Device_Button_Selection = 0x04; /* Device4 */
    P2P_Server_App_Context.ButtonControl.ButtonStatus            = 0x00;
#endif
#if ( P2P_SERVER5 != 0 )
    P2P_Server_App_Context.LedControl.Device_Led_Selection       = 0x05; /* Device5 */
    P2P_Server_App_Context.LedControl.Led1                       = 0x00; /* led OFF */
    P2P_Server_App_Context.ButtonControl.Device_Button_Selection = 0x05; /* Device5 */
    P2P_Server_App_Context.ButtonControl.ButtonStatus            = 0x00;
#endif
#if ( P2P_SERVER6 != 0 )
    P2P_Server_App_Context.LedControl.Device_Led_Selection       = 0x06; /* device6 */
    P2P_Server_App_Context.LedControl.Led1                       = 0x00; /* led OFF */
    P2P_Server_App_Context.ButtonControl.Device_Button_Selection = 0x06; /* Device6 */
    P2P_Server_App_Context.ButtonControl.ButtonStatus            = 0x00;
#endif
}

void P2PS_APP_SW1_Button_Action( void )
{
    NOTIFICATION_MESSAGE_T message;

    message.message_type = BUTTON_NOTIFICATION;
    message.data_length  = 0;    // No data for button notifications

    // Send message to queue (non-blocking)
    if ( tx_queue_send( notification_queue, &message, TX_NO_WAIT ) != TX_SUCCESS )
    {
        // Handle queue full condition if needed
    }
}

void Trigger_Send_Notification( SERIAL_DATA_OUT_BUF_T* p_data )
{
    NOTIFICATION_MESSAGE_T message;
    uint8_t* allocated_data;

    // Allocate memory for the data
    UINT status = tx_byte_allocate( msg_byte_pool,
                                    (VOID**)&allocated_data,
                                    p_data->bufDataLen,
                                    TX_WAIT_FOREVER  );

    if ( status == TX_SUCCESS )
    {
        message.message_type = DATA_TX_NOTIFICATION;
        message.data_length  = p_data->bufDataLen;
        message.data_ptr     = allocated_data;

        // Copy the data to allocated memory
        memcpy( allocated_data, p_data->outDataBuf, p_data->bufDataLen );

        // Send the message (now only 8 bytes)
        tx_queue_send( notification_queue, &message, TX_NO_WAIT );
    }
}

/* P2PS_Send_Notification */
static void thread_SendNotification_entry( ULONG argument )
{
    UNUSED( argument );
    SERIAL_DATA_OUT_BUF_T local_data;
    NOTIFICATION_MESSAGE_T received_message;

    ConfigureMaxThroughput();

    for ( ;; )
    {
        // Wait for message from queue
        if ( tx_queue_receive( notification_queue, &received_message, TX_WAIT_FOREVER )
             == TX_SUCCESS )
        {
            if ( received_message.message_type == DATA_TX_NOTIFICATION )
            {
                memcpy( local_data.outDataBuf,
                        received_message.data_ptr,
                        received_message.data_length );
                local_data.bufDataLen = received_message.data_length;

                // Free the allocated memory
                tx_byte_release( received_message.data_ptr );

                P2PS_Send_Notification_Data( &local_data );
            }
            else if ( received_message.message_type == BUTTON_NOTIFICATION )
            {
                // Handle button notification
                P2PS_Send_Notification();
            }
        }
    }
}

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void P2PS_Send_Notification( void )
{
    if ( P2P_Server_App_Context.ButtonControl.ButtonStatus == 0x00 )
    {
        P2P_Server_App_Context.ButtonControl.ButtonStatus = 0x01;
    }
    else
    {
        P2P_Server_App_Context.ButtonControl.ButtonStatus = 0x00;
    }

    if ( P2P_Server_App_Context.Notification_Status )
    {
        APP_DBG_MSG( "-- P2P APPLICATION SERVER  : INFORM CLIENT BUTTON 1 PUSHED \n " );
        APP_DBG_MSG( " \n\r" );
        P2PS_STM_App_Update_Char( P2P_NOTIFY_CHAR_UUID,
                                  2,
                                  (uint8_t*)&P2P_Server_App_Context.ButtonControl );
    }
    else
    {
        APP_DBG_MSG(
            "-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n " );
    }

    return;
}

void P2PS_Send_Notification_Data( SERIAL_DATA_OUT_BUF_T* p_data )
{
    P2PS_STM_App_Update_Char( P2P_NOTIFY_CHAR_UUID,
                              (uint8_t)( p_data->bufDataLen ),
                              (uint8_t*)&p_data->outDataBuf );
    return;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
