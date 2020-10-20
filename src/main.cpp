/*
 * Copyright (c) 2014-2018 Cesanta Software Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos.h"
#include "mgos_app.h"
#include "mgos_gpio.h"
#include "mgos_uart.h"
#include "mgos_neopixel.h"
#include "mgos_timers.h"
#include "mgos_system.h"
#include "mgos_arduino_lora.h"

/**** Delta Things IoT Cloud standard topics ****/
#define  TELEMETRY_TOPIC "v1/devices/me/telemetry"

#define LORA_BUF_SIZE 30
#define UART_BUF_SIZE 20
#define START_BYTE 0x02
#define END_BYTE 0x03
#define LORA_BUF_SIZE 30
#define ACK_BUF_SIZE 6
#define NODE_ID_INDEX 9
#define MODBUS_RX_BYTES_LENGTH 19
#define REQUIRE_BYTES_LENGTH 10
#define LORA_DATA_LENGTH 23


LoRaClass* lora;

struct mgos_neopixel *s_strip = NULL;

typedef enum {
  _101 = 0, _102,_103,_104,_105,_106,_107,_108,_109,_110
} NODE_IDS;

/**********************************************************************!
 * @fn          Check_Device
 * @brief       To check the device STATIC or DYNAMIC
 * @param[in]   buf
 * @return      device status 0-> STATIC 1-> DYNAMIC
 **********************************************************************/
uint8_t Check_Device(uint8_t *buf)
{
  if(buf[0] & (1 << 7))
  {
    LOG(LL_INFO, ("DYNAMIC DEVICE"));

    return (0x01);
  }

  else
  {
    LOG(LL_INFO, ("STATIC DEVICE"));  
    return(0x00);
  }
  
}
/**********************************************************************!
 * @fn          Rgb_Green
 * @brief       LED colour turns to RED 
 * @param[in]   NODE_IDS
 * @return      void
 **********************************************************************/
void Rgb_Red(NODE_IDS i)
{
  mgos_neopixel_set(s_strip,i,255,0,0);

  mgos_neopixel_show(s_strip);

  mgos_gpio_write(mgos_sys_config_get_modbus_buzzer(),1);

}
void Rgb_Green(int i)
{
 mgos_neopixel_set(s_strip,i,0,255,0);

 mgos_neopixel_show(s_strip);
}
/**********************************************************************!
 * @fn          All_Rgb_Green
 * @brief       Intially All Leds In Green
 * @param[in]   void
 * @return      void
 **********************************************************************/
void All_Rgb_Green()
{
 for(int i = 0; i <mgos_sys_config_get_neopixel_leds_num() ;i++)
  {
    if(i != 5 && i!= 6 && i != 7 && i != 8)
    {
     Rgb_Green(i);
     mgos_msleep(500);
    }
  }
}
/**********************************************************************!
 * @fn          Get_Node_ID
 * @brief       To get the device node number
 * @param[in]   buf
 * @return      node id
 **********************************************************************/
 uint8_t Get_Node_ID(uint8_t *buf)
{
  LOG(LL_INFO, ("NODE ID: %d",buf[NODE_ID_INDEX]));  

  return(buf[NODE_ID_INDEX]);
}


/**********************************************************************!
 * @fn          ModRTU_CRC
 * @brief       To caluclate crc
 * @param[in]   buf,len
 * @return      caluclated crc
 **********************************************************************/

uint16_t ModRTU_CRC(uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  
  for (uint8_t pos = 0; pos < len; pos++) 
	{
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
  
    for (uint8_t i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc; 

}

/**********************************************************************!
 * @fn          Send_Ack_To_Node
 * @brief       To send acknowledgement to the node
 * @param[in]   buffer
 * @return      void
 **********************************************************************/

void Send_Ack_To_Node(uint8_t* buffer)
{
  uint8_t Ack_txcmd_str[ACK_BUF_SIZE] ={'\0'};
  uint16_t MODBUS_CRC = 0;
  uint8_t MODBUS_CRC_MSB = 0;
  uint8_t MODBUS_CRC_LSB = 0;
  Ack_txcmd_str[0] = START_BYTE;                 //START BYTE
	Ack_txcmd_str[1] = Get_Node_ID(buffer);         // Received NODE ID
	MODBUS_CRC = ModRTU_CRC(&Ack_txcmd_str[1],1);  // CRC FOR ACK
	MODBUS_CRC_MSB = (MODBUS_CRC & 0xFF00) >> 8 ;  // CRC MSB
	MODBUS_CRC_LSB = (MODBUS_CRC & 0x00FF);        // CRC LSB
  Ack_txcmd_str[2] = MODBUS_CRC_MSB;
  Ack_txcmd_str[3] = MODBUS_CRC_LSB;
  Ack_txcmd_str[4] = END_BYTE;                    // END BYTE

 LOG(LL_INFO, ("ACKNOWLEDGEMENT PACKET : %s",Ack_txcmd_str));

  mgos_send_lora(lora,(char *)Ack_txcmd_str);
}
/**********************************************************************!
 * @fn          Lora_Data_Chk_CRC
 * @brief       To check the CRC in lora recieved data
 * @param[in]   r_buf
 * @return      0 or 1
 **********************************************************************/

uint8_t Check_CRC(uint8_t *r_buf,uint8_t r_buf_len)   //Lora_Data_Chk_CRC
{
 	uint16_t rx_crc = (r_buf[r_buf_len - 2]) << 8;

	rx_crc |= r_buf[r_buf_len - 1];

  LOG(LL_INFO, ("Rxd CRC: %X",rx_crc));

	uint16_t cal_crc = ModRTU_CRC(&r_buf[0],(r_buf_len - 2));

  LOG(LL_INFO, ("Calc CRC: %X",cal_crc));
  if(cal_crc == rx_crc)
	{
    if(cal_crc != 0)
    {
		return 1;
    }
	}
	return 0;
}
 /**********************************************************************!
 * @fn          Check_Button_State
 * @brief       To Check The Button State [Pressed (or) Not Pressed]
 * @param[in]   buffer
 * @return      void
 **********************************************************************/

void Check_Button_State(uint8_t *buffer)
{
   if(buffer[0] == 0x01 || ((buffer[0] & 0xC0)>>6) == 0x01)
    {
      LOG(LL_INFO, ("[INFO]: BUTTON PRESSED"));

      switch(buffer[1])                 // from 101 -> DYNAMIC DEVICES and from 151 -> STATIC DEVICES
       {
         case 101 : Rgb_Red(_110); 
         case 102 : Rgb_Red(_110); 
         case 103 : Rgb_Red(_110); 
         case 104 : Rgb_Red(_110); 
         case 105 : Rgb_Red(_110);
         case 106 : Rgb_Red(_110); 
         case 107 : Rgb_Red(_110);
         case 108 : Rgb_Red(_110); 
         case 109 : Rgb_Red(_110); 
         case 110 : Rgb_Red(_110); LOG(LL_INFO, ("[INFO]: NODE ID NUMBER: %d",buffer[1]));break;
         case 151 : Rgb_Red(_101); LOG(LL_INFO, ("[INFO]: NODE ID NUMBER: %d",buffer[1]));break;
         case 152 : Rgb_Red(_102); LOG(LL_INFO, ("[INFO]: NODE ID NUMBER: %d",buffer[1]));break;                                         
         case 153 : Rgb_Red(_103); LOG(LL_INFO, ("[INFO]: NODE ID NUMBER: %d",buffer[1]));break;
         case 154 : Rgb_Red(_104); LOG(LL_INFO, ("[INFO]: NODE ID NUMBER: %d",buffer[1]));break;
         case 155 : Rgb_Red(_105); LOG(LL_INFO, ("[INFO]: NODE ID NUMBER: %d",buffer[1]));break;
         default : All_Rgb_Green();break;
       }
    }
    else
    {
       LOG(LL_INFO, ("[INFO]: BUTTON NOT PRESSED"));
    }
}

 /**********************************************************************!
 * @fn          Validate_Rxed_Data
 * @brief       To validate lora received data
 * @param[in]   recvbuf
 * @return      void
 **********************************************************************/

 void Validate_Rxed_Data(uint8_t *rx_buf,uint8_t rx_buf_len) 
 {

     uint8_t cpy_recvbuf[LORA_BUF_SIZE]={'\0'};
 
     uint8_t btn_state_buf[3]={'\0'}; 

   if((rx_buf[0] == START_BYTE) && (rx_buf[rx_buf_len - 1] == END_BYTE)) // Checking start and end byte
   { 
       LOG(LL_INFO, ("START AND END BYTE MATCHED"));

    memset(cpy_recvbuf,0,sizeof(cpy_recvbuf));

    memcpy(cpy_recvbuf,&rx_buf[1],rx_buf_len-2); // lora data without start and end byte in cpy_recvbuf
    

    if(Check_CRC(cpy_recvbuf,rx_buf_len-2))
    {
          LOG(LL_INFO, ("CRC IS OK"));  
    
     Check_Device(cpy_recvbuf);

     Send_Ack_To_Node(cpy_recvbuf);
       
    btn_state_buf[0] = cpy_recvbuf[10]; //Button state Byte
    btn_state_buf[1] = cpy_recvbuf[9];  //Node ID Byte

     Check_Button_State(btn_state_buf);
  
     mgos_lora_receive_mode(lora);
    }
   
   }
 }

  /**********************************************************************!
 * @fn          uart_dispatcher
 * @brief       To Receive the data from the Modbus
 * @param[in]   uart_nmbr,arg
 * @return      void
 **********************************************************************/

 static void uart_dispatcher(int uart_nmbr, void *arg) 
{
  uint8_t rx_buf[UART_BUF_SIZE] = {0};  

  uint8_t btn_state_buf[3]={'\0'}; 
      
  if (mgos_uart_read_avail(uart_nmbr))
  {
     mgos_uart_read(uart_nmbr, &rx_buf[0], sizeof(rx_buf));
     LOG(LL_INFO, ("Serial Data Received: %s", rx_buf));

      LOG(LL_INFO, ("Serial Data : %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x", rx_buf[0], rx_buf[1], rx_buf[2]
      , rx_buf[3], rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7], rx_buf[8], rx_buf[9], rx_buf[10], rx_buf[11], rx_buf[12], rx_buf[13], rx_buf[14], rx_buf[15], rx_buf[16], rx_buf[17], rx_buf[18]));

  if(Check_CRC(rx_buf,MODBUS_RX_BYTES_LENGTH))
    {
      LOG(LL_INFO, ("[INFO]: MODBUS CRC IS OK"));

      btn_state_buf[0] = rx_buf[6]; //Button state Byte
      LOG(LL_INFO, ("[INFO]: BUTTON STATE BYTE: %x",rx_buf[6]));
      btn_state_buf[1] = rx_buf[4]; //Node ID Byte
      LOG(LL_INFO, ("[INFO]: NODE ID BYTE: %x",rx_buf[4]));
      Check_Button_State(btn_state_buf);
    }

  (void)arg;
}
}

 /**********************************************************************!
 * @fn          lora_data_receive
 * @brief      lora Receiver call back
 * @param[in]   packet_size
 * @return      void
 **********************************************************************/

void lora_data_receive(int packet_size)
{
      uint8_t recvbuf[LORA_BUF_SIZE] = {0};

      
        /*Receiving timer code */
   LOG(LL_INFO, ("\n[INFO]\t: LoRa Packet received"));

   LOG(LL_INFO, ("Packet size - %d",packet_size));

   if(packet_size == LORA_DATA_LENGTH)
   {
    for (int i = 0; i < packet_size; i++)
    {
      recvbuf[i] = (mgos_lora_read(lora));
    }
  
  LOG(LL_INFO, ("\n[INFO]\t: LoRa data : %s",recvbuf));
   
   Validate_Rxed_Data(recvbuf,packet_size);
   }
}

/**********************************************************************!
 * @fn          mgos_app_init_result
 * @brief       Initialization function 
 * @param[in]   void
 * @return      bool
 **********************************************************************/
enum mgos_app_init_result mgos_app_init(void) {

s_strip = mgos_neopixel_create(mgos_sys_config_get_neopixel_data_pin(), mgos_sys_config_get_neopixel_leds_num(), MGOS_NEOPIXEL_ORDER_GRB);

 
/********************LORA CONFIGURATION*******************************/  
if(mgos_sys_config_get_lora_enable())
{
lora = mgos_lora_create();

mgos_lora_setpins(lora,mgos_sys_config_get_spi_cs0_gpio(), mgos_sys_config_get_lora_reset(),mgos_sys_config_get_lora_dio0());

long int frequency = mgos_sys_config_get_lora_frequency();

 if(!mgos_lora_begin(lora,frequency))
  {
    LOG(LL_INFO,("[INFO]:LORA NOT INTIALIZED"));
  }
  else
  {
     LOG(LL_INFO,("[INFO]:LORA INTIALIZED"));
  }

  mgos_lora_receive_callback(lora,lora_data_receive);
  mgos_lora_receive_mode(lora);

}

  /********************UART CONFIGURATION*******************************/ 
if(mgos_sys_config_get_modbus_enable())
{
struct mgos_uart_config ucfg;
mgos_uart_config_set_defaults(mgos_sys_config_get_serial_num(), &ucfg);

  ucfg.baud_rate = mgos_sys_config_get_serial_baud_rate();
  ucfg.rx_buf_size = mgos_sys_config_get_serial_rx_buf_size();
  ucfg.tx_buf_size =  mgos_sys_config_get_serial_tx_buf_size();

if (!mgos_uart_configure(mgos_sys_config_get_serial_num(), &ucfg)) {
  LOG(LL_ERROR, ("Failed to configure UART%d", mgos_sys_config_get_serial_num()));
}
 mgos_uart_set_dispatcher(mgos_sys_config_get_serial_num(), uart_dispatcher, NULL /* arg */);

  /* Controls whether UART receiver is enabled. */
 mgos_uart_set_rx_enabled(mgos_sys_config_get_serial_num(), true);

  if((mgos_uart_is_rx_enabled( mgos_sys_config_get_serial_num())))
  {
	LOG(LL_ERROR, ("UART%d Receiver Enabled", mgos_sys_config_get_serial_num()));
  }
} 
  mgos_gpio_set_mode(mgos_sys_config_get_modbus_de_re(),MGOS_GPIO_MODE_OUTPUT);

  mgos_gpio_write(mgos_sys_config_get_modbus_de_re(),0);

  mgos_gpio_set_mode(mgos_sys_config_get_modbus_buzzer(),MGOS_GPIO_MODE_OUTPUT);

   mgos_gpio_write(mgos_sys_config_get_modbus_buzzer(),0);

  All_Rgb_Green();

return MGOS_APP_INIT_SUCCESS;
}
