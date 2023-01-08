/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* Bank's Server : Damien GABRIEL CALIXTE 
   Client's Computer : RONK Antoine 
   December 9th
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"


static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

//Declaration d une structure pour le message
typedef struct 
{
uint8_t Type;
uint8_t  Action;
uint8_t RequestID;
uint8_t PortID;
uint32_t Amount;
} message_t ;


//Declaration des queues
static QueueHandle_t rx_queue = NULL;
static QueueHandle_t tx_queue = NULL;

//Declaration du HEADER pour verifier le bon etat de la communication
const uint8_t HEADER[4] = {0xCA, 0xFE, 0xCA, 0xFE};




void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  
    //Initialisation des queues
     rx_queue = xQueueCreate(10, sizeof(uint32_t));
     tx_queue=xQueueCreate(10, sizeof(uint32_t));
}





int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    ESP_LOGI(logName, "Wrote %d bytes\n\n", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    ////Allocation dynamique de la memoire pour les buffers de messages
     uint32_t *transmit=(uint32_t*) malloc(sizeof(message_t)); 
      uint8_t *Header=(uint8_t*) malloc(4*sizeof(uint8_t)); 
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
    	//Stockage du premier message sur la queue dans le buffer avant envoi
    	 xQueueReceive(tx_queue,transmit,1000 / portTICK_PERIOD_MS);
    	 
    	 //Envoi du header
    	const int nb_transmitH=uart_write_bytes(UART_NUM_0, Header, sizeof(Header));
    	
    	vTaskDelay(2000 / portTICK_PERIOD_MS);
    	
    	//Envoi du message
        const int nb_transmit=uart_write_bytes(UART_NUM_0, transmit, sizeof(transmit));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    
    //Liberation des buffers
    free(transmit);
    free(Header);
}

static void rx_task(void *arg)
{	
    int i;
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    //Allocation dynamique de la memoire pour les buffers de messages
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    uint8_t *R_Head=(uint8_t*) malloc(4*sizeof(uint8_t)); 
    uint32_t *message=(uint32_t*) malloc(4*sizeof(uint32_t)); 
    
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'\n\n", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
        
        //Reception du header
        const int nb_bitH=uart_read_bytes(UART_NUM_0, R_Head, 4*sizeof(uint8_t), 1000 / portTICK_PERIOD_MS);
        
        if(nb_bitH==4)
        {
        	for(i=0;i<nb_bitH;i++)
        	{        	
        		if(R_Head[i]!=HEADER[i])
        			{
        			printf("Wrong Header ! Communication Failed !");
        			break;	
        			}
        	        	        		
        	}
        	       
        }
        
        //Reception du message et stockage dans la queue        
        const int nb_bitM=uart_read_bytes(UART_NUM_0, message, sizeof(message_t), 1000 / portTICK_PERIOD_MS);
        xQueueSend(rx_queue,message,1000 / portTICK_PERIOD_MS); 
      
    }
    // Liberation des buffers
    free(data);
    free(R_Head);
    free(message);
}


static void bank_task(void *arg)
{
    int balance; //Solde du compte
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world\n\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}


void app_main(void)
{
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(bank_task, "bank_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
