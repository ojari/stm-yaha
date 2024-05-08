#include "FreeRTOS.h"
#include <string.h>
#include "task.h"
//#include "queue.h"
//#include "semphr.h"
#include "main.h"
#include "leds.h"

#define MAX_COMMAND_LENGTH 50
#define NUM_COMMANDS 3

typedef struct {
    char* command;
    void (*handler)(void);
} Command;

void command1_handler(void);
void command2_handler(void);
void command3_handler(void);

Command commands[NUM_COMMANDS] = {
    {"c1", command1_handler},
    {"c2", command2_handler},
    {"c3", command3_handler}
};

void SendChar(char c) {
    HAL_UART_Transmit(&huart3, (uint8_t*)&c, 1, HAL_MAX_DELAY);
}

void SendString(const char* str) {
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void UART_Task(void *pvParameters) {
    char receivedChar;
    char receivedCommand[MAX_COMMAND_LENGTH];
    int commandIndex = 0;

    SendString("\r\nTerminal ready\r\n");

    while(1) {
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, (uint8_t*)&receivedChar, 1, 1000);
        if (status != HAL_OK) {
            SendChar('!');
            continue;
        }

        if(receivedChar == '\n' || receivedChar == '\r') {
            receivedCommand[commandIndex] = '\0'; // null terminate the string
            for(int i = 0; i < NUM_COMMANDS; i++) {
                if(strcmp(receivedCommand, commands[i].command) == 0) {
                    commands[i].handler();
                    break;
                }
            }
            SendChar('~');
            commandIndex = 0; // reset the command index
        } else if(commandIndex < MAX_COMMAND_LENGTH - 1) {
            SendChar(receivedChar);
            receivedCommand[commandIndex++] = receivedChar;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void command1_handler(void) {
    SendString("Command 1 ok\r\n");
    currentPattern = 0;
}

void command2_handler(void) {
    SendString("Command 2 ok\r\n");
    currentPattern = 1;
}

void command3_handler(void) {
    SendString("Command 3 ok\r\n");
    currentPattern = 2;
}