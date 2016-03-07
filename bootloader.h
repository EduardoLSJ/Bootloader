/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef BOOTLOADER_H
#define	BOOTLOADER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>



//  declarations
/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        40000000L
#define FCY             SYS_FREQ/4
#define _XTAL_FREQ      40000000L

#define USE_OR_MASKS
#define APP_START 0x810 //app memory origin
#define ROM_MAX 0x8000 //max rom space
//#define FLASH_ERASE_BLOCK 0x40 // 64bits

//make sure integer, now is (0x8000-0x800)/0x40=480 times repeat.
/*TODO corrigir a rotina para gravação de blocos 64bytes*/
#define COUNTS_TO_ERASE_APP (ROM_MAX-APP_START)/FLASH_ERASE_BLOCK

///**VERSÃO atual do bootloader*/
//#define VERSION 0x0100

#define STX 0x0F
//Commands
#define RD_VER 0x00
#define RD_MEM 0x01
#define WR_MEM 0x02
#define ER_MEM 0x03
#define RUN_APP 0xFF

#define CMD_INDEX 1

#define ADDRL_INDEX 2
#define ADDRH_INDEX 3
#define ADDRU_INDEX 4

#define ETX 0x04

#define BUFFER_MAX 23
#define DATA_MAX 16
#define TIMEOUT_RX 5000UL //em ms
#define BAUD_RATE_UART1	9600L
#define USART_BRG ((long) SYS_FREQ / (4 * (long)BAUD_RATE_UART1)) - 1
    
asm("PSECT intcode");
asm("\tgoto\t" ___mkstr(APP_START + 0x8));
asm("PSECT intcodelo");
asm("\tgoto\t" ___mkstr(APP_START + 0x18));

void init_USART(void);
void RomOperation(char isWrite);
void getAddress();
void sendResponse();
void ReadVer();
void ReadMem();
void WriteMem();
void EraseMem();
void comm_helper(uint8_t erro_detectado);

/* Salvar na FLASH:
 * 
 * RESET = 0
 * INT_VECTOR = 0X8
 * LOW_INT_VECTOR = 0X18
 * COMM_ROM_INIT_LOCATION = 0x7DC
 * COMM_ROM_SIZE_LOCATION = 0x7DE
 * COMM_ROM_HASH_LOCATION = 0x7E0
 * BOOT_ROM_HASH_LOCATION = 0x7FD
 * 
 * COMM_ROM_INIT_VALUE = //_comm_helper determinar após compilação
 * COMM_ROM_SIZE_VALUE = //_end_of_comm_helper - _comm_helper determinar após compilação
 * COMM_ROM_HASH_VALUE = //calcular após compilação
 * BOOT_ROM_HASH_VALUE = //calcular após compilação
 *  
 * COMM_ROM_INIT
 * COMM_ROM_SIZE
 * COMM_ROM_HASH
 * BOOT_ROM_HASH
 */
void main(void);
/**
 * Auxiliar na transmissão de código de falhas
 * 
 * Configura a USART em software(autobaudrate)
 * recebe o valor do erro detectado pelo programa principal
 * recebe qualquer mensagem do PC
 * a cada mensagem recebida, transmite a mensagem de erro codificada
 * 
 * @param erro_detectado - código do erro detectado pelo programa principal
 */
void comm_helper(uint8_t erro_detectado);

/* Salvar na FLASH:
 * 
 * RESET = 0
 * INT_VECTOR = 0X8
 * LOW_INT_VECTOR = 0X18
 * COMM_ROM_INIT_LOCATION = 0x7DC
 * COMM_ROM_SIZE_LOCATION = 0x7DE
 * COMM_ROM_HASH_LOCATION = 0x7E0
 * BOOT_ROM_HASH_LOCATION = 0x7FD
 * 
 * COMM_ROM_INIT_VALUE = //_comm_helper determinar após compilação
 * COMM_ROM_SIZE_VALUE = //_end_of_comm_helper - _comm_helper determinar após compilação
 * COMM_ROM_HASH_VALUE = //calcular após compilação
 * BOOT_ROM_HASH_VALUE = //calcular após compilação
 *  
 * COMM_ROM_INIT
 * COMM_ROM_SIZE
 * COMM_ROM_HASH
 * BOOT_ROM_HASH
 */
/**Endereço com a Versão de software do BOOTLOADER*/
#define _BOOT_VERSION_LOCATION 0x07DA
#define _COMM_ROM_INIT_LOCATION 0x07DC//0x07C0
#define _COMM_ROM_SIZE_LOCATION 0x07DE//0x07D0
#define _COMM_ROM_HASH_LOCATION 0x07E0
#define _BOOT_ROM_HASH_LOCATION 0x07F0

/*valores que serão informados à aplicação. determinar conteudo após compilação*/
/**Versão atual do BOOTLOADER*/
#define _BOOT_VERSION_VALUE {0x03, 0xE8} //1.00
/**Endereço de inicio da funcao _comm_helper.
 *  Usar MACRO ou determinar após compilaçã, no arquivo .map*/
#define _COMM_ROM_INIT_VALUE (uint16_t) 0x015A
/**Tamanho da função _comm_helper.
 *  Usar MACRO ou determinar após compilação, no arquivo .map*/
#define _COMM_ROM_SIZE_VALUE (uint16_t) 0x00E0
/**Hash MD5 de toda a memória FLASH 0x0000 a 0x07F0*/
#define _COMM_ROM_HASH_VALUE {0x2D,0x54,0x5D,0x50,0x94,0xE7,0xD7,0x60,\
        0xB4,0x39,0x51,0xEB,0x3E,0x94,0xE9,0xF0} //calcular após compilação
/**Hash MD5 do intervalo da função _comm_helper.*/
#define _BOOT_ROM_HASH_VALUE {0x44,0x5F,0xDD,0x8C,0x19,0x7C,0x13,0x92,\
        0x0A,0x4C,0x95,0x0C,0xB2,0xA8,0x0E,0x21} //calcular após compilação
#endif	/* BOOTLOADER_H */

