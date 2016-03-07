/* 
 * File:   main.c
 * Author: Tonghua
 * Notes:	In Project property -> XC8 linker -> Memory model -> ROM range
 *				set to 0-7FF.
 *				Since this bootloader is 0x7FF in size.
 * Created on December 25, 2012, 9:58 AM
 * Modified on November 11, 2015. Eduardo
 *  Incoming data format:
 *
 * <STX><---><CHKSUM><ETX>
 *	   	/ \
 * _____/	\____________________________
 * /												  \
 * <COMMAND><ADDRL><ADDRH><ADDRU><DATA>...
 *
 * Definitions:
 *
 * STX - Start of packet indicator
 * COMMAND - Base command
 * ADDR - Address up to 24 bits
 * DATA - General data 16 bytes
 * CHKSUM - The 8-bit two's compliment sum of  DATA
 * ETX - End of packet indicator
 *
 * ******************************************
 * Commands
 * RD_VER 0x00 Read Version Information
 * 
 * RD_MEM 0x01 Read Program Memory
 *		[<0x01><ADDRL><ADDRH><ADDRU>]
 *
 * WR_MEM 0x02 Write Program Memory
 *		WriteBlock = 32 bytes
 *		[<0x02><ADDRL><ADDRH><ADDRU>...LEN bytes of Data...]
 *
 * ER_MEM 0x03 Erase Program Memory
 *		FLASH_ERASE_BLOCK = 64 bytes (0x40)
 *		[<0x03><ADDRL><ADDRH><ADDRU>]
 */

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
    #include <stdbool.h>
#endif

#include "bootloader.h"

unsigned char m_buffer[BUFFER_MAX] = {0};
unsigned char m_buffer_Index = 0;

DWORD_VAL m_Flash_addr;

const uint8_t BOOT_VERSION[] __at(_BOOT_VERSION_LOCATION) = _BOOT_VERSION_VALUE;
const uint16_t COMM_ROM_INIT __at(_COMM_ROM_INIT_LOCATION) = _COMM_ROM_INIT_VALUE;//_comm_helper determinar após compilação
const uint16_t COMM_ROM_SIZE __at(_COMM_ROM_SIZE_LOCATION) = _COMM_ROM_SIZE_VALUE;//_end_of_comm_helper - _comm_helper determinar após compilação
const uint8_t COMM_ROM_HASH[] __at(_COMM_ROM_HASH_LOCATION) = _COMM_ROM_HASH_VALUE;//calcular após compilação
const uint8_t BOOT_ROM_HASH[] __at(_BOOT_ROM_HASH_LOCATION) = _BOOT_ROM_HASH_VALUE;//calcular após compilação

/*
 * 
 */
void main(void) {
    /**copia do byte recebido pela porta serial*/
	unsigned char ReceivedByte = 0;
    /**contador de tempo em milisegundos*/
    uint32_t contador = 0;

	init_USART();
    
    /*aguardar tempo predeterminado antes de abandonar o bootloader*/
    for(;;){
        if(PIR1bits.RCIF == 1)
            break;
        /*ir para APP_START, se não houver comunicação*/
        if(contador >= TIMEOUT_RX){
            asm("goto " ___mkstr(APP_START));
            comm_helper(0x06); //escape em caso de falha
        }
        __delay_ms(1);
        CLRWDT();
        contador++;
    }

    while (1) {
        if (PIR1bits.RCIF == 1) {
            ReceivedByte = RCREG;
            m_buffer[m_buffer_Index++] = ReceivedByte;
            //recarregar temporizador (novo tempo de espera) a cada byte recebido
            contador = 0;
            if (m_buffer_Index >= BUFFER_MAX) {
                if (m_buffer[0] == STX && ReceivedByte == ETX) { //get complete cmd
                    switch (m_buffer[CMD_INDEX]) {
                        case RD_VER:
                            ReadVer();
                            break;
                        case RD_MEM:
                            ReadMem();
                            break;
                        case WR_MEM:
                            WriteMem();
                            break;
                        case ER_MEM:
                            EraseMem();
                            break;
                        case RUN_APP:
                            asm("goto " ___mkstr(APP_START));
                        default: break;
                    }
                } else { //Send data error back
                    TXREG = '?';
                    while (TXSTAbits.TRMT == 0); //wait empty
                }
                m_buffer_Index = 0;
            }
        }
        /*ir para HELPER, se não houver comunicação*/
        if(contador >= TIMEOUT_RX){
            comm_helper(0x06); //escape em caso de falha
        }
        __delay_ms(1);
        CLRWDT();
        contador++;
    }
}

void init_USART(void) {
	CloseUSART();
	TXSTA = 0;
	RCSTA = 0;
	BAUDCON = 0;

	TXSTAbits.TXEN = 1;
	TXSTAbits.SYNC = 0;
	TXSTAbits.BRGH = 1;

	RCSTAbits.SPEN = 1;
	RCSTAbits.CREN = 1; //enable receiver

	BAUDCONbits.BRG16 = 1;
    SPBRG = USART_BRG;
    SPBRGH = USART_BRG >> 8;

	TRISCbits.RC6 = 0; //output
	TRISCbits.RC7 = 1; //input
}

void RomOperation(char isWrite) {
	EECON1bits.EEPGD = 1; // point to Flash program memory
	EECON1bits.CFGS = 0; // access Flash program memory
	EECON1bits.WREN = 1; // enable write to memory
	if (isWrite == 0) {
		EECON1bits.FREE = 1;
	}

	EECON2 = 0x55; // write 55h
	EECON2 = 0xAA; //write 0AAh
	EECON1bits.WR = 1; //start program (CPU stall)
	EECON1bits.WREN = 0; //disable write to memory
}

void getAddress() {
    /*TODO verificar se o endereço está fora da faixa protegida do bootloader*/
	m_Flash_addr.byte.LB = m_buffer[ADDRL_INDEX];
	m_Flash_addr.byte.HB = m_buffer[ADDRH_INDEX];
	m_Flash_addr.byte.UB = m_buffer[ADDRU_INDEX];
}

void sendResponse() {
	for (UINT8 i = 0; i < BUFFER_MAX; i++) {
		TXREG = m_buffer[i];
		while (TXSTAbits.TRMT == 0); //wait empty
	}
}

void ReadVer(){
    /* preencher m_buffer com a versao atual*/
    m_buffer[2] = BOOT_VERSION[0];
    m_buffer[3] = BOOT_VERSION[1];
    sendResponse();
}

void ReadMem() {
    /*TODO remover a função de leitura, por segurança*/
	getAddress(); //decode address and store in Flash_addr
	for (UINT32 i = 0; i < DATA_MAX; i++) {
		LoadFlashAddr(m_Flash_addr.Val + i);
		TableRead(m_buffer[i + ADDRU_INDEX + 1]); //buffer already stored address.
	}
	sendResponse();
}

void WriteMem() {
	getAddress(); //decode address and store in Flash_addr
	GIE = 0; //disable interrupts
	LoadFlashAddr(m_Flash_addr.Val); //Set rom address
	for (UINT8 i = 0; i < DATA_MAX; i++) { // Fill Flash block Write Buffer
		TABLAT = m_buffer[ADDRU_INDEX + i + 1]; // Load the holding registers
		asm ( "TBLWTPOSTINC"); // Store it to holding registers
	}
	LoadFlashAddr(m_Flash_addr.Val); ////Set back to address, for writting.
	RomOperation(1);
	GIE = 1; //re-enable interrupts
	sendResponse();
}

void EraseMem() {
    /*TODO corrigir o contador de 64bytes para que toda a memoria seja alcançável*/
	GIE = 0; //disable interrupts
	for (unsigned int i = 0; i < COUNTS_TO_ERASE_APP; i++) {
		LoadFlashAddr((UINT32) (APP_START) + i * FLASH_ERASE_BLOCK);
		RomOperation(0);
	}
	GIE = 1; //re-enable interrupts
	sendResponse();
}

void comm_helper(uint8_t erro_detectado) {
    static uint8_t falha;
    uint8_t RecivedByte = 0;
    union CRC_table{
        struct {//crc em bytes
            uint8_t high;
            uint8_t low;
        }crc8;
        
        struct {//tabela em 16 bits
            uint16_t crc16;
        }crc16;
    };

    union CRC_table crc[] = {0x9884, 0x1881, 0x188B, 0x988E, 0x189F,\
                             0x989A, 0x9890, 0x1895, 0x18B7, 0x98B2,\
                             0x98B8, 0x18BD, 0x98AC, 0x18A9, 0x18A3,\
                             0x98A6};
   
    falha = erro_detectado & 0x0F;
    /*inicia a interface serial*/
    CloseUSART();
	TXSTA = 0;
	RCSTA = 0;
	BAUDCON = 0;

	TXSTAbits.TXEN = 1;
	TXSTAbits.SYNC = 0;
	TXSTAbits.BRGH = 1;

	RCSTAbits.SPEN = 1;
	RCSTAbits.CREN = 1; //enable receiver

	BAUDCONbits.BRG16 = 1;
    SPBRG = USART_BRG;
    SPBRGH = USART_BRG >> 8;

	TRISCbits.RC6 = 0; //output
	TRISCbits.RC7 = 1; //input

    /*mantenha-se alerta*/
    while (true) {
        if (PIR1bits.RCIF == 1) {
            RecivedByte = RCREG;
            /*aguarda mensagem recebida pelo PC*/
            if (RecivedByte == '\xA4' ||
                    RecivedByte == '\xA2') {
                /*transmite a mensagem completa, com identificação do erro*/
                TXREG = '\xA5'; /*cabeçalho*/
                while (TXSTAbits.TRMT == 0); //wait empty
                TXREG = '\x00'; /*comando*/
                while (TXSTAbits.TRMT == 0); //wait empty
                TXREG = '\x00'; /*comando*/
                while (TXSTAbits.TRMT == 0); //wait empty
                TXREG = '\x02'; /*numero de bytes*/
                while (TXSTAbits.TRMT == 0); //wait empty
                TXREG = '\x00'; /*comando*/
                while (TXSTAbits.TRMT == 0); //wait empty
                TXREG = falha; /*codigo do erro*/
                while (TXSTAbits.TRMT == 0); //wait empty
                /*monta a mensagem de erro (CRC)*/
                TXREG = crc[falha].crc8.high; /*CRC*/
                while (TXSTAbits.TRMT == 0); //wait empty
                TXREG = crc[falha].crc8.low; /*CRC*/
                while (TXSTAbits.TRMT == 0); //wait empty
            }
        }
    }
}

