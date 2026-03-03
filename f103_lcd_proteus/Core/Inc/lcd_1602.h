#include "main.h"    // Περιέχει τα απαραίτητα αρχεία του STM32 (π.χ. stm32f4xx_hal.h)
#include <stdint.h>

// --- Ρυθμίσεις Pin & Θυρών (Προσάρμοσέ τα σύμφωνα με το CubeMX) ---
#define LCD_DB4_PORT GPIOB
#define LCD_DB4_PIN  GPIO_PIN_5
#define LCD_DB5_PORT GPIOB
#define LCD_DB5_PIN  GPIO_PIN_6
#define LCD_DB6_PORT GPIOB
#define LCD_DB6_PIN  GPIO_PIN_7
#define LCD_DB7_PORT GPIOB
#define LCD_DB7_PIN  GPIO_PIN_8

#define LCD_E_PORT   GPIOB
#define LCD_E_PIN    GPIO_PIN_4
#define LCD_RS_PORT  GPIOB
#define LCD_RS_PIN   GPIO_PIN_3

#define lcd_type 2        // 0=5x7, 1=5x10, 2=2 lines
#define lcd_line_two 0x40 // LCD RAM address for the 2nd line

const uint8_t LCD_INIT_STRING[4] = {
 0x20 | (lcd_type << 2), // Func set: 4-bit, 2 lines, 5x8 dots
 0x0C,                   // Display on (0x0C)
 0x01,                   // Clear display
 0x06                    // Increment cursor
};

// --- Πρωτότυπα Συναρτήσεων ---
void lcd_send_nibble(uint8_t nibble);
void lcd_send_byte(uint8_t address, uint8_t n);
void lcd_init(void);
void lcd_gotoxy(uint8_t x, uint8_t y);
void lcd_putc(char c);

// --- Συναρτήσεις Καθυστέρησης ---
// ΣΗΜΕΙΩΣΗ: Το HAL δεν έχει έτοιμη συνάρτηση για microseconds. 
// Αυτή η συνάρτηση χρησιμοποιεί τον SystemCoreClock (από το HAL) για μια προσεγγιστική λούπα.
void delay_us(uint32_t us) {
    uint32_t count = (SystemCoreClock / 1000000) * us / 4; 
    while(count--) {
        __NOP(); // No Operation - αποτρέπει τον compiler από το να "κόψει" τη λούπα
    }
}

// Προσομοίωση του delay_cycles(20) ~ περίπου 2us
#define delay_cycles() delay_us(2)


//=================================================================

void lcd_send_nibble(uint8_t nibble) {
    // Στέλνουμε κάθε bit στο αντίστοιχο Pin
    HAL_GPIO_WritePin(LCD_DB4_PORT, LCD_DB4_PIN, (nibble & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB5_PORT, LCD_DB5_PIN, (nibble & 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB6_PORT, LCD_DB6_PIN, (nibble & 4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB7_PORT, LCD_DB7_PIN, (nibble & 8) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    delay_cycles();
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    delay_us(45); // Καθυστέρηση ώστε η οθόνη να διαβάσει τα δεδομένα
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
}

void lcd_send_byte(uint8_t address, uint8_t n) {
    if(address) {
        HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
    }
     
    delay_cycles();
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);

    lcd_send_nibble(n >> 4);  // Στέλνουμε το High Nibble
    lcd_send_nibble(n & 0x0F); // Στέλνουμε το Low Nibble
}

void lcd_init(void) {
    uint8_t i;

    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
    
    HAL_Delay(15); // Απαραίτητη αναμονή για την εκκίνηση της οθόνης

    for(i = 0; i < 3; i++) {
        lcd_send_nibble(0x03);
        HAL_Delay(5);
    }

    lcd_send_nibble(0x02); // Θέτουμε την οθόνη σε λειτουργία 4-bit

    for(i = 0; i < sizeof(LCD_INIT_STRING); i++) {
        lcd_send_byte(0, LCD_INIT_STRING[i]);
        HAL_Delay(2);
    }
}

void lcd_gotoxy(uint8_t x, uint8_t y) {
    uint8_t address;

    if(y != 1)
        address = lcd_line_two;
    else
        address = 0;

    address += x - 1;
    lcd_send_byte(0, 0x80 | address);
}

void lcd_putc(char c) {
    switch(c) {
        case '\f': // Clear screen
            lcd_send_byte(0, 1);
            HAL_Delay(2);
            break;
        case '\n': // Νέα γραμμή
            lcd_gotoxy(1, 2);
            break;
        case '\b': // Backspace (μετακίνηση κέρσορα αριστερά)
            lcd_send_byte(0, 0x10);
            break;
        default:
            lcd_send_byte(1, c);
            break;
    }
}



void lcd_puts(char *s) {
    while (*s) {
        lcd_putc(*s); // Τυπώνει τον τρέχοντα χαρακτήρα
        s++;          // Προχωράει στον επόμενο χαρακτήρα
    }

}
