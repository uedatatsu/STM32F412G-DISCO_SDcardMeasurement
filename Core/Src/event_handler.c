#include "event_handler.h"
#include <stdio.h>
#include "fatfs.h"
#include <string.h>
#include "stm32f4xx_hal.h"
// #include "bsp_lcd.h"
#define BLOCK_SIZE 512

volatile uint8_t dataReady_sdFormat = 0; // データが準備完了したことを示すフラグ
volatile uint8_t ansFlag = 0; // フォーマット確認用フラグ
extern uint8_t rxBuffer[1]; // 受信データ用バッファ
extern int inputMode; // 入力モード

static int block_multiplier = 128; // 必要に応じてmain.c等から変更可能

void handle_event(uint8_t eventFlag) {
    switch (eventFlag) {
        case '1':
            printf("Hello\r\n");
            break;
        case '2':
            printf("Event 2 triggered\r\n");
            break;
        case '3': {
            FRESULT res;
            UINT bw;
            const char* text = "SD card write test!\r\n";
            res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
            if (res == FR_OK) {
                res = f_open(&SDFile, "test.txt", FA_WRITE | FA_CREATE_ALWAYS);
                if (res == FR_OK) {
                    res = f_write(&SDFile, text, strlen(text), &bw);
                    if (res == FR_OK && bw == strlen(text)) {
                        printf("SD write OK\r\n");
                    } else {
                        printf("SD write error\r\n");
                    }
                    f_close(&SDFile);
                } else {
                    printf("File open error\r\n");
                }
                f_mount(NULL, (TCHAR const*)SDPath, 1);
            } else {
                printf("SD mount error\r\n");
                uint8_t rtext[_MAX_SS];/* File read buffer */
                f_mkfs(SDPath, FM_ANY, 0, rtext, sizeof(rtext)); // SDカードの初期化
                res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1); // 再度マウント
                if (res == FR_OK) {
                    printf("SD card formatted and mounted successfully\r\n");
                } else {
                    printf("SD mount error after format\r\n");
                }
            }
            break;
        }
        case '4': {
            FRESULT res;
            UINT bw;
            uint32_t total_size = BLOCK_SIZE * block_multiplier;
            uint8_t *binData = malloc(total_size);
            if (binData == NULL) {
                printf("Memory alloc error\r\n");
                break;
            }
            for (uint32_t i = 0; i < total_size; i++) {
                binData[i] = (uint8_t)(i & 0xFF);
            }
            // DWTサイクルカウンタ有効化
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
            res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
            if (res == FR_OK) {
                res = f_open(&SDFile, "binary.bin", FA_WRITE | FA_CREATE_ALWAYS);
                if (res == FR_OK) {
                    uint32_t start = DWT->CYCCNT;
                    res = f_write(&SDFile, binData, total_size, &bw);
                    uint32_t end = DWT->CYCCNT;
                    float us = (float)(end - start) / (SystemCoreClock / 1000000.0f);
                    printf("f_write requested: %lu bytes, actually written: %lu bytes\r\n", total_size, (uint32_t)bw);
                    if (res == FR_OK && bw == total_size) {
                        printf("SD binary write OK (%lu bytes, %.2f us)\r\n", total_size, us);
                        // char lcdMsg[64];
                        // snprintf(lcdMsg, sizeof(lcdMsg), "SD: %luB %.2f us", total_size, us);
                        // BSP_LCD_Clear(LCD_COLOR_BLACK);
                        // BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
                        // BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t*)lcdMsg, CENTER_MODE);
                    } else {
                        printf("SD binary write error\r\n");
                    }
                    f_close(&SDFile);
                } else {
                    printf("File open error\r\n");
                }
                f_mount(NULL, (TCHAR const*)SDPath, 1);
            } else {
                printf("SD mount error\r\n");
            }
            free(binData);
            break;
        }
        case '9': {
            printf("SDカードをフォーマットすると全てのデータが消去されます。実行しますか？ (y/n): \r\n");
            ansFlag = 0; // フラグをリセット
            dataReady_sdFormat = 0; // フラグをリセット
            inputMode = INPUT_SDFORMAT; // フォーマットモードに変更        
            while(1){
                if (dataReady_sdFormat) {
                    if (ansFlag == 'y' || ansFlag == 'Y') {
                        break; // フォーマット実行
                    } else if (ansFlag == 'n' || ansFlag == 'N') {
                        printf("フォーマットを中止しました。\r\n");
                        return;
                    } else {
                        printf("無効な入力です。y/nで答えてください。\r\n");
                    }
                    dataReady_sdFormat = 0; // フラグをリセット
                }

            }
            

            FRESULT res;
            uint8_t rtext[_MAX_SS];/* File read buffer */
            printf("SD card format start...\r\n");

            f_mkfs(SDPath, FM_ANY, 0, rtext, sizeof(rtext)); // SDカードの初期化
            res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1); // 再度マウント
            if (res == FR_OK) {
                printf("SD card formatted and mounted successfully\r\n");
                DWORD fre_clust, fre_sect, tot_sect;
                FATFS *fs;
                fs = &SDFatFS;
                if (f_getfree(SDPath, &fre_clust, &fs) == FR_OK) {
                    #if _FS_EXFAT
                    if (fs->fs_type == FS_EXFAT) {
                        printf("FAT type: exFAT\r\n");
                        printf("Cluster size: %lu bytes\r\n", (DWORD)fs->csize * BLOCK_SIZE);
                        DWORD tot_sect = (fs->n_fatent - 2) * fs->csize;
                        DWORD fre_sect = fre_clust * fs->csize;
                        printf("Total size: %lu KB\r\n", tot_sect / 2);
                        printf("Free space: %lu KB\r\n", fre_sect / 2);
                    } else
                    #endif
                    {
                        printf("FAT type: FAT%u\r\n", (fs->fs_type == FS_FAT12) ? 12 : (fs->fs_type == FS_FAT16) ? 16 : 32);
                        printf("Cluster size: %lu bytes\r\n", (DWORD)fs->csize * BLOCK_SIZE);
                        DWORD tot_sect = (fs->n_fatent - 2) * fs->csize;
                        DWORD fre_sect = fre_clust * fs->csize;
                        printf("Total size: %lu KB\r\n", tot_sect / 2);
                        printf("Free space: %lu KB\r\n", fre_sect / 2);
                    }
                } else {
                    printf("Failed to get FS info\r\n");
                }
            } else {
                printf("SD mount error after format\r\n");
            }
            break;
        }
        default:
            break;
    }
}
