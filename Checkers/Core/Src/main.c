/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <retarget.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum spaceState {
  OCCUPIED,
  EMPTY
};

enum pieceState {
  IN_PLAY,
  CAPTURED
};

enum playerState {
  ACTING,
  WAITING
};

struct BOARD_SPACE{
  char column_letter;
  char row_number;
  enum spaceState SS;
};

struct CHECKER_PIECE{
  struct BOARD_SPACE curr_space;
  struct BOARD_SPACE prev_space;
  bool isKinged;
  enum pieceState PS;
  int color;
};

struct PLAYER{
  struct CHECKER_PIECE player_pieces[12];
  int color;
  int playerNum;
  enum playerState PLS;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_FONT FONT_6X8
#define BUFF_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void readFromHostPC( uint32_t timeout);
// things related to WiFi module;
void wifi_esp8266_init(void);
void readFromWiFi( uint32_t timeout, int debug);
void sendCommandToWiFi(char *command, uint32_t timeout, int debug);
void WiFi_module_rename(void);
void concatenate_strings( char *original, char *add);
void clear( char *buff, int len);
void sendHTTPResponse( int connectionId, char *content, int debug);
// Project-specific functions
void InitializeBoard(struct PLAYER p1, struct PLAYER p2);
void MovePiece(struct PLAYER *acting_player, uint8_t piece_num, struct BOARD_SPACE *dest_space);
//Helper functions
bool ValidDirection(struct CHECKER_PIECE *piece, int dist_row);
struct CHECKER_PIECE* PieceAt(struct BOARD_SPACE *space, struct PLAYER *p1, struct PLAYER *p2);
bool HasCaptureFrom(struct CHECKER_PIECE *piece, struct PLAYER *acting_player, struct PLAYER *opponent);
void Promote(struct CHECKER_PIECE *piece, int dst_row);
bool CheckPlayerDefeat(struct PLAYER *player);
int8_t ColumnLetterToIntTranslation(char col_let);
uint8_t PtToInt(char *effect);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char tx_buff_to_HostPC[BUFF_SIZE];
char rx_buff_from_HostPC[BUFF_SIZE];

int wifi_receive_n = 0;

char rx_buff_from_WiFi_module[BUFF_SIZE];
char tx_buff_to_WiFi_module[BUFF_SIZE];

char http_response_buff[BUFF_SIZE];
char *p_http_response;

char AT_cmd_buff[256];
char *p_AT_cmd;

struct BOARD_SPACE game_board[8][8];
struct PLAYER P1;
struct PLAYER P2;
struct PLAYER *P1p = &P1;
struct PLAYER *P2p = &P2;
bool gameIsOver = false;
bool P1_loses = false;
bool P2_loses = false;
uint8_t winning_player;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  char debug_str_buff[30];
  P1.playerNum = 1;
  P2.playerNum = 2;
  P1.color = C_RED;
  P2.color = C_BLUE;

  for(int row = 0; row < 8; row++){
    for(int col = 0; col < 8; col++){
      game_board[row][col].column_letter = col;
      game_board[row][col].row_number = row;
      game_board[row][col].SS = EMPTY;
    }
  }

  char *p_ipd = 0;
  char *p_effect = 0;
  char c; // used to receive characters from wifi module;
  int connectionId = 0; // used to build HTTP response only;
  int debug = 1;

  //Project Pointers
  struct PLAYER *AP; //Acting Player pointer
  uint8_t *moving_piece_idx;
  struct BOARD_SPACE *MTS; //Move To Space pointer
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);
  LCD_init();
  UG_FillScreen(C_BLUE);
  LCD_PutStr(32, 32, "WiFi ESP8266 Module", DEFAULT_FONT, C_YELLOW, C_BLACK);
  LCD_PutStr(32, 48, "Initialization...", DEFAULT_FONT, C_YELLOW, C_BLACK);
  LCD_PutStr(32, 64, "Welcome to Checkers!", DEFAULT_FONT, C_YELLOW, C_BLACK);
  // WiFi ESP8266 module initialization;
  // NOTE: once you run this program on the Nucleo board for the first time, then,
  // you should be ok with commenting out this wifi initialization, so that you do not
  // wait for resetting and all others of the wifi module every time when you rerun the
  // program...
  wifi_esp8266_init();

  // if all went ok so far, the wifi module should be ready for communication at this time...
  // print on the LCD display that wifi module is ready;
  LCD_PutStr(32, 80, "WiFi module ready!", DEFAULT_FONT, C_YELLOW, C_BLACK);
  UG_FillScreen(C_BLACK);
  InitializeBoard(P1, P2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (!gameIsOver)
  {

    // in each iteration, we just check on whether the wifi module is receiving anything;
    // and if so, then, we parse what was received in the WiFi receive buffer;
    readFromWiFi(1000, 0);
    wifi_receive_n = strlen( rx_buff_from_WiFi_module);
    if ( wifi_receive_n > 0) {

      c = rx_buff_from_WiFi_module[0];
      if ( c != 0) {

        connectionId = c - 48; // c is received as an ASCII code; subtract 48 (ASCII code of '0') to get the actual number;
        if ( debug == 1) {
          // print to host PC;
          sprintf(tx_buff_to_HostPC, "connectionID: %4d \r\n", connectionId);
          HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
        }

        p_ipd = strstr( (const char *)rx_buff_from_WiFi_module, "+IPD"); // find if the substring "+IPD" occurs in the receive buffer;
        if ( p_ipd != 0) {
          p_effect = strstr( (const char *)rx_buff_from_WiFi_module, "player"); // find if the substring "Player=" occurs in the receive buffer;
          if ( p_effect != 0) {
            //LCD_PutStr(32, 80, "Received via WiFi", DEFAULT_FONT, C_YELLOW, C_BLACK);
            HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY); //P=2 45 12
            // then, add 2 to the pointer such that we point to the number after the substring, "P" is 2
            p_effect += 7;
            //Check if the player denomination number is valid
            if (*p_effect == '1' || *p_effect == '2'){
              if (*p_effect == '1') {
                AP = &P1;
              } else if (*p_effect == '2') {
                AP = &P2;
              }

              // Shift pointer to piece number
              p_effect = strstr( (const char *)rx_buff_from_WiFi_module, "piece");
              p_effect += 6;
              uint8_t parsedIdx = 0;
              //Check if the parsed index is valid
              if(*p_effect == '0'){
                p_effect += 1;
                parsedIdx = PtToInt(p_effect);
              } else if (*p_effect == '1'){
                p_effect += 1;
                parsedIdx = PtToInt(p_effect) + 10;
              }

              if (parsedIdx < 12 && parsedIdx >= 0){
                //Assign the valid index to the moving piece index pointer
                moving_piece_idx = &parsedIdx;

                p_effect += 2;
                char moveTo_col_let = *p_effect;
                int8_t moveTo_col_num = ColumnLetterToIntTranslation(*p_effect);
                if(moveTo_col_num != -1) {

                  p_effect += 1;
                  uint8_t moveTo_row_num = PtToInt(p_effect);
                  if(moveTo_row_num <= 8 && moveTo_row_num > 0){
                    //moveTo_row_num -= 1;
                    //Assign the valid move-to space to the move-to-space pointer
                    MTS = &game_board[moveTo_row_num][moveTo_col_num];

                    //Move the appropriate piece
                    MovePiece(AP, *moving_piece_idx, MTS);

                    //Write down the last move performed on the LCD screen
                    sprintf(debug_str_buff, "Pl: %d, Piece: %d, Space: %c%d", AP->playerNum, *moving_piece_idx+1, moveTo_col_let, moveTo_row_num+1);
                    LCD_PutStr(16, 256, debug_str_buff, DEFAULT_FONT, C_WHITE, C_BLACK);
                    if(P1p->PLS == ACTING){
                      sprintf(debug_str_buff, "Player 1's turn");
                    } else {
                      sprintf(debug_str_buff, "Player 2's turn");
                    }
                    LCD_PutStr(16, 272, debug_str_buff, DEFAULT_FONT, C_WHITE, C_BLACK);

                    // print also on serial terminal of host PC;
                    sprintf(tx_buff_to_HostPC, "Move Registered: Player %d, Piece %d, Moved To %c%d \r\n", AP->playerNum, *moving_piece_idx+1, moveTo_col_let, moveTo_row_num+1);
                    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
                    if(P1p->PLS == ACTING){
                      sprintf(tx_buff_to_HostPC, "Player 1's turn.\r\n\n");
                    } else {
                      sprintf(tx_buff_to_HostPC, "Player 2's turn.\r\n\n");
                    }
                    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);

                    P1_loses = CheckPlayerDefeat(P1p);
                    P2_loses = CheckPlayerDefeat(P2p);
                    if(P1_loses || P2_loses) gameIsOver = true;
                  } else { // received nothing after "effect=" or a wrong command; error;
                    // print also on serial terminal of host PC;
                    sprintf(tx_buff_to_HostPC, "%s", "Received invalid ctrl: BAD ROW\r\n");
                    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
                    // send back response;
                    sendHTTPResponse(connectionId, "Received invalid ctrl", 1); // debug=1 true;
                  }
                } else { // received nothing after "effect=" or a wrong command; error;
                  // print also on serial terminal of host PC;
                  sprintf(tx_buff_to_HostPC, "%s", "Received invalid ctrl: BAD COL\r\n");
                  HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
                  // send back response;
                  sendHTTPResponse(connectionId, "Received invalid ctrl", 1); // debug=1 true;
                }
              } else { // received nothing after "effect=" or a wrong command; error;
                // print also on serial terminal of host PC;
                sprintf(tx_buff_to_HostPC, "%s", "Received invalid ctrl: BAD PIECE INDEX\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
                // send back response;
                sendHTTPResponse(connectionId, "Received invalid ctrl", 1); // debug=1 true;
              }
            } else { // received nothing after "effect=" or a wrong command; error;
              // print also on serial terminal of host PC;
              sprintf(tx_buff_to_HostPC, "%s", "Received invalid ctrl: BAD PLAYER NUMBER\r\n");
              HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
              // send back response;
              sendHTTPResponse(connectionId, "Received invalid ctrl", 1); // debug=1 true;
            }
          } else { // if ( p_effect != 0)
            // print also on serial terminal of host PC;
            sprintf(tx_buff_to_HostPC, "%s", "Received invalid ctrl\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
            // send back response;
            sendHTTPResponse(connectionId, "Received invalid ctrl", 1); // debug=1 true;
          } // if ( p_effect != 0)

        } // if ( p_ipd != 0)
      } // if ( c != 0)
    } // if ( wifi_receive_n > 0)

  } // while(gameIsOver)
  if(P1_loses) {
    winning_player = 2;
  } else if (P2_loses) {
    winning_player = 1;
  }
  sprintf(debug_str_buff, "                          ");
  LCD_PutStr(16, 256, debug_str_buff, DEFAULT_FONT, C_BLACK, C_BLACK);
  sprintf(debug_str_buff, "               ");
  LCD_PutStr(16, 272, debug_str_buff, DEFAULT_FONT, C_BLACK, C_BLACK);

  sprintf(debug_str_buff, "Game Over! Player %d Wins!", winning_player);
  LCD_PutStr(16, 256, debug_str_buff, DEFAULT_FONT, C_WHITE, C_BLACK);
  printf("Game Over! Player %d Wins!\r\n", winning_player);

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  //NOTE TO SELF: ALWAYS DOUBLE CHECK THAT BAUD RATE IS CORRECT
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
///////////////////////////////////////////////////////////////////////////////
//
// comm. with host PC via UART2 related functions
//
///////////////////////////////////////////////////////////////////////////////

void readFromHostPC( uint32_t timeout)
{
  // reads from host PC;
  int i = 0;

  // (1) clear receive buffer first;
  for (i = 0; i < BUFF_SIZE; i++) {
    rx_buff_from_HostPC[i] = 0;
  }

  // (2) check if anything was sent from host PC;
  // place received message rx_buff_from_HostPC;
  HAL_UART_Receive(&huart2, (uint8_t *)rx_buff_from_HostPC, BUFF_SIZE, timeout);
}

///////////////////////////////////////////////////////////////////////////////
//
// WiFi module related functions
//
///////////////////////////////////////////////////////////////////////////////

void concatenate_strings( char *original, char *add)
{
  // custom concatenation, without adding the '\0' at the end;
  while (*original != 0) {
    original++;
  }
  while (*add != 0) {
    *original = *add;
    add++;
    original++;
  }
  //*original = '\0';
}

void clear( char *buff, int len)
{
  // generic function to clear array "buff" of characters whose
  // length "len" is already known;
  while ( len > 0) {
    *buff = 0; buff++; len--;
  }
}

void readFromWiFi( uint32_t timeout, int debug)
{
  // reads from the WiFi module if it has received anything;
  int i = 0;

  // (1) clear receive buffer first;
  for (i = 0; i < BUFF_SIZE; i++) {
    rx_buff_from_WiFi_module[i] = 0;
  }

  // (2) check if anything was sent from WiFi module;
  // place received message into rx_buff_from_WiFi_module;
  HAL_UART_Receive(&huart1, (uint8_t *)rx_buff_from_WiFi_module, BUFF_SIZE, timeout);

  // (3) if debug is true, we also print to host PC;
  if (debug) {
    sprintf(tx_buff_to_HostPC, "%s", "\r\n<-------- START received data from WiFi -------->\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    strcpy(tx_buff_to_HostPC, rx_buff_from_WiFi_module);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    sprintf(tx_buff_to_HostPC, "%s", "\r\n>-------- END received data from WiFi --------<\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
  }
}

void sendCommandToWiFi(char *command, uint32_t timeout, int debug)
{
  int i = 0;
  // (1) clear receive buffer first;
  for (i = 0; i < BUFF_SIZE; i++) {
    rx_buff_from_WiFi_module[i] = 0;
  }

  // (2) send command to WiFi module;
  HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), timeout);

  // (3) check if WiFi module replied with anything; place received message
  // into rx_buff_from_WiFi_module;
  HAL_UART_Receive(&huart1, (uint8_t *)rx_buff_from_WiFi_module, BUFF_SIZE, timeout);

  // (4) if debug is true, we also print to host PC;
  if (debug) {
    sprintf(tx_buff_to_HostPC, "%s", "\r\n<-------- START response to sendCommandToWiFi() -------->\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    strcpy(tx_buff_to_HostPC, rx_buff_from_WiFi_module);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    sprintf(tx_buff_to_HostPC, "%s", "\r\n>-------- END response to sendCommandToWiFi() --------<\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
  }
}

void wifi_esp8266_init(void)
{
  // this function does some initializations; should be called only once;
  sendCommandToWiFi("AT\r\n", 2000, 1); // timeout of 2000 ms should be enough; debug set to true;
  HAL_Delay(1000);
  sendCommandToWiFi("AT+RST\r\n", 2000, 1); // reset wifi module
  HAL_Delay(1000);
  sendCommandToWiFi("AT+CWMODE=2\r\n", 2000, 1); // configure as access point
  HAL_Delay(1000);
  sendCommandToWiFi("AT+CIFSR\r\n", 2000, 1); // get ip address
  HAL_Delay(1000);
  sendCommandToWiFi("AT+CIPMUX=1\r\n", 2000, 1); // configure for multiple connections
  HAL_Delay(1000);
  sendCommandToWiFi("AT+CIPSERVER=1,80\r\n", 2000, 1); // turn on server on port 80
  HAL_Delay(1000);
}

void sendHTTPResponse( int connectionId, char *content, int debug)
{
  // function that sends HTTP 200, HTML UTF-8 response back to the Android App via the WiFi module;
  // the content argument can be: "Control received: 1" or "Control received: 2" or
  // "Received invalid ctrl" or "Received no ctrl"
  char len[4]; // a sketch small array; used for passing numbers during concatenations;

  // build HTTP response as HTTP Header + content
  p_http_response = http_response_buff; // access elements of http_response_buff via pointer during concatenations;
  clear( http_response_buff, BUFF_SIZE);
  clear( len, 4);
  sprintf( len, "%d", strlen(content));
  concatenate_strings( p_http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n");
  concatenate_strings( p_http_response, "Content-Length: ");
  concatenate_strings( p_http_response, len);
  concatenate_strings( p_http_response, "\r\n");
  concatenate_strings( p_http_response, "Connection: close\r\n\r\n");
  concatenate_strings( p_http_response, content);

  // build AT command;
  p_AT_cmd = AT_cmd_buff; // access elements of AT_cmd_buff via pointer during concatenations;
  clear( AT_cmd_buff, 256);
  clear( len, 4);
  sprintf( len, "%d", connectionId );
  concatenate_strings( p_AT_cmd, "AT+CIPSEND="); // "Sends Data" AT command
  concatenate_strings( p_AT_cmd, len);
  concatenate_strings( p_AT_cmd, ",");
  clear( len, 4);
  sprintf( len, "%d", strlen(http_response_buff));
  concatenate_strings( p_AT_cmd, len);
  concatenate_strings( p_AT_cmd, "\r\n");

  // send AT command and HTTP response to wifi module;
  sendCommandToWiFi( AT_cmd_buff, 2000, 1);
  sendCommandToWiFi( http_response_buff, 2000, 1);

  // if debug is true, we also print to host PC;
  if (debug) {
    sprintf(tx_buff_to_HostPC, "%s", "\r\n================== Sent AT Cmd =================\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    strcpy(tx_buff_to_HostPC, AT_cmd_buff);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    sprintf(tx_buff_to_HostPC, "%s", "\r\n=============== Sent HTTP response =============\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    strcpy(tx_buff_to_HostPC, http_response_buff);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
    sprintf(tx_buff_to_HostPC, "%s", "\r\n================================================\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buff_to_HostPC, strlen(tx_buff_to_HostPC), HAL_MAX_DELAY);
  }
}

void InitializeBoard(struct PLAYER p1, struct PLAYER p2)
{
  //Draw the board on the LCD
  LCD_DrawCheckerBoard();

  //Initialize of each of the players' pieces' locations, states and colors.
  unsigned int row, col, orientation, piece_idx;
  piece_idx = 0;
  orientation = 1;
  for(row = 0; row < 3;){
    for(col = 0; col < 4;)
    {
      if(orientation == 0){
        p1.player_pieces[piece_idx].curr_space = game_board[row][col*2];
        p1.player_pieces[piece_idx].prev_space = p1.player_pieces[piece_idx].curr_space;
        p1.player_pieces[piece_idx].curr_space.SS = OCCUPIED;
        p1.player_pieces[piece_idx].color = p1.color;
        game_board[row][col*2].SS = OCCUPIED;
      } else {
        p1.player_pieces[piece_idx].curr_space = game_board[row][1+(col*2)];
        p1.player_pieces[piece_idx].prev_space = p1.player_pieces[piece_idx].curr_space;
        p1.player_pieces[piece_idx].curr_space.SS = OCCUPIED;
        p1.player_pieces[piece_idx].color = p1.color;
        game_board[row][1+(col*2)].SS = OCCUPIED;
      }
      p1.player_pieces[piece_idx].isKinged = false;
      //Use temp variables to double check for correct assignment
      char P1_curr_space_col = p1.player_pieces[piece_idx].curr_space.column_letter;
      char P1_curr_space_row = p1.player_pieces[piece_idx].curr_space.row_number;
      bool P1_king_state = p1.player_pieces[piece_idx].isKinged;
      int P1_piece_color = p1.player_pieces[piece_idx].color;
      LCD_DrawCheckerPiece(piece_idx, P1_curr_space_row, P1_curr_space_col, P1_king_state, P1_piece_color);
      // DEBUG LINES
      printf("Player %d, Piece %d, Row %d, Column %d \r\n", p1.playerNum, piece_idx, P1_curr_space_row, P1_curr_space_col);

      piece_idx++;
      col++;
    }
    if(orientation==0){
      orientation = 1;
    } else {
      orientation = 0;
    }
    row++;
  }
  P1 = p1;
  P1.PLS = ACTING;
  piece_idx = 0;
  orientation = 0;
  for(row = 5; row < 8;){
    for(col = 0; col < 4;)
    {
      if(orientation == 0){
        p2.player_pieces[piece_idx].curr_space = game_board[row][col*2];
        p2.player_pieces[piece_idx].prev_space = p2.player_pieces[piece_idx].curr_space;
        p2.player_pieces[piece_idx].curr_space.SS = OCCUPIED;
        p2.player_pieces[piece_idx].color = p2.color;
        game_board[row][col*2].SS = OCCUPIED;
      } else {
        p2.player_pieces[piece_idx].curr_space = game_board[row][1+(col*2)];
        p2.player_pieces[piece_idx].prev_space = p2.player_pieces[piece_idx].curr_space;
        p2.player_pieces[piece_idx].curr_space.SS = OCCUPIED;
        p2.player_pieces[piece_idx].color = p2.color;
        game_board[row][col*2].SS = OCCUPIED;
      }
      p2.player_pieces[piece_idx].isKinged = false;
      //Use temp variables to double check for correct assignment
      char P2_curr_space_col = p2.player_pieces[piece_idx].curr_space.column_letter;
      char P2_curr_space_row = p2.player_pieces[piece_idx].curr_space.row_number;
      bool P2_king_state = p2.player_pieces[piece_idx].isKinged;
      int P2_piece_color = p2.player_pieces[piece_idx].color;
      LCD_DrawCheckerPiece(piece_idx, P2_curr_space_row, P2_curr_space_col, P2_king_state, P2_piece_color);
      // DEBUG LINES
      printf("Player %d, Piece %d, Row %d, Column %d \r\n", p2.playerNum, piece_idx, P2_curr_space_row, P2_curr_space_col);

      piece_idx++;
      col++;
    }
    if(orientation==0){
      orientation = 1;
    } else {
      orientation = 0;
    }
    row++;
  }
  P2 = p2;
  P2.PLS = WAITING;

  printf("\nWelcome to Checkers!\r\n");
  printf("Use your Wi-Fi Serial Adapter application on a wireless device to play!\r\n");
  printf("PLAYER 1: RED, PLAYER 2: BLUE\r\n");
  printf("It is Player 1's turn.\r\n\n");
}

//Note: the "dest_space" must have it's column letter translated before moving into the function.
void MovePiece(struct PLAYER *acting_player, uint8_t piece_num, struct BOARD_SPACE *dest_space)
{
  if(acting_player->PLS == WAITING){
    printf("It is not this player's turn.\r\n");
    return;
  } else {
    struct PLAYER *opponent;
    if(acting_player->color == C_RED){
      opponent = P2p;
    } else {
      opponent = P1p;
    }
    uint8_t piece_idx = piece_num;
    struct CHECKER_PIECE *piece = &acting_player->player_pieces[piece_idx];
    uint8_t src_row = piece->curr_space.row_number;
    uint8_t src_col = piece->curr_space.column_letter;
    uint8_t dest_row = dest_space->row_number;
    uint8_t dest_col = dest_space->column_letter;

    //Measure the distance of the move
    int dist_row = dest_row - src_row;
    int dist_col = dest_col - src_col;
    int abs_dist_row = abs(dist_row);
    int abs_dist_col = abs(dist_col);

    //Regular move process
    if(abs_dist_row == 1 && abs_dist_col == 1) {
      if(game_board[dest_row][dest_col].SS == EMPTY && ValidDirection(piece, dist_row)) {
        //Update the state of the game board spaces occupying the moved piece.
        game_board[src_row][src_col].SS = EMPTY;
        game_board[dest_row][dest_col].SS = OCCUPIED;
        //Move the piece to its new space;
        piece->prev_space = piece->curr_space;
        piece->curr_space = game_board[dest_row][dest_col];
        //Promote piece if it reached its promotion row
        Promote(piece, dest_row);
        //Create local variables to reduce cluttering.
        int prev_piece_row = piece->prev_space.row_number;
        int prev_piece_column = piece->prev_space.column_letter;
        char new_piece_row = piece->curr_space.row_number;
        char new_piece_column = piece->curr_space.column_letter;
        bool piece_king_state = piece->isKinged;
        //Redraw the piece onto the new space.
        LCD_EraseCheckerPiece(prev_piece_row, prev_piece_column);
        LCD_DrawCheckerPiece(piece_idx, new_piece_row, new_piece_column, piece_king_state, piece->color);
      } else {
        printf("Cannot move to this space: Row %d, Column %d\r\n", dest_space->row_number, dest_space->column_letter);
        printf("Space is either occupied, direction is invalid, or\r\n");
        printf("the moving piece has been captured.\r\n");
        return;
      }//Empty space and valid direction check
    } else if(abs_dist_row == 2 && abs_dist_col == 2) {
      int mid_row = src_row + dist_row/2;
      int mid_col = src_col + dist_col/2;
      struct BOARD_SPACE *mid = &game_board[mid_row][mid_col];
      if (mid->SS == OCCUPIED && PieceAt(mid, P1p, P2p)->color != acting_player->color && game_board[dest_row][dest_col].SS == EMPTY && ValidDirection(piece, dist_row)) {
        //Capture and remove piece in mid
        struct CHECKER_PIECE *captured = PieceAt(mid, P1p, P2p);
        captured->PS = CAPTURED;
        game_board[mid_row][mid_col].SS = EMPTY;
        LCD_EraseCheckerPiece(mid_row, mid_col);
        //Move the capturing piece to its new space
        game_board[src_row][src_col].SS = EMPTY;
        game_board[dest_row][dest_col].SS = OCCUPIED;
        piece->prev_space = piece->curr_space;
        piece->curr_space = game_board[dest_row][dest_col];
        Promote(piece, dest_row);
        //Create local variables to reduce cluttering.
        int prev_piece_row = piece->prev_space.row_number;
        int prev_piece_column = piece->prev_space.column_letter;
        char new_piece_row = piece->curr_space.row_number;
        char new_piece_column = piece->curr_space.column_letter;
        bool piece_king_state = piece->isKinged;
        //Redraw the piece onto the new space.
        LCD_EraseCheckerPiece(prev_piece_row, prev_piece_column);
        LCD_DrawCheckerPiece(piece_idx, new_piece_row, new_piece_column, piece_king_state, piece->color);
        //Check if capturing piece has additional captures from new position
        if (HasCaptureFrom(piece, acting_player, opponent)) {
          printf("The acting player still has another capture they can perform!\r\n");
          return;
        }
      } else {
        printf("Cannot move to this space: Row %d, Column %d\r\n", dest_space->row_number, dest_space->column_letter);
        printf("Space is either occupied, direction is invalid,\r\n");
        printf("there is no opponent piece in the middle, or\r\n");
        printf("the capturing piece is of the same color as the piece in the middle.");
        return;
      }
      //Mid piece check
    }//Capture check
    if(acting_player->color == C_RED) {
      P1.PLS = WAITING;
      P2.PLS = ACTING;
    } else {
      P1.PLS = ACTING;
      P2.PLS = WAITING;
    }
    return;
  }//Acting player check
}

/* Helper Functions */

//Returns true if a piece is moving in a valid direction, and false otherwise.
bool ValidDirection(struct CHECKER_PIECE *piece, int dist_row){
  //Kinged pieces can move in all directions
  if(piece->isKinged) return true;

  //Captured pieces can't move
  if(piece->PS == CAPTURED) {
    printf("This piece has been captured.\r\n\n");
    return false;
  }

  //P1's pieces can only move down
  if(piece->color == C_RED){
    return (dist_row > 0);
  } else {
    return (dist_row < 0);
  }
}

//Returns the address of the piece at the identified space pointer.
struct CHECKER_PIECE* PieceAt(struct BOARD_SPACE *space, struct PLAYER *p1, struct PLAYER *p2) {
  if(game_board[(int)space->row_number][(int)space->column_letter].SS == EMPTY){
    return NULL;
  } else {
    // Search both players' pieces for the piece at the space denoted in the space pointer
    for (int i = 0; i < 12; i++) {
        if (p1->player_pieces[i].PS == IN_PLAY &&
            p1->player_pieces[i].curr_space.row_number == space->row_number &&
            p1->player_pieces[i].curr_space.column_letter == space->column_letter) {
            return &p1->player_pieces[i];
        }
        if (p2->player_pieces[i].PS == IN_PLAY &&
            p2->player_pieces[i].curr_space.row_number == space->row_number &&
            p2->player_pieces[i].curr_space.column_letter == space->column_letter) {
            return &p2->player_pieces[i];
        }
    }
  }
  return NULL; // return NULL if the space contains no piece
}

bool HasCaptureFrom(struct CHECKER_PIECE *piece, struct PLAYER *acting_player, struct PLAYER *opponent) {
  int r = piece->curr_space.row_number;
  int c = piece->curr_space.column_letter;

  // Four diagonal directions
  int dirs[4][2] = { {2,2}, {2,-2}, {-2,2}, {-2,-2} };

  for (int i = 0; i < 4; i++) {
    int dr = dirs[i][0];
    int dc = dirs[i][1];
    int dst_r = r + dr;
    int dst_c = c + dc;
    int mid_r = r + dr/2;
    int mid_c = c + dc/2;

    // Bounds check
    if (dst_r < 0 || dst_r > 7 || dst_c < 0 || dst_c > 7) {
      // Destination must be empty
      if (game_board[dst_r][dst_c].SS != EMPTY) {
        // Middle square must contain opponent piece
        struct BOARD_SPACE *mid = &game_board[mid_r][mid_c];
        struct CHECKER_PIECE *captured = PieceAt(mid, acting_player, opponent);
        if (captured && captured->PS == IN_PLAY && captured->color != acting_player->color) {
          if (ValidDirection(piece, dr)) return true;
        }
      } // Destination empty
    } // Bounds valid
  } //for loop
  return false;
}

//Promotes a piece to King.
void Promote(struct CHECKER_PIECE *piece, int dst_row) {
  // Player 1 promotes at row 7
  if (piece->curr_space.row_number == 7 && piece->isKinged == false) {
      piece->isKinged = true;
      printf("Piece promoted to King!\r\n");
  }
  // Player 2 promotes at row 0
  if (piece->curr_space.row_number == 0 && piece->isKinged == false) {
      piece->isKinged = true;
      printf("Piece promoted to King!\r\n");
  }
}

bool CheckPlayerDefeat(struct PLAYER *player) {
  // Search a players' pieces to see if they have no more pieces "in-play", and have thus lost the game.
  for (int i = 0; i < 12; i++) {
    if (player->player_pieces[i].PS == IN_PLAY) {
      return false;
    }
  }
  return true;
}

int8_t ColumnLetterToIntTranslation(char col_let){
  int translated_int;
  switch(col_let)
  {
    case 'a':
      translated_int = 0;
      break;
    case 'b':
      translated_int = 1;
      break;
    case 'c':
      translated_int = 2;
      break;
    case 'd':
      translated_int = 3;
      break;
    case 'e':
      translated_int = 4;
      break;
    case 'f':
      translated_int = 5;
      break;
    case 'g':
      translated_int = 6;
      break;
    case 'h':
      translated_int = 7;
      break;
    default:
      translated_int = -1;
  }
  return translated_int;
}

uint8_t PtToInt(char *effect){
  int translated_int; // add one to the int after changing it when focusing one the pieces

  switch (*effect)
    {
      case '1':
        translated_int = 0;
        break;
      case '2':
        translated_int = 1;
        break;
      case '3':
        translated_int = 2;
        break;
      case '4':
        translated_int = 3;
        break;
      case '5':
        translated_int = 4;
        break;
      case '6':
        translated_int = 5;
        break;
      case '7':
        translated_int = 6;
        break;
      case '8':
        translated_int = 7;
        break;
      case '9':
        translated_int = 8;
        break;

      default:
        translated_int = -1;
    }
  return translated_int;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
