#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "stm32l4xx_hal.h"

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void sendTriggerPulse(void);
float measureDistance(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  float lastDistance = -1.0;
  char buffer[50];

  // Config LED LD2 e LEDs no CN9
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED LD2 desligado
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET); // LEDs vermelho, verde e branco desligados

  while (1)
  {
    float distance = measureDistance();
    if (distance > 0.0)
    {
      if (lastDistance >= 0.0)
      {
        if (distance < lastDistance)
        {
          sprintf(buffer, "DIST OBJ: %.2f CM, (POUSANDO) [PISTA BLOQUEADA]\r\n", distance);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Liga o LED LD2 (pouso)
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); // Liga o LED vermelho
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); // Desliga o LED verde
        }
        else if (distance > lastDistance)
        {
          sprintf(buffer, "DIST OBJ: %.2f CM, (DECOLANDO) [PISTA LIBERADA]\r\n", distance);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Liga o LED LD2 (decolagem)
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); // Desliga o LED vermelho
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); // Liga o LED verde
        }
      }
      else
      {
        sprintf(buffer, "DIST OBJ: %.2f CM, (AERONAVE DETECTADA NA PISTA)\r\n", distance);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Liga o LED LD2 (busca)
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // Liga o LED branco (piscando)
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Desliga o LED LD2 (busca)
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Desliga o LED branco (piscando)
        HAL_Delay(50);
      }

      HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
      lastDistance = distance;
    }
    else
    {
      HAL_UART_Transmit(&huart2, (uint8_t *)"BUSCANDO...\r\n", strlen("BUSCANDO...\r\n"), HAL_MAX_DELAY);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Liga o LED LD2 (busca)
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // Liga o LED branco (piscando)
      HAL_Delay(50);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Desliga o LED LD2 (busca)
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // Desliga o LED branco (piscando)
      HAL_Delay(50);
    }
    HAL_Delay(300); // Intervalo para 300ms entre as leituras
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Ativar o clock para as portas D5, D6 e D7 (GPIO D)
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // Configuração dos pinos de LED LD2, vermelho, verde e branco
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // Voltagem máxima
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // Inicializa os LEDs desligados
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
}

static void MX_USART2_UART_Init(void)
{
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
}

void sendTriggerPulse(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

#define MAX_DISTANCE 100.0 // distancia maxima em CM

float measureDistance(void)
{
  sendTriggerPulse();
  uint32_t startTime = 0;
  uint32_t stopTime = 0;
  uint32_t timeout = 1000000; // 1 segundo de timeout entre os pulsos

  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
  {
    startTime = HAL_GetTick();
    if (startTime > timeout)
    {
      return -1.0; // Valor inválido de distância
    }
  }

  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
  {
    stopTime = HAL_GetTick();
    if ((stopTime - startTime) > timeout)
    {
      return -1.0; // Valor inválido de distância
    }
  }

  uint32_t pulseDuration = stopTime - startTime;
  float distance = (34300.0 * pulseDuration) / (2.0 * 1000.0); // Converter para centímetros

  // Verifica se a distância medida é maior que a distância máxima
  if (distance > MAX_DISTANCE)
  {
    return -1.0; // Distância maior que a máxima desejada
  }

  return distance;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
