#include "user.h"

#include "stdint.h"
#include "dshot.h"
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

uint8_t rx[RDATA_SIZE];
uint8_t rxtmp[RDATA_SIZE];
uint8_t tx[TDATA_SIZE] = {0x12, 0x34};
uint16_t pwm[4] = {0, 0, 0, 0};
uint16_t pwm_tmp[4] = {0, 0, 0, 0};
uint32_t pwm_update_time = 0;
uint32_t pwm_update_time_last = 0;

uint32_t pwm_update_count = 0;
uint16_t uart_callback_count = 0;
uint32_t pwm_update_count_last = 0;
uint16_t loop_count = 0;
uint16_t loop_count_last = 0;
uint16_t uart_callback_count_last = 0;

uint32_t pkg_interval = 0;
uint32_t pkg_interval_min = 0xFFFFFFFF;
uint32_t pkg_interval_max = 0;
uint32_t connected_time = 0;

uint16_t test_count = 0;

bool is_startup = false;
bool is_connect = false;
bool last_state = false;
bool is_pkg_update = false;
uint32_t lock = 0;

void show_hex(uint8_t *buf, int len)
{
  for (int i = 0; i < len; i++)
  {
    printf("%02X ", buf[i]);
  }
}

uint8_t crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ 0x31;
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

int check_pkg(uint8_t *input)
{
  // uint16_t check_sum = 0;
  int ret = -1;

  if (input[0] == 0xAB && input[1] == 0xCD)
  {
    if (input[10] == crc8(input + 2, 8))
    {
      ret = 0;
    }
    // for (int i = 2; i < RDATA_SIZE - 2; i++)
    // {
    //   check_sum += input[i];
    // }

    // uint16_t rx_check_sum = (uint16_t)((input[RDATA_SIZE - 1] << 8) | input[RDATA_SIZE - 2]);
    // if (check_sum == rx_check_sum)
    // {
    //   ret = 0;
    // }
  }

  return ret;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static int err_count = 0;
  uint32_t uart_callback_time = TIM2->CNT;

  // if (huart->Instance != USART2)
  // {
  //   return;
  // }

  uart_callback_count++;

  if (is_connect == false)
  {
    if (check_pkg(rx) != 0)
    {
      err_count++;

      			// printf("%d : ",err_count);
      			// show_hex(rx, RDATA_SIZE);
      			// printf("\r\n");

      //			if(err_count >=3)
      //			{
      //				err_count = 0;
      //
      //				HAL_UART_DMAStop(&huart2);

      //        MX_USART2_UART_Init();
      //				HAL_UART_Receive_DMA(&huart2, rx, RDATA_SIZE);
      //			}
    }
    else
    {
      // __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
      connected_time = TIM2->CNT;
      memcpy(rxtmp, rx, RDATA_SIZE);
      pwm_update_time = TIM2->CNT;
      pwm_update_time_last = pwm_update_time;
      is_connect = true;
    }
  }
  else if (check_pkg(rx) == 0)
  {
    memcpy(rxtmp, rx, RDATA_SIZE);
    is_pkg_update = true;

    pwm_update_time = TIM2->CNT;

    pkg_interval = pwm_update_time - pwm_update_time_last;

    if (pkg_interval > pkg_interval_max)
    {
      pkg_interval_max = pkg_interval;
    }

    if (pkg_interval < pkg_interval_min)
    {
      pkg_interval_min = pkg_interval;
    }
    pwm_update_time_last = pwm_update_time;
  }
  else
  {
    is_connect = false;
  }
}

void set_pwm_all(uint16_t value)
{
    for (int i = 0; i < 4; i++)
    {
          pwm[i] = value;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == htim1.Instance)
  {

    if (is_connect == true)
    {
      if ((TIM2->CNT - pwm_update_time) > 100)
      {
        is_connect = false;
      }
    }

    if (is_connect == true && is_startup == true)
    {
      if (is_pkg_update == true)
      {
        is_pkg_update = false;
        if (check_pkg(rxtmp) == 0)
        {
          memcpy(pwm, rxtmp + 2, 8);
          memcpy(tx + 2 + 8, rxtmp + 2, 8);

          // if ((pwm[0] + 1 != pwm[1]) || (pwm[0] + 2 != pwm[2]) || (pwm[0] + 3 != pwm[3]))
          // {
          //   printf("ERROR %d %d %d %d\r\n", pwm[0], pwm[1], pwm[2], pwm[3]);
          //   show_hex(rxtmp, RDATA_SIZE);
          //   while (1)
          //   {
          //   };
          // }

          for (int i = 0; i < 4; i++)
          {
            if (pwm[i] > 1000)
            {
              pwm[i] = 1000;
            }

            if (pwm[i] > 0)
            {
              pwm[i] = DSHOT_MIN_THROTTLE + (pwm[i] - 1) * 1999 / 999;
            }
          }
          dshot_write(pwm);
          pwm_update_count++;
        }
      }
    }
   else
   {
     set_pwm_all(0);
     dshot_write(pwm);
   }

    // if (TIM2->CNT > 5000 && TIM2->CNT < 10000)
    // {
    //   set_pwm_all(0);
    //   dshot_write(pwm);
    // }
    // else if(TIM2->CNT <= 5000 || TIM2->CNT >= 11000)
		// {
			 
		// }
    // if(TIM2->CNT < 5000)
    // {
    //   set_pwm_all(0);
    //   dshot_write(pwm);
    // }
    // else
    // {
    //   set_pwm_all(1);
    //   dshot_write(pwm);
    // }

    for (int i = 0; i < 4; i++)
    {
      pwm_tmp[i] = pwm[i];
    }
  }
}

void loop_1s(void)
{
  static uint32_t loop_1s = 0;
  uint32_t now = TIM2->CNT;

  if (TIM2->CNT - loop_1s >= 1000)
  {
    loop_1s = now;

    printf("update:%d loop:%d rx:%d isconnect:%d max:%d min:%d pwm:%d %d %d %d dshot:%d %d %d %d\r\n",
           pwm_update_count_last, loop_count_last, uart_callback_count_last, is_connect,
           pkg_interval_max, pkg_interval_min,
           pwm[0], pwm[1], pwm[2], pwm[3],
           pwm_tmp[0], pwm_tmp[1], pwm_tmp[2], pwm_tmp[3]);
  }
}

void loop_100ms(void)
{
  static uint32_t loop_100ms = 0;
  uint32_t now = TIM2->CNT;

  if (TIM2->CNT - loop_100ms >= 100)
  {
    loop_100ms = now;

    ((uint16_t *)tx)[9] = pwm_update_count;
    ((uint16_t *)tx)[10] = loop_count;
    ((uint16_t *)tx)[11] = uart_callback_count;
    ((uint16_t *)tx)[12] = is_connect;

    // if (is_connect == true && is_startup == true && TIM2->CNT - connected_time > 3000)
    // {
    //   if(pwm_update_count != 40 && pwm_update_count != 39 && pwm_update_count != 41)
    //   {
    //     printf("ERROR PWM UPDATE %d\r\n",pwm_update_count);
    //     while(1){};
    //   }
    // }

    pwm_update_count_last = pwm_update_count;
    loop_count_last = loop_count;
    uart_callback_count_last = uart_callback_count;

    pwm_update_count = 0;
    loop_count = 0;
    uart_callback_count = 0;

    tx[29] = crc8(tx+2, TDATA_SIZE-3);

    HAL_UART_Transmit(&huart2, tx, TDATA_SIZE, 0xffff);
  }
}
