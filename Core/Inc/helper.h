/**
  ******************************************************************************
  * @file           : helper.h
  * @brief          : Header for helper.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#ifndef __HELPER_H
#define __HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "bmi270.h"

void bmi2_error_codes_print_result(int8_t rslt);



#ifdef __cplusplus
}
#endif

#endif



