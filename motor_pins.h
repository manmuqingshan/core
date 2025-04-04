/*
  motor_pins.h - pin mappings resolver for ganged/squared/ABC axes

  NOTE: This file is not used by the core, it may be used by drivers to simplify board map files

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#if N_GANGED

#if N_GANGED > N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#if N_AUTO_SQUARED
#define SQUARING_ENABLED
#endif

#if N_GANGED
#define GANGING_ENABLED
#endif

#if Z_GANGED
#define Z_DOUBLED N_ABC_MOTORS
#elif Y_GANGED
#define Y_DOUBLED N_ABC_MOTORS
#elif X_GANGED
#define X_DOUBLED N_ABC_MOTORS
#endif

#if Y_GANGED && !defined(Y_DOUBLED)
#define Y_DOUBLED (N_ABC_MOTORS - 1)
#elif X_GANGED && !defined(X_DOUBLED)
#define X_DOUBLED (N_ABC_MOTORS - 1)
#endif

#if X_GANGED && !defined(X_DOUBLED)
#define X_DOUBLED (N_ABC_MOTORS - 2)
#endif

#if X_DOUBLED == 1

#ifdef A_AXIS
#error "A-axis motor is used for ganged X motor"
#endif
#define X2_STEP_PORT        M3_STEP_PORT
#define X2_STEP_PIN         M3_STEP_PIN
#define X2_STEP_BIT         (1<<M3_STEP_PIN)
#define X2_DIRECTION_PORT   M3_DIRECTION_PORT
#define X2_DIRECTION_PIN    M3_DIRECTION_PIN
#define X2_DIRECTION_BIT    (1<<M3_DIRECTION_PIN)
#ifdef M3_HOME_PIN
 #if X_AUTO_SQUARE
  #define X2_HOME_PORT      M3_HOME_PORT
  #define X2_HOME_PIN       M3_HOME_PIN
  #define X2_HOME_BIT       (1<<M3_HOME_PIN)
 #endif
#elif X_AUTO_SQUARE && defined(X_HOME_PIN)
  #error "Auto squared X-axis requires second home pin input"
#endif
#ifdef M3_LIMIT_PIN
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT     M3_LIMIT_PORT
  #define X2_LIMIT_PIN      M3_LIMIT_PIN
  #define X2_LIMIT_BIT      (1<<M3_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !defined(M3_LIMIT_PIN_MAX)
  #define X_LIMIT_PORT_MAX  M3_LIMIT_PORT
  #define X_LIMIT_PIN_MAX   M3_LIMIT_PIN
  #define X_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared X-axis requires second limit pin input"
#endif
#ifdef M3_LIMIT_PIN_MAX
  #define X_LIMIT_PORT_MAX  M3_LIMIT_PORT_MAX
  #define X_LIMIT_PIN_MAX   M3_LIMIT_PIN_MAX
  #define X_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN_MAX)
#endif
#ifdef M3_ENABLE_PIN
  #define X2_ENABLE_PORT    M3_ENABLE_PORT
  #define X2_ENABLE_PIN     M3_ENABLE_PIN
  #define X2_ENABLE_BIT     (1<<M3_ENABLE_PIN)
#endif
#ifdef M3_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_PORT   M3_MOTOR_FAULT_PORT
  #define X2_MOTOR_FAULT_PIN    M3_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_BIT    (1<<M3_MOTOR_FAULT_PIN)
#endif

#elif X_DOUBLED == 2

#ifdef B_AXIS
#error "B-axis motor is used for ganged X motor"
#endif
#define X2_STEP_PORT        M4_STEP_PORT
#define X2_STEP_PIN         M4_STEP_PIN
#define X2_STEP_BIT         (1<<M4_STEP_PIN)
#define X2_DIRECTION_PORT   M4_DIRECTION_PORT
#define X2_DIRECTION_PIN    M4_DIRECTION_PIN
#define X2_DIRECTION_BIT    (1<<M4_DIRECTION_PIN)
#ifdef M4_HOME_PIN
 #if X_AUTO_SQUARE
  #define X2_HOME_PORT      M4_HOME_PORT
  #define X2_HOME_PIN       M4_HOME_PIN
  #define X2_HOME_BIT       (1<<M4_HOME_PIN)
 #endif
#elif X_AUTO_SQUARE && defined(X_HOME_PIN)
  #error "Auto squared X-axis requires second home pin input"
#endif
#ifdef M4_LIMIT_PIN
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT     M4_LIMIT_PORT
  #define X2_LIMIT_PIN      M4_LIMIT_PIN
  #define X2_LIMIT_BIT      (1<<M4_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !defined(M4_LIMIT_PIN_MAX)
  #define X_LIMIT_PORT_MAX  M4_LIMIT_PORT
  #define X_LIMIT_PIN_MAX   M4_LIMIT_PIN
  #define X_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared X-axis requires second limit pin input"
#endif
#ifdef M4_LIMIT_PIN_MAX
  #define X_LIMIT_PORT_MAX  M4_LIMIT_PORT_MAX
  #define X_LIMIT_PIN_MAX   M4_LIMIT_PIN_MAX
  #define X_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN_MAX)
#endif
#ifdef M4_ENABLE_PIN
  #define X2_ENABLE_PORT    M4_ENABLE_PORT
  #define X2_ENABLE_PIN     M4_ENABLE_PIN
  #define X2_ENABLE_BIT     (1<<M4_ENABLE_PIN)
#endif
#ifdef M4_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_PORT   M4_MOTOR_FAULT_PORT
  #define X2_MOTOR_FAULT_PIN    M4_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_BIT    (1<<M4_MOTOR_FAULT_PIN)
#endif

#elif X_DOUBLED == 3

#ifdef C_AXIS
#error "C-axis motor is used for ganged X motor"
#endif
#define X2_STEP_PORT        M5_STEP_PORT
#define X2_STEP_PIN         M5_STEP_PIN
#define X2_STEP_BIT         (1<<M5_STEP_PIN)
#define X2_DIRECTION_PORT   M5_DIRECTION_PORT
#define X2_DIRECTION_PIN    M5_DIRECTION_PIN
#define X2_DIRECTION_BIT    (1<<M5_DIRECTION_PIN)
#ifdef M5_HOME_PIN
 #if X_AUTO_SQUARE
  #define X2_HOME_PORT      M5_HOME_PORT
  #define X2_HOME_PIN       M5_HOME_PIN
  #define X2_HOME_BIT       (1<<M5_HOME_PIN)
 #endif
#elif X_AUTO_SQUARE && defined(X_HOME_PIN)
  #error "Auto squared X-axis requires second home pin input"
#endif
#ifdef M5_LIMIT_PIN
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT     M5_LIMIT_PORT
  #define X2_LIMIT_PIN      M5_LIMIT_PIN
  #define X2_LIMIT_BIT      (1<<M5_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !defined(M5_LIMIT_PIN_MAX)
  #define X_LIMIT_PORT_MAX  M5_LIMIT_PORT
  #define X_LIMIT_PIN_MAX   M5_LIMIT_PIN
  #define X_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared X-axis requires second limit pin input"
#endif
#ifdef M5_LIMIT_PIN_MAX
  #define X_LIMIT_PORT_MAX  M5_LIMIT_PORT_MAX
  #define X_LIMIT_PIN_MAX   M5_LIMIT_PIN_MAX
  #define X_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN_MAX)
#endif
#ifdef M5_ENABLE_PIN
  #define X2_ENABLE_PORT    M5_ENABLE_PORT
  #define X2_ENABLE_PIN     M5_ENABLE_PIN
  #define X2_ENABLE_BIT     (1<<M5_ENABLE_PIN)
#endif
#ifdef M5_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_PORT   M5_MOTOR_FAULT_PORT
  #define X2_MOTOR_FAULT_PIN    M5_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_BIT    (1<<M5_MOTOR_FAULT_PIN)
#endif

#elif X_DOUBLED == 4

#ifdef U_AXIS
#error "U-axis motor is used for ganged X motor"
#endif
#define X2_STEP_PORT        M6_STEP_PORT
#define X2_STEP_PIN         M6_STEP_PIN
#define X2_STEP_BIT         (1<<M6_STEP_PIN)
#define X2_DIRECTION_PORT   M6_DIRECTION_PORT
#define X2_DIRECTION_PIN    M6_DIRECTION_PIN
#define X2_DIRECTION_BIT    (1<<M6_DIRECTION_PIN)
#ifdef M6_HOME_PIN
 #if X_AUTO_SQUARE
  #define X2_HOME_PORT      M6_HOME_PORT
  #define X2_HOME_PIN       M6_HOME_PIN
  #define X2_HOME_BIT       (1<<M6_HOME_PIN)
 #endif
#elif X_AUTO_SQUARE && defined(X_HOME_PIN)
  #error "Auto squared X-axis requires second home pin input"
#endif
#ifdef M6_LIMIT_PIN
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT     M6_LIMIT_PORT
  #define X2_LIMIT_PIN      M6_LIMIT_PIN
  #define X2_LIMIT_BIT      (1<<M6_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !defined(M6_LIMIT_PIN_MAX)
  #define X_LIMIT_PORT_MAX  M6_LIMIT_PORT
  #define X_LIMIT_PIN_MAX   M6_LIMIT_PIN
  #define X_LIMIT_BIT_MAX   (1<<M6_LIMIT_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared X-axis requires second limit pin input"
#endif
#ifdef M6_LIMIT_PIN_MAX
  #define X_LIMIT_PORT_MAX  M6_LIMIT_PORT_MAX
  #define X_LIMIT_PIN_MAX   M6_LIMIT_PIN_MAX
  #define X_LIMIT_BIT_MAX   (1<<M6_LIMIT_PIN_MAX)
#endif
#ifdef M6_ENABLE_PIN
  #define X2_ENABLE_PORT    M6_ENABLE_PORT
  #define X2_ENABLE_PIN     M6_ENABLE_PIN
  #define X2_ENABLE_BIT     (1<<M6_ENABLE_PIN)
#endif
#ifdef M6_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_PORT   M6_MOTOR_FAULT_PORT
  #define X2_MOTOR_FAULT_PIN    M6_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_BIT    (1<<M6_MOTOR_FAULT_PIN)
#endif

#elif X_DOUBLED == 5

#ifdef V_AXIS
#error "V-axis motor is used for ganged X motor"
#endif
#define X2_STEP_PORT        M7_STEP_PORT
#define X2_STEP_PIN         M7_STEP_PIN
#define X2_STEP_BIT         (1<<M7_STEP_PIN)
#define X2_DIRECTION_PORT   M7_DIRECTION_PORT
#define X2_DIRECTION_PIN    M7_DIRECTION_PIN
#define X2_DIRECTION_BIT    (1<<M7_DIRECTION_PIN)
#ifdef M7_HOME_PIN
 #if X_AUTO_SQUARE
  #define X2_HOME_PORT      M7_HOME_PORT
  #define X2_HOME_PIN       M7_HOME_PIN
  #define X2_HOME_BIT       (1<<M7_HOME_PIN)
 #endif
#elif X_AUTO_SQUARE && defined(X_HOME_PIN)
  #error "Auto squared X-axis requires second home pin input"
#endif
#ifdef M7_LIMIT_PIN
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT     M7_LIMIT_PORT
  #define X2_LIMIT_PIN      M7_LIMIT_PIN
  #define X2_LIMIT_BIT      (1<<M7_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !defined(M7_LIMIT_PIN_MAX)
  #define X_LIMIT_PORT_MAX  M7_LIMIT_PORT
  #define X_LIMIT_PIN_MAX   M7_LIMIT_PIN
  #define X_LIMIT_BIT_MAX   (1<<M7_LIMIT_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared X-axis requires second limit pin input"
#endif
#ifdef M7_LIMIT_PIN_MAX
  #define X_LIMIT_PORT_MAX  M7_LIMIT_PORT_MAX
  #define X_LIMIT_PIN_MAX   M7_LIMIT_PIN_MAX
  #define X_LIMIT_BIT_MAX   (1<<M7_LIMIT_PIN_MAX)
#endif
#ifdef M7_ENABLE_PIN
  #define X2_ENABLE_PORT    M7_ENABLE_PORT
  #define X2_ENABLE_PIN     M7_ENABLE_PIN
  #define X2_ENABLE_BIT     (1<<M7_ENABLE_PIN)
#endif
#ifdef M7_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_PORT   M7_MOTOR_FAULT_PORT
  #define X2_MOTOR_FAULT_PIN    M7_MOTOR_FAULT_PIN
  #define X2_MOTOR_FAULT_BIT    (1<<M7_MOTOR_FAULT_PIN)
#endif

#endif // X_DOUBLED

#if Y_DOUBLED == 1

#ifdef A_AXIS
#error "A-axis motor is used for ganged Y motor"
#endif
#define Y2_STEP_PORT        M3_STEP_PORT
#define Y2_STEP_PIN         M3_STEP_PIN
#define Y2_STEP_BIT         (1<<M3_STEP_PIN)
#define Y2_DIRECTION_PORT   M3_DIRECTION_PORT
#define Y2_DIRECTION_PIN    M3_DIRECTION_PIN
#define Y2_DIRECTION_BIT    (1<<M3_DIRECTION_PIN)
#ifdef M3_HOME_PIN
 #if Y_AUTO_SQUARE
  #define Y2_HOME_PORT      M3_HOME_PORT
  #define Y2_HOME_PIN       M3_HOME_PIN
  #define Y2_HOME_BIT       (1<<M3_HOME_PIN)
 #endif
#elif Y_AUTO_SQUARE && defined(Y_HOME_PIN)
  #error "Auto squared Y-axis requires second home pin input"
#endif
#ifdef M3_LIMIT_PIN
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT     M3_LIMIT_PORT
  #define Y2_LIMIT_PIN      M3_LIMIT_PIN
  #define Y2_LIMIT_BIT      (1<<M3_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !defined(M3_LIMIT_PIN_MAX)
  #define Y_LIMIT_PORT_MAX  M3_LIMIT_PORT
  #define Y_LIMIT_PIN_MAX   M3_LIMIT_PIN
  #define Y_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#ifdef M3_LIMIT_PIN_MAX
  #define Y_LIMIT_PORT_MAX  M3_LIMIT_PORT_MAX
  #define Y_LIMIT_PIN_MAX   M3_LIMIT_PIN_MAX
  #define Y_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN_MAX)
#endif
#ifdef M3_ENABLE_PIN
  #define Y2_ENABLE_PORT    M3_ENABLE_PORT
  #define Y2_ENABLE_PIN     M3_ENABLE_PIN
  #define Y2_ENABLE_BIT     (1<<M3_ENABLE_PIN)
#endif
#ifdef M3_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_PORT   M3_MOTOR_FAULT_PORT
  #define Y2_MOTOR_FAULT_PIN    M3_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_BIT    (1<<M3_MOTOR_FAULT_PIN)
#endif

#elif Y_DOUBLED == 2

#ifdef B_AXIS
#error "B-axis motor is used for ganged Y motor"
#endif
#define Y2_STEP_PORT        M4_STEP_PORT
#define Y2_STEP_PIN         M4_STEP_PIN
#define Y2_STEP_BIT         (1<<M4_STEP_PIN)
#define Y2_DIRECTION_PORT   M4_DIRECTION_PORT
#define Y2_DIRECTION_PIN    M4_DIRECTION_PIN
#define Y2_DIRECTION_BIT    (1<<M4_DIRECTION_PIN)
#ifdef M4_HOME_PIN
 #if Y_AUTO_SQUARE
  #define Y2_HOME_PORT      M4_HOME_PORT
  #define Y2_HOME_PIN       M4_HOME_PIN
  #define Y2_HOME_BIT       (1<<M4_HOME_PIN)
 #endif
#elif Y_AUTO_SQUARE && defined(Y_HOME_PIN)
  #error "Auto squared Y-axis requires second home pin input"
#endif
#ifdef M4_LIMIT_PIN
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT     M4_LIMIT_PORT
  #define Y2_LIMIT_PIN      M4_LIMIT_PIN
  #define Y2_LIMIT_BIT      (1<<M4_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !defined(M4_LIMIT_PIN_MAX)
  #define Y_LIMIT_PORT_MAX  M4_LIMIT_PORT
  #define Y_LIMIT_PIN_MAX   M4_LIMIT_PIN
  #define Y_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#ifdef M4_LIMIT_PIN_MAX
  #define Y_LIMIT_PORT_MAX  M4_LIMIT_PORT_MAX
  #define Y_LIMIT_PIN_MAX   M4_LIMIT_PIN_MAX
  #define Y_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN_MAX)
#endif
#ifdef M4_ENABLE_PIN
  #define Y2_ENABLE_PORT    M4_ENABLE_PORT
  #define Y2_ENABLE_PIN     M4_ENABLE_PIN
  #define Y2_ENABLE_BIT     (1<<M4_ENABLE_PIN)
#endif
#ifdef M4_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_PORT   M4_MOTOR_FAULT_PORT
  #define Y2_MOTOR_FAULT_PIN    M4_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_BIT    (1<<M4_MOTOR_FAULT_PIN)
#endif

#elif Y_DOUBLED == 3

#ifdef C_AXIS
#error "C-axis motor is used for ganged Y motor"
#endif
#define Y2_STEP_PORT        M5_STEP_PORT
#define Y2_STEP_PIN         M5_STEP_PIN
#define Y2_STEP_BIT         (1<<M5_STEP_PIN)
#define Y2_DIRECTION_PORT   M5_DIRECTION_PORT
#define Y2_DIRECTION_PIN    M5_DIRECTION_PIN
#define Y2_DIRECTION_BIT    (1<<M5_DIRECTION_PIN)
#ifdef M5_HOME_PIN
 #if Y_AUTO_SQUARE
  #define Y2_HOME_PORT      M5_HOME_PORT
  #define Y2_HOME_PIN       M5_HOME_PIN
  #define Y2_HOME_BIT       (1<<M5_HOME_PIN)
 #endif
#elif Y_AUTO_SQUARE && defined(Y_HOME_PIN)
  #error "Auto squared Y-axis requires second home pin input"
#endif
#ifdef M5_LIMIT_PIN
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT     M5_LIMIT_PORT
  #define Y2_LIMIT_PIN      M5_LIMIT_PIN
  #define Y2_LIMIT_BIT      (1<<M5_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !defined(M5_LIMIT_PIN_MAX)
  #define Y_LIMIT_PORT_MAX  M5_LIMIT_PORT
  #define Y_LIMIT_PIN_MAX   M5_LIMIT_PIN
  #define Y_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#ifdef M5_LIMIT_PIN_MAX
  #define Y_LIMIT_PORT_MAX  M5_LIMIT_PORT_MAX
  #define Y_LIMIT_PIN_MAX   M5_LIMIT_PIN_MAX
  #define Y_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN_MAX)
#endif
#ifdef M5_ENABLE_PIN
  #define Y2_ENABLE_PORT    M5_ENABLE_PORT
  #define Y2_ENABLE_PIN     M5_ENABLE_PIN
  #define Y2_ENABLE_BIT     (1<<M5_ENABLE_PIN)
#endif
#ifdef M5_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_PORT   M5_MOTOR_FAULT_PORT
  #define Y2_MOTOR_FAULT_PIN    M5_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_BIT    (1<<M5_MOTOR_FAULT_PIN)
#endif

#elif Y_DOUBLED == 4

#ifdef U_AXIS
#error "U-axis motor is used for ganged Y motor"
#endif
#define Y2_STEP_PORT        M6_STEP_PORT
#define Y2_STEP_PIN         M6_STEP_PIN
#define Y2_STEP_BIT         (1<<M6_STEP_PIN)
#define Y2_DIRECTION_PORT   M6_DIRECTION_PORT
#define Y2_DIRECTION_PIN    M6_DIRECTION_PIN
#define Y2_DIRECTION_BIT    (1<<M6_DIRECTION_PIN)
#ifdef M6_HOME_PIN
 #if Y_AUTO_SQUARE
  #define Y2_HOME_PORT      M6_HOME_PORT
  #define Y2_HOME_PIN       M6_HOME_PIN
  #define Y2_HOME_BIT       (1<<M6_HOME_PIN)
 #endif
#elif Y_AUTO_SQUARE && defined(Y_HOME_PIN)
  #error "Auto squared Y-axis requires second home pin input"
#endif
#ifdef M6_LIMIT_PIN
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT     M6_LIMIT_PORT
  #define Y2_LIMIT_PIN      M6_LIMIT_PIN
  #define Y2_LIMIT_BIT      (1<<M6_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !defined(M6_LIMIT_PIN_MAX)
  #define Y_LIMIT_PORT_MAX  M6_LIMIT_PORT
  #define Y_LIMIT_PIN_MAX   M6_LIMIT_PIN
  #define Y_LIMIT_BIT_MAX   (1<<M6_LIMIT_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#ifdef M6_LIMIT_PIN_MAX
  #define Y_LIMIT_PORT_MAX  M6_LIMIT_PORT_MAX
  #define Y_LIMIT_PIN_MAX   M6_LIMIT_PIN_MAX
  #define Y_LIMIT_BIT_MAX   (1<<M6_LIMIT_PIN_MAX)
#endif
#ifdef M6_ENABLE_PIN
  #define Y2_ENABLE_PORT    M6_ENABLE_PORT
  #define Y2_ENABLE_PIN     M6_ENABLE_PIN
  #define Y2_ENABLE_BIT     (1<<M6_ENABLE_PIN)
#endif
#ifdef M6_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_PORT   M6_MOTOR_FAULT_PORT
  #define Y2_MOTOR_FAULT_PIN    M6_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_BIT    (1<<M6_MOTOR_FAULT_PIN)
#endif

#elif Y_DOUBLED == 5

#ifdef V_AXIS
#error "V-axis motor is used for ganged Y motor"
#endif
#define Y2_STEP_PORT        M7_STEP_PORT
#define Y2_STEP_PIN         M7_STEP_PIN
#define Y2_STEP_BIT         (1<<M7_STEP_PIN)
#define Y2_DIRECTION_PORT   M7_DIRECTION_PORT
#define Y2_DIRECTION_PIN    M7_DIRECTION_PIN
#define Y2_DIRECTION_BIT    (1<<M7_DIRECTION_PIN)
#ifdef M7_HOME_PIN
 #if Y_AUTO_SQUARE
  #define Y2_HOME_PORT      M7_HOME_PORT
  #define Y2_HOME_PIN       M7_HOME_PIN
  #define Y2_HOME_BIT       (1<<M7_HOME_PIN)
 #endif
#elif Y_AUTO_SQUARE && defined(Y_HOME_PIN)
  #error "Auto squared Y-axis requires second home pin input"
#endif
#ifdef M7_LIMIT_PIN
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT     M7_LIMIT_PORT
  #define Y2_LIMIT_PIN      M7_LIMIT_PIN
  #define Y2_LIMIT_BIT      (1<<M7_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !defined(M7_LIMIT_PIN_MAX)
  #define Y_LIMIT_PORT_MAX  M7_LIMIT_PORT
  #define Y_LIMIT_PIN_MAX   M7_LIMIT_PIN
  #define Y_LIMIT_BIT_MAX   (1<<M7_LIMIT_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#ifdef M7_LIMIT_PIN_MAX
  #define Y_LIMIT_PORT_MAX  M7_LIMIT_PORT_MAX
  #define Y_LIMIT_PIN_MAX   M7_LIMIT_PIN_MAX
  #define Y_LIMIT_BIT_MAX   (1<<M7_LIMIT_PIN_MAX)
#endif
#ifdef M7_ENABLE_PIN
  #define Y2_ENABLE_PORT    M7_ENABLE_PORT
  #define Y2_ENABLE_PIN     M7_ENABLE_PIN
  #define Y2_ENABLE_BIT     (1<<M7_ENABLE_PIN)
#endif
#ifdef M7_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_PORT   M7_MOTOR_FAULT_PORT
  #define Y2_MOTOR_FAULT_PIN    M7_MOTOR_FAULT_PIN
  #define Y2_MOTOR_FAULT_BIT    (1<<M7_MOTOR_FAULT_PIN)
#endif

#endif // Y_DOUBLED

#if Z_DOUBLED == 1

#ifdef A_AZIS
#error "A-axis motor is used for ganged Z motor"
#endif
#define Z2_STEP_PORT        M3_STEP_PORT
#define Z2_STEP_PIN         M3_STEP_PIN
#define Z2_STEP_BIT         (1<<M3_STEP_PIN)
#define Z2_DIRECTION_PORT   M3_DIRECTION_PORT
#define Z2_DIRECTION_PIN    M3_DIRECTION_PIN
#define Z2_DIRECTION_BIT    (1<<M3_DIRECTION_PIN)
#ifdef M3_HOME_PIN
 #if Z_AUTO_SQUARE
  #define Z2_HOME_PORT      M3_HOME_PORT
  #define Z2_HOME_PIN       M3_HOME_PIN
  #define Z2_HOME_BIT       (1<<M3_HOME_PIN)
 #endif
#elif Z_AUTO_SQUARE && defined(Y_HOME_PIN)
  #error "Auto squared Z-axis requires second home pin input"
#endif
#ifdef M3_LIMIT_PIN
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT     M3_LIMIT_PORT
  #define Z2_LIMIT_PIN      M3_LIMIT_PIN
  #define Z2_LIMIT_BIT      (1<<M3_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !defined(M3_LIMIT_PIN_MAX)
  #define Z_LIMIT_PORT_MAX  M3_LIMIT_PORT
  #define Z_LIMIT_PIN_MAX   M3_LIMIT_PIN
  #define Z_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Z-axis requires second limit pin input"
#endif
#ifdef M3_LIMIT_PIN_MAX
  #define Z_LIMIT_PORT_MAX  M3_LIMIT_PORT_MAX
  #define Z_LIMIT_PIN_MAX   M3_LIMIT_PIN_MAX
  #define Z_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN_MAX)
#endif
#ifdef M3_ENABLE_PIN
  #define Z2_ENABLE_PORT    M3_ENABLE_PORT
  #define Z2_ENABLE_PIN     M3_ENABLE_PIN
  #define Z2_ENABLE_BIT     (1<<M3_ENABLE_PIN)
#endif
#ifdef M3_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_PORT   M3_MOTOR_FAULT_PORT
  #define Z2_MOTOR_FAULT_PIN    M3_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_BIT    (1<<M3_MOTOR_FAULT_PIN)
#endif

#elif Z_DOUBLED == 2

#ifdef B_AZIS
#error "B-axis motor is used for ganged Z motor"
#endif
#define Z2_STEP_PORT        M4_STEP_PORT
#define Z2_STEP_PIN         M4_STEP_PIN
#define Z2_STEP_BIT         (1<<M4_STEP_PIN)
#define Z2_DIRECTION_PORT   M4_DIRECTION_PORT
#define Z2_DIRECTION_PIN    M4_DIRECTION_PIN
#define Z2_DIRECTION_BIT    (1<<M4_DIRECTION_PIN)
#ifdef M4_HOME_PIN
 #if Z_AUTO_SQUARE
  #define Z2_HOME_PORT      M4_HOME_PORT
  #define Z2_HOME_PIN       M4_HOME_PIN
  #define Z2_HOME_BIT       (1<<M4_HOME_PIN)
 #endif
#elif Z_AUTO_SQUARE && defined(Z_HOME_PIN)
  #error "Auto squared Z-axis requires second home pin input"
#endif
#ifdef M4_LIMIT_PIN
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT     M4_LIMIT_PORT
  #define Z2_LIMIT_PIN      M4_LIMIT_PIN
  #define Z2_LIMIT_BIT      (1<<M4_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !defined(M4_LIMIT_PIN_MAX)
  #define Z_LIMIT_PORT_MAX  M4_LIMIT_PORT
  #define Z_LIMIT_PIN_MAX   M4_LIMIT_PIN
  #define Z_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Z-axis requires second limit pin input"
#endif
#ifdef M4_LIMIT_PIN_MAX
  #define Z_LIMIT_PORT_MAX  M4_LIMIT_PORT_MAX
  #define Z_LIMIT_PIN_MAX   M4_LIMIT_PIN_MAX
  #define Z_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN_MAX)
#endif
#ifdef M4_ENABLE_PIN
  #define Z2_ENABLE_PORT    M4_ENABLE_PORT
  #define Z2_ENABLE_PIN     M4_ENABLE_PIN
  #define Z2_ENABLE_BIT     (1<<M4_ENABLE_PIN)
#endif
#ifdef M4_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_PORT   M4_MOTOR_FAULT_PORT
  #define Z2_MOTOR_FAULT_PIN    M4_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_BIT    (1<<M4_MOTOR_FAULT_PIN)
#endif

#elif Z_DOUBLED == 3

#ifdef C_AZIS
#error "C-axis motor is used for ganged Z motor"
#endif
#define Z2_STEP_PORT        M5_STEP_PORT
#define Z2_STEP_PIN         M5_STEP_PIN
#define Z2_STEP_BIT         (1<<M5_STEP_PIN)
#define Z2_DIRECTION_PORT   M5_DIRECTION_PORT
#define Z2_DIRECTION_PIN    M5_DIRECTION_PIN
#define Z2_DIRECTION_BIT    (1<<M5_DIRECTION_PIN)
#ifdef M5_HOME_PIN
 #if Z_AUTO_SQUARE
  #define Z2_HOME_PORT      M5_HOME_PORT
  #define Z2_HOME_PIN       M5_HOME_PIN
  #define Z2_HOME_BIT       (1<<M5_HOME_PIN)
 #endif
#elif Z_AUTO_SQUARE && defined(Z_HOME_PIN)
  #error "Auto squared Z-axis requires second home pin input"
#endif
#ifdef M5_LIMIT_PIN
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT     M5_LIMIT_PORT
  #define Z2_LIMIT_PIN      M5_LIMIT_PIN
  #define Z2_LIMIT_BIT      (1<<M5_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !defined(M5_LIMIT_PIN_MAX)
  #define Z_LIMIT_PORT_MAX  M5_LIMIT_PORT
  #define Z_LIMIT_PIN_MAX   M5_LIMIT_PIN
  #define Z_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Z-axis requires second limit pin input"
#endif
#ifdef M5_LIMIT_PIN_MAX
  #define Z_LIMIT_PORT_MAX  M5_LIMIT_PORT_MAX
  #define Z_LIMIT_PIN_MAX   M5_LIMIT_PIN_MAX
  #define Z_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN_MAX)
#endif
#ifdef M5_ENABLE_PIN
  #define Z2_ENABLE_PORT    M5_ENABLE_PORT
  #define Z2_ENABLE_PIN     M5_ENABLE_PIN
  #define Z2_ENABLE_BIT     (1<<M5_ENABLE_PIN)
#endif
#ifdef M5_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_PORT   M5_MOTOR_FAULT_PORT
  #define Z2_MOTOR_FAULT_PIN    M5_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_BIT    (1<<M5_MOTOR_FAULT_PIN)
#endif

#elif Z_DOUBLED == 4

#ifdef U_AZIS
#error "U-axis motor is used for ganged Z motor"
#endif
#define Z2_STEP_PORT        M6_STEP_PORT
#define Z2_STEP_PIN         M6_STEP_PIN
#define Z2_STEP_BIT         (1<<M6_STEP_PIN)
#define Z2_DIRECTION_PORT   M6_DIRECTION_PORT
#define Z2_DIRECTION_PIN    M6_DIRECTION_PIN
#define Z2_DIRECTION_BIT    (1<<M6_DIRECTION_PIN)
#ifdef M6_HOME_PIN
 #if Z_AUTO_SQUARE
  #define Z2_HOME_PORT      M6_HOME_PORT
  #define Z2_HOME_PIN       M6_HOME_PIN
  #define Z2_HOME_BIT       (1<<M6_HOME_PIN)
 #endif
#elif Z_AUTO_SQUARE && defined(Z_HOME_PIN)
  #error "Auto squared Z-axis requires second home pin input"
#endif
#ifdef M6_LIMIT_PIN
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT     M6_LIMIT_PORT
  #define Z2_LIMIT_PIN      M6_LIMIT_PIN
  #define Z2_LIMIT_BIT      (1<<M6_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !defined(M6_LIMIT_PIN_MAX)
  #define Z_LIMIT_PORT_MAX  M6_LIMIT_PORT
  #define Z_LIMIT_PIN_MAX   M6_LIMIT_PIN
  #define Z_LIMIT_BIT_MAX   (1<<M6_LIMIT_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Z-axis requires second limit pin input"
#endif
#ifdef M6_LIMIT_PIN_MAX
  #define Z_LIMIT_PORT_MAX  M6_LIMIT_PORT_MAX
  #define Z_LIMIT_PIN_MAX   M6_LIMIT_PIN_MAX
  #define Z_LIMIT_BIT_MAX   (1<<M6_LIMIT_PIN_MAX)
#endif
#ifdef M6_ENABLE_PIN
  #define Z2_ENABLE_PORT    M6_ENABLE_PORT
  #define Z2_ENABLE_PIN     M6_ENABLE_PIN
  #define Z2_ENABLE_BIT     (1<<M6_ENABLE_PIN)
#endif
#ifdef M6_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_PORT   M6_MOTOR_FAULT_PORT
  #define Z2_MOTOR_FAULT_PIN    M6_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_BIT    (1<<M6_MOTOR_FAULT_PIN)
#endif

#elif Z_DOUBLED == 5

#ifdef V_AZIS
#error "V-axis motor is used for ganged Z motor"
#endif
#define Z2_STEP_PORT        M7_STEP_PORT
#define Z2_STEP_PIN         M7_STEP_PIN
#define Z2_STEP_BIT         (1<<M7_STEP_PIN)
#define Z2_DIRECTION_PORT   M7_DIRECTION_PORT
#define Z2_DIRECTION_PIN    M7_DIRECTION_PIN
#define Z2_DIRECTION_BIT    (1<<M7_DIRECTION_PIN)
#ifdef M7_HOME_PIN
 #if Z_AUTO_SQUARE
  #define Z2_HOME_PORT      M7_HOME_PORT
  #define Z2_HOME_PIN       M7_HOME_PIN
  #define Z2_HOME_BIT       (1<<M7_HOME_PIN)
 #endif
#elif Z_AUTO_SQUARE && defined(Z_HOME_PIN)
  #error "Auto squared Z-axis requires second home pin input"
#endif
#ifdef M7_LIMIT_PIN
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT     M7_LIMIT_PORT
  #define Z2_LIMIT_PIN      M7_LIMIT_PIN
  #define Z2_LIMIT_BIT      (1<<M7_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !defined(M7_LIMIT_PIN_MAX)
  #define Z_LIMIT_PORT_MAX  M7_LIMIT_PORT
  #define Z_LIMIT_PIN_MAX   M7_LIMIT_PIN
  #define Z_LIMIT_BIT_MAX   (1<<M7_LIMIT_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Z-axis requires second limit pin input"
#endif
#ifdef M7_LIMIT_PIN_MAX
  #define Z_LIMIT_PORT_MAX  M7_LIMIT_PORT_MAX
  #define Z_LIMIT_PIN_MAX   M7_LIMIT_PIN_MAX
  #define Z_LIMIT_BIT_MAX   (1<<M7_LIMIT_PIN_MAX)
#endif
#ifdef M7_ENABLE_PIN
  #define Z2_ENABLE_PORT    M7_ENABLE_PORT
  #define Z2_ENABLE_PIN     M7_ENABLE_PIN
  #define Z2_ENABLE_BIT     (1<<M7_ENABLE_PIN)
#endif
#ifdef M7_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_PORT   M7_MOTOR_FAULT_PORT
  #define Z2_MOTOR_FAULT_PIN    M7_MOTOR_FAULT_PIN
  #define Z2_MOTOR_FAULT_BIT    (1<<M7_MOTOR_FAULT_PIN)
#endif

#endif // Z_DOUBLED

#ifdef X_DOUBLED
#define X2_MOTOR (X_DOUBLED + 2)
#endif
#ifdef Y_DOUBLED
#define Y2_MOTOR (Y_DOUBLED + 2)
#endif
#ifdef Z_DOUBLED
#define Z2_MOTOR (Z_DOUBLED + 2)
#endif

#endif // N_GANGED

#if defined(X2_HOME_PIN) || defined(Y2_HOME_PIN) || defined(Z2_HOME_PIN)
#define DUAL_HOME_SWITCHES
#ifndef X2_HOME_BIT
#define X2_HOME_BIT 0
#endif
#ifndef Y2_HOME_BIT
#define Y2_HOME_BIT 0
#endif
#ifndef Z2_HOME_BIT
#define Z2_HOME_BIT 0
#endif
#define HOME2_MASK (X2_HOME_BIT|Y2_HOME_BIT|Z2_HOME_BIT)
#define HOME2_MASK_SUM (X2_HOME_BIT+Y2_HOME_BIT+Z2_HOME_BIT)
#else
#define HOME2_MASK 0
#define HOME2_MASK_SUM 0
#endif

#if defined(X2_MOTOR_FAULT_PIN) || defined(Y2_MOTOR_FAULT_PIN) || defined(Z2_MOTOR_FAULT_PIN)
#define DUAL_MOTOR_FAULT_SWITCHES
#ifndef X2_MOTOR_FAULT_BIT
#define X2_MOTOR_FAULT_BIT 0
#endif
#ifndef Y2_MOTOR_FAULT_BIT
#define Y2_MOTOR_FAULT_BIT 0
#endif
#ifndef Z2_MOTOR_FAULT_BIT
#define Z2_MOTOR_FAULT_BIT 0
#endif
#define MOTOR_FAULT2_MASK (X2_MOTOR_FAULT_BIT|Y2_MOTOR_FAULT_BIT|Z2_MOTOR_FAULT_BIT)
#define MOTOR_FAULT2_MASK_SUM (X2_MOTOR_FAULT_BIT+Y2_MOTOR_FAULT_BIT+Z2_MOTOR_FAULT_BIT)
#else
#define MOTOR_FAULT2_MASK 0
#define MOTOR_FAULT2_MASK_SUM 0
#endif

#if defined(X2_LIMIT_PIN) || defined(Y2_LIMIT_PIN) || defined(Z2_LIMIT_PIN)
#define DUAL_LIMIT_SWITCHES
#ifndef X2_LIMIT_BIT
#define X2_LIMIT_BIT 0
#endif
#ifndef Y2_LIMIT_BIT
#define Y2_LIMIT_BIT 0
#endif
#ifndef Z2_LIMIT_BIT
#define Z2_LIMIT_BIT 0
#endif
#define LIMIT2_MASK (X2_LIMIT_BIT|Y2_LIMIT_BIT|Z2_LIMIT_BIT)
#define LIMIT2_MASK_SUM (X2_LIMIT_BIT+Y2_LIMIT_BIT+Z2_LIMIT_BIT)
#else
#define LIMIT2_MASK 0
#define LIMIT2_MASK_SUM 0
#endif

#if defined(X_LIMIT_PIN_MAX) || defined(Y_LIMIT_PIN_MAX) || defined(Z_LIMIT_PIN_MAX) || defined(A_LIMIT_PIN_MAX) || defined(B_LIMIT_PIN_MAX) || defined(C_LIMIT_PIN_MAX)
#define MAX_LIMIT_SWITCHES
#endif

#ifdef A_AXIS

#ifndef M3_AVAILABLE
  #error "A_AXIS pins are not available"
#endif
#define A_STEP_PORT         M3_STEP_PORT
#define A_STEP_PIN          M3_STEP_PIN
#define A_STEP_BIT          (1<<M3_STEP_PIN)
#define A_DIRECTION_PORT    M3_DIRECTION_PORT
#define A_DIRECTION_PIN     M3_DIRECTION_PIN
#define A_DIRECTION_BIT     (1<<M3_DIRECTION_PIN)
#ifdef M3_HOME_PIN
  #define A_HOME_PORT       M3_HOME_PORT
  #define A_HOME_PIN        M3_HOME_PIN
  #define A_HOME_BIT        (1<<M3_HOME_PIN)
#endif
#ifdef M3_LIMIT_PIN
  #define A_LIMIT_PORT      M3_LIMIT_PORT
  #define A_LIMIT_PIN       M3_LIMIT_PIN
  #define A_LIMIT_BIT       (1<<M3_LIMIT_PIN)
#endif
#ifdef M3_LIMIT_PIN_MAX
  #define A_LIMIT_PORT_MAX  M3_LIMIT_PORT_MAX
  #define A_LIMIT_PIN_MAX   M3_LIMIT_PIN_MAX
  #define A_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN_MAX)
#endif
#ifdef M3_ENABLE_PIN
  #define A_ENABLE_PORT     M3_ENABLE_PORT
  #define A_ENABLE_PIN      M3_ENABLE_PIN
  #define A_ENABLE_BIT      (1<<M3_ENABLE_PIN)
#endif
#ifdef M3_MOTOR_FAULT_PIN
  #define A_MOTOR_FAULT_PORT    M3_MOTOR_FAULT_PORT
  #define A_MOTOR_FAULT_PIN     M3_MOTOR_FAULT_PIN
  #define A_MOTOR_FAULT_BIT     (1<<M3_MOTOR_FAULT_PIN)
#endif

#endif // A_AXIS

#ifdef B_AXIS

#ifndef M4_AVAILABLE
  #error "B_AXIS pins are not available"
#endif
#define B_STEP_PORT         M4_STEP_PORT
#define B_STEP_PIN          M4_STEP_PIN
#define B_STEP_BIT          (1<<M4_STEP_PIN)
#define B_DIRECTION_PORT    M4_DIRECTION_PORT
#define B_DIRECTION_PIN     M4_DIRECTION_PIN
#define B_DIRECTION_BIT     (1<<M4_DIRECTION_PIN)
#ifdef M4_HOME_PIN
  #define B_HOME_PORT       M4_HOME_PORT
  #define B_HOME_PIN        M4_HOME_PIN
  #define B_HOME_BIT        (1<<M4_HOME_PIN)
#endif
#ifdef M4_LIMIT_PIN
  #define B_LIMIT_PORT      M4_LIMIT_PORT
  #define B_LIMIT_PIN       M4_LIMIT_PIN
  #define B_LIMIT_BIT       (1<<M4_LIMIT_PIN)
#endif
#ifdef M4_LIMIT_PIN_MAX
  #define B_LIMIT_PORT_MAX  M4_LIMIT_PORT_MAX
  #define B_LIMIT_PIN_MAX   M4_LIMIT_PIN_MAX
  #define B_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN_MAX)
#endif
#ifdef M4_ENABLE_PIN
  #define B_ENABLE_PORT     M4_ENABLE_PORT
  #define B_ENABLE_PIN      M4_ENABLE_PIN
  #define B_ENABLE_BIT      (1<<M4_ENABLE_PIN)
#endif
#ifdef M4_MOTOR_FAULT_PIN
  #define B_MOTOR_FAULT_PORT    M4_MOTOR_FAULT_PORT
  #define B_MOTOR_FAULT_PIN     M4_MOTOR_FAULT_PIN
  #define B_MOTOR_FAULT_BIT     (1<<M4_MOTOR_FAULT_PIN)
#endif

#endif //B_AXIS

#ifdef C_AXIS

#ifndef M5_AVAILABLE
  #error "C_AXIS pins are not available"
#endif
#define C_STEP_PORT         M5_STEP_PORT
#define C_STEP_PIN          M5_STEP_PIN
#define C_STEP_BIT          (1<<M5_STEP_PIN)
#define C_DIRECTION_PORT    M5_DIRECTION_PORT
#define C_DIRECTION_PIN     M5_DIRECTION_PIN
#define C_DIRECTION_BIT     (1<<M5_DIRECTION_PIN)
#ifdef M5_HOME_PIN
  #define C_HOME_PORT       M5_HOME_PORT
  #define C_HOME_PIN        M5_HOME_PIN
  #define C_HOME_BIT        (1<<M5_HOME_PIN)
#endif
#ifdef M5_LIMIT_PIN
  #define C_LIMIT_PORT      M5_LIMIT_PORT
  #define C_LIMIT_PIN       M5_LIMIT_PIN
  #define C_LIMIT_BIT       (1<<M5_LIMIT_PIN)
#endif
#ifdef M5_LIMIT_PIN_MAX
  #define C_LIMIT_PORT_MAX  M5_LIMIT_PORT_MAX
  #define C_LIMIT_PIN_MAX   M5_LIMIT_PIN_MAX
  #define C_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN_MAX)
#endif
#ifdef M5_ENABLE_PIN
  #define C_ENABLE_PORT     M5_ENABLE_PORT
  #define C_ENABLE_PIN      M5_ENABLE_PIN
  #define C_ENABLE_BIT      (1<<M5_ENABLE_PIN)
#endif
#ifdef M5_MOTOR_FAULT_PIN
  #define C_MOTOR_FAULT_PORT    M5_MOTOR_FAULT_PORT
  #define C_MOTOR_FAULT_PIN     M5_MOTOR_FAULT_PIN
  #define C_MOTOR_FAULT_BIT     (1<<M5_MOTOR_FAULT_PIN)
#endif

#endif // C_AXIS

#ifdef U_AXIS

#ifndef M6_AVAILABLE
  #error "U_AXIS pins are not available"
#endif
#define U_STEP_PORT         M6_STEP_PORT
#define U_STEP_PIN          M6_STEP_PIN
#define U_STEP_BIT          (1<<M6_STEP_PIN)
#define U_DIRECTION_PORT    M6_DIRECTION_PORT
#define U_DIRECTION_PIN     M6_DIRECTION_PIN
#define U_DIRECTION_BIT     (1<<M6_DIRECTION_PIN)
#ifdef M6_HOME_PIN
  #define U_HOME_PORT       M6_HOME_PORT
  #define U_HOME_PIN        M6_HOME_PIN
  #define U_HOME_BIT        (1<<M6_HOME_PIN)
#endif
#ifdef M6_LIMIT_PIN
  #define U_LIMIT_PORT      M6_LIMIT_PORT
  #define U_LIMIT_PIN       M6_LIMIT_PIN
  #define U_LIMIT_BIT       (1<<M6_LIMIT_PIN)
#endif
#ifdef M6_LIMIT_PIN_MAX
  #define U_LIMIT_PORT_MAX  M6_LIMIT_PORT_MAX
  #define U_LIMIT_PIN_MAX   M6_LIMIT_PIN_MAX
  #define U_LIMIT_BIT_MAX   (1<<M6_LIMIT_PIN_MAX)
#endif
#ifdef M6_ENABLE_PIN
  #define U_ENABLE_PORT     M6_ENABLE_PORT
  #define U_ENABLE_PIN      M6_ENABLE_PIN
  #define U_ENABLE_BIT      (1<<M6_ENABLE_PIN)
#endif
#ifdef M6_MOTOR_FAULT_PIN
  #define U_MOTOR_FAULT_PORT    M6_MOTOR_FAULT_PORT
  #define U_MOTOR_FAULT_PIN     M6_MOTOR_FAULT_PIN
  #define U_MOTOR_FAULT_BIT     (1<<M6_MOTOR_FAULT_PIN)
#endif

#endif // U_AXIS

#ifdef V_AXIS

#ifndef M7_AVAILABLE
  #error "V_AXIS pins are not available"
#endif
#define V_STEP_PORT         M7_STEP_PORT
#define V_STEP_PIN          M7_STEP_PIN
#define V_STEP_BIT          (1<<M7_STEP_PIN)
#define V_DIRECTION_PORT    M7_DIRECTION_PORT
#define V_DIRECTION_PIN     M7_DIRECTION_PIN
#define V_DIRECTION_BIT     (1<<M7_DIRECTION_PIN)
#ifdef M7_HOME_PIN
  #define V_HOME_PORT       M7_HOME_PORT
  #define V_HOME_PIN        M7_HOME_PIN
  #define V_HOME_BIT        (1<<M7_HOME_PIN)
#endif
#ifdef M7_LIMIT_PIN
  #define V_LIMIT_PORT      M7_LIMIT_PORT
  #define V_LIMIT_PIN       M7_LIMIT_PIN
  #define V_LIMIT_BIT       (1<<M7_LIMIT_PIN)
#endif
#ifdef M7_LIMIT_PIN_MAX
  #define V_LIMIT_PORT_MAX  M7_LIMIT_PORT_MAX
  #define V_LIMIT_PIN_MAX   M7_LIMIT_PIN_MAX
  #define V_LIMIT_BIT_MAX   (1<<M7_LIMIT_PIN_MAX)
#endif
#ifdef M7_ENABLE_PIN
  #define V_ENABLE_PORT     M7_ENABLE_PORT
  #define V_ENABLE_PIN      M7_ENABLE_PIN
  #define V_ENABLE_BIT      (1<<M7_ENABLE_PIN)
#endif
#ifdef M7_MOTOR_FAULT_PIN
  #define V_MOTOR_FAULT_PORT    M7_MOTOR_FAULT_PORT
  #define V_MOTOR_FAULT_PIN     M7_MOTOR_FAULT_PIN
  #define V_MOTOR_FAULT_BIT     (1<<M6_MOTOR_FAULT_PIN)
#endif

#endif // V_AXIS

#ifdef STEP_PORT
#ifndef X_STEP_PORT
#define X_STEP_PORT STEP_PORT
#endif
#ifndef Y_STEP_PORT
#define Y_STEP_PORT STEP_PORT
#endif
#ifndef Z_STEP_PORT
#define Z_STEP_PORT STEP_PORT
#endif
#if defined(A_AXIS) && !defined(A_STEP_PORT)
#define A_STEP_PORT STEP_PORT
#endif
#if defined(B_AXIS) && !defined(B_STEP_PORT)
#define B_STEP_PORT STEP_PORT
#endif
#if defined(C_AXIS) && !defined(C_STEP_PORT)
#define C_STEP_PORT STEP_PORT
#endif
#if defined(U_AXIS) && !defined(U_STEP_PORT)
#define U_STEP_PORT STEP_PORT
#endif
#if defined(V_AXIS) && !defined(V_STEP_PORT)
#define V_STEP_PORT STEP_PORT
#endif
#endif

#ifndef X_STEP_BIT
#define X_STEP_BIT (1<<X_STEP_PIN)
#endif
#ifndef Y_STEP_BIT
#define Y_STEP_BIT (1<<Y_STEP_PIN)
#endif
#ifndef Z_STEP_BIT
#define Z_STEP_BIT (1<<Z_STEP_PIN)
#endif

#ifdef DIRECTION_PORT
#ifndef X_DIRECTION_PORT
#define X_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Y_DIRECTION_PORT
#define Y_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Z_DIRECTION_PORT
#define Z_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(A_AXIS) && !defined(A_DIRECTION_PORT)
#define A_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(B_AXIS) && !defined(B_DIRECTION_PORT)
#define B_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(C_AXIS) && !defined(B_DIRECTION_PORT)
#define C_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(U_AXIS) && !defined(U_DIRECTION_PORT)
#define U_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(V_AXIS) && !defined(V_DIRECTION_PORT)
#define V_DIRECTION_PORT DIRECTION_PORT
#endif
#endif

#ifndef X_DIRECTION_BIT
#define X_DIRECTION_BIT (1<<X_DIRECTION_PIN)
#endif
#ifndef Y_DIRECTION_BIT
#define Y_DIRECTION_BIT (1<<Y_DIRECTION_PIN)
#endif
#ifndef Z_DIRECTION_BIT
#define Z_DIRECTION_BIT (1<<Z_DIRECTION_PIN)
#endif

#ifdef LIMIT_PORT
#ifndef X_LIMIT_PORT
#define X_LIMIT_PORT LIMIT_PORT
#endif
#ifndef Y_LIMIT_PORT
#define Y_LIMIT_PORT LIMIT_PORT
#endif
#ifndef Z_LIMIT_PORT
#define Z_LIMIT_PORT LIMIT_PORT
#endif
#if defined(A_AXIS) && !defined(A_LIMIT_PORT)
#define A_LIMIT_PORT LIMIT_PORT
#endif
#if defined(B_AXIS) && !defined(B_LIMIT_PORT)
#define B_LIMIT_PORT LIMIT_PORT
#endif
#if defined(C_AXIS) && !defined(C_LIMIT_PORT)
#define C_LIMIT_PORT LIMIT_PORT
#endif
#if defined(U_AXIS) && !defined(U_LIMIT_PORT)
#define U_LIMIT_PORT LIMIT_PORT
#endif
#if defined(V_AXIS) && !defined(V_LIMIT_PORT)
#define V_LIMIT_PORT LIMIT_PORT
#endif
#endif

#ifndef X_HOME_BIT
#ifdef X_HOME_PIN
#define X_HOME_BIT (1<<X_HOME_PIN)
#else
#define X_HOME_BIT 0
#endif
#endif
#ifndef Y_HOME_BIT
#ifdef Y_HOME_PIN
#define Y_HOME_BIT (1<<Y_HOME_PIN)
#else
#define Y_HOME_BIT 0
#endif
#endif
#ifndef Z_HOME_BIT
#ifdef Z_HOME_PIN
#define Z_HOME_BIT (1<<Z_HOME_PIN)
#else
#define Z_HOME_BIT 0
#endif
#endif

#ifndef X_MOTOR_FAULT_BIT
#ifdef X_MOTOR_FAULT_PIN
#define X_MOTOR_FAULT_BIT (1<<X_MOTOR_FAULT_PIN)
#else
#define X_MOTOR_FAULT_BIT 0
#endif
#endif
#ifndef Y_MOTOR_FAULT_BIT
#ifdef Y_MOTOR_FAULT_PIN
#define Y_MOTOR_FAULT_BIT (1<<Y_MOTOR_FAULT_PIN)
#else
#define Y_MOTOR_FAULT_BIT 0
#endif
#endif
#ifndef Z_MOTOR_FAULT_BIT
#ifdef Z_MOTOR_FAULT_PIN
#define Z_MOTOR_FAULT_BIT (1<<Z_MOTOR_FAULT_PIN)
#else
#define Z_MOTOR_FAULT_BIT 0
#endif
#endif

#ifndef X_LIMIT_BIT
#define X_LIMIT_BIT (1<<X_LIMIT_PIN)
#endif
#ifndef Y_LIMIT_BIT
#define Y_LIMIT_BIT (1<<Y_LIMIT_PIN)
#endif
#ifndef Z_LIMIT_BIT
#define Z_LIMIT_BIT (1<<Z_LIMIT_PIN)
#endif
#ifndef X_LIMIT_BIT_MAX
#ifdef X_LIMIT_PIN_MAX
#define X_LIMIT_BIT_MAX (1<<X_LIMIT_PIN_MAX)
#else
#define X_LIMIT_BIT_MAX 0
#endif
#endif
#ifndef Y_LIMIT_BIT_MAX
#ifdef Y_LIMIT_PIN_MAX
#define Y_LIMIT_BIT_MAX (1<<Y_LIMIT_PIN_MAX)
#else
#define Y_LIMIT_BIT_MAX 0
#endif
#endif
#ifndef Z_LIMIT_BIT_MAX
#ifdef Z_LIMIT_PIN_MAX
#define Z_LIMIT_BIT_MAX (1<<Z_LIMIT_PIN_MAX)
#else
#define Z_LIMIT_BIT_MAX 0
#endif
#endif
#ifndef A_LIMIT_BIT_MAX
#ifdef A_LIMIT_PIN_MAX
#define A_LIMIT_BIT_MAX (1<<A_LIMIT_PIN_MAX)
#else
#define A_LIMIT_BIT_MAX 0
#endif
#endif
#ifndef B_LIMIT_BIT_MAX
#ifdef B_LIMIT_PIN_MAX
#define B_LIMIT_BIT_MAX (1<<B_LIMIT_PIN_MAX)
#else
#define B_LIMIT_BIT_MAX 0
#endif
#endif
#ifndef C_LIMIT_BIT_MAX
#ifdef C_LIMIT_PIN_MAX
#define C_LIMIT_BIT_MAX (1<<C_LIMIT_PIN_MAX)
#else
#define C_LIMIT_BIT_MAX 0
#endif
#endif
#ifndef U_LIMIT_BIT_MAX
#ifdef U_LIMIT_PIN_MAX
#define U_LIMIT_BIT_MAX (1<<U_LIMIT_PIN_MAX)
#else
#define U_LIMIT_BIT_MAX 0
#endif
#endif
#ifndef V_LIMIT_BIT_MAX
#ifdef V_LIMIT_PIN_MAX
#define V_LIMIT_BIT_MAX (1<<V_LIMIT_PIN_MAX)
#else
#define V_LIMIT_BIT_MAX 0
#endif
#endif

#define LIMIT_MAX_MASK (X_LIMIT_BIT_MAX|Y_LIMIT_BIT_MAX|Z_LIMIT_BIT_MAX|A_LIMIT_BIT_MAX|B_LIMIT_BIT_MAX|C_LIMIT_BIT_MAX|U_LIMIT_BIT_MAX|V_LIMIT_BIT_MAX)
#define LIMIT_MAX_SUM (X_LIMIT_BIT_MAX+Y_LIMIT_BIT_MAX+Z_LIMIT_BIT_MAX+A_LIMIT_BIT_MAX+B_LIMIT_BIT_MAX+C_LIMIT_BIT_MAX+U_LIMIT_BIT_MAX+V_LIMIT_BIT_MAX)

#if !defined(X_ENABLE_BIT) && defined(X_ENABLE_PIN)
#define X_ENABLE_BIT (1<<X_ENABLE_PIN)
#endif
#if !defined(Y_ENABLE_BIT) && defined(Y_ENABLE_PIN)
#define Y_ENABLE_BIT (1<<Y_ENABLE_PIN)
#endif
#if !defined(Z_ENABLE_BIT) && defined(Z_ENABLE_PIN)
#define Z_ENABLE_BIT (1<<Z_ENABLE_PIN)
#endif

#ifndef STEP_MASK
#if N_AXIS == 3
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT)
#elif N_AXIS == 4
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT)
#elif N_AXIS == 5
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT|B_STEP_BIT)
#elif N_AXIS == 6
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT|B_STEP_BIT|C_STEP_BIT)
#elif N_AXIS == 7
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT|B_STEP_BIT|C_STEP_BIT|U_STEP_BIT)
#else
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT|B_STEP_BIT|C_STEP_BIT|U_STEP_BIT|V_STEP_BIT)
#endif
#endif

#ifndef DIRECTION_MASK
#if N_AXIS == 3
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT)
#elif N_AXIS == 4
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT)
#elif N_AXIS == 5
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT|B_DIRECTION_BIT)
#elif N_AXIS == 6
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT|B_DIRECTION_BIT|C_DIRECTION_BIT)
#elif N_AXIS == 7
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT|B_DIRECTION_BIT|C_DIRECTION_BIT|U_DIRECTION_BIT)
#else
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT|B_DIRECTION_BIT|C_DIRECTION_BIT|U_DIRECTION_BIT|V_DIRECTION_BIT)
#endif
#endif

#if defined(STEPPERS_ENABLE_PIN) && !defined(STEPPERS_ENABLE_BIT)
#define STEPPERS_ENABLE_BIT (1<<STEPPERS_ENABLE_PIN)
#endif

#ifndef STEPPERS_ENABLE_MASK

#ifdef STEPPERS_ENABLE_BIT
#define STEPPERS_ENABLE_MASK STEPPERS_ENABLE_BIT
#else

#if N_AXIS >= 4 && !defined(A_ENABLE_BIT)
#define A_ENABLE_BIT 0
#endif
#if N_AXIS >= 5 && !defined(B_ENABLE_BIT)
#define B_ENABLE_BIT 0
#endif
#if N_AXIS >= 6 && !defined(C_ENABLE_BIT)
#define C_ENABLE_BIT 0
#endif

#if N_AXIS == 3
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT)
#elif N_AXIS == 4
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT)
#elif N_AXIS == 5
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT|B_ENABLE_BIT)
#elif N_AXIS == 6
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT|B_ENABLE_BIT|C_ENABLE_BIT)
#elif N_AXIS == 7
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT|B_ENABLE_BIT|C_ENABLE_BIT|U_ENABLE_BIT)
#else
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT|B_ENABLE_BIT|C_ENABLE_BIT|U_ENABLE_BIT|V_ENABLE_BIT)
#endif
#endif

#endif // STEPPERS_ENABLE_MASK

#ifndef HOME_MASK

#define HOME_MASK_BASE (X_HOME_BIT|Y_HOME_BIT|Z_HOME_BIT|HOME2_MASK)
#define HOME_MASK_BASE_SUM (X_HOME_BIT+Y_HOME_BIT+Z_HOME_BIT+HOME2_MASK_SUM)

#if N_AXIS == 3
#define HOME_MASK HOME_MASK_BASE
#define HOME_MASK_SUM HOME_MASK_BASE_SUM
#define HOME_MIN_CAP AXES_BITMASK
#elif N_AXIS == 4
#define HOME_MASK (HOME_MASK_BASE|A_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+A_HOME_BIT)
#elif N_AXIS == 5
#define HOME_MASK (HOME_MASK_BASE|A_HOME_BIT|B_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+A_HOME_BIT+B_HOME_BIT)
#elif N_AXIS == 6
#define HOME_MASK (HOME_MASK_BASE|A_HOME_BIT|B_HOME_BIT|C_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+A_HOME_BIT+B_HOME_BIT+C_HOME_BIT)
#elif N_AXIS == 7
#define HOME_MASK (HOME_MASK_BASE|A_HOME_BIT|B_HOME_BIT|C_HOME_BIT|U_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+A_HOME_BIT+B_HOME_BIT+C_HOME_BIT+U_HOME_BIT)
#else
#define HOME_MASK (HOME_MASK_BASE|A_HOME_BIT|B_HOME_BIT|C_HOME_BIT|U_HOME_BIT|V_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+A_HOME_BIT+B_HOME_BIT+C_HOME_BIT+U_HOME_BIT+V_HOME_BIT)
#endif

#endif // HOME_MASK

#ifndef LIMIT_MASK

#if N_AXIS >=4 && !defined(A_LIMIT_BIT)
#ifdef A_LIMIT_PIN
#define A_LIMIT_BIT (1<<A_LIMIT_PIN)
#else
#define A_LIMIT_BIT 0
#endif
#endif
#if N_AXIS >=5 && !defined(B_LIMIT_BIT)
#ifdef B_LIMIT_PIN
#define B_LIMIT_BIT (1<<B_LIMIT_PIN)
#else
#define B_LIMIT_BIT 0
#endif
#endif
#if N_AXIS >= 6 && !defined(C_LIMIT_BIT)
#ifdef C_LIMIT_PIN
#define C_LIMIT_BIT (1<<C_LIMIT_PIN)
#else
#define C_LIMIT_BIT 0
#endif
#endif
#if N_AXIS >= 7 && !defined(U_LIMIT_BIT)
#ifdef U_LIMIT_PIN
#define U_LIMIT_BIT (1<<U_LIMIT_PIN)
#else
#define U_LIMIT_BIT 0
#endif
#endif
#if N_AXIS == 8 && !defined(V_LIMIT_BIT)
#ifdef V_LIMIT_PIN
#define V_LIMIT_BIT (1<<V_LIMIT_PIN)
#else
#define V_LIMIT_BIT 0
#endif
#endif

#ifdef Z_LIMIT_POLL
#define LIMIT_MASK_BASE (X_LIMIT_BIT|Y_LIMIT_BIT|LIMIT2_MASK|LIMIT_MAX_MASK)
#define LIMIT_MASK_BASE_SUM (X_LIMIT_BIT+Y_LIMIT_BIT+LIMIT2_MASK_SUM+LIMIT_MAX_SUM)
#else
#define LIMIT_MASK_BASE (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|LIMIT2_MASK|LIMIT_MAX_MASK)
#define LIMIT_MASK_BASE_SUM (X_LIMIT_BIT+Y_LIMIT_BIT+Z_LIMIT_BIT+LIMIT2_MASK_SUM+LIMIT_MAX_SUM)
#endif

#if N_AXIS == 3
#define LIMIT_MASK LIMIT_MASK_BASE
#define LIMIT_MASK_SUM LIMIT_MASK_BASE_SUM
#define LIMIT_MIN_CAP AXES_BITMASK
#elif N_AXIS == 4
#define LIMIT_MASK (LIMIT_MASK_BASE|A_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+A_LIMIT_BIT)
#elif N_AXIS == 5
#define LIMIT_MASK (LIMIT_MASK_BASE|A_LIMIT_BIT|B_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+A_LIMIT_BIT+B_LIMIT_BIT)
#elif N_AXIS == 6
#define LIMIT_MASK (LIMIT_MASK_BASE|A_LIMIT_BIT|B_LIMIT_BIT|C_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+A_LIMIT_BIT+B_LIMIT_BIT+C_LIMIT_BIT)
#elif N_AXIS == 7
#define LIMIT_MASK (LIMIT_MASK_BASE|A_LIMIT_BIT|B_LIMIT_BIT|C_LIMIT_BIT|U_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+A_LIMIT_BIT+B_LIMIT_BIT+C_LIMIT_BIT+U_LIMIT_BIT)
#else
#define LIMIT_MASK (LIMIT_MASK_BASE|A_LIMIT_BIT|B_LIMIT_BIT|C_LIMIT_BIT|U_LIMIT_BIT|V_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+A_LIMIT_BIT+B_LIMIT_BIT+C_LIMIT_BIT+U_LIMIT_BIT+V_LIMIT_BIT)
#endif

#endif // LIMIT_MASK

#ifndef MOTOR_FAULT_MASK

#define MOTOR_FAULT_MASK_BASE (X_MOTOR_FAULT_BIT|Y_MOTOR_FAULT_BIT|Z_MOTOR_FAULT_BIT|MOTOR_FAULT2_MASK)
#define MOTOR_FAULT_MASK_BASE_SUM (X_MOTOR_FAULT_BIT+Y_MOTOR_FAULT_BIT+Z_MOTOR_FAULT_BIT+MOTOR_FAULT2_MASK_SUM)

#if N_AXIS == 3
#define MOTOR_FAULT_MASK MOTOR_FAULT_MASK_BASE
#define MOTOR_FAULT_MASK_SUM MOTOR_FAULT_MASK_BASE_SUM
#define MOTOR_FAULT_MIN_CAP AXES_BITMASK
#elif N_AXIS == 4
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|A_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+A_MOTOR_FAULT_BIT)
#elif N_AXIS == 5
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|A_MOTOR_FAULT_BIT|B_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+A_MOTOR_FAULT_BIT+B_MOTOR_FAULT_BIT)
#elif N_AXIS == 6
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|A_MOTOR_FAULT_BIT|B_MOTOR_FAULT_BIT|C_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+A_MOTOR_FAULT_BIT+B_MOTOR_FAULT_BIT+C_MOTOR_FAULT_BIT)
#elif N_AXIS == 7
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|A_MOTOR_FAULT_BIT|B_MOTOR_FAULT_BIT|C_MOTOR_FAULT_BIT|U_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+A_MOTOR_FAULT_BIT+B_MOTOR_FAULT_BIT+C_MOTOR_FAULT_BIT+U_MOTOR_FAULT_BIT)
#else
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|A_MOTOR_FAULT_BIT|B_MOTOR_FAULT_BIT|C_MOTOR_FAULT_BIT|U_MOTOR_FAULT_BIT|V_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+A_MOTOR_FAULT_BIT+B_MOTOR_FAULT_BIT+C_MOTOR_FAULT_BIT+U_MOTOR_FAULT_BIT+V_MOTOR_FAULT_BIT)
#endif

#endif // MOTOR_FAULT_MASK

#ifndef N_GANGED
#define N_GANGED 0
#endif

static void motor_iterator (motor_iterator_callback_ptr callback)
{
    motor_map_t motor;

    if(callback) for(motor.id = 0; motor.id < N_AXIS + N_GANGED; motor.id++)
    {
        if(motor.id < N_AXIS)
            motor.axis = motor.id;
        else switch (motor.id) {
#ifdef X2_MOTOR
            case X2_MOTOR:
                motor.axis = X_AXIS;
                break;
#endif
#ifdef Y2_MOTOR
            case Y2_MOTOR:
                motor.axis = Y_AXIS;
                break;
#endif
#ifdef Z2_MOTOR
            case Z2_MOTOR:
                motor.axis = Z_AXIS;
                break;
#endif
        }
        callback(motor);
    }
}

static inline limit_signals_t get_limits_cap (void)
{
    limit_signals_t limits = {0};

#if X_LIMIT_BIT
    limits.min.x = On;
#endif
#if Y_LIMIT_BIT
    limits.min.y = On;
#endif
#if Z_LIMIT_BIT
    limits.min.z = On;
#endif
#if A_LIMIT_BIT
    limits.min.a = On;
#endif
#if B_LIMIT_BIT
    limits.min.b = On;
#endif
#if C_LIMIT_BIT
    limits.min.c = On;
#endif
#if U_LIMIT_BIT
    limits.min.u = On;
#endif
#if V_LIMIT_BIT
    limits.min.v = On;
#endif

#if X2_LIMIT_BIT
    limits.min2.x = On;
#endif
#if Y2_LIMIT_BIT
    limits.min2.y = On;
#endif
#if Z2_LIMIT_BIT
    limits.min2.z = On;
#endif

#if X_LIMIT_BIT_MAX
    limits.max.x = On;
#endif
#if Y_LIMIT_BIT_MAX
    limits.max.y = On;
#endif
#if Z_LIMIT_BIT_MAX
    limits.max.z = On;
#endif
#if A_LIMIT_BIT_MAX
    limits.max.a = On;
#endif
#if B_LIMIT_BIT_MAX
    limits.max.b = On;
#endif
#if C_LIMIT_BIT_MAX
    limits.max.c = On;
#endif
#if U_LIMIT_BIT_MAX
    limits.max.u = On;
#endif
#if V_LIMIT_BIT_MAX
    limits.max.v = On;
#endif

    return limits;
}

static inline home_signals_t get_home_cap (void)
{
    home_signals_t home = {0};

#if HOME_MASK

#if X_HOME_BIT
    home.a.x = On;
#endif
#if Y_HOME_BIT
    home.a.y = On;
#endif
#if Z_HOME_BIT
    home.a.z = On;
#endif
#if A_HOME_BIT
    home.a.a = On;
#endif
#if B_HOME_BIT
    home.a.b = On;
#endif
#if C_HOME_BIT
    home.a.c = On;
#endif
#if U_HOME_BIT
    home.a.u = On;
#endif
#if V_HOME_BIT
    home.a.v = On;
#endif

#if X2_HOME_BIT
    home.b.x = On;
#endif
#if Y2_HOME_BIT
    home.b.y = On;
#endif
#if Z2_HOME_BIT
    home.b.z = On;
#endif

#endif // HOME_MASK

    return home;
}

static inline home_signals_t get_motor_fault_cap (void)
{
    home_signals_t motor_fault = {0};

#if MOTOR_FAULT_MASK

#if X_MOTOR_FAULT_BIT
    motor_fault.a.x = On;
#endif
#if Y_MOTOR_FAULT_BIT
    motor_fault.a.y = On;
#endif
#if Z_MOTOR_FAULT_BIT
    motor_fault.a.z = On;
#endif
#if A_MOTOR_FAULT_BIT
    motor_fault.a.a = On;
#endif
#if B_MOTOR_FAULT_BIT
    motor_fault.a.b = On;
#endif
#if C_MOTOR_FAULT_BIT
    motor_fault.a.c = On;
#endif
#if U_MOTOR_FAULT_BIT
    motor_fault.a.u = On;
#endif
#if V_MOTOR_FAULT_BIT
    motor_fault.a.v = On;
#endif

#if X2_MOTOR_FAULT_BIT
    motor_fault.b.x = On;
#endif
#if Y2_MOTOR_FAULT_BIT
    motor_fault.b.y = On;
#endif
#if Z2_MOTOR_FAULT_BIT
    motor_fault.b.z = On;
#endif

#endif // MOTOR_FAULT_MASK

    return motor_fault;
}

/*EOF*/
