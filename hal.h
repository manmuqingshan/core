/*
  hal.h - HAL (Hardware Abstraction Layer) entry points structures and capabilities type

  Part of grblHAL

  Copyright (c) 2016-2025 Terje Io

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

/*! \file
    \brief HAL function pointers and data structures definitions.
*/

#ifndef _HAL_H_
#define _HAL_H_

#include <time.h>

#include "grbl.h"
#include "core_handlers.h"
#include "gcode.h"
#include "coolant_control.h"
#include "spindle_control.h"
#include "crossbar.h"
#include "stepper.h"
#include "nvs.h"
#include "probe.h"
#include "ioports.h"
#include "rgb.h"
#include "plugins.h"

#define HAL_VERSION 10

// Bitmap flags for driver capabilities, to be set by driver in driver_init() or plugins,
// flags may be cleared by plugins or the core to switch off capabilities.
typedef union {
    uint32_t value; //!< All bitmap flags.
    struct {
        uint32_t software_debounce         :1, //!< Software debounce of input switches signals is supported.
                 step_pulse_delay          :1, //!< Stepper step pulse delay is supported.
                 limits_pull_up            :1, //!< Pullup resistors for limit inputs are are supported.
                 control_pull_up           :1, //!< Pullup resistors for control inputs are supported.
                 probe_pull_up             :1, //!< Pullup resistors for probe inputs are supported.
                 amass_level               :2, // 0...3 Deprecated?
                 spindle_encoder           :1, //!< Spindle encoder is supported.
                 spindle_sync              :1, //!< Spindle synced motion is supported.
                 sd_card                   :1,
                 littlefs                  :1,
                 bluetooth                 :1,
                 ethernet                  :1,
                 wifi                      :1,
                 spindle_pid               :1,
                 mpg_mode                  :1,
                 laser_ppi_mode            :1, //!< Laser PPI (Pulses Per Inch) mode is supported.
                 atc                       :1, //!< Automatic tool changer (ATC) is supported.
                 no_gcode_message_handling :1,
                 odometers                 :1,
                 pwm_spindle               :1,
                 probe_latch               :1, //!< Deprecated.
                 probe                     :1, //!< Primary (default) probe input is supported.
                 probe2                    :1, //!< 2nd or 3rd probe input is supported.
                 toolsetter                :1, //!< Toolsetter (2nd probe) input is supported.
                 rtc                       :1,
                 rtc_set                   :1,
                 bltouch_probe             :1,
                 unassigned                :5;
    };
} driver_cap_t;


/*! \brief Pointer to function called to set up driver peripherals after settings are loaded. */
typedef bool (*driver_setup_ptr)(settings_t *settings);

/*! \brief Pointer to function to be called when a soft reset occurs. */
typedef void (*driver_reset_ptr)(void);

/*! \brief Pointer to function for getting free memory (as sum of all free blocks in the heap). */
typedef uint32_t (*get_free_mem_ptr)(void);

/*! \brief Pointer to function for registering information about a peripheral pin.
\param pin as periph_pin_t struct containing pin information.
*/
typedef void (*register_periph_pin_ptr)(const periph_pin_t *pin);

/*! \brief Pointer to function for setting pin description for a peripheral pin.
\param function as #pin_function_t enum.
\param group as #pin_group_t enum.
\param description pointer to null terminated description string.
*/
typedef void (*set_periph_pin_description_ptr)(const pin_function_t function, const pin_group_t group, const char *description);

typedef struct {
    register_periph_pin_ptr register_pin;               //!< Opional handler for registering information about a peripheral pin (with the driver).
    set_periph_pin_description_ptr set_pin_description; //!< Optional handler for setting a description of a peripheral pin.
} periph_port_t;

/*! \brief Pointer to callback function for pin enumerations.
\param pin pointer to the \a xbar_t structure holding the pin information.
\param data pointer to \a void that may hold data data useful to the callback function.
*/
typedef void (*pin_info_ptr)(xbar_t *pin, void *data);

/*! \brief Pointer to function for enumerate pin information.
\param low_level true if low level information is required, false if used for reporting.
\param callback pointer to a \a pin_info_ptr type function to receive the pin information.
\param data pointer to \a void that can be used to pass data to the callback function. May be NULL.
The callback function will be called for each pin.
*/
typedef void (*enumerate_pins_ptr)(bool low_level, pin_info_ptr callback, void *data);


/*************
 *  Coolant  *
 *************/

/*! \brief Pointer to function for setting the coolant state.
\param state a \a coolant_state_t union variable.
*/
typedef void (*coolant_set_state_ptr)(coolant_state_t state);

/*! \brief Pointer to function for getting the coolant state.
\returns state in a \a coolant_state_t union variable.
*/
typedef coolant_state_t (*coolant_get_state_ptr)(void);

//! Handlers for coolant support.
typedef struct {
    coolant_set_state_ptr set_state;    //!< Handler for setting coolant state.
    coolant_get_state_ptr get_state;    //!< Handler for getting coolant state.
} coolant_ptrs_t;


/********************
 *  Limit switches  *
 ********************/

/*! \brief Pointer to function for enabling/disabling limit switches functionality.
\param on true to enable limit switches interrupts.
\param homing true when machine is in a homing cycle. Usually ignored by driver code, may be used if Trinamic drivers are supported.
*/
typedef void (*limits_enable_ptr)(bool on, axes_signals_t homing_cycle);

/*! \brief Pointer to function for getting limit switches state.
\returns switch states in a limit_signals_t struct.
*/
typedef limit_signals_t (*limits_get_state_ptr)(void);

/*! \brief Pointer to callback function for reporting limit switches events (interrupts). _Set by the core on startup._
\param state input pin states in a limit_signals_t struct.
*/
typedef void (*limit_interrupt_callback_ptr)(limit_signals_t state);

//! Limit switches handlers
typedef struct {
    limits_enable_ptr enable;                           //!< Handler for enabling limits handling mode.
    limits_get_state_ptr get_state;                     //!< Handler for getting limit switches status.
    limit_interrupt_callback_ptr interrupt_callback;    //!< Callback for informing about limit switches events. _Set by the core at startup._
} limits_ptrs_t;


/************
 *  Homing  *
 ************/

/*! \brief Pointer to function for getting home switches state.
\returns switch states in a home_signals_t struct.
*/
typedef home_signals_t (*home_get_state_ptr)(void);
typedef float (*homing_get_feedrate_ptr)(axes_signals_t axes, homing_mode_t mode);

//! Limit switches handler for homing cycle.
typedef struct {
    home_get_state_ptr get_state;                     //!< Handler for getting homing switches status. Usually read from _hal.limits.get_state_.
    homing_get_feedrate_ptr get_feedrate;
} homing_ptrs_t;

/*****************************
 *  Control signal switches  *
 *****************************/

/*! \brief Pointer to function for getting control switches state.
\returns switch states in a control_signals_t struct.
*/
typedef control_signals_t (*control_signals_get_state_ptr)(void);

/*! \brief Pointer to callback function for reporting control switches events (interrupts). _Set by the core on startup._
\param signals input pin states in a control_signals_t struct.
*/
typedef void (*control_signals_callback_ptr)(control_signals_t signals);

//! Control switches handlers
typedef struct {
    control_signals_get_state_ptr get_state;            //!< Handler for getting limit switches status.
    control_signals_callback_ptr interrupt_callback;    //!< Callback for informing about control switches events. _Set by the core at startup.
} control_signals_ptrs_t;


/**************
 *  Steppers  *
 **************/

/*! \brief Motor vs. axis mapping
__NOTE:__ id and axis values are equal for primary motors, unequal for secondary (ganged) motors.
*/
typedef union {
    uint32_t value;
    struct {
        uint32_t id   : 8,
                 axis : 8;
    };
} motor_map_t;

/*! \brief Signature of the callback function to receive motor vs. axis mappings.
\param motor a motor_map_t struct.
*/
typedef void (*motor_iterator_callback_ptr)(motor_map_t motor);


/*! \brief Pointer to function for iterating over stepper motor vs. axis mappings.

\param callback pointer to a #motor_iterator_callback_ptr function to be called for each motor.
*/
typedef void (*motor_iterator_ptr)(motor_iterator_callback_ptr callback);


/*! \brief Pointer to function for enabling all stepper motors and the main stepper interrupt.

The first interrupt should be generated after a short delay to allow the drivers time to apply power to the motors.
*/
typedef void (*stepper_wake_up_ptr)(void);


/*! \brief Pointer to function for disabling the main stepper interrupt.

\param clear_signals when true stepper motor outputs can be reset to the default state. This parameter can often be ignored.

__NOTE:__ this function will be called from an interrupt context.
*/
typedef void (*stepper_go_idle_ptr)(bool clear_signals);

/*! \brief Pointer to function for enabling/disabling stepper motors.

\param enable a \a axes_signals_t union containing separate flags for each motor to enable/disable.
\param hold a \a true to keep motor powered with reduced current, \a false otherwise.

__NOTE:__ this function may be called from an interrupt context.
*/
typedef void (*stepper_enable_ptr)(axes_signals_t enable, bool hold);

/*! \brief Pointer to function for enabling/disabling step signals for individual motors.

Used by auto squaring functionality where two motors are employed for one or more axes.

\param axes a \a axes_signals_t union containing separate flags for each motor to enable/disable.
\param mode a \a #squaring_mode_t enum for which side to enable/disable.
*/
typedef void (*stepper_disable_motors_ptr)(axes_signals_t axes, squaring_mode_t mode);

/*! \brief Pointer to function for setting the step pulse interrupt rate for the next motion segment.

The driver should limit the maximum time between step interrupts to a few seconds if a 32-bit timer is used.

\param cycles_per_tick number of clock ticks between main stepper interrupts.

__NOTE:__ this function will be called from an interrupt context.
*/
typedef void (*stepper_cycles_per_tick_ptr)(uint32_t cycles_per_tick);

/*! \brief Pointer to function for setting up steppers for the next step pulse.

For plain drivers the most of the information pointed to by the \a stepper argument can be ignored.
The most used fields are these:
<br>\ref stepper.step_outbits
<br>\ref stepper.dir_outbits - these are only necessary to output to the drivers when \ref stepper.dir_change is true.

If the driver is to support spindle synced motion many more needs to be referenced...

\param stepper pointer to a \ref stepper struct containing information about the stepper signals to be output.

__NOTE:__ this function will be called from an interrupt context.
*/
typedef void (*stepper_pulse_start_ptr)(stepper_t *stepper);

/*! \brief Pointer to function for outputting a single step pulse and direction signal.

This is for an experimental implementation of plasma Torch Height Control (THC) and may be removed and/or changed.

\param step_outbits a \a #axes_signals_t union containing the axes to output a step signal for.
\param dir_outbits a \a #axes_signals_t union containing the axes to output a direction signal for.

__NOTE:__ this function will be called from an interrupt context.
*/
typedef void (*stepper_output_step_ptr)(axes_signals_t step_outbits, axes_signals_t dir_outbits);

/*! \brief Pointer to function for getting which axes are configured for auto squaring.

\returns which axes are configured for ganging or auto squaring in an \a axes_signals_t union.
*/
typedef axes_signals_t (*stepper_get_ganged_ptr)(bool auto_squared);

/*! \brief Pointer to function for claiming/releasing motor(s) from/to normal step/dir signalling.

\param axis_id a \a the axis to claim/release motor(s) for.
\param claim \a true to claim a motor(s), \a false to release.
*/
typedef void (*stepper_claim_motor_ptr)(uint_fast8_t axis_id, bool claim);

/*! \brief Pointer to function for querying or resetting stepper driver status.

\param reset \a true to reset status, \a false to query it.
\returns stepper driver warning and fault status in a \a stepper_status_t struct.
*/
typedef stepper_status_t (*stepper_status_ptr)(bool reset);

/*! \brief Pointer to callback function for outputting the next direction and step pulse signals. _Set by the core on startup._

To be called by the driver from the main stepper interrupt handler (when the timer times out).
*/
typedef void (*stepper_interrupt_callback_ptr)(void);

//! Stepper motor handlers
typedef struct {
    stepper_wake_up_ptr wake_up;                        //!< Handler for enabling stepper motor power and main stepper interrupt.
    stepper_go_idle_ptr go_idle;                        //!< Handler for disabling main stepper interrupt and optionally reset stepper signals. Called from interrupt context.
    stepper_enable_ptr enable;                          //!< Handler for enabling/disabling stepper motor power for individual motors. Called from interrupt context.
    stepper_disable_motors_ptr disable_motors;          //!< Optional handler for enabling/disabling stepper motor step signals for individual motors.
    stepper_cycles_per_tick_ptr cycles_per_tick;        //!< Handler for setting the step pulse rate for the next motion segment. Called from interrupt context.
    stepper_pulse_start_ptr pulse_start;                //!< Handler for starting outputting direction signals and a step pulse. Called from interrupt context.
    stepper_interrupt_callback_ptr interrupt_callback;  //!< Callback for informing about the next step pulse to output. _Set by the core at startup._
    stepper_get_ganged_ptr get_ganged;                  //!< Optional handler getting which axes are configured for ganging or auto squaring.
    stepper_claim_motor_ptr claim_motor;                //!< Optional handler for claiming/releasing motor(s) from normal step/dir control.
    stepper_output_step_ptr output_step;                //!< Optional handler for outputting a single step pulse. _Experimental._ Called from interrupt context.
    motor_iterator_ptr motor_iterator;                  //!< Optional handler iteration over motor vs. axis mappings. Required for the motors plugin (Trinamic drivers).
    stepper_status_ptr status;                          //!< Optional handler handler for querying steppper driver status or attempting to reset it.
} stepper_ptrs_t;


/**************
 *  ms delay  *
 **************/

/*! \brief Signature of delay callback functions. */
typedef void (*delay_callback_ptr)(void);

//! Delay struct, currently not used by core - may be used by drivers.
typedef struct {
   volatile uint32_t ms;
   delay_callback_ptr callback;
} delay_t;


/************
 *  Probes  *
 ************/

/*! \brief Pointer to function for getting probe status.
\returns probe state in a \a #probe_state_t union.

__NOTE:__ this function will be called from an interrupt context.
*/
typedef probe_state_t (*probe_get_state_ptr)(void);

/*! \brief Pointer to function for setting probe operation mode.
\param is_probe_away true if probing away from the workpiece, false otherwise. When probing away the signal must be inverted in the probe_get_state_ptr() implementation.
\param probing true if probe cycle is active, false otherwise.
*/
typedef void (*probe_configure_ptr)(bool is_probe_away, bool probing);

/*! \brief Pointer to function for selecting probe input.
\param probe_id probe number. 0 - default probe, 1 - toolsetter, ...
\returns true if selectetion succeded, false otherwise.
*/
typedef bool (*probe_select_ptr)(probe_id_t probe_id);

/*! \brief Pointer to function for toggling probe connected status.


If the driver does not support a probe connected input signal this can be used to implement
toggling of probe connected status via a #CMD_PROBE_CONNECTED_TOGGLE real time command.
*/
typedef void (*probe_connected_toggle_ptr)(void);

//! Handlers for probe input(s).
typedef struct {
    probe_configure_ptr configure;                  //!< Optional handler for setting probe operation mode.
    probe_get_state_ptr get_state;                  //!< Optional handler for getting probe status. Called from interrupt context.
    probe_select_ptr select;                        //!< Optional handler for selecting probe to use.
    probe_connected_toggle_ptr connected_toggle;    //!< Optional handler for toggling probe connected status.
} probe_ptrs_t;


/*******************************
 *  Tool selection and change  *
 *******************************/

typedef enum {
    ATC_None = 0,
    ATC_Offline,
    ATC_Online
} atc_status_t;

/*! \brief Pointer to function for selecting a tool.
\param tool pointer to tool_data_t struct.
\param next \a true if tool is selected for next the next tool change (M6), \a false to as set current tool.
*/
typedef void (*tool_select_ptr)(tool_data_t *tool, bool next);

/*! \brief Pointer to function for executing a tool change.
\param gc_state pointer to parser_state_t struct.
*/
typedef status_code_t (*tool_change_ptr)(parser_state_t *gc_state);

/*! \brief Pointer to function for checking ATC status.
\returns \a true if online.
*/
typedef atc_status_t (*atc_get_state_ptr)(void);

/*! \brief Handlers for tool changes.

If the driver (or a plugin) does not set these handlers the core will set them to its own
handlers for manual or semi-automatic tool change if the current input stream supports
the tool change protocol.
 */
typedef struct {
    tool_select_ptr select;    //!< Optional handler for selecting a tool.
    tool_change_ptr change;    //!< Optional handler for executing a tool change (M6).
    atc_get_state_ptr atc_get_state; //!< Optional handler for checking ATC status.
} tool_ptrs_t;

/*******************
 *  Encoder input  *
 *******************/

/*! \brief Pointer to function for getting number of encoders supported.
\returns number of encoders.
*/
typedef uint8_t (*encoder_get_n_encoders_ptr)(void);

/*! \brief Pointer to callback function to receive encoder events.
\param encoder pointer to a \a encoder_t struct.
\param position encoder position.
*/
typedef void (*encoder_on_event_ptr)(encoder_t *encoder, int32_t position);

/*! \brief Pointer to function for resetting encoder data.
\param id encoder id.
*/
typedef void (*encoder_reset_ptr)(uint_fast8_t id);

typedef struct {
    encoder_get_n_encoders_ptr get_n_encoders;  //!< Optional handler for getting number of encoders supported.
    encoder_on_event_ptr on_event;              //!< Optional callback handler for receiving encoder events.
    encoder_reset_ptr reset;                    //!< Optional handler for resetting data for an encoder.
} encoder_ptrs_t;

/*! \brief Pointer to function for claiming higher level interrupt requests (irq).
\param irq irq type as a #irq_type_t enum value.
\param id irq id, normally 0, > 0 if there are several sources for the same irq type.
\param callback function as a \a irq_callback_ptr function pointer.
*/
typedef bool (*irq_claim_ptr)(irq_type_t irq, uint_fast8_t id, irq_callback_ptr callback);

/************
 *  Timers  *
 ************/

typedef void *hal_timer_t;  //!< Timer handle, actual type defined by driver implementation.

typedef enum {
    Timer_16bit = 0,
    Timer_32bit,
    Timer_64bit
} timer_resolution_t;

typedef union {
    uint8_t value; //!< All bitmap flags.
    struct {
        uint8_t periodic :1, //!<
                up       :1, //!< Timer supports upcounting
                comp1    :1, //!< Timer supports compare interrupt 0
                comp2    :1; //!< Timer supports compare interrupt 1
    };
} timer_cap_t;

typedef void (*timer_irq_handler_ptr)(void *context);

typedef struct {
    void *context;                          //!< Pointer to data to be passed on to the interrupt handlers
    bool single_shot;                       //!< Set to true if timer is single shot
    timer_irq_handler_ptr timeout_callback; //!< Pointer to main timeout callback
    uint32_t irq0;                          //!< Compare value for compare interrupt 0
    timer_irq_handler_ptr irq0_callback;    //!< Pointer to compare interrupt 0 callback
    uint32_t irq1;                          //!< Compare value for compare interrupt 10
    timer_irq_handler_ptr irq1_callback;    //!< Pointer to compare interrupt 1 callback
} timer_cfg_t;

/*! \brief Pointer to function for claiming a timer.
\param cap pointer to a \a timer_cap_t struct containing the required capabilities.
\param timebase timebase in ns.
\returns a \a hal_timer_t pointer if successful, \a NULL if not.
*/
typedef hal_timer_t (*timer_claim_ptr)(timer_cap_t cap, uint32_t timebase);

/*! \brief Pointer to function for configuring a timer.
\param timer a \a hal_timer_t pointer.
\param cfg pointer to a \a timer_cfg_t struct.
\returns \a true if successful.
*/
typedef bool (*timer_cfg_ptr)(hal_timer_t timer, timer_cfg_t *cfg);

/*! \brief Pointer to function for starting a timer.
\param timer a \a hal_timer_t pointer.
\param period delay in.
\returns \a true if successful.
*/
typedef bool (*timer_start_ptr)(hal_timer_t timer, uint32_t period);

/*! \brief Pointer to function for stopping a running timer.
\param timer a \a hal_timer_t pointer.
\returns \a true if successful.
*/
typedef bool (*timer_stop_ptr)(hal_timer_t timer);

typedef struct {
    timer_claim_ptr claim;
    timer_cfg_ptr configure;
    timer_start_ptr start;
    timer_stop_ptr stop;
} timer_ptrs_t;

/**************************
 *  RTC (Real Time Clock  *
 **************************/

/*! \brief Pointer to function for setting the current datetime.
\param datetime pointer to a \a tm struct.
\\returns \a true if successful.
*/
typedef bool (*rtc_get_datetime_ptr)(struct tm *datetime);

/*! \brief Pointer to function for setting the current datetime.
\param datetime pointer to a \a tm struct.
\returns \a true if successful.
*/
typedef bool (*rtc_set_datetime_ptr)(struct tm *datetime);

typedef struct {
    rtc_get_datetime_ptr get_datetime;  //!< Optional handler getting the current datetime.
    rtc_set_datetime_ptr set_datetime;  //!< Optional handler setting the current datetime.
} rtc_ptrs_t;

/**/

/*! \brief Pointer to function for performing a pallet shuttle.
*/
typedef void (*pallet_shuttle_ptr)(void);

/*! \brief HAL structure used for the driver interface.

This structure contains properties and function pointers (to handlers) that the core uses to communicate with the driver.

Required function pointers has to be set up by the driver in the driver_init() function.

__NOTE:__ Those not marked as optional are required. If not assigned by the driver behaviour is undefined and
is likely to cause a controller crash at some point.
*/
typedef struct {
    uint32_t version;               //!< HAL version, _set by the core_. Driver should check against this in the _driver_init()_ function.
    char *info;                     //!< Pointer to driver info string, typically name of processor/platform.
    char *driver_version;           //!< Pointer to driver version date string in YYMMDD format.
    char *driver_options;           //!< Pointer to optional comma separated string with driver options.
    char *driver_url;               //!< Pointer to optional URL for the driver.
    char *board;                    //!< Pointer to optional board name string.
    char *board_url;                //!< Pointer to optional URL for the board.
    float step_us_min;              //!< Minimum step pulse width (microseconds).
    uint32_t f_step_timer;          //!< Frequency of main stepper timer in Hz.
    uint32_t f_mcu;                 //!< Frequency of MCU in MHz.
    uint32_t rx_buffer_size;        //!< Input stream buffer size in bytes.
    uint32_t max_step_rate;         //!< Currently unused.
    uint8_t driver_axis_settings;   //!< Currently unused.

    /*! \brief Optional pointer to function for getting free memory (as sum of all free blocks in the heap). */
    get_free_mem_ptr get_free_mem;

    /*! \brief Driver setup handler.
    Called once by the core after settings has been loaded. The driver should enable MCU peripherals in the provided function.
    \param settings pointer to settings_t structure.
    \returns \a true if completed successfully and the driver supports the _settings->version_ number, false otherwise.
    */
    driver_setup_ptr driver_setup;

    /*! \brief Millisecond delay.

    If the callback parameter is \a NULL the call blocks until the delay has expired otherwise it returns immediately
    and the callback function is called when the timeout expires.

    A pending callback can be terminated by calling the function with the delay set to \a 0 and the callback to \a NULL.
    \param ms number of milliseconds to delay.
    \param callback function as a \a delay_callback_ptr function pointer.
    */
    void (*delay_ms)(uint32_t ms, delay_callback_ptr callback);

    /*! \brief Atomically set bits.
    \param value pointer to 16 bit unsigned integer.
    \param bits bits to set.
    */
    void (*set_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);

    /*! \brief Atomically clear bits.
    \param value pointer to 16 bit unsigned integer.
    \param bits bits to clear.
    \returns value before bits were cleared.
    */
    uint_fast16_t (*clear_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t v);

    /*! \brief Atomically set value.
    \param value pointer to 16 bit unsigned integer.
    \param v value to set.
    \returns value before value were set.
    */
    uint_fast16_t (*set_value_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);

    //! \brief Optional handler to disable global interrupts.
    void (*irq_enable)(void);

    //! \brief Optional handler to enable global interrupts.
    void (*irq_disable)(void);

    //! \brief Optional handler for claiming higher level interrupts. Set to a dummy handler on startup.
    irq_claim_ptr irq_claim;

    limits_ptrs_t limits;                   //!< Handlers for limit switches.
    homing_ptrs_t homing;                   //!< Handlers for homing switches, used by homing cycle.
    control_signals_ptrs_t control;         //!< Handlers for control switches.
    coolant_ptrs_t coolant;                 //!< Handlers for coolant.
    spindle_data_ptrs_t spindle_data;       //!< Handlers for getting/resetting spindle data (RPM, angular position, ...).
    stepper_ptrs_t stepper;                 //!< Handlers for stepper motors.
    io_stream_t stream;                     //!< Handlers for stream I/O.
    settings_changed_ptr settings_changed;  //!< Deprecated, hook into grbl.on_settings_changed in new code.
    probe_ptrs_t probe;                     //!< Optional handlers for probe input(s).
    tool_ptrs_t tool;                       //!< Optional handlers for tool changes.
    timer_ptrs_t timer;                     //!< Optional handlers for claiming and controlling timers.
    rtc_ptrs_t rtc;                         //!< Optional handlers for real time clock (RTC).
    io_port_t port;                         //!< Optional handlers for axuillary I/O (adds support for M62-M66).
    rgb_ptr_t rgb0;                         //!< Optional handler for RGB output to LEDs (neopixels) or lamps.
    rgb_ptr_t rgb1;                         //!< Optional handler for RGB output to LEDs (neopixels) or lamps.
    periph_port_t periph_port;              //!< Optional handlers for peripheral pin registration.
    driver_reset_ptr driver_reset;          //!< Optional handler, called on soft resets. Set to a dummy handler by the core at startup.
    nvs_io_t nvs;                           //!< Optional handlers for storing/retrieving settings and data to/from non-volatile storage (NVS).
    enumerate_pins_ptr enumerate_pins;      //!< Optional handler for enumerating pins used by the driver.
    bool (*driver_release)(void);           //!< Optional handler for releasing hardware resources before exiting.
    uint32_t (*get_elapsed_ticks)(void);    //!< Optional handler for getting number of elapsed 1 ms tics since startup. Rolls over every 49.71 days.  Required by a number of plugins.
    uint64_t (*get_micros)(void);           //!< Optional handler for getting number of elapsed 1 us tics since startup. Rolls over every 1.19 hours. Required by a number of plugins.
    pallet_shuttle_ptr pallet_shuttle;      //!< Optional handler for performing a pallet shuttle on program end (M60).
    void (*reboot)(void);                   //!< Optoional handler for rebooting the controller. This will be called when #ASCII_ESC followed by #CMD_REBOOT is received.

    encoder_ptrs_t encoder;                 //!< Optional handlers for encoder support.

    /*! \brief Optional handler for getting the current axis positions.
    \returns the axis positions in an int32_array.
    */
    bool (*get_position)(int32_t (*position)[N_AXIS]);

#ifdef DEBUGOUT
    void (*debug_out)(bool on);
    io_stream_t debug;                     //!< Handlers for debug stream I/O.
#endif

    /*! \brief Check for a soft reset or abort in blocking calls.
    Set up by core at startup.

    Typically called from stream output functions when the output buffer is full and
    they are blocking waiting for space in the buffer to become available.
    \returns false when the blocking loop should exit, true otherwise.
    */
    bool (*stream_blocking_callback)(void);

    driver_cap_t driver_cap;                        //!< Basic driver capabilities flags.
    control_signals_t signals_cap;                  //!< Control input signals supported by the driver.
    control_signals_t signals_pullup_disable_cap;   //!< Control input signals pullup disable supported by the driver.
    limit_signals_t limits_cap;                     //!< Limit input signals supported by the driver.
    home_signals_t home_cap;                        //!< Home input signals supported by the driver.
    coolant_state_t coolant_cap;                    //!< Coolant outputs supported by the driver.
    home_signals_t motor_warning_cap;               //!< Motor warning input signals (per motor) supported by the driver.
    home_signals_t motor_fault_cap;                 //!< Motor fault input signals (per motor) supported by the driver.
} grbl_hal_t;

extern grbl_hal_t hal; //!< Global HAL struct.

/*! \brief Driver main entry point.
This will be called once by the core after the HAL structure has been nulled and handlers provided by the core is set up.
The driver should initialize all the required handlers and capabilities flags in this function.
On successful return from this function the core will load settings from non-volatile storage (NVS) if handler functions were provided or load default values if not.
Then _hal.driver_setup()_ will be called so that the driver can configure the remaining MCU peripherals.

__NOTE__: This is the only driver function that is called directly from the core, all others are called via HAL function pointers.

\returns \a true if completed successfully and the driver matches the _hal.version number_, \a false otherwise.
*/
extern bool driver_init (void);

#endif
