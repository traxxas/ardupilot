/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/diydrones/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.com/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

#include "Copter.h"

#define SCHED_TASK(func) FUNCTOR_BIND(&copter, &Copter::func, void)
#include <fcntl.h>  // open()
// Drivers
#include <uORB/topics/tpfc_sensors.h>
#include <uORB/topics/tpfc_input_cmd_rsp.h>
#include <board_config.h>
#include <stm32_gpio.h>

#include <systemlib/git_version.h>

#define MAX_DIST_TO_HOME               1000   // 1000cm = 10m

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 2.5ms units) and the maximum time they are expected to take (in
  microseconds)
  1    = 400hz
  2    = 200hz
  4    = 100hz
  8    = 50hz
  20   = 20hz
  40   = 10hz
  133  = 3hz
  400  = 1hz
  4000 = 0.1hz
  
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] PROGMEM = {
    { SCHED_TASK(rc_loop),               4,    130 },
    { SCHED_TASK(throttle_loop),         8,     75 },
    { SCHED_TASK(update_GPS),            8,    200 },
#if OPTFLOW == ENABLED
    { SCHED_TASK(update_optical_flow),   2,    160 },
#endif
    { SCHED_TASK(update_batt_compass),  40,    120 },
    { SCHED_TASK(read_aux_switches),    40,     50 },
    { SCHED_TASK(check_cmd_input),      20,     50 },
    { SCHED_TASK(arm_motors_check),     40,     50 },
    { SCHED_TASK(auto_trim),            40,     75 },
    { SCHED_TASK(update_altitude),      40,    140 },
    { SCHED_TASK(update_ctrl_output),   4,     300 },
    { SCHED_TASK(check_flight_mode),    40,     75 },
    { SCHED_TASK(run_nav_updates),       8,    100 },
    { SCHED_TASK(update_thr_average),    4,     90 },
    { SCHED_TASK(three_hz_loop),       133,     75 },
    { SCHED_TASK(compass_accumulate),    8,    100 },
    { SCHED_TASK(barometer_accumulate),  8,     90 },
#if FRAME_CONFIG == HELI_FRAME
    { SCHED_TASK(check_dynamic_flight),  8,     75 },
#endif
    { SCHED_TASK(update_notify),         8,     90 },
    { SCHED_TASK(one_hz_loop),         400,    100 },
    { SCHED_TASK(ekf_check),            40,     75 },
    { SCHED_TASK(landinggear_update),   40,     75 },
    { SCHED_TASK(lost_vehicle_check),   40,     50 },
    { SCHED_TASK(gcs_check_input),       1,    180 },
    { SCHED_TASK(gcs_send_heartbeat),  400,    110 },
    { SCHED_TASK(gcs_send_deferred),     8,    550 },
    { SCHED_TASK(gcs_data_stream_send),  8,    550 },
    { SCHED_TASK(update_mount),          8,     75 },
    { SCHED_TASK(ten_hz_logging_loop),  40,    350 },
    { SCHED_TASK(fifty_hz_logging_loop), 8,    110 },
    { SCHED_TASK(full_rate_logging_loop),1,    100 },
    { SCHED_TASK(perf_update),        4000,     75 },
    { SCHED_TASK(read_receiver_rssi),   40,     75 },
    { SCHED_TASK(rpm_update),           40,    200 },
#if FRSKY_TELEM_ENABLED == ENABLED
    { SCHED_TASK(frsky_telemetry_send), 80,     75 },
#endif
#if EPM_ENABLED == ENABLED
    { SCHED_TASK(epm_update),           40,     75 },
#endif
#ifdef USERHOOK_FASTLOOP
    { SCHED_TASK(userhook_FastLoop),     4,     75 },
#endif
#ifdef USERHOOK_50HZLOOP
    { SCHED_TASK(userhook_50Hz),         8,     75 },
#endif
#ifdef USERHOOK_MEDIUMLOOP
    { SCHED_TASK(userhook_MediumLoop),  40,     75 },
#endif
#ifdef USERHOOK_SLOWLOOP
    { SCHED_TASK(userhook_SlowLoop),    120,    75 },
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    { SCHED_TASK(userhook_SuperSlowLoop),400,   75 },
#endif
};


__EXPORT int tpfc_send_ekf_data = 1;      // send to FC board?

__EXPORT int tpfc_new_loiter_stop_algorithm = 0;
__EXPORT int tpfc_baro_new_algorithm = 0;

__EXPORT uint32_t tpfc_rtl_brake = 1;
__EXPORT uint32_t tpfc_rtl_brake_duration_ms = 3000;
__EXPORT uint32_t tpfc_rtl_brake_velocity_cms = 300;


__EXPORT int tpfc_display_ekf_data = 0;   // display ekf data to console
__EXPORT int tpfc_display_error_data = 0; // display stats to console
__EXPORT float tpfc_jerk_ratio = 1700.0f;


__EXPORT uint32_t tpfc_mtd_writes = 0;
__EXPORT uint32_t tpfc_mtd_reads  = 0;
__EXPORT uint32_t tpfc_eeprom_writes = 0;
__EXPORT uint32_t tpfc_eeprom_reads  = 0;


uint16_t tpfc_ekf_errors = 0;
uint16_t tpfc_bad_fd_errors = 0;
uint16_t tpfc_ioctl_errors = 0;
uint16_t tpfc_ioctl_success = 0;
const unsigned int tpfc_display_stats_period = 500; // in loops
const unsigned int tpfc_display_data_period  = 1200; // in loops, 1/3Hz



void Copter::setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    tpfc_fd = open(PX4IO_DEVICE_PATH, O_WRONLY);

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = hal.scheduler->micros();
}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Copter::compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
void Copter::barometer_accumulate(void)
{
    barometer.accumulate();
}

void Copter::perf_update(void)
{
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        gcs_send_text_fmt(PSTR("PERF: %u/%u %lu %lu\n"),
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void Copter::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available);
}

uint64_t tpfc_loop_last_timestamp = 0;
uint32_t tpfc_loop_count = 0;


__EXPORT bool tpfc_print_loop_timings = false;
__EXPORT uint16_t tpfc_ekf_health_filter = 0;
__EXPORT uint16_t tpfc_ekf_health_ratio = 0;
__EXPORT uint16_t tpfc_ekf_health_imu = 0;
__EXPORT uint16_t tpfc_ekf_health_innovations = 0;


// Main loop - 400hz
void Copter::fast_loop()
{
    uint64_t now = hal.scheduler->micros64();
    
    if (tpfc_print_loop_timings) {

        tpfc_print_loop_timings = 0;

        printf("Armed state: %s\n", (motors.armed()?"ON":"OFF"));

        printf("Flight mode:%d\n", control_mode);
        
        printf("Takeoff throttle: %d + %d = %f\n",
               channel_throttle->get_control_mid(),
               g.takeoff_trigger_dz,
               get_takeoff_trigger_throttle());

        Vector3f angle_targets = attitude_control.angle_ef_targets();
            
        uint16_t target_yaw = angle_targets.z;

        printf("Target yaw: %6.1f, converted: %d\n", angle_targets.z, target_yaw);

        float hAcc = 0.0f;
        gps.horizontal_accuracy(hAcc);

        printf("EKF/GPS, status:%d primary:%d  hAcc:%f  hdop:%d  sats:%d home dist:%d\n",
               gps.status(),
               gps.primary_sensor(),
               hAcc,
               gps.get_hdop(),
               gps.num_sats(),
               distance_to_home_cm
               );


        printf("ap bit fields: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d 0x%04x\n",
               ap.unused1,
               ap.simple_mode,
               ap.pre_arm_rc_check,
               ap.pre_arm_check,
               ap.auto_armed,
               ap.logging_started,
               ap.land_complete,
               ap.new_radio_frame,
               ap.usb_connected,
               ap.rc_receiver_present,
               ap.compass_mot,
               ap.motor_test,
               ap.initialised,
               ap.land_complete_maybe,
               ap.throttle_zero,
               ap.system_time_set,
               ap.gps_base_pos_set,
               ap.home_state,
               ap.using_interlock,
               ap.motor_emergency_stop,
               ap.value);
               

        
#if 0
        printf("Radio in, yaw:%d, pitch:%d, roll:%d, throttle:%d , Control In yaw:%d, pitch:%d, roll:%d, throttle:%d\n",
               channel_yaw->radio_in,  channel_pitch->radio_in, channel_roll->radio_in, channel_throttle->radio_in,
               channel_yaw->control_in,  channel_pitch->control_in, channel_roll->control_in, channel_throttle->control_in);

        printf("Yaw scale, min:%d, max:%d, trim:%d, reverse:%d dz:%d  high:%d\n",
               channel_yaw->radio_min, channel_yaw->radio_max,
               channel_yaw->radio_trim, (channel_yaw->get_reverse()?-1:1), channel_yaw->_dead_zone, channel_yaw->_high);
        printf("Pitch scale, min:%d, max:%d, trim:%d, reverse:%d dz:%d  high:%d\n",
               channel_pitch->radio_min, channel_pitch->radio_max,
               channel_pitch->radio_trim, (channel_pitch->get_reverse()?-1:1), channel_pitch->_dead_zone, channel_pitch->_high);
        printf("Roll scale, min:%d, max:%d, trim:%d, reverse:%d dz:%d  high:%d\n",
               channel_roll->radio_min, channel_roll->radio_max,
               channel_roll->radio_trim, (channel_roll->get_reverse()?-1:1), channel_roll->_dead_zone, channel_roll->_high);
        printf("Throttle scale, min:%d, max:%d, trim:%d, reverse:%d dz:%d  high:%d\n",
               channel_throttle->radio_min, channel_throttle->radio_max,
               channel_throttle->radio_trim, (channel_throttle->get_reverse()?-1:1), channel_throttle->_dead_zone, channel_throttle->_high);
#endif 
        
        // Recompute now so the printf time will be excluded.
        now = hal.scheduler->micros64();
    }

    tpfc_loop_last_timestamp = now;


    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // run low level rate controllers that only require IMU data
    attitude_control.rate_controller_run();
    
#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
#endif //HELI_FRAME

    // send outputs to the motors library
    motors_output();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

    // Embedded flight control update
    update_tpfc();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_thr_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif
}

// update_mount - update camera mount position
// should be run at 50hz
void Copter::update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif

#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle()/1000.0f);
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS)) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Rate();
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, g.pid_rate_roll.get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, g.pid_rate_pitch.get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, g.pid_rate_yaw.get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration(ins);
    }

    if (should_log(MASK_LOG_FCU) && !should_log(MASK_LOG_FCU_FAST)) {
        uint16_t pSize = 16;
        uint16_t parms[pSize];
    
        if (ioctl(tpfc_fd, TPFC_IOC_FCU_LOG_GET, (unsigned long)parms) == 0)  {
            Log_Write_FCU(parms, pSize);
        }
    }

#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
}

// fifty_hz_logging_loop
// should be run at 50hz
void Copter::fifty_hz_logging_loop()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Rate();
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, g.pid_rate_roll.get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, g.pid_rate_pitch.get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, g.pid_rate_yaw.get_pid_info() );
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info() );
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW))) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif
}

// full_rate_logging_loop
// should be run at the MAIN_LOOP_RATE
void Copter::full_rate_logging_loop()
{
    if (should_log(MASK_LOG_IMU_FAST) && !should_log(MASK_LOG_IMU_RAW)) {
         DataFlash.Log_Write_IMU(ins);
    }

    if (should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMUDT(ins);
    }
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    // update ch6 in flight tuning
    tuning();
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    // If we have new calibration, save it to the EEPROM on the FC processor.
    // FIXME: Remove when we determine cause of FRAM erase.
    if (save_calibration) {
        Vector3f mag = compass.get_offsets();
        Vector3f trim = ahrs.get_trim();
        Vector3f accel = ins.get_accel_offsets();

        if (!is_zero(mag.length()) && (mag.length() < COMPASS_OFFSETS_MAX)) {
            TpfcFloatVector v;

            v.x = mag.x;
            v.y = mag.y;
            v.z = mag.z;
            
            // printf("Set mag offsets: (%3.2f, %3.2f, %3.2f)\n",
            //        v.x, v.y, v.z);
            
            ioctl(tpfc_fd, TPFC_IOC_MAG_OFFSETS_SET, (unsigned long)&v);

            // printf("saved mag offsets (%3.2f, %3.2f, %3.2f).\n", mag.x, mag.y, mag.z);
        }

        float trim_limit = ToRad(AP_AHRS_TRIM_LIMIT);
        
        if (!is_zero(trim.length()) && (fabsf(trim.x) < trim_limit) && (fabsf(trim.y) <= trim_limit)) {
            TpfcFloatVector v;

            v.x = trim.x;
            v.y = trim.y;
            v.z = trim.z;
            
            // printf("Set trim offsets: (%2.4f, %2.4f, %2.4f)\n",
            //        v.x, v.y, v.z);
            ioctl(tpfc_fd, TPFC_IOC_TRIM_OFFSETS_SET, (unsigned long)&v);
        }

        if (!is_zero(accel.length()) && (accel.length() < 3.0f)) {
            TpfcFloatVector v;

            v.x = accel.x;
            v.y = accel.y;
            v.z = accel.z;

            // printf("Set accel offsets: (%2.4f, %2.4f, %2.4f)\n",
            //        v.x, v.y, v.z);

            ioctl(tpfc_fd, TPFC_IOC_ACCEL_OFFSETS_SET, (unsigned long)&v);

            // Also save the scale.
            //
            accel = ins.get_accel_scale();

            if (accel.length() < 2.0f) {
                v.x = accel.x;
                v.y = accel.y;
                v.z = accel.z;

                ioctl(tpfc_fd, TPFC_IOC_ACCEL_SCALE_SET, (unsigned long)&v);
            }
        }

        save_calibration = false;
    }
    
    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = 15;
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= 30) {
        pre_arm_checks(true);
        pre_arm_display_counter = 0;
    }else{
        pre_arm_checks(false);
    }

    // auto disarm checks
    auto_disarm_check();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

#if FRAME_CONFIG == HELI_FRAME
        // helicopters are always using motor interlock
        set_using_interlock(true);
#else
        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);

        // check if we are using motor interlock control on an aux switch
        set_using_interlock(check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK));

        // set all throttle channel settings
        motors.set_throttle_range(g.throttle_min, channel_throttle->radio_min, channel_throttle->radio_max);
        // set hover throttle
        motors.set_hover_throttle(g.throttle_mid);
#endif
    }

    // update assigned functions and enable auxiliar servos
    RC_Channel_aux::enable_aux_servos();

    check_usb_mux();

#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // tell the rangefinder our height, so it can go into power saving
    // mode if available
#if CONFIG_SONAR == ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        sonar.set_estimated_terrain_height(height);
    }
#endif
#endif

    // update position controller alt limits
    update_poscon_alt_max();

    // enable/disable raw gyro/accel logging
    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
}

// called at 50hz
void Copter::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    static uint32_t last_good_hacc_ms(0);
    static bool     initial_lock(false);
    
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // Check horizontal accuracy as a more strict GPS check.
            // We do not want to allow arming with a marginal GPS
            // signal.
            //
            float hAcc = 0.0f;
            if (gps.horizontal_accuracy(hAcc)) {
                if (!initial_lock) {
                    if (hAcc < GPS_REQUIRED_INITIAL_HACC) {
                        initial_lock = true;
                        gps_monitor_ok = true;
                    }
                }
                else {
                    if (hAcc < GPS_REQUIRED_HACC) {
                        gps_monitor_ok = true;
                        last_good_hacc_ms = millis();
                    }
                    else {
                        if (gps_monitor_ok) {
                            if ((millis() - last_good_hacc_ms) > GPS_OUTAGE_REPORT_DELAY_MS) {
                                // State transition 
                                gps_monitor_ok = false;
                            }
                        }
                    }
                }       
            }

            // log GPS message
            if (should_log(MASK_LOG_GPS)) {
                DataFlash.Log_Write_GPS(gps, i, current_loc.alt);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

        // checks to initialise home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

#if CAMERA == ENABLED
            if (camera.update_location(current_loc) == true) {
                do_take_picture();
            }
#endif
        }
    }
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->control_in*simple_cos_yaw - channel_pitch->control_in*simple_sin_yaw;
        pitchx = channel_roll->control_in*simple_sin_yaw + channel_pitch->control_in*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->control_in*super_simple_cos_yaw - channel_pitch->control_in*super_simple_sin_yaw;
        pitchx = channel_roll->control_in*super_simple_sin_yaw + channel_pitch->control_in*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->control_in = rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw();
    channel_pitch->control_in = -rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw();
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

void Copter::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
}

// read baro and sonar altitude at 10hz
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // read in sonar altitude
    sonar_alt           = read_sonar();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}


// Check for any commands from flight control
void Copter::check_cmd_input() {

    uint32_t reqRegister;
    
    if (ioctl(tpfc_fd, TPFC_IOC_INPUT_REQ_GET, (unsigned long)&reqRegister) == 0)  {
        switch(reqRegister) {
            // Some commands are handled immediately inline.  Others are
            // handled at different frequencies by different loops.

        case PX4IO_P_CONTROL_INPUT_REQ_SETHOME:
            input_cmd = reqRegister;
            publish_input_cmd_rsp(set_home_to_current_location_and_lock());
            break;
        case PX4IO_P_CONTROL_INPUT_REQ_LOST_ON:
            input_cmd = reqRegister;
            sound_lost_vehicle_alarm = true;
            publish_input_cmd_rsp(true);
            break;
        case PX4IO_P_CONTROL_INPUT_REQ_LOST_OFF:
            input_cmd = reqRegister;
            sound_lost_vehicle_alarm = false;
            publish_input_cmd_rsp(true);
            break;
        case PX4IO_P_CONTROL_INPUT_REQ_ACC_RESET:
            input_cmd = reqRegister;
            float trim_roll, trim_pitch;
            if(ins.calibrate_trim(trim_roll, trim_pitch)) {
                // reset ahrs trim
                //
                ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                set_new_calibration();
                publish_input_cmd_rsp(true);
            }
            else {
                publish_input_cmd_rsp(false);
            }
            break;
        case PX4IO_P_CONTROL_INPUT_REQ_NONE:
        case PX4IO_P_CONTROL_INPUT_REQ_DISARM:
        case PX4IO_P_CONTROL_INPUT_REQ_ARM:
            input_cmd = reqRegister;
        default:
            // Ignore unknown requests
            break;
        }
    }
}

void Copter::publish_input_cmd_rsp(bool success) {

    tpfc_input_cmd_rsp_s rspData;

    if (success) {
        rspData.rsp_code = 0;  // 0 indicates success
    }
    else {
        rspData.rsp_code = 1;
    }

    printf("cmd %d, reply %d\n", input_cmd, rspData.rsp_code);
    
    if (tpfc_input_cmd_rsp_handle != 0) {
        orb_publish(ORB_ID(tpfc_input_cmd_rsp), tpfc_input_cmd_rsp_handle, &rspData);
    }
    else {
        tpfc_input_cmd_rsp_handle = orb_advertise(ORB_ID(tpfc_input_cmd_rsp), &rspData);
    }

    // We are done with this command, clear it
    //
    input_cmd = PX4IO_P_CONTROL_INPUT_REQ_NONE;
}


void Copter::update_ctrl_output()  {

    tpfc_autopilot_s apData;
    int8_t flight_mode = control_mode;

    if (flight_mode == LAND && !landing_with_GPS()) {
        flight_mode = LAND_NO_GPS;
    }

    Vector3f angle_targets = attitude_control.angle_ef_targets();

    switch (flight_mode) {
        case STABILIZE:
        case ACRO:
        case ALT_HOLD:
        case SPORT:
        case LAND_NO_GPS:
            apData.target_pitch = 0;
            apData.target_roll  = 0;
            apData.target_yaw   = angle_targets.z;
            break;
        default:
            // Autopilot mode.  Send the target angles to FC
            //
            apData.target_pitch = angle_targets.y;
            apData.target_roll  = angle_targets.x;
            apData.target_yaw   = angle_targets.z;
            break;
    }

    
    apData.target_throttle = motors.get_throttle();
    apData.is_armed     = motors.armed();
    apData.flight_mode  = flight_mode;

    apData.gps_state   = position_ok() && !failsafe_ekf_bad_variance();
    apData.mag_state   = compass.healthy();
    apData.ahrs_state  = ahrs.healthy();
    apData.gyro_state  = ins.get_gyro_health_all();
    apData.accel_state = ins.get_accel_health_all();
    apData.home_info   = 0;

    if (distance_to_home_cm > MAX_DIST_TO_HOME) {
        apData.home_info |= PX4IO_P_CONTROL_OUTPUT_HOME_FAR;
    }

    if (ap.home_state != HOME_UNSET) {
        apData.home_info |= PX4IO_P_CONTROL_OUTPUT_HOME_SET;
    }

    if (barometer.healthy()) {
        // baro temp
        apData.temperature = (barometer.get_temperature()*9/5)+32.0f;
    }
    else {
        apData.temperature = 0xffff;
    }

    apData.fence_breach = (fence.get_breaches() != AC_FENCE_TYPE_NONE);
    
    if (tpfc_autopilot_handle != 0) {
        orb_publish(ORB_ID(tpfc_autopilot), tpfc_autopilot_handle, &apData);
    }
    else {
        tpfc_autopilot_handle = orb_advertise(ORB_ID(tpfc_autopilot), &apData);
    }


    if (should_log(MASK_LOG_CONTROL)) {
        Log_Write_Control(apData);
    }

    if (should_log(MASK_LOG_FCU_FAST)) {
        uint16_t pSize = 16;
        uint16_t parms[pSize];
    
        if (ioctl(tpfc_fd, TPFC_IOC_FCU_LOG_GET, (unsigned long)parms) == 0)  {
            Log_Write_FCU(parms, pSize);
        }
    }
}


void Copter::check_flight_mode()  {

    uint32_t mode = 0;
    
    if (ioctl(tpfc_fd, TPFC_IOC_MODE_GET, (unsigned long) &mode) == 0)  {
        uint8_t eventId = 0;

        if (mode != (uint32_t)control_mode)  {
            eventId = 100 + mode;
            Log_Write_Event(eventId);
        }
            
        switch (mode)  {
            case STABILIZE:
            case ALT_HOLD:
            case ACRO:
            case RTL:
            case LAND:
            case BRAKE:
            case LOITER:
                set_mode(mode);
                break;
            case LAND_NO_GPS:
                set_mode(LAND);
                break;
            default:
                // Ignore
                break;
        }
    }
    else  {
        tpfc_ioctl_errors++;
    }
}



void Copter::update_tpfc()
{
    tpfc_loop_count++;
    tpfc_sensors_s data;

    if (tpfc_send_ekf_data > 0)   {
                
        Vector3f      v;

        ahrs.get_NavEKF_const().getEulerAngles(v);

        data.euler_z = v.z;
        data.euler_y = v.y;
        data.euler_x = v.x;

        v = ahrs.get_gyro();

        data.gyro_z = v.z;
        data.gyro_y = v.y;
        data.gyro_x = v.x;

        v = ahrs.get_accel_ef_blended();

        data.accel_z = v.z;
        data.accel_y = v.y;
        data.accel_x = v.x;

        ahrs.get_NavEKF_const().getHAGL(data.altitude);

        //  publish the sensor data to tpfc
        if (tpfc_sensor_handle != 0) {
            orb_publish(ORB_ID(tpfc_sensors), tpfc_sensor_handle, &data);
        }
        else {
            tpfc_sensor_handle = orb_advertise(ORB_ID(tpfc_sensors), &data);
        }
    }

    if (tpfc_display_ekf_data > 0 &&
        (tpfc_loop_count % tpfc_display_data_period) == 0)  {

        printf("tpfc data: euler(%3.6f, %3.6f, %3.6f)  gyro(%3.6f, %3.6f, %3.6f)   Alt ekf:%8.3f baro:%d\n",
               data.euler_x, data.euler_y, data.euler_z,
               data.gyro_x, data.gyro_y, data.gyro_z,
               //                 data.accel_x, data.accel_y, data.accel_z,
               data.altitude, baro_alt);
    }

    if (tpfc_display_error_data > 0 &&
        (tpfc_loop_count % tpfc_display_stats_period) == 0)  {
        
        printf("tpfc stats, loops:%d  efk err:%d  fd err:%d  ioctl err:%d  ioctl ok:%d\n",
               tpfc_loop_count, tpfc_ekf_errors, tpfc_bad_fd_errors, tpfc_ioctl_errors, tpfc_ioctl_success);
    }
}

void Copter::set_new_calibration() {
    save_calibration = true;
}

int16_t Copter::set_px4io_param(uint8_t id, int32_t value) {
    TpfcFcuParam v;
    int16_t rc = -1;

    v.id = id;
    // TODO: Range check for int16_t, we are reducing precision from
    // int32_t to int16_t here.
    v.value = value;

    //    printf("Set_px4io_param: id=%d  value=%d\n",
    //            v.id, v.value);

    if (id == PX4IO_PARAM_RESET_TO_DEFAULT && value != 0) {
        rc = ioctl(tpfc_fd, TPFC_IOC_FCU_PARAM_RESET_ALL, 0);

        if (rc == 0) {
            px4io_parms_reset = true;
        }
    }
    else {
        rc = ioctl(tpfc_fd, TPFC_IOC_FCU_PARAM_SET, (unsigned long)&v);
    }
    
    return rc;
}


int32_t Copter::get_px4io_param(uint8_t id) {

    uint32_t rc;
    
    switch (id) {
        case PX4IO_PARAM_FMU_SW_VERSION: {
            int a,b,c,d;

            if (sscanf(flt_sw_version, "%d.%d.%d.%d", &a, &b, &c, &d) != 4) {
                a = 0;
                b = 0;
                c = 0;
                d = 0;
            }

            rc = (a << 24) |
                ((b & 0x000000ff) << 16) |
                ((c & 0x000000ff) << 8) | 
                (d & 0x000000ff);
        }
            break;
        case PX4IO_PARAM_FC_SW_VERSION:
            if(ioctl(tpfc_fd, TPFC_IOC_FC_FW_VERSION_GET, (unsigned long)&rc) == 0)  {
                rc = rc & 0x0000ffff;
            }

            break;
        case PX4IO_PARAM_ESC_SW_VERSION:
            if(ioctl(tpfc_fd, TPFC_IOC_ESC_FW_VERSION_GET, (unsigned long)&rc) == 0)  {
                rc = rc & 0x0000ffff;
            }

            break;
        case PX4IO_PARAM_RESET_TO_DEFAULT:
            rc = px4io_parms_reset;
            break;
        default:
            rc = ioctl(tpfc_fd, TPFC_IOC_FCU_PARAM_GET, (unsigned long)id);
            break;
    }

    //    printf("get_px4io_param: id=%d, val=%d\n", id, rc);

    return rc;
}

int16_t Copter::reset_px4io_param_to_default()
{
    return (ioctl(tpfc_fd, TPFC_IOC_FCU_PARAM_RESET_ALL, 0));
}


int16_t Copter::get_px4io_battery() {
    //printf("Get_px4io_batt\n");
    return ioctl(tpfc_fd, TPFC_IOC_FCU_BATTERY_GET, (unsigned long) 0);
}

uint16_t Copter::get_px4io_status_led() {
    return ioctl(tpfc_fd, TPFC_IOC_FCU_STATUS_LED_GET, (unsigned long) 0);
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    copter.setup();
}
void loop(void)
{
    copter.loop();
}

AP_HAL_MAIN();

