/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#include <board_config.h>
#include <stm32_bkp.h>
#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint8_t auto_disarming_counter;


// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
void Copter::arm_motors_check()
{
    // ensure throttle is down
    if (channel_throttle->control_in > 0) {
        return;
    }

    if (input_cmd == PX4IO_P_CONTROL_INPUT_REQ_ARM && !motors.armed()) {
        bool armSuccess = init_arm_motors(false);
        publish_input_cmd_rsp(armSuccess);
    }
    else if (input_cmd == PX4IO_P_CONTROL_INPUT_REQ_DISARM && motors.armed()) {
        init_disarm_motors();
        publish_input_cmd_rsp(true);
    }
}


// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
// called at 1hz
void Copter::auto_disarm_check()
{
    uint8_t disarm_delay = constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors.armed() || disarm_delay == 0) {
        auto_disarming_counter = 0;
        return;
    }

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors.get_interlock()) || ap.motor_emergency_stop) {
        auto_disarming_counter++;
        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay /= 2;
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (mode_has_manual_throttle(control_mode) || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = g.rc_3.get_control_mid() + g.throttle_deadzone;
            thr_low = g.rc_3.control_in <= deadband_top;
        }

        if (thr_low && ap.land_complete) {
            // increment counter
            auto_disarming_counter++;
        } else {
            // reset counter
            auto_disarming_counter = 0;
        }
    }

    // disarm once counter expires
    if (auto_disarming_counter >= disarm_delay) {
        init_disarm_motors();
        auto_disarming_counter = 0;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool Copter::init_arm_motors(bool arming_from_gcs)
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // run pre-arm-checks and display failures
    if(!pre_arm_checks(true) || !arm_checks(true, arming_from_gcs)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // reset battery failsafe
    set_failsafe_battery(false);

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call update_notify a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        update_notify();
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    if (ap.home_state == HOME_UNSET) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        ahrs.get_NavEKF().resetHeightDatum();
        Log_Write_Event(DATA_EKF_ALT_RESET);
    } else if (ap.home_state == HOME_SET_NOT_LOCKED) {
        // Set and lock home position if it is unlocked
        set_home_to_current_location_and_lock();
    }
    calc_distance_and_bearing();

    if(did_ground_start == false) {
        startup_ground(true);
        // final check that gyros calibrated successfully
        if (((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) && !ins.gyro_calibrated_ok_all()) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Gyro calibration failed"));
            AP_Notify::flags.armed = false;
            failsafe_enable();
            in_arm_motors = false;
            return false;
        }
        did_ground_start = true;
    }

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

#if SPRAYER == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif

    // short delay to allow reading of rc inputs
    delay(30);

    // enable output to motors
    enable_motor_output();

    // finally actually arm the motors
    motors.armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // log flight mode in case it was changed while vehicle was disarmed
    DataFlash.Log_Write_Mode(control_mode);

    // reenable failsafe
    failsafe_enable();

    // perf monitor ignores delay due to arming
    perf_ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // return success
    return true;
}

// perform pre-arm checks and set ap.pre_arm_check flag
//  return true if the checks pass successfully
bool Copter::pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (motors.armed()) {
        return true;
    }

    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    if (check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK) && check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Interlock/E-Stop Conflict"));
        }
        return false;
    }

    // exit immediately if we've already successfully performed the pre-arm check
    if (ap.pre_arm_check) {
        // run gps checks because results may change and affect LED colour
        // no need to display failures because arm_checks will do that if the pilot tries to arm
        pre_arm_gps_checks(false);
        return true;
    }

    // succeed if pre arm checks are disabled
    if(g.arming_check == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        set_pre_arm_rc_check(true);
        return true;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: RC not calibrated"));
        }
        return false;
    }

    // Check AHRS (EKF) health.  This will make sure we do not play
    // the ready tone until the EKF is ready to fly.  Users complained
    // about arming failure after ready tone was played.
    //
    if(!ahrs.healthy()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: AHRS not healthy"));
        }
        return false;
    }

    // check Baro
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // barometer health check
        if(!barometer.all_healthy()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Barometer not healthy"));
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Altitude disparity"));
                }
                return false;
            }
        }
    }

    // check Compass
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_COMPASS)) {
        // check the primary compass is healthy
        if(!compass.healthy()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy"));
            }
            return false;
        }

        // check compass learning is on or offsets have been set
        if(!compass.configured()) {

            // Offsets may have been inadvertently lost due to storage
            // manager/fram issue we cannot find.  See if they can be
            // restored from TPFC backups.
            //
            TpfcFloatVector v;
            if (ioctl(tpfc_fd, TPFC_IOC_MAG_OFFSETS_GET, (unsigned long)&v) == 0)  {
                Vector3f offsets(v.x, v.y, v.z);

                if (!is_zero(offsets.length())) {
                    
                    //   printf("Restoring backup offsets: (%3.2f, %3.2f, %3.2f)\n",
                    //          offsets.x, offsets.y, offsets.z);

                    // Count restores in backup register
                    (*(uint32_t *) STM32_BKP_DR10)++;

                    compass.set_and_save_offsets(compass.get_primary(), offsets);
                }
            }

            // Recheck in case we restored the offsets above.
            if(!compass.configured()) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
                }
                return false;
            }
        }

        // check for unreasonable compass offsets
        Vector3f offsets = compass.get_offsets();
        if(offsets.length() > COMPASS_OFFSETS_MAX) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass offsets too high"));
            }
            return false;
        }

        // check for unreasonable mag field length
        float mag_field = compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65f || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35f) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check mag field"));
            }
            return false;
        }

        // check all compasses point in roughly same direction
        if (!compass.consistent()) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent compasses"));
                    }
                    return false;
                }

    }

    // check GPS
    // if (!pre_arm_gps_checks(display_failure)) {
    //     return false;
    // }

#if AC_FENCE == ENABLED
    // check fence is initialised
    if(!fence.pre_arm_check()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: check fence"));
        }
        return false;
    }
#endif


    // Check AHRS trim...
    Vector3f trim = ahrs.get_trim();

    if (is_zero(trim.length())) {
        // Offsets may have been inadvertently lost due to storage
        // manager/fram issue we cannot find.  See if they can be
        // restored from TPFC backups.
        //
        TpfcFloatVector v;
        if (ioctl(tpfc_fd, TPFC_IOC_TRIM_OFFSETS_GET, (unsigned long)&v) == 0)  {
            trim.x = v.x;
            trim.y = v.y;
            trim.z = v.z;

            // Count restores in backup register
            (*(uint32_t *) STM32_BKP_DR10)++;

            ahrs.set_trim(trim);
            // printf("Trim: Restoring backup offsets: (%3.2f, %3.2f, %3.2f)\n",
            //        v.x, v.y, v.z);

        }
    }

    // Check accelerometer calibration offsets.
    //
    Vector3f accel_offsets = ins.get_accel_offsets();
    
    if (is_zero(accel_offsets.length())) {
        // Offsets may have been inadvertently lost due to storage
        // manager/fram issue we cannot find.  See if they can be
        // restored from TPFC backups.
        //
        TpfcFloatVector offset, scale;
        if (ioctl(tpfc_fd, TPFC_IOC_ACCEL_OFFSETS_GET, (unsigned long)&offset) == 0 &&
            ioctl(tpfc_fd, TPFC_IOC_ACCEL_SCALE_GET, (unsigned long)&scale) == 0)  {
            accel_offsets.x = offset.x;
            accel_offsets.y = offset.y;
            accel_offsets.z = offset.z;

            ins.set_accel_offsets(accel_offsets);

            // Count restores in backup register
            (*(uint32_t *) STM32_BKP_DR10)++;

            // printf("Accel: Restoring backup offsets: (%3.2f, %3.2f, %3.2f)\n",
            //        offset.x, offset.y, offset.z);

            Vector3f accel_scale;
                
            accel_scale.x = scale.x;
            accel_scale.y = scale.y;
            accel_scale.z = scale.z;

            ins.set_accel_scale(accel_scale);

            // printf("Accel: Restoring backup scale: (%3.2f, %3.2f, %3.2f)\n",
            //         scale.x, scale.y, scale.z);
        }
    }

    
    // check INS
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if(!ins.accel_calibrated_ok_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Accels not calibrated"));
            }
            return false;
        }

        // check accels are healthy
        if(!ins.get_accel_health_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Accelerometers not healthy"));
            }
            return false;
        }

#if INS_MAX_INSTANCES > 1
        // check all accelerometers point in roughly same direction
        if (ins.get_accel_count() > 1) {
            const Vector3f &prime_accel_vec = ins.get_accel();
            for(uint8_t i=0; i<ins.get_accel_count(); i++) {
                // get next accel vector
                const Vector3f &accel_vec = ins.get_accel(i);
                Vector3f vec_diff = accel_vec - prime_accel_vec;
                float threshold = PREARM_MAX_ACCEL_VECTOR_DIFF;
                if (i >= 2) {
                    /*
                      for boards with 3 IMUs we only use the first two
                      in the EKF. Allow for larger accel discrepancy
                      for IMU3 as it may be running at a different temperature
                     */
                    threshold *= 2;
                }
                if (vec_diff.length() > threshold) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent Accelerometers"));
                    }
                    return false;
                }
            }
        }
#endif

        // check gyros are healthy
        if(!ins.get_gyro_health_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Gyros not healthy"));
            }
            return false;
        }

#if INS_MAX_INSTANCES > 1
        // check all gyros are consistent
        if (ins.get_gyro_count() > 1) {
            for(uint8_t i=0; i<ins.get_gyro_count(); i++) {
                // get rotation rate difference between gyro #i and primary gyro
                Vector3f vec_diff = ins.get_gyro(i) - ins.get_gyro();
                if (vec_diff.length() > PREARM_MAX_GYRO_VECTOR_DIFF) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent Gyros"));
                    }
                    return false;
                }
            }
        }
#endif
    }

#if CONFIG_HAL_BOARD != HAL_BOARD_VRBRAIN
#ifndef CONFIG_ARCH_BOARD_PX4FMU_V1
    // check board voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if(hal.analogin->board_voltage() < BOARD_VOLTAGE_MIN || hal.analogin->board_voltage() > BOARD_VOLTAGE_MAX) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check Board Voltage"));
            }
            return false;
        }
    }
#endif
#endif

    // check battery voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if (failsafe.battery || (!ap.usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah))) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check Battery"));
            }
            return false;
        }
    }

    // check various parameter values
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS)) {

        // ensure ch7 and ch8 have different functions
        if (check_duplicate_auxsw()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Duplicate Aux Switch Options"));
            }
            return false;
        }

        // failsafe parameter checks
        if (g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (channel_throttle->radio_min <= g.failsafe_throttle_value+10 || g.failsafe_throttle_value < 910) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check FS_THR_VALUE"));
                }
                return false;
            }
        }

        // lean angle parameter check
        if (aparm.angle_max < 1000 || aparm.angle_max > 8000) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check ANGLE_MAX"));
            }
            return false;
        }

        // acro balance parameter check
        if ((g.acro_balance_roll > g.p_stabilize_roll.kP()) || (g.acro_balance_pitch > g.p_stabilize_pitch.kP())) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: ACRO_BAL_ROLL/PITCH"));
            }
            return false;
        }

#if CONFIG_SONAR == ENABLED && OPTFLOW == ENABLED
        // check range finder if optflow enabled
        if (optflow.enabled() && !sonar.pre_arm_check()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: check range finder"));
            }
            return false;
        }
#endif
#if FRAME_CONFIG == HELI_FRAME
        // check helicopter parameters
        if (!motors.parameter_check()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check Heli Parameters"));
            }
            return false;
        }
#endif // HELI_FRAME
    }

    // check throttle is above failsafe throttle
    // this is near the bottom to allow other failures to be displayed before checking pilot throttle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_RC)) {
        if (g.failsafe_throttle != FS_THR_DISABLED && channel_throttle->radio_in < g.failsafe_throttle_value) {
            if (display_failure) {
    #if FRAME_CONFIG == HELI_FRAME
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Collective below Failsafe"));
    #else
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Throttle below Failsafe"));
    #endif
            }
            return false;
        }
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(true);
    return true;
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
void Copter::pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.pre_arm_rc_check ) {
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_RC)) {
        set_pre_arm_rc_check(true);
        return;
    }

    // check if radio has been calibrated
    // DGS: would like to do this by clearning the ARMING_CHECK_RC bit
    // in the arming_check mask but ALL is not the bitwise or of the
    // individual parts.  Revisit later...
    // if(!channel_throttle->radio_min.load() && !channel_throttle->radio_max.load()) {
    //     printf("read throttle failed, throttle min: %d, max: %d\n",
    //            channel_throttle->radio_min,
    //            channel_throttle->radio_max);
    //     return;
    // }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (channel_roll->radio_min > 1300 || channel_roll->radio_max < 1700 || channel_pitch->radio_min > 1300 || channel_pitch->radio_max < 1700) {
        return;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (channel_throttle->radio_min > 1300 || channel_throttle->radio_max < 1700 || channel_yaw->radio_min > 1300 || channel_yaw->radio_max < 1700) {
        return;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (channel_roll->radio_trim < 1300 || channel_roll->radio_trim > 1700 || channel_pitch->radio_trim < 1300 || channel_pitch->radio_trim > 1700) {
        return;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (channel_yaw->radio_trim < 1300 || channel_yaw->radio_trim > 1700) {
        return;
    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(true);
}

// performs pre_arm gps related checks and returns true if passed
bool Copter::pre_arm_gps_checks(bool display_failure)
{
    // always check if inertial nav has started and is ready
    if(!ahrs.get_NavEKF().healthy()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Waiting for Nav Checks"));
        }
        return false;
    }

    // check if flight mode requires GPS
    bool gps_required = mode_requires_GPS(control_mode);

#if AC_FENCE == ENABLED
    // if circular fence is enabled we need GPS
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) {
        //        gps_required = true;
    }
#endif

    // return true if GPS is not required
    if (!gps_required) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // ensure GPS is ok
    if (!position_ok()) {
        if (display_failure) {
            const char *reason = ahrs.prearm_failure_reason();
            if (reason) {
                GCS_MAVLINK::send_statustext_all(SEVERITY_HIGH, PSTR("PreArm: %s"), reason);
            } else {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Need 3D Fix"));
        }
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check EKF compass variance is below failsafe threshold
    // float vel_variance, pos_variance, hgt_variance, tas_variance;
    // Vector3f mag_variance;
    // Vector2f offset;
    // ahrs.get_NavEKF().getVariances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    // if (mag_variance.length() >= g.fs_ekf_thresh) {
    //     if (display_failure) {
    //         gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: EKF compass variance"));
    //     }
    //     return false;
    // }

    // check home and EKF origin are not too far
    if (far_from_EKF_origin(ahrs.get_home())) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: EKF-home variance"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // return true immediately if gps check is disabled
    if (!(g.arming_check == ARMING_CHECK_ALL || g.arming_check & ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (gps.get_hdop() > g.gps_hdop_good) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: High GPS HDOP"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
bool Copter::arm_checks(bool display_failure, bool arming_from_gcs)
{
#if LOGGING_ENABLED == ENABLED
    // start dataflash
    start_logging();
#endif

    // check accels and gyro are healthy
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        if(!ins.get_accel_health_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Accelerometers not healthy"));
            }
            return false;
        }
        if(!ins.get_gyro_health_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Gyros not healthy"));
            }
            return false;
        }
    }

    // always check if inertial nav has started and is ready
    if(!ahrs.healthy()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Waiting for Nav Checks"));
        }
        return false;
    }

    // always check if the current mode allows arming
    if (!mode_allows_arming(control_mode, arming_from_gcs)) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Mode not armable"));
        }
        return false;
    }

    // always check gps
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

#if FRAME_CONFIG == HELI_FRAME
	// heli specific arming check
    // check if rotor is spinning on heli because this could disrupt gyro calibration
    if (!motors.allow_arming()){
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Rotor is Spinning"));
        }
        return false;
    }
#endif

    // if we are using motor interlock switch and it's enabled, fail to arm
    if (ap.using_interlock && motors.get_interlock()){
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Motor Interlock Enabled"));
        AP_Notify::flags.armed = false;
        return false;
    }

    // if we are not using Emergency Stop switch option, force Estop false to ensure motors
    // can run normally
    if (!check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        set_motor_emergency_stop(false);
    // if we are using motor Estop switch, it must not be in Estop position
    } else if (check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP) && ap.motor_emergency_stop){
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Motor Emergency Stopped"));
        return false;
    }

    // succeed if arming checks are disabled
    if (g.arming_check == ARMING_CHECK_NONE) {
        return true;
    }

    // baro checks
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // baro health check
        if (!barometer.all_healthy()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Barometer not healthy"));
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref && (fabsf(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM)) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Altitude disparity"));
            }
            return false;
        }
    }

#if AC_FENCE == ENABLED
    // check vehicle is within fence
    if(!fence.pre_arm_check()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: check fence"));
        }
        return false;
    }
#endif

    // check lean angle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > aparm.angle_max) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Leaning"));
            }
            return false;
        }
    }

    // check battery voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if (failsafe.battery || (!ap.usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah))) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Check Battery"));
            }
            return false;
        }
    }

    // check throttle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_RC)) {
        // check throttle is not too low - must be above failsafe throttle
        if (g.failsafe_throttle != FS_THR_DISABLED && channel_throttle->radio_in < g.failsafe_throttle_value) {
            if (display_failure) {
#if FRAME_CONFIG == HELI_FRAME
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Collective below Failsafe"));
#else
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle below Failsafe"));
#endif
            }
            return false;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(arming_from_gcs && control_mode == GUIDED)) {
            // above top of deadband is too always high
            if (channel_throttle->control_in > get_takeoff_trigger_throttle()) {
                if (display_failure) {
#if FRAME_CONFIG == HELI_FRAME
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Collective too high"));
#else
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle too high"));
#endif
                }
                return false;
            }
            // in manual modes throttle must be at zero
            if ((mode_has_manual_throttle(control_mode) || control_mode == DRIFT) && channel_throttle->control_in > 0) {
                if (display_failure) {
#if FRAME_CONFIG == HELI_FRAME
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Collective too high"));
#else
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle too high"));
#endif
                }
                return false;
            }
        }
    }

    // DGS-comment for now
    // check if safety switch has been pushed
    // if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
    //     if (display_failure) {
    //         gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Safety Switch"));
    //     }
    //     printf("%s(%d)\n", __FILE__, __LINE__);
    //     return false;
    // }

    // if we've gotten this far all is ok
    return true;
}

// init_disarm_motors - disarm motors
void Copter::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors.armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    // save compass offsets learned by the EKF
    Vector3f magOffsets;
    if (ahrs.use_compass() && ahrs.getMagOffsets(magOffsets)) {
        compass.set_and_save_offsets(compass.get_primary(), magOffsets);
    }

#if AUTOTUNE_ENABLED == ENABLED
    // save auto tuned parameters
    autotune_save_tuning_gains();
#endif

    // we are not in the air
    set_land_complete(true);
    set_land_complete_maybe(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // send disarm command to motors
    motors.armed(false);

    // reset the mission
    mission.reset();

    // suspend logging
    if (!(g.log_bitmask & MASK_LOG_WHEN_DISARMED)) {
        DataFlash.EnableWrites(false);
    }

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
    // check if we are performing the motor test
    if (ap.motor_test) {
        motor_test_output();
    } else {
        if (!ap.using_interlock){
            // if not using interlock switch, set according to Emergency Stop status
            // where Emergency Stop is forced false during arming if Emergency Stop switch
            // is not used. Interlock enabled means motors run, so we must
            // invert motor_emergency_stop status for motors to run.
            motors.set_interlock(!ap.motor_emergency_stop);
        }
        motors.output();
    }
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (check_if_auxsw_mode_used(AUXSW_LOST_COPTER_SOUND)) {
        return;
    }

    int16_t pitch_in = channel_pitch->control_in;
    int16_t roll_in  = channel_roll->control_in;
    
    // Two cases here:
    // 1. user is requesting the lost vehicle tone by putting pitch
    //    and roll at max.
    // 2. Flight control is requesting due to lost radio transmission
    //
    // In either case, require the motors to be disarmed.
    //
    if ((((roll_in > 4000) && (pitch_in > 4000)) || sound_lost_vehicle_alarm) && !motors.armed()) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Locate Copter Alarm!"));
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}
