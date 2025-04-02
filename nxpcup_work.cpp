/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

 #include "nxpcup_work.hpp"
 #include "nxpcup_race.h"
 #include <time.h>
 
 #include <drivers/drv_hrt.h>
 #include <px4_platform_common/posix.h>
 #include "drivers/drv_pwm_output.h"
 
 
 
 using namespace time_literals;
 
 static struct distance_sensor_s distance_sensor_data;
 
 NxpCupWork::NxpCupWork() :
     ModuleParams(nullptr),
     ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
 {
 }
 
 NxpCupWork::~NxpCupWork()
 {
     perf_free(_loop_perf);
     perf_free(_loop_interval_perf);
 }
 
 bool NxpCupWork::init()
 {
     ScheduleOnInterval(50_ms); // 1000 us interval, 1000 Hz rate
 
     return true;
 }
 
 void NxpCupWork::roverSteerSpeed(roverControl control, int fd)
 {
     // Steering control of the Rover
     // 2000 is extreme left -1
     // 1500 is 0
     
     // 1000 is extreme right 1
     // if (control.steer>60){
     // 		control.steer=60;
     // 	}
     // if (control.steer<-60){
     // 	control.steer=-60;
     // }
     control.steer *= 60.0f;
     int servo_pwm_rate = (control.steer + 90.0f) * 1000.0f / 180.0f + 1000.0f;
     servo_pwm_rate -= 140;
     int ret = 0;
 
     // Motor values are mapped from 0 to 1 based on MOTOR_ACTIVATION_PWM
     // The 0 is binded to 1000.
     int motor_pwm_rate = 1000;
 
     // activationValue + normalizedValue * (2000 - activationValue)
     // normalized value is control.speed which is between 0 and 1.
 
     if (!(control.speed <= 0.0f && control.speed >= 0.0f)) {
         if (control.speed < 0.0f) {
             control.speed = 0.0f;
 
         } else if (control.speed > 1.0f) {
             control.speed = 1.0f;
         }
 
         motor_pwm_rate = MOTOR_ACTIVATION_PWM + control.speed * 40;
 
     }
 
     ::ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE);
     // Change speed then steerting.
     // 3 is motor.
     ret = px4_ioctl(fd, PWM_SERVO_SET(3), motor_pwm_rate);
     // 1 is servo.
     ret = px4_ioctl(fd, PWM_SERVO_SET(1), servo_pwm_rate);
     // Hack for unsed value.
     printf("%d", ret);
 
 }
 
 void NxpCupWork::Run()
 {
     const char *dev = PWM_OUTPUT0_DEVICE_PATH;
     static int fd = px4_open(dev, 0);
     // static PID_t PID;
     // pid_init(&PID, PID_MODE_DERIVATIV_CALC_NO_SP, 1.0f);
     // pid_set_parameters(&PID, PID_P, PID_I, PID_D, 0.1f, 1.0f);
 
     if (should_exit()) {
         ScheduleClear();
         exit_and_cleanup();
         return;
     }
 
     perf_begin(_loop_perf);
     perf_count(_loop_interval_perf);
 
     // DO WORK
     roverControl motorControl;
 
     /*Get distance data*/
     distance_sensor_sub.update();
     distance_sensor_data = distance_sensor_sub.get();
     // printf("AAAA %f", (double)distance_sensor_data.current_distance);
 
     /* Get pixy data */
     pixy_sub.update();
     const pixy_vector_s &pixy = pixy_sub.get();
     motorControl = raceTrack(pixy/*, PID*/);
 
     NxpCupWork::roverSteerSpeed(motorControl, fd);
 
     perf_end(_loop_perf);
 }
 
 int NxpCupWork::task_spawn(int argc, char *argv[])
 {
     NxpCupWork *instance = new NxpCupWork();
 
     if (instance) {
         _object.store(instance);
         _task_id = task_id_is_work_queue;
 
         if (instance->init()) {
             return PX4_OK;
         }
 
     } else {
         PX4_ERR("alloc failed");
     }
 
     delete instance;
     _object.store(nullptr);
     _task_id = -1;
 
     return PX4_ERROR;
 }
 
 int NxpCupWork::print_status()
 {
     perf_print_counter(_loop_perf);
     perf_print_counter(_loop_interval_perf);
     //PX4_INFO("Distance sensor data: %f", (double)distance_sensor_data.current_distance);
     return 0;
 }
 
 int NxpCupWork::custom_command(int argc, char *argv[])
 {
     return print_usage("unknown command");
 }
 
 int NxpCupWork::print_usage(const char *reason)
 {
     if (reason) {
         PX4_WARN("%s\n", reason);
     }
 
     PRINT_MODULE_DESCRIPTION(
         R"DESCR_STR(
 ### Description
 Example of a simple module running out of a work queue.
 
 )DESCR_STR");
 
     PRINT_MODULE_USAGE_NAME("work_item_example", "template");
     PRINT_MODULE_USAGE_COMMAND("start");
     PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
 
     return 0;
 }
 
 extern "C" __EXPORT int nxpcup_work_main(int argc, char *argv[])
 {
     return NxpCupWork::main(argc, argv);
 }
 