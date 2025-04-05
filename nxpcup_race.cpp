/****************************************************************************
 *
 *   Copyright 2019 NXP.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

/**
 * @file hello_example.cpp
 * Race code for NXP Cup
 *
 * @author Katrin Moritz
 */

 #include "nxpcup_race.h"

 #include <stdio.h>
 #include <string.h>
 #include <math.h>
 
 int flag = 0 ;
 static uint8_t w;
 // float Kp = 0.002f;
 float fast = 0.3f;
 
 uint8_t get_num_vectors(Vector &vec1, Vector &vec2) {
     uint8_t numVectors = 0;
     if(!(vec1.m_x0 == 0 && vec1.m_x1 == 0 && vec1.m_y0 == 0 && vec1.m_y1 == 0)) numVectors++;
     if(!(vec2.m_x0 == 0 && vec2.m_x1 == 0 && vec2.m_y0 == 0 && vec2.m_y1 == 0)) numVectors++;
     return numVectors;
 }
 
 // float calculate_error(float line_center_x, float camera_center_x) {
 //     return line_center_x - camera_center_x;
 // }
 
 // float calculate_line_center(Vector &line) {
 //     // Weight by line length (longer lines have more influence)
 //     float line_length = sqrt(pow(line.m_x1 - line.m_x0, 2) + pow(line.m_y1 - line.m_y0, 2));
 //     float weighted_center = (line.m_x0 + line.m_x1) / 2.0f;
 //     return (weighted_center * (line_length / 100.0f));
 // }
 
 Vector copy_vectors(const pixy_vector_s &pixy, uint8_t num) {
     Vector vec;
     if(num == 1) {
         vec.m_x0 = pixy.m0_x0;
         vec.m_x1 = pixy.m0_x1;
         vec.m_y0 = pixy.m0_y0;
         vec.m_y1 = pixy.m0_y1;
     }
     if(num == 2) {
         vec.m_x0 = pixy.m1_x0;
         vec.m_x1 = pixy.m1_x1;
         vec.m_y0 = pixy.m1_y0;
         vec.m_y1 = pixy.m1_y1;
     }
     return vec;
 }
 
 // roverControl control_track(const pixy_vector_s &pixy) {
 //     roverControl control = {0.0f, 0.0f}; // Default neutral control
 
 //     // Center of the camera's view (assumed to be in the middle of the camera's resolution)
 //     float camera_center_x = 160.0f; // Assuming a 320x240 image
 
 //     // Get the first detected line's center (you can extend this to handle multiple lines)
 //     Vector line = copy_vectors(pixy, 1); // Assuming the 1st line detection
 
 //     // Calculate the line center
 //     float line_center_x = calculate_line_center(line);
 
 //     // Calculate the error in X-axis (how far the line center is from the camera's center)
 //     float error = calculate_error(line_center_x, camera_center_x);
 
 //     // Apply proportional control to the error
 //     control.steer = error * Kp;
 
 //     // Limit the steer value to be between -1 and 1
 //     if (control.steer > 1.0f) control.steer = 1.0f;
 //     if (control.steer < -1.0f) control.steer = -1.0f;
 
 //     // Adjust speed based on the error (if the error is too large, slow down)
 //     if (error > 50 || error < -50) {
 //         control.speed = 0.3f; // Slow down if the error is large (likely off the track)
 //     } else {
 //         control.speed = 0.8f; // Normal speed if we're on track
 //     }
 
 //     return control;
 // }
 float produit_scalaire(Vector vec1, Vector vec2)
 {
     float x;
     float y;
     x = (vec1.m_x1 +vec1.m_x0) * (vec2.m_x1 + vec2.m_x0);
     y = (vec1.m_y1 +vec1.m_y0) * (vec2.m_y1 + vec2.m_y0);
     return (x*y);
 }
 
 roverControl raceTrack(const pixy_vector_s &pixy)
 {
     Vector main_vec;
     Vector vec1 = copy_vectors(pixy, 1);
     Vector vec2 = copy_vectors(pixy, 2);
     // roverControl control = control_track(pixy);
     roverControl control;
     float dy;
     float dx;
     float angle;
     static hrt_abstime no_line_time = 0;
     hrt_abstime time_diff = 0;
     static bool first_call = true;
     uint8_t num_vectors = get_num_vectors(vec1,vec2);
     static uint8_t track_width;
     float diff;
     if (flag++ == 0)
     {
         w = abs(vec1.m_x0-vec2.m_x0)/2;
     }
     switch (num_vectors) {
         case 0:{
             if (first_call) {
                 no_line_time = hrt_absolute_time();
                 first_call = false;
             } else {
                 time_diff = hrt_elapsed_time_atomic(&no_line_time);
                 control.steer = 0.0f;
                 if (time_diff > 10000) {
                     control.steer = 0.0f;
                     control.speed = 0.0f;
                 }
             }
             break;
         }
         case 2:{
             first_call = true;
             fast += 0.02f;
             control.speed = fast;
             if (control.speed > 0.6f) {
                 control.speed = 0.4f;
                 fast = 0.4f;
             }
             float p = produit_scalaire(vec1, vec2);
             if (p < 4 || p >= 0) {
                 control.steer = -0.3;
                 break;
             } else if (p > -4 || p <= 0) {
                 control.steer = 0.3;
                 break;
             }
             diff = vec1.m_x0 - vec2.m_x0;
             track_width = abs(diff);
             main_vec.m_x1 = (vec1.m_x1 + vec2.m_x1) / 2;
             main_vec.m_x0 = (vec1.m_x0 + vec2.m_x0) / 2;
             main_vec.m_y0 = (vec1.m_y0 + vec2.m_y0) / 2;
             main_vec.m_y1 = (vec1.m_y1 + vec2.m_y1) / 2;
     
             dy = (main_vec.m_y0 - main_vec.m_y1);
             dx = (main_vec.m_x1 - main_vec.m_x0);
     
             angle = atan2(dy, dx) * 180 / M_PI;
             if (dx < 1 || dx > -1) {
                 angle = 0;
             }
             control.steer = angle / 60;
             if (track_width < w) {
                 control.speed = 0.1f;
                 if (diff < 0) {
                     control.steer = 1;
                 } else {
                     control.steer = -1;
                 }
             }
             if (control.steer < -1) {
                 control.steer = -1;
             }
             if (control.steer > 1) {
                 control.steer = 1;
             }
             break;
         }
         default:{
             first_call = true;
     
             // IF X0(TAIL) < X1(HEAD) ==>> LEFT VECTOR ==>> STEERING TO THE RIGHT
             if (vec1.m_x0 > vec1.m_x1) {
                 control.steer = 1;
             } else {
                 control.steer = -1;
             }
             control.speed = 0.1f;
             break;
         }
     }
     
 
     return control;
 }