%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
 %}


clc
intrinsic_matrix = [616.3681640625, 0.0,            319.93463134765625;
                    0.0,            616.7451171875, 243.6385955810547;
                    0.0, 0.0, 1.0];
Good_dataset = 11;
Good_transorm_r = 82.0122;
Good_transorm_p = -0.0192 ;
Good_transorm_h = 87.7953;
Good_transform_T = [0.0228
   -0.2070
   -0.0783];
                
Bad_dataset = 4;
Bad_transorm_r = 81.9883;
Bad_transorm_p = -0.5840;
Bad_transorm_h = 87.2922;
Bad_transform_T = [    0.0053
   -0.2040
   -0.0544];                
                
% Good transform, bad training data                
X = [BagData(Bad_dataset).lidar_target(1).scan(1).corners];
Y = [BagData(Bad_dataset).camera_target(1).corners];
disp("Good transform, bad training data")
cost = cost4Points(Good_transorm_r, Good_transorm_p, Good_transorm_h, Good_transform_T, X, Y, intrinsic_matrix)

% bad transform, bad training data
X = [BagData(Bad_dataset).lidar_target(1).scan(1).corners];
Y = [BagData(Bad_dataset).camera_target(1).corners];
disp("bad transform, bad training data")
cost = cost4Points(Bad_transorm_r, Bad_transorm_p, Bad_transorm_h, Bad_transform_T, X, Y, intrinsic_matrix)

% Good transform, good training data
X = [BagData(Good_dataset).lidar_target(1).scan(1).corners];
Y = [BagData(Good_dataset).camera_target(1).corners];
disp("Good transform, good training data")
cost = cost4Points(Good_transorm_r, Good_transorm_p, Good_transorm_h, Good_transform_T, X, Y, intrinsic_matrix)

% bad transform, good training data
X = [BagData(Good_dataset).lidar_target(1).scan(1).corners];
Y = [BagData(Good_dataset).camera_target(1).corners];
disp("bad transform, good training data")
cost = cost4Points(Bad_transorm_r, Bad_transorm_p, Bad_transorm_h, Bad_transform_T, X, Y, intrinsic_matrix)
