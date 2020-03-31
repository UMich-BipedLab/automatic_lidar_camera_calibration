%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
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

function [X_clean, bag_data] = cleanLiDARTarget(scan_num, tag_num, bag_data, X, target_len, opt)
    [X_clean, X_ref, X_std, L_infinity] = cleanLiDARTargetCore(opt, X, target_len);
    bag_data.lidar_target(tag_num).scan(scan_num).clean_up.std = N*(X_std(1));
    bag_data.lidar_target(tag_num).scan(scan_num).clean_up.L_infinity = L_infinity;
    bag_data.lidar_target(tag_num).scan(scan_num).clean_up.L_1 = sum(X_ref(1:3,:), 1);
%     figure(200);
%     clf('reset')
%     scatter3(X_ref(1,:), X_ref(2,:), X_ref(3,:))
%     xlabel('x') 
%     ylabel('y') 
%     zlabel('z') 
%     axis equal
%     hold on;
%     scatter3(X_ref_clean_yz(1,:), X_ref_clean_yz(2,:), X_ref_clean_yz(3,:))
%     scatter3(X_ref_clean(1,:), X_ref_clean(2,:), X_ref_clean(3,:))
%     axis equal
%     view(90,0)
%     hold off;    
end