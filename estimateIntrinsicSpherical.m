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


function  [delta, plane] = estimateIntrinsicSpherical(num_beams, num_targets, num_scans, data_split_with_ring, data_split_with_ring_raw)
    delta(num_beams).D = struct();
    delta(num_beams).theta = struct();
    delta(num_beams).phi = struct();
    %%
    for i = 1: num_scans
        % Calculate 'ground truth' points by projecting the angle onto the
        % normal plane
        %
        % Assumption: we have the normal plane at this step in the form:
        % plane_normal = [nx ny nz]

        % example normal lies directly on the x axis
        opt.corners.rpy_init = [45 2 3];
        opt.corners.T_init = [2, 0, 0];
        opt.corners.H_init = eye(4);
        opt.corners.method = "Constraint Customize"; %% will add more later on
        opt.corners.UseCentroid = 1;

        plane = cell(1,num_targets);
        for t = 1:num_targets
            X = [];
            for j = 1: num_beams
                X = [X,data_split_with_ring_raw{t}(j).points];
            end
            [plane{t}, ~] = estimateNormal(opt.corners, X(1:3, :), 0.8051);
        end

        opt.delta.D_corr_init = 0;
        opt.delta.theta_corr_init = 0;
        opt.delta.phi_corr_init = 0;
        delta = estimateDeltaSpherical(opt.delta, data_split_with_ring, plane, delta(num_beams), num_beams, num_targets);
    end

end
