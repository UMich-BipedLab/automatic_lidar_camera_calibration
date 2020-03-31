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
% clc, clear
% test2D_nonhomo = rand(2,5);
% test2D_homo = [test2D_nonhomo; ones(1,5)];
% test3D_nonhomo = rand(3, 5);
% test3D_homo = [test3D_nonhomo; ones(1,5)];
% test_random = rand(4, 5);
% t_checkHomogeneousCorners(test2D_nonhomo)
% t_checkHomogeneousCorners(test2D_homo)
% t_checkHomogeneousCorners(test3D_nonhomo)
% t_checkHomogeneousCorners(test3D_homo)
% t_checkHomogeneousCorners(test_random)


function corners = checkHomogeneousCorners(corners)
    % Assuming corners is either 2xn, 3xn or 4xn
    % check 2D or 3D
    corners = makeWideMatrix(corners);
    [n, ~] = size(corners);
    if n < 2 || n > 4
        error("wrong usage: input rows are smaller than 2 or bigger than 4 (currently is %i)", n)
    elseif n == 2 % 2D non-homo
        corners = [corners; 
                   ones(1, size(corners, 2))];
    elseif n == 3
        if ~isempty(corners(3, corners(3, :)==1)) % 2D homogeneous
            
            return;
        else % 3D none-homo
            corners = [corners; 
                       ones(1, size(corners, 2))];
        end
    elseif n==4
        if isempty(corners(4, corners(4, :)==1)) % 3D but not in homogeneous
            error("wrong usage: mean of the last rows is not one but %f", mean(corners(4,:), 2))
        end
    end
end