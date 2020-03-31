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


function cost = compare3D2DLines(lines_3D, lines_2D, P)
    N = size(lines_3D, 2);
    cost = 0;
    for i = 1:N
        line_3D = lines_3D(i).line;
        x = line_3D.dX*(line_3D.m)';    % project residuals on R(:,1)
        x_min = min(x);
        x_max = max(x);
        dx = x_max-x_min;
        Xa = P * [(x_min-0.05*dx)*line_3D.m + line_3D.X0 1]';
        Xa = Xa ./ Xa(3,:);
        Xb = P * [(x_max+0.05*dx)*line_3D.m + line_3D.X0 1]';
        Xb = Xb ./ Xb(3,:);

        cost1 = pointToLineDistance((Xa(1:2,1))', ...
                            [lines_2D(i).x(1) lines_2D(i).y(1)], ...
                            [lines_2D(i).x(2) lines_2D(i).y(2)]);
        cost2 = pointToLineDistance((Xb(1:2,1))', ...
                            [lines_2D(i).x(1) lines_2D(i).y(1)], ...
                            [lines_2D(i).x(2) lines_2D(i).y(2)]);
        cost = cost + cost1 + cost2;
    end
end

%         x=dX*R(:,1);    % project residuals on R(:,1)
%         x_min=min(x);
%         x_max=max(x);
%         dx=x_max-x_min;
%         Xa=(x_min-0.05*dx)*R(:,1)' + X_ave;
%         Xb=(x_max+0.05*dx)*R(:,1)' + X_ave;
%         X_end=[Xa;Xb];
%         figure(3000)
%         axis equal
%         hold on
%         Hl=plot3(X_end(:,1),X_end(:,2),X_end(:,3),'-r','LineWidth',3); % best fit line
