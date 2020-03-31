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


function [model] = iQP(points,Beta)
%
%  My function to fit a rectangle using a QP
mu1=Beta(1);mu2=-1/Beta(1);
PhiEdges=points(:,1:end-1);
YEdges=points(:,end);
n=length(YEdges);

for k = 1:100
    %Set up for rectangle; then add more for a square.
    %First line is m1*m2=-1
    %Next two are opposite sides are parallel.
    Aeq=[mu2 mu1 0 0 zeros(1,4); ...
        1    0 -1 0 zeros(1,4); ...
        0    1  0 -1 zeros(1,4)];
    beq=[-1+mu1*mu2; 0; 0];
    Q=PhiEdges'*PhiEdges;
    f=-(YEdges')*PhiEdges;
    Beta = quadprog(Q,f,[],[],Aeq,beq);
    mu1=Beta(1); mu2=Beta(2); mu3=Beta(3); mu4=Beta(4); b1=Beta(5);b2=Beta(6);b3=Beta(7);b4=Beta(8);
    
    V1=-[mu1 -1; mu2 -1]\[b1;b2];
    V2=-[mu2 -1; mu3 -1]\[b2;b3];
    V3=-[mu3 -1; mu4 -1]\[b3;b4];
    V4=-[mu4 -1; mu1 -1]\[b4;b1];
    
    
%     e1=norm(V1-V2)-d;
%     e2=norm(V2-V3)-d;
%     e3=norm(V3-V4)-d;
%     e4=norm(V4-V1)-d;
    
    delta=1e-7;
    if abs(1+mu1*mu2)<delta
        break
    end
    
end
model=Beta;
%n
abs(1+mu1*mu2)
max(abs(PhiEdges*Beta-YEdges))
model'


