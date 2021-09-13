%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [R] = Rotation_Link1(a,b,c,d)


C=cross(a,b);
D=cross(C,a);

R=[D/norm(D),C/norm(C),a];


end

