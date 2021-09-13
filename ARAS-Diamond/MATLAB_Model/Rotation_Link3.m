%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [R] = Rotation_Link3(a,b,c,d)


C=cross(b,d);
D=cross(C,b);

R=[D/norm(D),C/norm(C),b];

end

