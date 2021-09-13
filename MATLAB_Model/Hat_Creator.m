%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [OMG] = Hat_Creator(omega)

wx=omega(1);
wy=omega(2);
wz=omega(3);

OMG=[wx,wy,wz,0,0,0;0,wx,0,wy,wz,0;0,0,wx,0,wy,wz];
end

