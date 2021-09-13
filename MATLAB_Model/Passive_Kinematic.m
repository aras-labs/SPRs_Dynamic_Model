%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [psi__i] = Passive_Kinematic(U_i,V_i,W_i,alpha__1i,alpha__2i)



n1_i=(cross(U_i,W_i))/(abs(sin(alpha__1i)));
n2_i=(cross(W_i,V_i))/(abs(sin(alpha__2i)));


c_psi_i=dot(n1_i,n2_i);

psi__i=acos(c_psi_i);



end

