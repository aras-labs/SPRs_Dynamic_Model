%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J1dot] = Jacobian_dot_Link1(alpha,beta,gamma,A,a,b,c,d,gamma_dot)

G=gamma;
zero=[0;0;0];
K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
R=(((-cos(alpha) * cos(G) ^ 2 + cos(alpha)) * cos(A) - sin(G) * sin(alpha) * cos(G)) * K - sin(A) * cos(A) * sin(alpha)) / sin(G) ^ 2 / sin(alpha) / sin(A) ^ 2;

J1dot=[zero,-R*gamma_dot*a];



end

