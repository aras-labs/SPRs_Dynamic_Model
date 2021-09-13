%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J] = Passive_Jacobian(a,b,c,d,gamma,beta)


ad=cross(a,d)/(sin(gamma));

J_x=[dot(ad,a),dot(ad,ad);dot(ad,a),dot(ad,ad)];
J_q=[-dot(ad,c),0;0,-dot(ad,b)];

J=inv(J_q)*(J_x);


end

