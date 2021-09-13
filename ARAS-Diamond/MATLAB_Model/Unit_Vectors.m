%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [A] = Unit_Vectors(theta__1,theta__2,alpha,beta,gamma,phi)

a=[0,0,1]';
b=[sin(alpha)*cos(theta__2),sin(alpha)*sin(theta__2),cos(alpha)]';
d=[sin(gamma)*cos(phi),sin(gamma)*sin(phi),cos(gamma)]';
c=[sin(alpha)*cos(theta__1),sin(alpha)*sin(theta__1),cos(alpha)]';

A=[a;b;c;d];

end

