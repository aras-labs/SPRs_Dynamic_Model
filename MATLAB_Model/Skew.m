%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [S] = Skew(a)


x1=a(1);
x2=a(2);
x3=a(3);


S=[0,-x3,x2;x3,0,-x1;-x2,x1,0];


end

