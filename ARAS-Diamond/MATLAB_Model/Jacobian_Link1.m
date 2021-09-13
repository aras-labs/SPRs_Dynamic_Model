%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J1] = Jacobian_Link1(alpha,beta,gamma,A,a,b,c,d)


K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
J1=[a,-K*a];

end

