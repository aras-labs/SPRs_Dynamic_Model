%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J] = Jacobian(alpha,beta,gamma,A)

K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
J=[1,K;1,-K];

end

