%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J3] = Jacobian_Link3(alpha,beta,gamma,A,B,a,b,c,d)



K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
Q=((cos(gamma)*sin(A)+sin(gamma)*K*cos(A)))/((cos(B)*sin(beta)));


J3=[a,-K*a-Q*b];

end

