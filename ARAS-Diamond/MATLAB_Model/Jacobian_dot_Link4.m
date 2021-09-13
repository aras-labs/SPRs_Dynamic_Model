%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J4dot] = Jacobian_dot_Link4(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J2dot,theta__1_dot)

G=gamma;
zero=[0;0;0];
K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
R=(((-cos(alpha) * cos(G) ^ 2 + cos(alpha)) * cos(A) - sin(G) * sin(alpha) * cos(G)) * K - sin(A) * cos(A) * sin(alpha)) / sin(G) ^ 2 / sin(alpha) / sin(A) ^ 2;


Q=((cos(gamma)*sin(A)+sin(gamma)*K*cos(A)))/((cos(B)*sin(beta)));

GG=gamma_dot;
QQ=(cos(B) * (-sin(A) * sin(G) * K + cos(A) * cos(G)) * K * GG + sin(B) * (cos(G) * sin(A) + sin(G) * K * cos(A)) * Q * GG + cos(B) * ((cos(G) * K * cos(A) - sin(A) * sin(G)) * GG + sin(G) * R * GG * cos(A))) / cos(B) ^ 2 / sin(beta);

cc=[-sin(theta__1)*sin(alpha);+cos(theta__1)*sin(alpha);0];

J14dot=[zero,(QQ*c+Q*cc*theta__1_dot)];

J4dot=J14dot+J2dot;

end

