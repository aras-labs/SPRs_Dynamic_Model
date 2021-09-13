%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [V_i,U_i,W_i] = Unit_Vectors(X,theta__i,Gamma,beta,eta__i,lambda__i,alpha__1i)
q__1=X(1);
q__2=X(2);
q__3=X(3);

%% Symbolic Define(Verified Numerically)

% v_i=[-cos(q__1) * cos(q__2) * sin(eta__i) * sin(beta) + (-sin(q__1) * cos(q__3) + cos(q__1) * sin(q__2) * sin(q__3)) * cos(eta__i) * sin(beta) + (sin(q__1) * sin(q__3) + cos(q__1) * sin(q__2) * cos(q__3)) * cos(beta) -sin(q__1) * cos(q__2) * sin(eta__i) * sin(beta) + (cos(q__1) * cos(q__3) + sin(q__1) * sin(q__2) * sin(q__3)) * cos(eta__i) * sin(beta) + (-cos(q__1) * sin(q__3) + sin(q__1) * sin(q__2) * cos(q__3)) * cos(beta) sin(q__2) * sin(eta__i) * sin(beta) + cos(q__2) * sin(q__3) * cos(eta__i) * sin(beta) + cos(q__2) * cos(q__3) * cos(beta)]';
% u_i=[-sin(lambda__i) * sin(Gamma) cos(lambda__i) * sin(Gamma) -cos(Gamma)]';
% w_i=[-(-cos(lambda__i) * sin(theta__i) + sin(lambda__i) * cos(Gamma) * cos(theta__i)) * sin(alpha__1i) - sin(lambda__i) * sin(Gamma) * cos(alpha__1i) -(-sin(lambda__i) * sin(theta__i) - cos(lambda__i) * cos(Gamma) * cos(theta__i)) * sin(alpha__1i) + cos(lambda__i) * sin(Gamma) * cos(alpha__1i) sin(Gamma) * cos(theta__i) * sin(alpha__1i) - cos(Gamma) * cos(alpha__1i)]';

%%
One=[0;0;1];
F1=[sin(alpha__1i) 0 0; 0 -sin(alpha__1i) 0; 0 0 cos(alpha__1i);];
F2=[sin(theta__i);cos(theta__i);1];

R_ee=Rz(q__1)*Ry(q__2)*Rx(q__3);
v1S=Rz(eta__i)*Rx(-beta)*One;

V_i=R_ee*v1S;   
U_i=Rz(lambda__i)*Rx(Gamma-pi)*One;

W_i=Rz(lambda__i)*Rx(Gamma-pi)*F1*F2;


end

