%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [theta_i] = Inverse_Kinematic(X,Gamma,beta,lambda__i,eta__i,alpha__1i,alpha__2i)

q__1=X(1);
q__2=X(2);
q__3=X(3);

One=[0;0;1];

%% Calculation unit Vector v
R_ee=Rz(q__1)*Ry(q__2)*Rx(q__3);
v1S=Rz(eta__i)*Rx(-beta)*One;
V_i=R_ee*v1S; 

F1=[sin(alpha__1i) 0 0; 0 -sin(alpha__1i) 0; 0 0 cos(alpha__1i);];
C=F1*Rx(Gamma-pi)'*Rz(lambda__i)'*V_i;
a=C(2);
b=C(1);
c=cos(alpha__2i)-C(3);

theta_i=atan2(b,a)-atan2(sqrt(a^2+b^2-c^2),c);
%theta_i=atan2(b,a)+atan2(sqrt(a^2+b^2-c^2),c);


end

