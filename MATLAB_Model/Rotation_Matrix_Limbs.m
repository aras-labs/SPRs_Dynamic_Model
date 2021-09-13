%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [R__1i,R__2i] = Rotation_Matrix_Limbs(X,theta__i,psi__i,Gamma,lambda__i,alpha__1i)

 R__1i=Rz(lambda__i)*Rx(Gamma-pi)*Rz(theta__i);
R__12i=Rz(lambda__i)*Rx(Gamma-pi)*Rz(theta__i)*Rx(alpha__1i)*Rz(psi__i);

R__2i=R__12i;


end

