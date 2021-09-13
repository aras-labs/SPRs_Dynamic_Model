%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J] = Jacobian_In_Dynamic(X)

phi=X(1);
gamma=X(2);

%% Geometry Define

alpha=pi/4;
beta=pi/4;
g=[0,-10,0]';


    %% Inverse Kinematic
    [theta_New,Passive]=Inverse_Kinematic(gamma,phi,alpha,beta);
    theta__1=theta_New(1);  theta__2=theta_New(2);    
    A=Passive(1);B=Passive(2);D=Passive(3);

    %% Jacobian Analysis of End_Effector:
    J= Jacobian(alpha,beta,gamma,A);             %Scalar approach 
  

end

