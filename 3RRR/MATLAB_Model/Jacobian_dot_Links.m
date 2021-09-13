%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [dJW1_1,dJW2_1,dJW1_2,dJW2_2,dJW1_3,dJW2_3] = Jacobian_dot_Links(J,Jp,Jdot,Jpdot,V1,W1,U1,V2,W2,U2,V3,W3,U3,X,Xdot,theta,Theta_dot,Gamma,Beta,Eta,Lambda,alpha__1)

    
    %% Vector Derivative Analysis
    
    [hdot,P1dot,P2dot,dW] =Vector_Derivatives(X,Xdot,theta,Theta_dot,Gamma,Beta,Eta,Lambda,alpha__1); 

    dW1=dW(:,1);
    dW2=dW(:,2);
    dW3=dW(:,3);

    %% Jacobian dot Analysis
    
    dJW1_1=Jdot(1,:).*U1;
    dJW1_2=Jdot(2,:).*U2;
    dJW1_3=Jdot(3,:).*U3;
    
    dJW12_1=Jpdot(1,:).*W1+Jp(1,:).*dW1;
    dJW12_2=Jpdot(2,:).*W2+Jp(2,:).*dW2;
    dJW12_3=Jpdot(3,:).*W3+Jp(3,:).*dW3;

    dJW2_1=dJW1_1+dJW12_1;
    dJW2_2=dJW1_2+dJW12_2;
    dJW2_3=dJW1_3+dJW12_3;

    
    
end

