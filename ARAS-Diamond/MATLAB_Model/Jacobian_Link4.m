%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J4] = Jacobian_Link4(alpha,beta,gamma,A,B,a,b,c,d)


    K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
    Q=((cos(gamma)*sin(A)+sin(gamma)*K*cos(A)))/((cos(B)*sin(beta)));
    
    
        %Bdot=gamma_dot*Q;
%     theta__1_dot=phi_dot+K*gamma_dot
%     
%     Omega4=(phi_dot+K*gamma_dot)*a+gamma_dot*Q*c;
    
J4=[a,K*a+Q*c];

end

