%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [JW1_1,JW2_1,JW1_2,JW2_2,JW1_3,JW2_3] = Jacobian_Links(J,Jp,V1,W1,U1,V2,W2,U2,V3,W3,U3)


    %% Jacobian Analysis
    JW1_1=J(1,:).*U1;
    JW1_2=J(2,:).*U2;
    JW1_3=J(3,:).*U3;

    JW12_1=Jp(1,:).*W1;
    JW12_2=Jp(2,:).*W2;
    JW12_3=Jp(3,:).*W3;

    JW2_1=JW1_1+JW12_1;
    JW2_2=JW1_2+JW12_2;
    JW2_3=JW1_3+JW12_3;
    

    
    
end

