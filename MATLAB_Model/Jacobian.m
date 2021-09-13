%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J,Jp,E] = Jacobian(X,V1,W1,U1,V2,W2,U2,V3,W3,U3)


q1=X(1);
q2=X(2);
q3=X(3);
E=[0 -sin(q1) cos(q1) * cos(q2); 0 cos(q1) sin(q1) * cos(q2); 1 0 -sin(q2);];

%% Chain 1

P1_1=cross(V1,W1);
P2_1=cross(U1,V1);
h1=dot(P1_1,U1);

%dot(cross(V1,W1),U1)-dot(cross(U1,V1),W1)

J1_1=(h1^-1*P1_1'*E);
J2_1=(h1^-1*P2_1'*E);


%% Chain2

P1_2=cross(V2,W2);
P2_2=cross(U2,V2);
h2=dot(P1_2,U2);

J1_2=(h2^-1*P1_2'*E);
J2_2=(h2^-1*P2_2'*E);


%% Chain3

P1_3=cross(V3,W3);
P2_3=cross(U3,V3);
h3=dot(P1_3,U3);

J1_3=(h3^-1*P1_3'*E);
J2_3=(h3^-1*P2_3'*E);


%% Jacobian Define

J=[J1_1;
   J1_2;
   J1_3];


%% Passive Jacobian Define
Jp=[J2_1;
    J2_2;
    J2_3];
  
    
end

