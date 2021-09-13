%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [Jdot,Jpdot,Edot] = Jacobian_dot(X,Xdot,theta,Theta_dot,V1,W1,U1,V2,W2,U2,V3,W3,U3,Gamma,Beta,Eta,Lambda,alpha__1)

q1=X(1);
q2=X(2);
q3=X(3);

Q1=Xdot(1);
Q2=Xdot(2);
Q3=Xdot(3);

E=[0 -sin(q1) cos(q1) * cos(q2); 0 cos(q1) sin(q1) * cos(q2); 1 0 -sin(q2);];
Edot=[0 -Q1 * cos(q1) -Q1 * sin(q1) * cos(q2) - cos(q1) * Q2 * sin(q2); 0 -Q1 * sin(q1) Q1 * cos(q1) * cos(q2) - sin(q1) * Q2 * sin(q2); 0 0 -Q2 * cos(q2);];

%% Vector Derivatives (Calculated with Maple)

[hdot,P1dot,P2dot,dW] =Vector_Derivatives(X,Xdot,theta,Theta_dot,Gamma,Beta,Eta,Lambda,alpha__1); 

%% Chain 1

P1_1=cross(V1,W1);
P2_1=cross(U1,V1);
h1=dot(P1_1,U1);

h1dot=hdot(1);
P1dot_1= P1dot(:,1);
P2dot_1= P2dot(:,1);

% J1_1=(h1^-1*P1_1'*E);
% J2_1=(h1^-1*P2_1'*E);

dJ1_1=((P1dot_1'*E)/h1+(P1_1'*Edot)/h1-(P1_1'*E*h1dot)/h1^2);
dJ2_1=((P2dot_1'*E)/h1+(P2_1'*Edot)/h1-(P2_1'*E*h1dot)/h1^2);


%% Chain2

P1_2=cross(V2,W2);
P2_2=cross(U2,V2);
h2=dot(P1_2,U2);

h2dot=hdot(2);
P1dot_2= P1dot(:,2);
P2dot_2= P2dot(:,2);

% J1_2=(h2^-1*P1_2'*E);
% J2_2=(h2^-1*P2_2'*E);

dJ1_2=((P1dot_2'*E)/h2+(P1_2'*Edot)/h2-(P1_2'*E*h2dot)/h2^2);
dJ2_2=((P2dot_2'*E)/h2+(P2_2'*Edot)/h2-(P2_2'*E*h2dot)/h2^2);

%% Chain3

P1_3=cross(V3,W3);
P2_3=cross(U3,V3);
h3=dot(P1_3,U3);

h3dot=hdot(3);
P1dot_3= P1dot(:,3);
P2dot_3= P2dot(:,3);

% J1_3=(h3^-1*P1_3'*E);
% J2_3=(h3^-1*P2_3'*E);

dJ1_3=((P1dot_3'*E)/h3+(P1_3'*Edot)/h3-(P1_3'*E*h3dot)/h3^2);
dJ2_3=((P2dot_3'*E)/h3+(P2_3'*Edot)/h3-(P2_3'*E*h3dot)/h3^2);

%% Jacobian Define

Jdot=[dJ1_1;
      dJ1_2;
      dJ1_3];

%% Passive Jacobian Define
Jpdot=[dJ2_1;
      dJ2_2;
      dJ2_3];

end

