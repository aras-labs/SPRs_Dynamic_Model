%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [ Y_Reg] = Full_Regressor(X,X_dot,X_dot_dot)
       
   %% Geometry Define

alpha=pi/4;
beta=pi/4;


%%
    g=[0,-10,0]';


%%
        
    phi=X(1);
    gamma=X(2);
      
    phi_dot=X_dot(1);
    gamma_dot=X_dot(2);

    phi_dot_dot=X_dot_dot(1);
    gamma_dot_dot=X_dot_dot(2);
    
        %% Inverse Kinematic

    [theta_New,Passive]=Inverse_Kinematic(gamma,phi,alpha,beta);
    theta__1=theta_New(1);
    theta__2=theta_New(2);   
        
    A=Passive(1);
    B=Passive(2);
    D=Passive(3);
    
        
    %% Calculating Unit Vectors
    Unit= Unit_Vectors(theta__1,theta__2,alpha,beta,gamma,phi);
    a=Unit(1:3);b=Unit(4:6);c=Unit(7:9);d=Unit(10:12);
    
        %% Rotation Matrix 
    R1=Rotation_Link1(a,b,c,d);
    R2=Rotation_Link2(a,b,c,d);
    R3=Rotation_Link3(a,b,c,d);
    R4=Rotation_Link4(a,b,c,d);

    %% Jacobian Analysis of End_Effector:
    J= Jacobian(alpha,beta,gamma,A);             %Scalar approach 
    %J_a=Actuated_Jacobian(a,b,c,d,gamma,beta);   %Verified

    %% Passive Jacobian
    J_p=Passive_Jacobian(a,b,c,d,gamma,beta);     % Semi Verified
    
    %K=-((cot(alpha)-cos(A)*cot(gamma))/(sin(A)));
    %Q=((cos(gamma)*sin(A)+sin(gamma)*K*cos(A)))/((cos(B)*sin(beta)));
    %J_P=[0,-Q;0,Q];
    
    %J_p-J_P
    
    q_dot=J*X_dot;
    theta__1_dot=q_dot(1);
    theta__2_dot=q_dot(2);  
        
        
        
        %% Jacobain of Link 1
    J1=Jacobian_Link1(alpha,beta,gamma,A,a,b,c,d);
    Omega1=J1*X_dot;  

    %% Jacobain of Link 2
    J2=Jacobian_Link2(alpha,beta,gamma,A,a,b,c,d);
    Omega2=J2*X_dot;  

    %% Jacobain of Link 3
    J3=Jacobian_Link3(alpha,beta,gamma,A,B,a,b,c,d);
    Omega3=J3*X_dot;

    %% Jacobain of Link 3
    J4=Jacobian_Link4(alpha,beta,gamma,A,B,a,b,c,d);
    Omega4=J4*X_dot;
        
        
        
            %% Acceleration_Analysis
    
    J1dot=Jacobian_dot_Link1(alpha,beta,gamma,A,a,b,c,d,gamma_dot);
    alpha1=J1*X_dot_dot+J1dot*X_dot;
   
    J2dot=Jacobian_dot_Link2(alpha,beta,gamma,A,a,b,c,d,gamma_dot);
    alpha2=J2*X_dot_dot+J2dot*X_dot;
    
    J3dot=Jacobian_dot_Link3(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J1dot,theta__2_dot);
    alpha3=J3*X_dot_dot+J3dot*X_dot;
  
    J4dot=Jacobian_dot_Link4(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J2dot,theta__1_dot);
    alpha4=J4*X_dot_dot+J4dot*X_dot;    
    
        
        %% Regressor Creator
    
    % Link1
    Omega1Local=R1'*Omega1;
    alpha1Local=R1'*alpha1;
    g1local=R1'*g;
    Omega1H=Hat_Creator(Omega1Local);
    alpha1H=Hat_Creator(alpha1Local);   
    Reg1=J1'*[R1*Skew(g1local),R1*(alpha1H+Skew(Omega1Local)*Omega1H)];
    
    % Link2
    Omega2Local=R2'*Omega2;
    alpha2Local=R2'*alpha2;
    g2local=R2'*g;
    Omega2H=Hat_Creator(Omega2Local);
    alpha2H=Hat_Creator(alpha2Local);   
    Reg2=J2'*[R2*Skew(g2local),R2*(alpha2H+Skew(Omega2Local)*Omega2H)];
    
    % Link3
    Omega3Local=R3'*Omega3;
    alpha3Local=R3'*alpha3;
    g3local=R3'*g;
    Omega3H=Hat_Creator(Omega3Local);
    alpha3H=Hat_Creator(alpha3Local);   
    Reg3=J3'*[R3*Skew(g3local),R3*(alpha3H+Skew(Omega3Local)*Omega3H)];
    
    % Link4
    Omega4Local=R4'*Omega4;
    alpha4Local=R4'*alpha4;
    g4local=R4'*g;
    Omega4H=Hat_Creator(Omega4Local);
    alpha4H=Hat_Creator(alpha4Local);   
    Reg4=J4'*[R4*Skew(g4local),R4*(alpha4H+Skew(Omega4Local)*Omega4H)];
    
    Y_Reg=[Reg1,Reg2,Reg3,Reg4];

end

