%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [YS_Reg] = Slotine_Regressor(X,X_dot,XR_dot,XR_dot_dot)
     
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

%     phi_dot_dot=X_dot_dot(1);
%     gamma_dot_dot=X_dot_dot(2);
    
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
%     alpha1=J1*X_dot_dot+J1dot*X_dot;
   
    J2dot=Jacobian_dot_Link2(alpha,beta,gamma,A,a,b,c,d,gamma_dot);
%     alpha2=J2*X_dot_dot+J2dot*X_dot;
    
    J3dot=Jacobian_dot_Link3(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J1dot,theta__2_dot);
%     alpha3=J3*X_dot_dot+J3dot*X_dot;
  
    J4dot=Jacobian_dot_Link4(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J2dot,theta__1_dot);
%     alpha4=J4*X_dot_dot+J64dot*X_dot;    
    
        

%% Converting Coordinate

%Link1
    Omega1Local=R1'*Omega1;   
    Omega1_R=J1*XR_dot;
    Omega1_R_Local=R1'*Omega1_R;
    
    Alpha1_R=J1*XR_dot_dot+J1dot*XR_dot;
    Alpha1_R_Local=R1'*Alpha1_R;
    
    Omega1_RH=Hat_Creator(Omega1_R_Local);
    Alpha1_RH=Hat_Creator(Alpha1_R_Local); 
    
    g1local=R1'*g;

 %Link2
    Omega2Local=R2'*Omega2;
    Omega2_R=J2*XR_dot;
    Omega2_R_Local=R2'*Omega2_R;
    
    Alpha2_R=J2*XR_dot_dot+J2dot*XR_dot;
    Alpha2_R_Local=R2'*Alpha2_R;
    
    Omega2_RH=Hat_Creator(Omega2_R_Local);
    Alpha2_RH=Hat_Creator(Alpha2_R_Local); 
    
    g2local=R2'*g;
    
    %Link3
    Omega3Local=R3'*Omega3;
    Omega3_R=J3*XR_dot;
    Omega3_R_Local=R3'*Omega3_R;
    
    Alpha3_R=J3*XR_dot_dot+J3dot*XR_dot;
    Alpha3_R_Local=R3'*Alpha3_R;
    
    Omega3_RH=Hat_Creator(Omega3_R_Local);
    Alpha3_RH=Hat_Creator(Alpha3_R_Local); 
    
    g3local=R3'*g;
    
    %Link4
    Omega4Local=R4'*Omega4;
    Omega4_R=J4*XR_dot;
    Omega4_R_Local=R4'*Omega4_R;
    
    Alpha4_R=J4*XR_dot_dot+J4dot*XR_dot;
    Alpha4_R_Local=R4'*Alpha4_R;
    
    Omega4_RH=Hat_Creator(Omega4_R_Local);
    Alpha4_RH=Hat_Creator(Alpha4_R_Local); 
    
    g4local=R4'*g;



%%

    RegS1=J1'*[R1*Skew(g1local),R1*(Alpha1_RH+Skew(Omega1Local)*Omega1_RH)];
    RegS2=J2'*[R2*Skew(g2local),R2*(Alpha2_RH+Skew(Omega2Local)*Omega2_RH)];
    RegS3=J3'*[R3*Skew(g3local),R3*(Alpha3_RH+Skew(Omega3Local)*Omega3_RH)];
    RegS4=J4'*[R4*Skew(g4local),R4*(Alpha4_RH+Skew(Omega4Local)*Omega4_RH)];

    YS_Reg=[RegS1,RegS2,RegS3,RegS4];
%     YS_Reg=[RegS4];




end

