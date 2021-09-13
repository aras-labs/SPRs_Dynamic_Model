%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [Y_MP,Y_Links] = Full_Regressor(X,X_dot,Xddot)

%% Kinematic Parameters

Lambda=[0;120;240]*(pi/180);Eta=Lambda;
Beta=acos(sqrt(3)/3);Gamma=Beta;

alpha__1=[80*(pi/180);80*(pi/180);80*(pi/180)];
alpha__2=[70*(pi/180);70*(pi/180);70*(pi/180)];
g=[0;0;-9.80665];
     
%% Constants Define
    theta=zeros(3,1);psi=zeros(3,1);R1=zeros(3,3,3);R2=zeros(3,3,3);
  
%% Kinematic Analysis
    for j=1:3
    %% Inverse Kinematic    
    theta(j,1)=Inverse_Kinematic(X,Gamma,Beta,Lambda(j),Eta(j),alpha__1(j),alpha__2(j));
    %% Unit Vectors
	[V_i(:,j),U_i(:,j),W_i(:,j)] = Unit_Vectors(X,theta(j,1),Gamma,Beta,Eta(j),Lambda(j),alpha__1(j));
    %% Passive_Kinematc
    psi(j,1)=Passive_Kinematic(U_i(:,j),V_i(:,j),W_i(:,j),alpha__1(j),alpha__2(j));  
    %% Rotation Matrix
    [R1(:,:,j),R2(:,:,j)]=Rotation_Matrix_Limbs(X,theta(j,1),psi(j,1),Gamma,Lambda(j),alpha__1(j));
    %% Velocity Analysis(theta_dot)
    %[Theta_dot_II(j,1),Psi_dot_II(j,1)] = theta_dot(X,X_dot,U_i(:,j),V_i(:,j),W_i(:,j));
    end
    
%% Rotation Matrix of Moving Platform

    X1=X(1);X2=X(2);X3=X(3);
    Rp=Rz(X1)*Ry(X2)*Rx(X3);   
    
%% Discreate Chanins
    %% Chain 1
    V1=V_i(:,1);U1=U_i(:,1);W1=W_i(:,1);R1_1=R1(:,:,1);R2_1=R2(:,:,1);
    %% Chain 2
    V2=V_i(:,2);U2=U_i(:,2);W2=W_i(:,2);R1_2=R1(:,:,2);R2_2=R2(:,:,2);   
    %% Chain 3
    W3=W_i(:,3);U3=U_i(:,3);V3=V_i(:,3);R1_3=R1(:,:,3);R2_3=R2(:,:,3);
     
%% Jacobian Analysis
    
	[J,Jp,E] = Jacobian(X,V1,W1,U1,V2,W2,U2,V3,W3,U3);
    Theta_dot=J*X_dot;     
    %Psi_dot=Jp*X_dot; 

%% Jacobian Analysis of Each Link
     [Jw1_1,Jw2_1,Jw1_2,Jw2_2,Jw1_3,Jw2_3] = Jacobian_Links(J,Jp,V1,W1,U1,V2,W2,U2,V3,W3,U3);
        
%% Angular Velocity Analysis
     %% Chain 1
     omega1_1=Jw1_1*X_dot;omega2_1=Jw2_1*X_dot;
     %% Chain 2
     omega1_2=Jw1_2*X_dot;omega2_2=Jw2_2*X_dot;
     %% Chain 3
     omega1_3=Jw1_3*X_dot;omega2_3=Jw2_3*X_dot;  
    
%% End_Effector Velocity
     Omega_EE=E*X_dot;

    %% Jacobian Dot Analysis
         
 	 [Jdot,Jpdot,Edot] = Jacobian_dot(X,X_dot,theta,Theta_dot,V1,W1,U1,V2,W2,U2,V3,W3,U3,Gamma,Beta,Eta,Lambda,alpha__1);    
     %Theta_ddot=J*Xddot+Jdot*X_dot;
     %Psi_ddot=Jp*Xddot+Jpdot*X_dot;

%% End_Effector Acceleration
     Alpha_EE=E*Xddot+Edot*X_dot;
    
%% Jacobian Dot Analysis of Each Link 

    [dJw1_1,dJw2_1,dJw1_2,dJw2_2,dJw1_3,dJw2_3] = Jacobian_dot_Links(J,Jp,Jdot,Jpdot,V1,W1,U1,V2,W2,U2,V3,W3,U3,X,X_dot,theta,Theta_dot,Gamma,Beta,Eta,Lambda,alpha__1);
     
%% Angular Acceleration Analysis
    %% Chain 1
    alpha1_1=Jw1_1*Xddot+dJw1_1*X_dot;alpha2_1=Jw2_1*Xddot+dJw2_1*X_dot;
    %% Chain 2
    alpha1_2=Jw1_2*Xddot+dJw1_2*X_dot;alpha2_2=Jw2_2*Xddot+dJw2_2*X_dot;
    %% Chain 3    
    alpha1_3=Jw1_3*Xddot+dJw1_3*X_dot;alpha2_3=Jw2_3*Xddot+dJw2_3*X_dot;

      
%% Regressor Creator
    
    %% Chain 1
    Omega1Local_1=R1_1'*omega1_1;
    alpha1Local_1=R1_1'*alpha1_1;
    g1local_1=R1_1'*g;
    Omega1H_1=Hat_Creator(Omega1Local_1);
    alpha1H_1=Hat_Creator(alpha1Local_1);   
    Reg1_1=Jw1_1'*[R1_1*Skew(g1local_1),R1_1*(alpha1H_1+Skew(Omega1Local_1)*Omega1H_1)];

    Omega2Local_1=R2_1'*omega2_1;
    alpha2Local_1=R2_1'*alpha2_1;
    g2local_1=R2_1'*g;
    Omega2H_1=Hat_Creator(Omega2Local_1);
    alpha2H_1=Hat_Creator(alpha2Local_1);   
    Reg2_1=Jw2_1'*[R2_1*Skew(g2local_1),R2_1*(alpha2H_1+Skew(Omega2Local_1)*Omega2H_1)];
    
    Reg1=[Reg1_1,Reg2_1];
    %% Chain 2
    Omega1Local_2=R1_2'*omega1_2;
    alpha1Local_2=R1_2'*alpha1_2;
    g1local_2=R1_2'*g;
    Omega1H_2=Hat_Creator(Omega1Local_2);
    alpha1H_2=Hat_Creator(alpha1Local_2);   
    Reg1_2=Jw1_2'*[R1_2*Skew(g1local_2),R1_2*(alpha1H_2+Skew(Omega1Local_2)*Omega1H_2)];

    Omega2Local_2=R2_2'*omega2_2;
    alpha2Local_2=R2_2'*alpha2_2;
    g2local_2=R2_2'*g;
    Omega2H_2=Hat_Creator(Omega2Local_2);
    alpha2H_2=Hat_Creator(alpha2Local_2);   
    Reg2_2=Jw2_2'*[R2_2*Skew(g2local_2),R2_2*(alpha2H_2+Skew(Omega2Local_2)*Omega2H_2)];
    
    Reg2=[Reg1_2,Reg2_2];
    
    
    %% Chain 3
    Omega1Local_3=R1_3'*omega1_3;
    alpha1Local_3=R1_3'*alpha1_3;
    g1local_3=R1_3'*g;
    Omega1H_3=Hat_Creator(Omega1Local_3);
    alpha1H_3=Hat_Creator(alpha1Local_3);   
    Reg1_3=Jw1_3'*[R1_3*Skew(g1local_3),R1_3*(alpha1H_3+Skew(Omega1Local_3)*Omega1H_3)];

    Omega2Local_3=R2_3'*omega2_3;
    alpha2Local_3=R2_3'*alpha2_3;
    g2local_3=R2_3'*g;
    Omega2H_3=Hat_Creator(Omega2Local_3);
    alpha2H_3=Hat_Creator(alpha2Local_3);   
    Reg2_3=Jw2_3'*[R2_3*Skew(g2local_3),R2_3*(alpha2H_3+Skew(Omega2Local_3)*Omega2H_3)];
    
    Reg3=[Reg1_3,Reg2_3];

    %% Regressor of Links
    Y_Links=[Reg1,Reg2,Reg3];
        
    %% Regressor of Moving_Platform  
    glocal=Rp'*g;
    OmegaLocal=Rp'*Omega_EE;
    alphaLocal=Rp'*Alpha_EE;
    
    OmegaH=Hat_Creator(OmegaLocal);
    alphaH=Hat_Creator(alphaLocal);   
    
    Y_MP=E'*[Rp*Skew(glocal),(Rp*alphaH+Skew(Omega_EE)*Rp*OmegaH)];

    
end

