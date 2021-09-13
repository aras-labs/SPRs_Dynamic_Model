%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [Gp_Final,G_Links] = G_Vector(X)

%% Kinematic Parameters

Lambda=[0;120;240]*(pi/180);Eta=Lambda;
Beta=acos(sqrt(3)/3);Gamma=Beta;

alpha__1=[80*(pi/180);80*(pi/180);80*(pi/180)];
alpha__2=[70*(pi/180);70*(pi/180);70*(pi/180)];
g=[0;0;-9.80665];
%% Inertial Parameters

    %% Masses
        %% Chain1  
        m1_1=0.5012423762;
        m2_1=0.3891400595;
        %% Chain2  
        m1_2=m1_1;
        m2_2=m2_1;
        %% Chain3 
        m1_3=m1_1;
        m2_3=m2_1;
        %% Moving Platform
        mp=0.6045042773;
        
    %% CG Of Each link
        %% Chain1  
        rho1_1=[0;-0.117465246575864;0.139983998075476];
        rho2_1=[0;-0.093540986885397;0.133590695351393];
        %% Chain2 
        rho1_2=[0;-0.117465246575864;0.139983998075476];
        rho2_2=[0;-0.093540986885397;0.133590695351393];
        %% Chain3   
        rho1_3=[0;-0.117465246575864;0.139983998075476];
        rho2_3=[0;-0.093540986885397;0.133590695351393];
        %% Moving Platform
        rho_p=[0;0;0.084583480323000];

    %% Moment Inertia
        %% Chain1      
        II1_1=[3.3906023479E-03,0,0;0,3.2718954694E-03,0;0,0,1.7101532875E-04];
        II2_1=[1.628011789E-03,0,0;0,1.5834981056E-03,0;0,0,8.5149842496E-05];
        %% Chain2  
        II1_2=II1_1;
        II2_2=II2_1;
        %% Chain3 
        II1_3=II1_1;    
        II2_3=II2_1;
        %% Moving Platform
        IIp=[3.7203038672E-03,0,0;0,1.879213797E-03,0;0,0,1.8787528739E-03]; 
                
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
    %Theta_dot=J*X_dot;     
    %Psi_dot=Jp*X_dot; 

%% Jacobian Analysis of Each Link
     [Jw1_1,Jw2_1,Jw1_2,Jw2_2,Jw1_3,Jw2_3] = Jacobian_Links(J,Jp,V1,W1,U1,V2,W2,U2,V3,W3,U3);
        
%% Angular Velocity Analysis
     %% Chain 1
     %omega1_1=Jw1_1*X_dot;omega2_1=Jw2_1*X_dot;
     %% Chain 2
     %omega1_2=Jw1_2*X_dot;omega2_2=Jw2_2*X_dot;
     %% Chain 3
     %omega1_3=Jw1_3*X_dot;omega2_3=Jw2_3*X_dot;  
    
%% End_Effector Velocity
     %Omega_EE=E*X_dot;

    %% Jacobian Dot Analysis
         
 	 %[Jdot,Jpdot,Edot] = Jacobian_dot(X,X_dot,theta,Theta_dot,V1,W1,U1,V2,W2,U2,V3,W3,U3,Gamma,Beta,Eta,Lambda,alpha__1);    
     %Theta_ddot=J*Xddot+Jdot*X_dot;
     %Psi_ddot=Jp*Xddot+Jpdot*X_dot;

%% End_Effector Acceleration
     %Alpha_EE=E*Xddot+Edot*X_dot;
    
%% Jacobian Dot Analysis of Each Link 

    %[dJw1_1,dJw2_1,dJw1_2,dJw2_2,dJw1_3,dJw2_3] = Jacobian_dot_Links(J,Jp,Jdot,Jpdot,V1,W1,U1,V2,W2,U2,V3,W3,U3,X,X_dot,theta,Theta_dot,Gamma,Beta,Eta,Lambda,alpha__1);
     
%% Angular Acceleration Analysis
    %% Chain 1
    %alpha1_1=Jw1_1*Xddot+dJw1_1*X_dot;alpha2_1=Jw2_1*Xddot+dJw2_1*X_dot;
    %% Chain 2
    %alpha1_2=Jw1_2*Xddot+dJw1_2*X_dot;alpha2_2=Jw2_2*Xddot+dJw2_2*X_dot;
    %% Chain 3    
    %alpha1_3=Jw1_3*Xddot+dJw1_3*X_dot;alpha2_3=Jw2_3*Xddot+dJw2_3*X_dot;

%% Parallel Axes Theorem

    %% Chain1
    IA1_1=II1_1+m1_1*Skew(rho1_1)'*Skew(rho1_1);    
    IA2_1=II2_1+m2_1*Skew(rho2_1)'*Skew(rho2_1); 
    %% Chain2
    IA1_2=II1_2+m1_2*Skew(rho1_2)'*Skew(rho1_2);    
    IA2_2=II2_2+m2_2*Skew(rho2_2)'*Skew(rho2_2); 
    %% Chain3
    IA1_3=II1_3+m1_3*Skew(rho1_3)'*Skew(rho1_3);    
    IA2_3=II2_3+m2_3*Skew(rho2_3)'*Skew(rho2_3); 
    %% Moving Platform
    IAp=IIp+mp*Skew(rho_p)'*Skew(rho_p); 
    
%% Rotation Matrix Mapping

    %% Chain1
    I1_1=R1_1*IA1_1*R1_1';  
    I2_1=R2_1*IA2_1*R2_1';  
    %% Chain 2
    I1_2=R1_2*IA1_2*R1_2';  
    I2_2=R2_2*IA2_2*R2_2';  
    %% Chain3
    I1_3=R1_3*IA1_3*R1_3';  
    I2_3=R2_3*IA2_3*R2_3';  
    %% Moving Platform
    Ip=Rp*IAp*Rp';  
    
%% Conver Local rho to Base rho
    %% Chain1    
    Rho1_1=R1_1*Skew(rho1_1)*R1_1';
    Rho2_1=R2_1*Skew(rho2_1)*R2_1';
    %% Chain2    
    Rho1_2=R1_2*Skew(rho1_2)*R1_2';
    Rho2_2=R2_2*Skew(rho2_2)*R2_2';
    %% Chain3  
    Rho1_3=R1_3*Skew(rho1_3)*R1_3';
    Rho2_3=R2_3*Skew(rho2_3)*R2_3';    
    %% Moving Platform
    Rho_p=Rp*Skew(rho_p)*Rp';    
    
%% G Vector
  
    %% Chain1
    G1_1=-m1_1*Jw1_1'*Rho1_1*g;  
    G2_1=-m2_1*Jw2_1'*Rho2_1*g;  
    G1=G1_1+G2_1;
    %% Chain 2  
    G1_2=-m1_2*Jw1_2'*Rho1_2*g;  
    G2_2=-m2_2*Jw2_2'*Rho2_2*g;  
    G2=G1_2+G2_2;
    %% Chain3 
    G1_3=-m1_3*Jw1_3'*Rho1_3*g;  
    G2_3=-m2_3*Jw2_3'*Rho2_3*g;  
    G3=G1_3+G2_3;
    
    
    %% Moving Platform
    Gp=-mp*Rho_p*g;
    Gp_Final=E'*Gp;
    %% Total Links 
	G_Links=G1+G2+G3;   
 
end

