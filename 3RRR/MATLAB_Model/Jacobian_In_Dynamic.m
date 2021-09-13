%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [J] = Jacobian_In_Dynamic(X)


%% Kinematic Parameters

Lambda=[0;120;240]*(pi/180);Eta=Lambda;
Beta=acos(sqrt(3)/3);Gamma=Beta;

alpha__1=[80*(pi/180);80*(pi/180);80*(pi/180)];
alpha__2=[70*(pi/180);70*(pi/180);70*(pi/180)];

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


end

