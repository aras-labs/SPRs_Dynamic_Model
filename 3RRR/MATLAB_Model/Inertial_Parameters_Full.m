%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [pi_Mp,pi_Links] = Inertial_Parameters_Full()


%% Kinematic Parameters

% Lambda=[0;120;240]*(pi/180);Eta=Lambda;
% Beta=acos(sqrt(3)/3);Gamma=Beta;
% 
% alpha__1=[80*(pi/180);80*(pi/180);80*(pi/180)];
% alpha__2=[70*(pi/180);70*(pi/180);70*(pi/180)];
% g=[0;0;-9.80665];

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
      
%% Pi Creator
    %% Chain1
    Ibar1_1=Moment_Inertia_Linear_Form(IA1_1);
    phi1_1=[m1_1*rho1_1;Ibar1_1]; 

    Ibar2_1=Moment_Inertia_Linear_Form(IA2_1);
    phi2_1=[m2_1*rho2_1;Ibar2_1]; 
    
    phi1=[phi1_1;phi2_1];
    %% Chain2
    Ibar1_2=Moment_Inertia_Linear_Form(IA1_2);
    phi1_2=[m1_2*rho1_2;Ibar1_2]; 

    Ibar2_2=Moment_Inertia_Linear_Form(IA2_2);
    phi2_2=[m2_2*rho2_2;Ibar2_2]; 
    
    phi2=[phi1_2;phi2_2];

    %% Chain3

    Ibar1_3=Moment_Inertia_Linear_Form(IA1_3);
    phi1_3=[m1_3*rho1_3;Ibar1_3]; 

    Ibar2_3=Moment_Inertia_Linear_Form(IA2_3);
    phi2_3=[m2_3*rho2_3;Ibar2_3]; 
    
    phi3=[phi1_3;phi2_3];
    
    %% Full Inertial Parameters of Links
    pi_Links=[phi1;phi2;phi3];
    
    %% Moving Platform
    Ibar_p=Moment_Inertia_Linear_Form(IAp);
    pi_Mp=[mp*rho_p;Ibar_p]; 
        

end

