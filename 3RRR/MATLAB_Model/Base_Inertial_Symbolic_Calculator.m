%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc;
load('Gauss_Jordan.mat')
Gauss_B=round(Gauss_B,5);

%% Define Syms
syms m_1_1;syms rho_x1_1 rho_y1_1 rho_z1_1; syms I_xx1_1 I_xy1_1 I_xz1_1 I_yy1_1 I_yz1_1 I_zz1_1;
syms m_2_1;syms rho_x2_1 rho_y2_1 rho_z2_1; syms I_xx2_1 I_xy2_1 I_xz2_1 I_yy2_1 I_yz2_1 I_zz2_1;

syms m_1_2;syms rho_x1_2 rho_y1_2 rho_z1_2; syms I_xx1_2 I_xy1_2 I_xz1_2 I_yy1_2 I_yz1_2 I_zz1_2;
syms m_2_2;syms rho_x2_2 rho_y2_2 rho_z2_2; syms I_xx2_2 I_xy2_2 I_xz2_2 I_yy2_2 I_yz2_2 I_zz2_2;

syms m_1_3;syms rho_x1_3 rho_y1_3 rho_z1_3; syms I_xx1_3 I_xy1_3 I_xz1_3 I_yy1_3 I_yz1_3 I_zz1_3;
syms m_2_3;syms rho_x2_3 rho_y2_3 rho_z2_3; syms I_xx2_3 I_xy2_3 I_xz2_3 I_yy2_3 I_yz2_3 I_zz2_3;

syms mp;syms rho_p_x rho_p_y  rho_p_z; syms I_xx_p I_xy_p I_xz_p I_yy_p I_yz_p I_zz_p;

%% Symbole Base Inertial Parameters

    %% Chain 1
    rho1_1=[rho_x1_1;rho_y1_1;rho_z1_1];
    I1_1=[I_xx1_1,I_xy1_1,I_xz1_1;I_xy1_1,I_yy1_1,I_yz1_1;I_xz1_1,I_yz1_1,I_zz1_1];
    I1_1_Lin=Moment_Inertia_Linear_Form(I1_1);
    phi1_1=[m_1_1*rho1_1;I1_1_Lin];

    rho2_1=[rho_x2_1;rho_y2_1;rho_z2_1];
    I2_1=[I_xx2_1,I_xy2_1,I_xz2_1;I_xy2_1,I_yy2_1,I_yz2_1;I_xz2_1,I_yz2_1,I_zz2_1];
    I2_1_Lin=Moment_Inertia_Linear_Form(I2_1);
    phi2_1=[m_2_1*rho2_1;I2_1_Lin];    

    phi1=[phi1_1;phi2_1];
    %% Chain 2
    rho1_2=[rho_x1_2;rho_y1_2;rho_z1_2];
    I1_2=[I_xx1_2,I_xy1_2,I_xz1_2;I_xy1_2,I_yy1_2,I_yz1_2;I_xz1_2,I_yz1_2,I_zz1_2];
    I1_2_Lin=Moment_Inertia_Linear_Form(I1_2);
    phi1_2=[m_1_2*rho1_2;I1_2_Lin];

    rho2_2=[rho_x2_2;rho_y2_2;rho_z2_2];
    I2_2=[I_xx2_2,I_xy2_2,I_xz2_2;I_xy2_2,I_yy2_2,I_yz2_2;I_xz2_2,I_yz2_2,I_zz2_2];
    I2_2_Lin=Moment_Inertia_Linear_Form(I2_2);
    phi2_2=[m_2_2*rho2_2;I2_2_Lin];   
    
    phi2=[phi1_2;phi2_2];

    %% Chain 3
    rho1_3=[rho_x1_3;rho_y1_3;rho_z1_3];
    I1_3=[I_xx1_3,I_xy1_3,I_xz1_3;I_xy1_3,I_yy1_3,I_yz1_3;I_xz1_3,I_yz1_3,I_zz1_3];
    I1_3_Lin=Moment_Inertia_Linear_Form(I1_3);
    phi1_3=[m_1_3*rho1_3;I1_3_Lin];

    rho2_3=[rho_x2_3;rho_y2_3;rho_z2_3];
    I2_3=[I_xx2_3,I_xy2_3,I_xz2_3;I_xy2_3,I_yy2_3,I_yz2_3;I_xz2_3,I_yz2_3,I_zz2_3];
    I2_3_Lin=Moment_Inertia_Linear_Form(I2_3);
    phi2_3=[m_2_3*rho2_3;I2_3_Lin];    

    phi3=[phi1_3;phi2_3];
    %% pi Links 
    pi_Links=[phi1;phi2;phi3];
    
    %% pi Moving Platform
    rho_p=[rho_p_x;rho_p_y;rho_p_z];
    IAp=[I_xx_p,I_xy_p,I_xz_p;I_xy_p,I_yy_p,I_yz_p;I_xz_p,I_yz_p,I_zz_p];
    Ibar_p=Moment_Inertia_Linear_Form(IAp);
    pi_Mp=[mp*rho_p;Ibar_p]; 
    
    %% Full Inertial Parameters
    pi_Full=[pi_Mp;pi_Links];
    %% Base Inertial Parameters 

    pi_Reduced_Gauss=Gauss_B*pi_Full;
    pi_Reduced_Gauss=vpa(pi_Reduced_Gauss,5);


    for i=1:33
        Base_Inertial(1,i)=pi_Reduced_Gauss(i,1);
    end


%% For Symbolic Usage
    Base_Inertial=vpa(Base_Inertial,5)

