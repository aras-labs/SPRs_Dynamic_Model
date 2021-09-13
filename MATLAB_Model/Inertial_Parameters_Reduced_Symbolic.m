%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [pi_Reduced] = Inertial_Parameters_Reduced_Symbolic()


%% Inertial Parameters

    %% Masses
        %% Chain1  
        m_1_1=0.5012423762;
        m_2_1=0.3891400595;
        %% Chain2  
        m_1_2=m_1_1;
        m_2_2=m_2_1;
        %% Chain3 
        m_1_3=m_1_1;
        m_2_3=m_2_1;
        %% Moving Platform
        mp=0.6045042773;
            
    %% CG Of Each link
        %% Chain1  
        rho1_1=[0;-0.117465246575864;0.139983998075476];rho_x1_1=rho1_1(1);rho_y1_1=rho1_1(2);rho_z1_1=rho1_1(3);
        rho2_1=[0;-0.093540986885397;0.133590695351393];rho_x2_1=rho2_1(1);rho_y2_1=rho2_1(2);rho_z2_1=rho2_1(3);
        %% Chain2 
        rho1_2=[0;-0.117465246575864;0.139983998075476];rho_x1_2=rho1_2(1);rho_y1_2=rho1_2(2);rho_z1_2=rho1_2(3);
        rho2_2=[0;-0.093540986885397;0.133590695351393];rho_x2_2=rho2_2(1);rho_y2_2=rho2_2(2);rho_z2_2=rho2_2(3);
        %% Chain3   
        rho1_3=[0;-0.117465246575864;0.139983998075476];rho_x1_3=rho1_3(1);rho_y1_3=rho1_3(2);rho_z1_3=rho1_3(3);
        rho2_3=[0;-0.093540986885397;0.133590695351393];rho_x2_3=rho2_3(1);rho_y2_3=rho2_3(2);rho_z2_3=rho2_3(3);
        %% Moving Platform
        rho_p=[0;0;0.084583480323000];rho_p_x=rho_p(1);rho_p_y=rho_p(2);rho_p_z=rho_p(3);
        
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
    IA1_1=II1_1+m_1_1*Skew(rho1_1)'*Skew(rho1_1);   
    I_xx1_1=IA1_1(1,1);I_xy1_1=IA1_1(1,2);I_xz1_1=IA1_1(1,3);I_yy1_1=IA1_1(2,2);I_yz1_1=IA1_1(2,3);I_zz1_1=IA1_1(3,3);   
    
    IA2_1=II2_1+m_2_1*Skew(rho2_1)'*Skew(rho2_1); 
    I_xx2_1=IA2_1(1,1);I_xy2_1=IA2_1(1,2);I_xz2_1=IA2_1(1,3);I_yy2_1=IA2_1(2,2);I_yz2_1=IA2_1(2,3);I_zz2_1=IA2_1(3,3);        
    
    %% Chain2
    IA1_2=II1_2+m_1_2*Skew(rho1_2)'*Skew(rho1_2);   
    I_xx1_2=IA1_2(1,1);I_xy1_2=IA1_2(1,2);I_xz1_2=IA1_2(1,3);I_yy1_2=IA1_2(2,2);I_yz1_2=IA1_2(2,3);I_zz1_2=IA1_2(3,3);
    
    IA2_2=II2_2+m_2_2*Skew(rho2_2)'*Skew(rho2_2); 
    I_xx2_2=IA2_2(1,1);I_xy2_2=IA2_2(1,2);I_xz2_2=IA2_2(1,3);I_yy2_2=IA2_2(2,2);I_yz2_2=IA2_2(2,3);I_zz2_2=IA2_2(3,3);        
    
    %% Chain3
    IA1_3=II1_3+m_1_3*Skew(rho1_3)'*Skew(rho1_3);   
    I_xx1_3=IA1_3(1,1);I_xy1_3=IA1_3(1,2);I_xz1_3=IA1_3(1,3);I_yy1_3=IA1_3(2,2);I_yz1_3=IA1_3(2,3);I_zz1_3=IA1_3(3,3);
    
    IA2_3=II2_3+m_2_3*Skew(rho2_3)'*Skew(rho2_3); 
    I_xx2_3=IA2_3(1,1);I_xy2_3=IA2_3(1,2);I_xz2_3=IA2_3(1,3);I_yy2_3=IA2_3(2,2);I_yz2_3=IA2_3(2,3);I_zz2_3=IA2_3(3,3);        

    %% Moving Platform
    IAp=IIp+mp*Skew(rho_p)'*Skew(rho_p); 
    I_xx_p=IAp(1,1); I_xy_p=IAp(1,2); I_xz_p=IAp(1,3); I_yy_p=IAp(2,2); I_yz_p=IAp(2,3); I_zz_p=IAp(3,3);
    
    %% Numerical Pi Creator
    pi_Reduced=[ I_yz2_3 - 0.36397*I_zz2_3, I_xz2_3, I_xy2_3, I_xx2_3 - 1.0*I_yy2_3 - 1.0*I_zz2_3, m_2_3*rho_x2_3, 0.96985*I_yy2_3 + I_zz1_3 - 0.12848*I_zz2_3, m_1_3*rho_y1_3 - 0.35844*m_2_3*rho_y2_3 - 0.98481*m_2_3*rho_z2_3, m_1_3*rho_x1_3, I_yz2_2 - 0.36397*I_zz2_2, I_xz2_2, I_xy2_2, I_xx2_2 - 1.0*I_yy2_2 - 1.0*I_zz2_2, m_2_2*rho_x2_2, 0.96985*I_yy2_2 + I_zz1_2 - 0.12848*I_zz2_2, m_1_2*rho_y1_2 - 0.35844*m_2_2*rho_y2_2 - 0.98481*m_2_2*rho_z2_2, m_1_2*rho_x1_2, I_yz2_1 - 0.36397*I_zz2_1, I_xz2_1, I_xy2_1, I_xx2_1 - 1.0*I_yy2_1 - 1.0*I_zz2_1, m_2_1*rho_x2_1, 0.96985*I_yy2_1 + I_zz1_1 - 0.12848*I_zz2_1, m_1_1*rho_y1_1 - 0.35844*m_2_1*rho_y2_1 - 0.98481*m_2_1*rho_z2_1, m_1_1*rho_x1_1, 0.75498*I_zz2_1 + 0.75498*I_zz2_2 + 0.75498*I_zz2_3 + I_zz_p, I_yz_p - 0.53385*I_zz2_1 + 0.26693*I_zz2_2 + 0.26693*I_zz2_3, I_yy_p + 0.37749*I_zz2_1 + 0.94373*I_zz2_2 + 0.94373*I_zz2_3, I_xz_p + 0.46233*I_zz2_2 - 0.46233*I_zz2_3, I_xy_p - 0.32692*I_zz2_2 + 0.32692*I_zz2_3, I_xx_p + 1.1325*I_zz2_1 + 0.56624*I_zz2_2 + 0.56624*I_zz2_3, mp*rho_p_z - 0.6144*m_2_2*rho_y2_2 - 0.6144*m_2_3*rho_y2_3 - 0.6144*m_2_1*rho_y2_1, 0.43445*m_2_2*rho_y2_2 - 0.8689*m_2_1*rho_y2_1 + 0.43445*m_2_3*rho_y2_3 + mp*rho_p_y, 0.75249*m_2_2*rho_y2_2 - 0.75249*m_2_3*rho_y2_3 + mp*rho_p_x]';

end
    
    