%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [phi_Reg] = Inertial_Parameters_Full()

%% Geometry Define

alpha=pi/4;
beta=pi/4;


 
    %% Dynamic Parameters
    
    g=[0,-10,0]';
    
    II1=[6.4413629155E-04,0,0;0,6.3501139794E-04,0;0,0,1.7161749378E-05];
    II2=[5.3590531389E-04,0,0;0,5.2840271497E-04,0;0,0,1.5074544696E-05];
    
    II3=[9.8499010048E-04,0,0;0,9.3427520943E-04,0;0,0,6.1016538598E-05];
    II4=[7.5779670437E-04,0,0;0,7.496980046E-04,0;0,0,2.3482517406E-05];

    m_1=0.1170118419;
    m_2=0.1121923268;
    m_3=0.1555782254;
    m_4=0.1453301931;
    
    %% Parallel axes Theroem
    
    L_Link1=252/1000;    
    rho1 = Ry(22.9518*(pi/180))*[0,0,L_Link1]'; 

    L_Link2=228/1000;    
    rho2 = Ry(22.6335*(pi/180))*[0,0,L_Link2]'; 
    
    L_Link3=276/1000;
    rho3 = Ry(22.8026*(pi/180))*[0,0,L_Link3]';   

    L_Link4=204/1000;
    rho4 = Ry(22.5*(pi/180))*[0,0,L_Link4]'; 
    
    IA1=II1+m_1*Skew(rho1)'*Skew(rho1);
    IA2=II2+m_2*Skew(rho2)'*Skew(rho2);
    IA3=II3+m_3*Skew(rho3)'*Skew(rho3);
    IA4=II4+m_4*Skew(rho4)'*Skew(rho4);    

 % Link1
    Ibar1=Moment_Inertia_Linear_Form(IA1);
    phi1=[m_1*rho1;Ibar1]; 

    % Link2    
    Ibar2=Moment_Inertia_Linear_Form(IA2);
    phi2=[m_2*rho2;Ibar2];  
    
    % Link3    
    Ibar3=Moment_Inertia_Linear_Form(IA3);
    phi3=[m_3*rho3;Ibar3];  
    
    % Link4  
    Ibar4=Moment_Inertia_Linear_Form(IA4);
    phi4=[m_4*rho4;Ibar4];  


    %%
        phi_Reg=[phi1;phi2;phi3;phi4];

end

