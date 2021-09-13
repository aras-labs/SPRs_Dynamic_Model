%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [Base_Inertial_Parameters] = Inertial_Parameters_Reduced()


%% Geometry Define

alpha=pi/4;
beta=pi/4;


 
    %% Dynamic Parameters
    
    g=[0,-10,0]';
    
    II1=[6.4413629155E-04,0,0;0,6.3501139794E-04,0;0,0,1.7161749378E-05];
    II2=[5.3590531389E-04,0,0;0,5.2840271497E-04,0;0,0,1.5074544696E-05];
    
    II3=[9.8499010048E-04,0,0;0,9.3427520943E-04,0;0,0,6.1016538598E-05];
    II4=[7.5779670437E-04,0,0;0,7.496980046E-04,0;0,0,2.3482517406E-05];

    m__1=0.1170118419;
    m__2=0.1121923268;
    m__3=0.1555782254;
    m__4=0.1453301931;
    
        %%
    
    I__xx1=II1(1,1);
    I__xy1=II1(1,2);
    I__xz1=II1(1,3);
    I__yz1=II1(2,3);
    I__yy1=II1(2,2);
    I__zz1=II1(3,3);
    
    I__xx2=II2(1,1);
    I__xy2=II2(1,2);
    I__xz2=II2(1,3);
    I__yz2=II2(2,3);
    I__yy2=II2(2,2);
    I__zz2=II2(3,3);
    
    I__xx3=II3(1,1);
    I__xy3=II3(1,2);
    I__xz3=II3(1,3);
    I__yz3=II3(2,3);
    I__yy3=II3(2,2);
    I__zz3=II3(3,3);
    
    I__xx4=II4(1,1);
    I__xy4=II4(1,2);
    I__xz4=II4(1,3);
    I__yz4=II4(2,3);
    I__yy4=II4(2,2);
    I__zz4=II4(3,3);
    
    
    %% Parallel axes Theroem
    
    L_Link1=252/1000;    
    rho1 = Ry(22.9518*(pi/180))*[0,0,L_Link1]'; 

    L_Link2=228/1000;    
    rho2 = Ry(22.6335*(pi/180))*[0,0,L_Link2]'; 
    
    L_Link3=276/1000;
    rho3 = Ry(22.8026*(pi/180))*[0,0,L_Link3]';   

    L_Link4=204/1000;
    rho4 = Ry(22.5*(pi/180))*[0,0,L_Link4]'; 
    
    rho__x1=rho1(1);  
    rho__y1=rho1(2);  
    rho__z1=rho1(3);  

    rho__x2=rho2(1);  
    rho__y2=rho2(2);  
    rho__z2=rho2(3);  
    
    rho__x3=rho3(1);  
    rho__y3=rho3(2);  
    rho__z3=rho3(3);  
    
    rho__x4=rho4(1);  
    rho__y4=rho4(2);  
    rho__z4=rho4(3);  

%%
Base_Inertial_Parameters=[ - 1.0*I__yz4 - 1.0*m__4*rho__y4*rho__z4, I__zz4 - 1.0*I__xz4 + m__4*(rho__x4^2 + rho__y4^2) - 1.0*m__4*rho__x4*rho__z4, - 1.0*I__xy4 - 1.0*m__4*rho__x4*rho__y4, I__xx4 - 1.0*I__yy4 + I__zz4 + m__4*(rho__x4^2 + rho__y4^2) - 1.0*m__4*(rho__x4^2 + rho__z4^2) + m__4*(rho__y4^2 + rho__z4^2), I__zz3 + I__zz4 + m__3*(rho__x3^2 + rho__y3^2) + m__4*(rho__x4^2 + rho__y4^2), - 1.0*I__yz3 - 1.0*m__3*rho__y3*rho__z3, - 1.0*I__xz3 - 1.0*I__zz4 - 1.0*m__4*(rho__x4^2 + rho__y4^2) - 1.0*m__3*rho__x3*rho__z3, - 1.0*I__xy3 - 1.0*m__3*rho__x3*rho__y3, I__xx3 - 1.0*I__yy3 - 1.0*I__zz4 - 1.0*m__3*(rho__x3^2 + rho__z3^2) + m__3*(rho__y3^2 + rho__z3^2) - 1.0*m__4*(rho__x4^2 + rho__y4^2), m__3*rho__y3 - 1.0*m__4*rho__y4, m__3*rho__x3 + m__4*rho__x4, 0.5*I__yy4 + I__zz2 - 1.0*I__zz4 + m__2*(rho__x2^2 + rho__y2^2) - 1.0*m__4*(rho__x4^2 + rho__y4^2) + 0.5*m__4*(rho__x4^2 + rho__z4^2), m__2*rho__y2 + m__4*rho__y4, m__2*rho__x2 - 0.70711*m__4*rho__x4 + 0.70711*m__4*rho__z4, 0.5*I__yy3 + I__zz1 + I__zz4 + m__1*(rho__x1^2 + rho__y1^2) + 0.5*m__3*(rho__x3^2 + rho__z3^2) + m__4*(rho__x4^2 + rho__y4^2), m__1*rho__y1 + m__4*rho__y4, m__1*rho__x1 + 0.70711*m__3*rho__z3 + 0.70711*m__4*rho__x4]';



end

