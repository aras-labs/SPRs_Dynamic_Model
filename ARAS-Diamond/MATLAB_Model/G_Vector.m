%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [G] = G_Vector(X)

phi=X(1);
gamma=X(2);

%% Geometry Define

alpha=pi/4;
beta=pi/4;
g=[0,-10,0]';


%% Inertial Parameters
    
    m_1=0.1170118419;
    L_Link1=252/1000;    
    rho1 = Ry(22.9518*(pi/180))*[0,0,L_Link1]'; 
    rho__x1=rho1(1);rho__y1=rho1(2);rho__z1=rho1(3);
    I__xx1=6.4413629155E-04;I__xy1=0;I__xz1=0;I__yy1=6.3501139794E-04;I__yz1=0;I__zz1=1.7161749378E-05;
    II1=[I__xx1,I__xy1,I__xz1;I__xy1,I__yy1,I__yz1;I__xz1,I__yz1,I__zz1];

    m_2=0.1121923268;
    L_Link2=228/1000;    
    rho2 = Ry(22.6335*(pi/180))*[0,0,L_Link2]';
    rho__x2=rho2(1);rho__y2=rho2(2);rho__z2=rho2(3);
    I__xx2=5.3590531389E-04;I__xy2=0;I__xz2=0;I__yy2=5.2840271497E-04;I__yz2=0;I__zz2=1.5074544696E-05;
    II2=[I__xx2,I__xy2,I__xz2;I__xy2,I__yy2,I__yz2;I__xz2,I__yz2,I__zz2];
    
    
    m_3=0.1555782254;
    L_Link3=276/1000;
    rho3 = Ry(22.8026*(pi/180))*[0,0,L_Link3]';   
    rho__x3=rho3(1);rho__y3=rho3(2);rho__z3=rho3(3);
    I__xx3=9.8499010048E-04;I__xy3=0;I__xz3=0;I__yy3=9.3427520943E-04;I__yz3=0;I__zz3=6.1016538598E-05;
    II3=[I__xx3,I__xy3,I__xz3;I__xy3,I__yy3,I__yz3;I__xz3,I__yz3,I__zz3];
 
    m_4=0.1453301931;
    L_Link4=204/1000;
    rho4 = Ry(22.5*(pi/180))*[0,0,L_Link4]'; 
    rho__x4=rho4(1);rho__y4=rho4(2);rho__z4=rho4(3);
    I__xx4=7.5779670437E-04;I__xy4=0;I__xz4=0;I__yy4=7.496980046E-04;I__yz4=0;I__zz4=2.3482517406E-05;
    II4=[I__xx4,I__xy4,I__xz4;I__xy4,I__yy4,I__yz4;I__xz4,I__yz4,I__zz4];

   
    %% Inverse Kinematic
    [theta_New,Passive]=Inverse_Kinematic(gamma,phi,alpha,beta);
    theta__1=theta_New(1);  theta__2=theta_New(2);    
    A=Passive(1);B=Passive(2);D=Passive(3);
    
    %% Calculating Unit Vectors
    Unit= Unit_Vectors(theta__1,theta__2,alpha,beta,gamma,phi);
    a=Unit(1:3);b=Unit(4:6);c=Unit(7:9);d=Unit(10:12);
    
    %% Rotation Matrix 
    R1=Rotation_Link1(a,b,c,d); R2=Rotation_Link2(a,b,c,d); R3=Rotation_Link3(a,b,c,d); R4=Rotation_Link4(a,b,c,d);

    %% Jacobian Analysis of End_Effector:
    J= Jacobian(alpha,beta,gamma,A);             %Scalar approach 
    
    %% Passive Jacobian
    J_p=Passive_Jacobian(a,b,c,d,gamma,beta);     % Semi Verified
    
    %% Jacobain of Links
    J1=Jacobian_Link1(alpha,beta,gamma,A,a,b,c,d);
    J2=Jacobian_Link2(alpha,beta,gamma,A,a,b,c,d);
    J3=Jacobian_Link3(alpha,beta,gamma,A,B,a,b,c,d);
    J4=Jacobian_Link4(alpha,beta,gamma,A,B,a,b,c,d);
   
	%% Conver Local rho to Base rho
    Rho1=R1*Skew(rho1)*R1';
    Rho2=R2*Skew(rho2)*R2';
    Rho3=R3*Skew(rho3)*R3';
    Rho4=R4*Skew(rho4)*R4';      

    %% Gravity Vectors ( Rho1>> Base Measured ) 
    G1=-m_1*J1'*Rho1*g;  
    G2=-m_2*J2'*Rho2*g;  
    G3=-m_3*J3'*Rho3*g;  
    G4=-m_4*J4'*Rho4*g;  
    G = G1 + G2 + G3 + G4; 
 

end

