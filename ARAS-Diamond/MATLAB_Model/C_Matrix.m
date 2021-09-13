%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [C] = C_Matrix(X,X_dot)

phi=X(1);
gamma=X(2);

phi_dot=X_dot(1);
gamma_dot=X_dot(2);

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
    
    %% Parallel Axis Theorem
    IA1=II1+m_1*Skew(rho1)'*Skew(rho1);
    IA2=II2+m_2*Skew(rho2)'*Skew(rho2);
    IA3=II3+m_3*Skew(rho3)'*Skew(rho3);
    IA4=II4+m_4*Skew(rho4)'*Skew(rho4);
    
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
    q_dot=J*X_dot;
    theta__1_dot=q_dot(1);  theta__2_dot=q_dot(2);   
    
    %% Passive Jacobian
    J_p=Passive_Jacobian(a,b,c,d,gamma,beta);     % Semi Verified
    
    %% Jacobain of Links
    J1=Jacobian_Link1(alpha,beta,gamma,A,a,b,c,d);
    J2=Jacobian_Link2(alpha,beta,gamma,A,a,b,c,d);
    J3=Jacobian_Link3(alpha,beta,gamma,A,B,a,b,c,d);
    J4=Jacobian_Link4(alpha,beta,gamma,A,B,a,b,c,d);
      
    omega1=J1*X_dot;
    omega2=J2*X_dot;
    omega3=J3*X_dot; 
    omega4=J4*X_dot;   
    
    %% Acceleration_Analysis
    J1dot=Jacobian_dot_Link1(alpha,beta,gamma,A,a,b,c,d,gamma_dot);
    J2dot=Jacobian_dot_Link2(alpha,beta,gamma,A,a,b,c,d,gamma_dot);
    J3dot=Jacobian_dot_Link3(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J1dot,theta__2_dot);
    J4dot=Jacobian_dot_Link4(alpha,beta,gamma,phi,A,B,a,b,c,d,theta__1,theta__2,phi_dot,gamma_dot,J2dot,theta__1_dot);
    
    %% Moment Inertia Mapping
    I1=R1*IA1*R1';
    I2=R2*IA2*R2';
    I3=R3*IA3*R3';
    I4=R4*IA4*R4';   

    %% C Matrix
    C1= J1'*I1*J1dot+J1'*Skew(J1*X_dot)*I1*J1;    
    C2= J2'*I2*J2dot+J2'*Skew(J2*X_dot)*I2*J2;
    C3= J3'*I3*J3dot+J3'*Skew(J3*X_dot)*I3*J3;
    C4= J4'*I4*J4dot+J4'*Skew(J4*X_dot)*I4*J4;
    C = C1 + C2 + C3 + C4 ; 

    %% Mdot-2C
    Mdot1=J1dot'*I1*J1+J1'*(Skew(omega1)*I1+I1*Skew(omega1)')*J1+J1'*I1*J1dot;
    Mdot2=J2dot'*I2*J2+J2'*(Skew(omega2)*I2+I2*Skew(omega2)')*J2+J2'*I2*J2dot;
    Mdot3=J3dot'*I3*J3+J3'*(Skew(omega3)*I3+I3*Skew(omega3)')*J3+J3'*I3*J3dot;
    Mdot4=J4dot'*I4*J4+J4'*(Skew(omega4)*I4+I4*Skew(omega4)')*J4+J4'*I4*J4dot;
    %MdotP=(Skew(omega)*I1+I1*Skew(omega)')
    
    Mdot=Mdot1+Mdot2+Mdot3+Mdot4;
    % Cheak
    Skew_Sym=Mdot-2*C;
    Skew_Sym_Cheak=Skew_Sym+Skew_Sym';
    

end

