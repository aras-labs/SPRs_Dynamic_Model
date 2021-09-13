%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc;
load('ADAMS_Data2.mat')

%% Time of Simulation Define

T_final=1;
Step=0.005;
time=0:Step:T_final;
i=1;

%% Trajectory cofficients Define (Trajectory Define in Sperical Coordinate)

phiJ=Trajectory(0*(pi/180),120*(pi/180),0,0,0,T_final);
gammaJ=Trajectory(70*(pi/180),10*(pi/180),0,0,0,T_final);

phiJJ=Trajectory(5*(pi/180),100*(pi/180),0,0,0,T_final);
gammaJJ=Trajectory(60*(pi/180),5*(pi/180),0,0,0,T_final);


for t=0:Step:T_final

    %% Trajectory Define
    gamma=gammaJ(1)+gammaJ(2)*t+gammaJ(3)*t^2+gammaJ(4)*t^3;
    phi=phiJ(1)+phiJ(2)*t+phiJ(3)*t^2+phiJ(4)*t^3;
    X=[phi;gamma];
    
    gamma_dot=gammaJ(2)+2*gammaJ(3)*t+3*gammaJ(4)*t^2;
    phi_dot=phiJ(2)+2*phiJ(3)*t+3*phiJ(4)*t^2;
    X_dot=[phi_dot;gamma_dot];  

    gamma_dot_dot=2*gammaJ(3)+6*gammaJ(4)*t;
    phi_dot_dot=2*phiJ(3)+6*phiJ(4)*t;
    X_dot_dot=[phi_dot_dot;gamma_dot_dot];  

    %% Jacobian Define  
    J=Jacobian_In_Dynamic(X);
  
    %% Explicit Dynamic
    M=Mass_Matrix(X);
    C=C_Matrix(X,X_dot);
    G=G_Vector(X);
    
    F_Explicit=M*X_dot_dot+C*X_dot+G;
    Tau_Explicit=J'\F_Explicit;
    Tau1_Explicit(i,1)=Tau_Explicit(1);Tau2_Explicit(i,1)=Tau_Explicit(2);    
    
    i=i+1;
end

%% Plots
figure(1)
subplot(211)
plot(time,Tau1_Explicit,'linewidth',4)
hold on;
plot(TIME,T1_ADAMS,'r--','linewidth',4)
grid on;
%legend('Explicit Virtual Work','MSC ADAMS','Location', 'Best')
xlabel('Time(s)')
ylabel('\tau_1 (N.m)')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)


subplot(212)
plot(time,Tau2_Explicit,'linewidth',4)
hold on;
plot(TIME,T2_ADAMS,'r--','linewidth',4)
grid on;
legend('Explicit Virtual Work','MSC ADAMS','Location', 'Best')
xlabel('Time(s)')
ylabel('\tau_2 (N.m)')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)


figure(2)
subplot(211)
plot(time',(Tau1_Explicit-T1_ADAMS),'b+')
grid on;
xlabel('Time(s)')
ylabel('e_{\tau_1}')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(212)
plot(time',(Tau2_Explicit-T2_ADAMS),'r*')
grid on;
xlabel('Time(s)')
ylabel('e_{\tau_1}')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

