%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc;
load('Gauss_Jordan.mat')


%% Time of Simulation Define

T_final=5;
Step=0.005;
time=0:Step:T_final;
i=1;


%% Trajectory cofficients Define (Trajectory Define in Sperical Coordinate)

phiJ=Trajectory(1*(pi/180),179*(pi/180),0,0,0,T_final);
gammaJ=Trajectory(89*(pi/180),1*(pi/180),0,0,0,T_final);

for t=0:Step:T_final

    %% Random Trajectory Define
    Data_phi=Data_Generator_phi;
    Data_gamma=Data_Generator_gamma;  
    
            X=[Data_phi(1,1);Data_gamma(1,1)];  
        X_dot=[Data_phi(2,1);Data_gamma(2,1)];  
    X_dot_dot=[Data_phi(3,1);Data_gamma(3,1)];

    %% Explicit Dynamic
    M=Mass_Matrix(X);
    C=C_Matrix(X,X_dot);
    G=G_Vector(X);
    
    F_Explicit=M*X_dot_dot+C*X_dot+G;
    F1_Explicit=F_Explicit(1);F2_Explicit=F_Explicit(2);    

    %% Regressor Dynamic Analysis
    Y_Full=Full_Regressor(X,X_dot,X_dot_dot);
    pi_Full=Inertial_Parameters_Full;
    
    F_Full=Y_Full*pi_Full;    
    F1_Full=F_Full(1);F2_Full=F_Full(2);    

    %% Reduced Regressor
    Y_Reduced_Gauss=Y_Full*Gauss_BB;
    pi_Reduced_Gauss=Gauss_B*pi_Full;
    
    F_Reduced=Y_Reduced_Gauss*pi_Reduced_Gauss;
    F1_Reduced=F_Reduced(1);F2_Reduced=F_Reduced(2);    
    
    %% Verification
    E1_Full(i)=F1_Full-F1_Explicit;
    E2_Full(i)=F2_Full-F2_Explicit;
    
    E1_Reduced(i)=F1_Reduced-F1_Explicit;
    E2_Reduced(i)=F2_Reduced-F2_Explicit;
    
    i=i+1;
end

%% Plot
figure(1)
subplot(211)
plot(time,E1_Full,'bo')
hold on;
plot(time,E1_Reduced,'r+')
grid on;
%legend('Full Regressor','Reduced Regressor','Location', 'Best')
xlabel('Time(s)')
ylabel('E_{\tau_1} ')
ylim([-3e-13 3e-13])
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(212)
plot(time,E2_Full,'bo')
hold on;
plot(time,E2_Reduced,'r+')
grid on;
legend('Full Regressor','Reduced Regressor')
xlabel('Time(s)')
ylabel('E_{\tau_2} ')
ylim([-3e-13 3e-13])
%suptitle('Reduced Regressor Verification for ARAS-Diamond Robot')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

