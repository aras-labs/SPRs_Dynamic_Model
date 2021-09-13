%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc
close all;
load('Gauss_Jordan.mat')

%% Time of Simulation Define
i=1;
T_final=5;
Step=0.005;
time=0:Step:T_final;

%%
for t=0:Step:T_final
    
%% Random Trajectory Define
    Data_X1=Data_Generator;
    Data_X2=Data_Generator;  
    Data_X3=Data_Generator;  
    
            X=[Data_X1(1,1);Data_X2(1,1);Data_X3(1,1)];  
        X_dot=[Data_X1(2,1);Data_X2(2,1);Data_X3(2,1)];  
    Xddot=[Data_X1(3,1);Data_X2(3,1);Data_X3(3,1)];  
   
    %% Jacobian (Force Disfurbation) Define    
    J=Jacobian_In_Dynamic(X);
    
    %% Explicit Dynamic 
    [Mp,M_Links]=Mass_Matrix(X);
    [Cp,C_Links]=C_Matrix(X,X_dot);
    [Gp,G_Links]=G_Vector(X);

	F_Explicit_Mp=Mp*Xddot+Cp*X_dot+Gp;
	F_Explicit_Links=M_Links*Xddot+C_Links*X_dot+G_Links;

    F_Explicit= F_Explicit_Mp+F_Explicit_Links;
    Tau_Explicit=J'^(-1)*F_Explicit;  
    Tau1_Explicit=Tau_Explicit(1);Tau2_Explicit=Tau_Explicit(2);Tau3_Explicit=Tau_Explicit(3);    

    %% Linear Regressor 
    [Y_Mp,Y_Links]=Full_Regressor(X,X_dot,Xddot);
    [pi_Mp,pi_Links]=Inertial_Parameters_Full;
    Y_Full=[Y_Mp,Y_Links];
    pi_Full=[pi_Mp;pi_Links];

    F_Full=Y_Full*pi_Full;
    Tau_Full=J'^(-1)*F_Full;  
    Tau1_Full=Tau_Full(1);Tau2_Full=Tau_Full(2); Tau3_Full=Tau_Full(3);    

    %% Reduced Linear Regressor
	Y_Reduced=Reduced_Regressor(Y_Full,Gauss_BB);
	pi_Reduced=Inertial_Parameters_Reduced(pi_Full,Gauss_B);
	pi_Reduced_Symbolic=Inertial_Parameters_Reduced_Symbolic();
    
    F_Reduced=Y_Reduced*pi_Reduced;
    Tau_Reduced=J'^(-1)*F_Reduced; 
    Tau1_Reduced=Tau_Reduced(1);Tau2_Reduced=Tau_Reduced(2); Tau3_Reduced=Tau_Reduced(3);    

    %% Plot Generator

    E1_Full(i)=Tau1_Full-Tau1_Explicit;
    E2_Full(i)=Tau2_Full-Tau2_Explicit;
    E3_Full(i)=Tau3_Full-Tau3_Explicit;
  
    E1_Reduced(i)=Tau1_Reduced-Tau1_Explicit;
    E2_Reduced(i)=Tau2_Reduced-Tau2_Explicit;
    E3_Reduced(i)=Tau2_Reduced-Tau2_Explicit;
    
    i=i+1;
end

%% Plot
figure(1)
subplot(311)
plot(time,E1_Full,'r*')
hold on;
plot(time,E1_Reduced,'b+')
grid on;
xlabel('Time(s)')
ylabel('E_{\tau_1}')
ylim([-1e-014 1e-014])
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(312)
plot(time,E2_Full,'r*')
hold on;
plot(time,E2_Reduced,'b+')
grid on;
xlabel('Time(s)')
ylabel('E_{\tau_2}')
ylim([-1e-014 1e-014])
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(313)
plot(time,E3_Full,'r*')
hold on;
plot(time,E3_Reduced,'b+')
grid on;
legend('Full Regressor','Reduced Regressor','Location','best')
xlabel('Time(s)')
ylabel('E_{\tau_3}')
ylim([-1e-014 1e-014])
set(gca,'FontWeight','bold','FontName','times','FontSize',17)



