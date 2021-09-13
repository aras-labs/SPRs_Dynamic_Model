%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc;
close all;
load('Gauss_Jordan.mat')

%% Time of Simulation Define

T_final=2;
Step=0.005;
time=0:Step:T_final;
i=1;


for t=0:Step:T_final
    
%% Random Trajectory Define
    Data_X1=Data_Generator;
    Data_X2=Data_Generator;  
    Data_X3=Data_Generator;  
    
            X=[Data_X1(1,1);Data_X2(1,1);Data_X3(1,1)];  
        X_dot=[Data_X1(2,1);Data_X2(2,1);Data_X3(2,1)];  
       XR_dot=[Data_X1(2,1);Data_X2(2,1);Data_X3(2,1)];  
   XR_dot_dot=[Data_X1(3,1);Data_X2(3,1);Data_X3(3,1)];  
    
%% Jacobian Define

    J=Jacobian_In_Dynamic(X);

%% Explicit Dynamic 

    %% Jacobian (Force Disfurbation) Define    
    J=Jacobian_In_Dynamic(X);
    
    %% Explicit Dynamic 
    [Mp,M_Links]=Mass_Matrix(X);
    [Cp,C_Links]=C_Matrix(X,X_dot);
    [Gp,G_Links]=G_Vector(X);

	F_Explicit_Mp=Mp*XR_dot_dot+Cp*XR_dot+Gp;
	F_Explicit_Links=M_Links*XR_dot_dot+C_Links*XR_dot+G_Links;

    F_Explicit= F_Explicit_Mp+F_Explicit_Links;
    Tau_Explicit=J'^(-1)*F_Explicit;  
    Tau1_Explicit=Tau_Explicit(1);Tau2_Explicit=Tau_Explicit(2);Tau3_Explicit=Tau_Explicit(3);   
    
    %% Slotine Regressor Verification
    [YS_Mp,YS_Links]=Slotine_Regressor(X,X_dot,XR_dot,XR_dot_dot);
    [pi_Mp,pi_Links]=Inertial_Parameters_Full;
    YSlotine_Full=[YS_Mp,YS_Links];
    pi_Full=[pi_Mp;pi_Links];   
    
    F_Slotine_Regressor=YSlotine_Full*pi_Full; 
    Tau_Slotine_Regressor=J'\F_Slotine_Regressor;
    
    E2=Tau_Explicit-Tau_Slotine_Regressor;
    EE1_2(i)=E2(1);
    EE2_2(i)=E2(2);
    EE3_2(i)=E2(3);
       
    %% Slotine-Li Regressor Reduction
    Y_Reduced_Gauss=YSlotine_Full*Gauss_BB;
    pi_Reduced_Gauss=Gauss_B*pi_Full;
    
    F_Slotine_Reduced=Y_Reduced_Gauss*pi_Reduced_Gauss;
    Tau_Slotine_Reduced=J'\F_Slotine_Reduced;
    
    E3=Tau_Explicit-Tau_Slotine_Reduced;
    EE1_3(i)=E3(1);
    EE2_3(i)=E3(2);
    EE3_3(i)=E3(2);
    
    i=i+1;
end

figure(1)
subplot(311)
hold on
plot(time,EE1_2,'b+')
grid on;
hold on
plot(time,EE1_3,'g*')
%legend('Slotine-Li Regressor','Reduced Slotine-Li Regressor','Location','best')
xlabel('Time(s)')
ylim([-1e-014 1e-014])
ylabel('E_{\tau_1} ')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(312)
hold on
plot(time,EE2_2,'b+')
hold on
plot(time,EE1_3,'g*')
grid on;
%legend('Slotine-Li Regressor','Reduced Slotine-Li Regressor','Location','best')
xlabel('Time(s)')
ylabel('E_{\tau_2}')
ylim([-1e-014 1e-014])
set(gca,'FontWeight','bold','FontName','times','FontSize',17)


subplot(313)
hold on
plot(time,EE3_2,'b+')
hold on
plot(time,EE3_3,'g*')
grid on;
legend('SL Regressor','Reduced SL Regressor','Location','best')
xlabel('Time(s)')
ylabel('E_{\tau_3}')
ylim([-1e-014 1e-014])
set(gca,'FontWeight','bold','FontName','times','FontSize',17)




