%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc;
load('Gauss_Jordan.mat')

%% Time of Simulation Define
T_final=2;
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
    Data_phi=Data_Generator_phi;
    Data_gamma=Data_Generator_gamma;  
    
             X=[Data_phi(1,1);Data_gamma(1,1)];  
         X_dot=[Data_phi(2,1);Data_gamma(2,1)];  
        XR_dot=[Data_phi(2,1);Data_gamma(2,1)];  
     X_dot_dot=[Data_phi(3,1);Data_gamma(3,1)];
    XR_dot_dot=[Data_phi(3,1);Data_gamma(3,1)];
    
    %% Explicit Dynamic
    M=Mass_Matrix(X);
    C=C_Matrix(X,X_dot);
    G=G_Vector(X);
    F_Explicit_Slotine=(M)*XR_dot_dot + (C)*XR_dot + (G);
   
    %% Slotine Regressor Verification
    YSlotine_Full=Slotine_Regressor(X,X_dot,XR_dot,XR_dot_dot);
    pi_Full=Inertial_Parameters_Full;
    
    F_Slotine_Regressor=YSlotine_Full*pi_Full;     
    E2=F_Explicit_Slotine-F_Slotine_Regressor;
    
    EE1_2(i)=E2(1);
    EE2_2(i)=E2(2);
       
    %% Slotine-Li Regressor Reduction
    Y_Reduced_Gauss=YSlotine_Full*Gauss_BB;
    pi_Reduced_Gauss=Gauss_B*pi_Full;
    
    F_Slotine_Reduced=Y_Reduced_Gauss*pi_Reduced_Gauss;
    E3=F_Slotine_Reduced-F_Slotine_Regressor;
    EE1_3(i)=E3(1);
    EE2_3(i)=E3(2);
    
    i=i+1;
end

figure(1)
subplot(211)
plot(time,EE1_2,'g+')
grid on;
hold on
plot(time,EE1_3,'b*')
xlabel('Time(s)')
ylim([-1e-13 1e-13])
ylabel('E_{\tau_1}')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(212)
plot(time,EE2_2,'g+')
hold on
plot(time,EE1_3,'b*')
grid on;
legend('S-L Regressor','Reduced SL Regressor')
xlabel('Time(s)')
ylabel('E_{\tau_2}')
ylim([-1e-13 1e-13])
set(gca,'FontWeight','bold','FontName','times','FontSize',17)



