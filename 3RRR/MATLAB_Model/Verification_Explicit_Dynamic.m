%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc
close all;
load('Adams_Data')

i=1;
%% Geometry Define

Lambda=[0;120;240]*(pi/180);Eta=Lambda;
Beta=acos(sqrt(3)/3);Gamma=Beta;

alpha__1=[80*(pi/180);80*(pi/180);80*(pi/180)];
alpha__2=[70*(pi/180);70*(pi/180);70*(pi/180)];
g=[0;0;-9.80665];

%% Time of Simulation 
T_final=1;
Step=0.005;
time=0:Step:T_final;

%% Trajectory Define
X1J=Trajectory(0*(pi/180),10*(pi/180),0,0,0,T_final);
X2J=Trajectory(0*(pi/180),30*(pi/180),0,0,0,T_final);
X3J=Trajectory(0*(pi/180),20*(pi/180),0,0,0,T_final);

for t=0:Step:T_final
    
    X1=X1J(1)+X1J(2)*t+X1J(3)*t^2+X1J(4)*t^3;
    X1_dot=X1J(2)+2*X1J(3)*t+3*X1J(4)*t^2;
    X1_dot_dot=2*X1J(3)+6*X1J(4)*t;    

    X2=X2J(1)+X2J(2)*t+X2J(3)*t^2+X2J(4)*t^3;
    X2_dot=X2J(2)+2*X2J(3)*t+3*X2J(4)*t^2;
    X2_dot_dot=2*X2J(3)+6*X2J(4)*t;    
    
    X3=X3J(1)+X3J(2)*t+X3J(3)*t^2+X3J(4)*t^3;
    X3_dot=X3J(2)+2*X3J(3)*t+3*X3J(4)*t^2;
    X3_dot_dot=2*X3J(3)+6*X3J(4)*t;  

    X=[X1;X2;X3];    
    X_dot=[X1_dot;X2_dot;X3_dot];      
    Xddot=[X1_dot_dot;X2_dot_dot;X3_dot_dot];  

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
    
    %% Plot Generation
    Plot_Tau(:,i)=(Tau_Explicit);
    
    i=i+1;
end

%% Notice: ADAMS Model Measured Magnitude of Actuated Joint Variables 1. In this section, ADAMS data are modified.
j=1;
for i=1:201
    if i<150
        Tau1_ADAMS_C(j,1)=Tau1_ADAMS(i);
        j=j+1;
    end
    if i>=150
        Tau1_ADAMS_C(j,1)=-Tau1_ADAMS(i);
        j=j+1;
    end
end

%% Plot
figure(1)
subplot(311)
plot(time,Plot_Tau(1,:),'linewidth',4)
grid on;
hold on;
plot(time_ADAMS,Tau1_ADAMS_C,'r--','linewidth',4)
%legend('Explicit Virtual Work','MSC ADAMS','Location','best')
ylabel('\tau_1 (N.m)')
xlabel('Time(s)')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(312)
plot(time,Plot_Tau(2,:),'linewidth',4)
grid on;
hold on;
plot(time_ADAMS,Tau2_ADAMS,'r--','linewidth',4)
%legend('Explicit Virtual Work','MSC ADAMS','Location','best')
ylabel('\tau_2 (N.m)')
xlabel('Time(s)')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)


subplot(313)
plot(time,Plot_Tau(3,:),'linewidth',4)
grid on;
hold on;
plot(time_ADAMS,Tau3_ADAMS,'r--','linewidth',4)
legend('Explicit Virtual Work','MSC ADAMS','Location','best')
ylabel('\tau_3 (N.m)')
xlabel('Time(s)')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)


figure(2)
subplot(311)
plot(time,Plot_Tau(1,:)'-Tau1_ADAMS_C,'+')
grid on;
ylabel('E_{\tau_1}')
xlabel('Time(s)')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(312)
plot(time,Plot_Tau(2,:)'-Tau2_ADAMS,'ro')
ylabel('E_{\tau_2}')
grid on;
xlabel('Time(s)')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)

subplot(313)
plot(time,Plot_Tau(3,:)'-Tau3_ADAMS,'g*')
ylabel('E_{\tau_3}')
grid on;
xlabel('Time(s)')
set(gca,'FontWeight','bold','FontName','times','FontSize',17)


