%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology

%%
clear;
clc;

%% Time of Simulation Define

T_final=25;
Step=0.005;
time=0:Step:T_final;

m=2;    
k=T_final/Step;
i=1;

%%
for t=0:Step:T_final
    %% Random Trajectory Define
    Data_phi=Data_Generator_phi;
    Data_gamma=Data_Generator_gamma;  
    
    X=[Data_phi(1,1);Data_gamma(1,1)];  
    X_dot=[Data_phi(2,1);Data_gamma(2,1)];  
    X_dot_dot=[Data_phi(3,1);Data_gamma(3,1)];  
    %% Regressor Dynamic Analysis
    Y_Full=Full_Regressor(X,X_dot,X_dot_dot);
    pi_Full=Inertial_Parameters_Full;
    F_Full=Y_Full*pi_Full;    
    F1_Full(i)=F_Full(1);F2_Full(i)=F_Full(2);    
    
    %% Reduction_Regressor
    j=i-1;
    
    Observation_Matrix((j*2+1):(2*j+2),:)=Y_Full;
    i=i+1;
end


%% Gauss_Jordan

km=k*m
Rank=rank(Observation_Matrix)


tic
Gauss_A=rref(Observation_Matrix);
row_is_zero = all(Gauss_A==0,2);
Gauss_B=unique( Gauss_A(~row_is_zero, :), 'rows');
toc   

Gauss_S=eye(36,36);
Gauss_BB=Gauss_S*Gauss_B'*(Gauss_B*Gauss_S*Gauss_B')^-1;
    
save('Gauss_Jordan','Gauss_B','Gauss_BB')

