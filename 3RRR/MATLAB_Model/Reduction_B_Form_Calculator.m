%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc;

%% Time of Simulation

T_final=50;
Step=0.005;
time=0:Step:T_final;

m=3;    
k=T_final/Step;
i=1;

%%
for t=0:Step:T_final
    %% Random Trajectory Define
    Data_X1=Data_Generator;
    Data_X2=Data_Generator;  
    Data_X3=Data_Generator;  
    
            X=[Data_X1(1,1);Data_X2(1,1);Data_X3(1,1)];  
        X_dot=[Data_X1(2,1);Data_X2(2,1);Data_X3(2,1)];  
    X_dot_dot=[Data_X1(3,1);Data_X2(3,1);Data_X3(3,1)];  

    %% Regressor Dynamic Analysis

    [Y_Mp,Y_Links]=Full_Regressor(X,X_dot,X_dot_dot);
    [pi_Mp,pi_Links]=Inertial_Parameters_Full;
    Y_Full=[Y_Mp,Y_Links];
    pi_Full=[pi_Mp;pi_Links];
    
    %% Reduction_Regressor
    j=i-1;
    
    Observation_Matrix((j*3+1):(3*j+3),:)=Y_Full;
    i=i+1;
end


%% Gauss_Jordan Elimination

km=k*m
Rank=rank(Observation_Matrix)


tic
Gauss_A=rref(Observation_Matrix);
row_is_zero = all(Gauss_A==0,2);
Gauss_B=unique( Gauss_A(~row_is_zero, :), 'rows');
toc   

Gauss_S=eye(63,63);
Gauss_BB=Gauss_S*Gauss_B'*(Gauss_B*Gauss_S*Gauss_B')^-1;
    
save('Gauss_Jordan','Gauss_B','Gauss_BB')

