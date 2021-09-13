%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
%%
clear;
clc;
load('Gauss_Jordan.mat')

%% Define Syms
syms m__1; syms m__2;syms m__3;syms m__4;
syms I__xx1;syms I__yy1;syms I__zz1;syms I__xy1;syms I__xz1;syms I__yz1;
syms I__xx2;syms I__yy2;syms I__zz2;syms I__xy2;syms I__xz2;syms I__yz2;
syms I__xx3;syms I__yy3;syms I__zz3;syms I__xy3;syms I__xz3;syms I__yz3;
syms I__xx4;syms I__yy4;syms I__zz4;syms I__xy4;syms I__xz4;syms I__yz4;


syms rho__x1;syms rho__y1;syms rho__z1;
syms rho__x2;syms rho__y2;syms rho__z2;
syms rho__x3;syms rho__y3;syms rho__z3;
syms rho__x4;syms rho__y4;syms rho__z4;

Gauss_B=round(Gauss_B,5);

%% Symbole Base Inertial Parameters

pi_Full=[m__1 * rho__x1; m__1 * rho__y1; m__1 * rho__z1; I__xx1 + m__1 * (rho__y1 ^ 2 + rho__z1 ^ 2); -m__1 * rho__x1 * rho__y1 - I__xy1; -m__1 * rho__x1 * rho__z1 - I__xz1; I__yy1 + m__1 * (rho__x1 ^ 2 + rho__z1 ^ 2); -m__1 * rho__y1 * rho__z1 - I__yz1; I__zz1 + m__1 * (rho__x1 ^ 2 + rho__y1 ^ 2); m__2 * rho__x2; m__2 * rho__y2; m__2 * rho__z2; I__xx2 + m__2 * (rho__y2 ^ 2 + rho__z2 ^ 2); -m__2 * rho__x2 * rho__y2 - I__xy2; -m__2 * rho__x2 * rho__z2 - I__xz2; I__yy2 + m__2 * (rho__x2 ^ 2 + rho__z2 ^ 2); -m__2 * rho__y2 * rho__z2 - I__yz2; I__zz2 + m__2 * (rho__x2 ^ 2 + rho__y2 ^ 2); m__3 * rho__x3; m__3 * rho__y3; m__3 * rho__z3; I__xx3 + m__3 * (rho__y3 ^ 2 + rho__z3 ^ 2); -m__3 * rho__x3 * rho__y3 - I__xy3; -m__3 * rho__x3 * rho__z3 - I__xz3; I__yy3 + m__3 * (rho__x3 ^ 2 + rho__z3 ^ 2); -m__3 * rho__y3 * rho__z3 - I__yz3; I__zz3 + m__3 * (rho__x3 ^ 2 + rho__y3 ^ 2); m__4 * rho__x4; m__4 * rho__y4; m__4 * rho__z4; I__xx4 + m__4 * (rho__y4 ^ 2 + rho__z4 ^ 2); -m__4 * rho__x4 * rho__y4 - I__xy4; -m__4 * rho__x4 * rho__z4 - I__xz4; I__yy4 + m__4 * (rho__x4 ^ 2 + rho__z4 ^ 2); -m__4 * rho__y4 * rho__z4 - I__yz4; I__zz4 + m__4 * (rho__x4 ^ 2 + rho__y4 ^ 2)];
pi_Reduced_Gauss=Gauss_B*pi_Full;

pi_Reduced_Gauss=vpa(pi_Reduced_Gauss,5);

for i=1:17
    Base_Inertial(1,i)=pi_Reduced_Gauss(i,1);
end

%% For Symbolic Usage
Base_Inertial=vpa(Base_Inertial,5)





