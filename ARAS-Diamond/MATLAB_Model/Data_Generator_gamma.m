%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function [Data] = Data_Generator_gamma()

%-pi/2
% setq=[-pi/2,pi/2];
% setQ=[-1,0,1];
% setQQ=[0,1];

setq=[0+0.001:0.001:pi/2-0.001];
setQ=[-4:0.005:4];
setQQ=[-4:0.005:4];

%setQQ=[-1,0,1];

q1=setq(randperm(numel(setq),1));
Q1=setQ(randperm(numel(setQ),1));
QQ1=setQQ(randperm(numel(setQQ),1));
Data=[q1,Q1,QQ1];
Data=Data';
end

