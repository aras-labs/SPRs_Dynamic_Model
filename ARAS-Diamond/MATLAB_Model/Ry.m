%%
%  Author(s):  Ali Hassani, Abbas Bataleblu , S. Ahmad Khalilpour, Hamid D.Taghirad, Philippe Cardou
%  Created on: August 28, 2021
%  Copyright (c)  2021, Advanced Robotics and Automated Systems (ARAS), K.N. Toosi University of Technology
function output = Ry(input)

output     = [cos(input), 0 , sin(input);...
         0     , 1 , 0     ;...
        -sin(input), 0 , cos(input);];
    
% output(1,1) = cos(input);
% output(1,2) = 0;
% output(1,3) = sin(input);
% 
% output(2,1) = 0;
% output(2,2) = 1;
% output(2,3) = 0;
% 
% output(3,1) = -sin(input);
% output(3,2) =0;
% output(3,3) = cos(input);
end