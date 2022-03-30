function Vb = adjoint(Va,T)
% ADJOINT Calculates the adjoint transformations corresponding to a
% homogenous transformation matrix T_AB between two frames.
%
% This function represents the known twist in one frame with respect to another
% frame, given the homogenous transformation matrix between the two frames.
%
% Inputs: twist_inA - a 6X1 vector representing the robot twist in frame A
%         T_AB - homogenous transformation matrix from frame A to B
%
% Output: twist_inB - a 6X1 vector representing the robot twist in frame B
%
% Author: Fenil Desai <fdesai@wpi.edu>
% Last modified: 11/03/2021
    R = [T(1,1),T(1,2),T(1,3); T(2,1), T(2,2), T(2,3); T(3,1), T(3,2), T(3,3)];
    p = [T(1,4), T(2,4), T(3,4)]';
    
    p_bracket = [0,-p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    K = p_bracket*R;
    O = [0 0 0; 0 0 0; 0 0 0];
    AD_T = [R,O;K,R];
    
    Vb = AD_T*Va;
end

