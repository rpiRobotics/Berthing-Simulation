function sigma = inequality_bound(h,c,eta,epsilon,e)
% This function generates inequality bounds for the resolved velocity
% motion controller based on scaling parameters and the evaluation of the
% inequality bound.
%
% Inputs:
%   h: evaulation of the inequality bound (e.g., joint stops)
%   c: scaling factor - see below
%   eta: scaling factor - see below
%   epsilon: scaling factor - see below
%   e: scaling factor - see below
%
% Outputs:
%   sigma: differential inequality constraint - see below
%
% Inequality constraint of the form h >= eta > 0
% Convert to differential constraint of the form 
%   (partial h/ partial q)*dq >= sigma
% sigma = -tan(c*pi/2) if h>= eta + epsilon (essentially unconstrained)
% sigma = -tan(c*pi(h-eta)/(2*epsilon)) if eta < h < eta + epsilon
%   (near constraint boundary, move away from constraint at slow rate)
% sigma = epsilon(eta-h)/eta if 0<=h<=eta (constraint is violated, 
%   move away from constraint at quicker rate)
% sigma = epsilon if else (constraint is violated, move away from 
%   constraint at quicker rate)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sigma = zeros(size(h));
    h2 = h - eta;
    sigma(h2 >= epsilon) = -tan(c*pi/2);
    sigma(h2 >= 0 & h2 < epsilon) = ...
                    -tan(c*pi/2/epsilon*h2(h2 >= 0 & h2 < epsilon));
    sigma(h >= 0 & h2 < 0) = -e*h2(h >= 0 & h2 < 0)/eta;
    sigma(h < 0) = e;
end