function [ListXs, ListYs, ListZs]=Fn_sph2cart(ListTheta, ListPhi, ListR, flag_convention)

% inputs:
%   - flag_convention
%       - if 0: matlab convention: theta=0 at Ox, Phi=0 in the x-y plane, phi=pi/2 on the positive z-axis
%       - if 1: Banno convention: theta=0 at Ox, Phi=0 on the positive z-axis (so phi=pi/2 on the x-y plane)
%                  - i.e. theta is same but PhiBanno=-PhiMatlab+pi/2
%                  - i.e. theta is same but PhiMatlab=-PhiBanno+pi/2


if flag_convention==0
    % nothing
elseif flag_convention==1
% transform from banno's convention to matlab
    ListPhi=-ListPhi+pi/2;
else
    error('wrong case')
end

    [ListXs, ListYs, ListZs]=sph2cart(ListTheta, ListPhi, ListR);