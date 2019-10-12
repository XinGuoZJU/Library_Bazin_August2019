function [ListTheta, ListPhi, ListR]=Fn_cart2sph(ListXs, ListYs, ListZs, flag_convention)

% inputs:
%   - flag_convention
%       - if 0: matlab convention: theta=0 at Ox, Phi=0 in the x-y plane, phi=pi/2 on the positive z-axis
%       - if 1: Banno convention: theta=0 at Ox, Phi=0 on the positive z-axis (so phi=pi/2 on the x-y plane)
%                  - i.e. theta is same but PhiBanno=-PhiMatlab+pi/2
%                  - i.e. theta is same but PhiMatlab=-PhiBanno+pi/2

% Note: cart2sph returns theta in [-pi,+pi] and phi in [-pi/2,+pi/2]
% associated code for this test:
%     [ListXs,ListYs,ListZs] = sphere(50);
%     [ListTheta, ListPhi, ListR]=cart2sph(ListXs, ListYs, ListZs);
%     ThetaMin=min(min(ListTheta(:))), ThetaMax=max(max(ListTheta(:)))
%     PhiMin=min(min(ListPhi(:))), PhiMax=max(max(ListPhi(:)))


[ListTheta, ListPhi, ListR]=cart2sph(ListXs, ListYs, ListZs);

if flag_convention==0
    % nothing
elseif flag_convention==1
% transform from matlab to banno's convention
    ListPhi=-ListPhi+pi/2;
    %[rows cols]=find(ListTheta<0);
    %indices=sub2ind(ListTheta,rows,cols);
    indices=find(ListTheta<0);
    ListTheta(indices)=ListTheta(indices)+2*pi;
else
    error('wrong case')
end