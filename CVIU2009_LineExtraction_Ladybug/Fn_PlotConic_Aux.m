function [ConicPts,GreatCirclePts]=Fn_PlotConic_Aux(my_normal,image_size)
    
% inputs:
%   - my_normal. 1x3. can be 3x1??

% outputs:
%   - ConicPts: Nx2
%   - GreatCirclePts: Nx3


if nargin~=2, nargin, error('wrong nb of inputs'); end
if nargout~=2, nargout, error('wrong nb of outputs'); end 


global flag_ProjectionModel;
if length(flag_ProjectionModel)==0; error('not provided'); end


height_image=image_size(1);
width_image=image_size(2);


if flag_ProjectionModel==0 % 0-Barreto Model, 1-Mei model, 2-ladybug linear model, 3-Scaramuzza model
    global Hc;
    if length(Hc)==0; error('not provided'); end
    
 Omega=[-my_normal(3)*my_normal(3) 0 my_normal(1)*my_normal(3)
        0 -my_normal(3)*my_normal(3) my_normal(2)*my_normal(3)
        my_normal(1)*my_normal(3) my_normal(2)*my_normal(3) my_normal(3)*my_normal(3)];
     OmegaI=(inv(Hc))'*Omega*inv(Hc);
      [ConicPts]=Fn_PlotConic_Barreto(OmegaI,[height_image width_image]);
      GreatCirclePts=[];
      
elseif flag_ProjectionModel==1 || flag_ProjectionModel==2 || flag_ProjectionModel==3
    % compute the great circles on sphere
[Array_GreatCirclePts] = Fn_ComputeGreatCirclesPointsOnSphere(my_normal);
% of GreatCirclePts=[];


%Array_GreatCirclePts, size(Array_GreatCirclePts)
[Array_GreatCirclePts,Array_ConicPts] = Fn_PlotConic_Mei(my_normal, Array_GreatCirclePts);


NbPoints=Array_GreatCirclePts(1,1,1);
GreatCirclePts=squeeze(Array_GreatCirclePts(1,1+1:1+NbPoints,:));
ConicPts=squeeze(Array_ConicPts(1,1+1:1+NbPoints,:));
clear Array_GreatCirclePts;
clear Array_ConicPts;

else
    error('wrong case')

end
