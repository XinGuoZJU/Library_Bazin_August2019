function ListInfoCalibration=Fn_BuildListInfoCalibration()

%       - flag_ProjectionModel:
%               - 0-Barreto Model. It requires the global variables Hc and Phi
%               - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc (size is 1x5)
%               - 2-Ladybug linear model. It requires the global variables: height_image and width_image
%               - 3-Scaramuzza model
%               - 10-Stereo-omni by Jang Gijeong

global flag_ProjectionModel Hc Phi xi kc height_image width_image;
if length(flag_ProjectionModel)==0; error('not defined'); end
ListInfoCalibration.flag_ProjectionModel=flag_ProjectionModel;

if flag_ProjectionModel==0
    if length(Hc)==0; error('not defined'); end
    if length(Phi)==0; error('not defined'); end
    ListInfoCalibration.Hc=Hc;
    ListInfoCalibration.Phi=Phi;
    
elseif flag_ProjectionModel==1
    if length(Hc)==0; error('not defined'); end
    if length(xi)==0; error('not defined'); end
    %if length(kc)==0; error('not defined'); end
    ListInfoCalibration.Hc=Hc;
    ListInfoCalibration.xi=xi;
    ListInfoCalibration.kc=kc;
    
elseif flag_ProjectionModel==2
    if length(height_image)==0; error('not defined'); end
    if length(width_image)==0; error('not defined'); end
    ListInfoCalibration.height_image=height_image;
    ListInfoCalibration.width_image=width_image;
    
elseif flag_ProjectionModel==3
    error('to do maybe')
else
    error('wrong case')
end



% if length(Hc)==0; error('not defined'); end
% if length(Phi)==0; error('not defined'); end
% if length(xi)==0; error('not defined'); end
% if length(kc)==0; error('not defined'); end
% ListInfoCalibration.Hc=Hc;
% ListInfoCalibration.Phi=Phi;
% ListInfoCalibration.xi=xi;
% ListInfoCalibration.kc=kc;