
function [Array_GreatCirclePts] = Fn_ComputeGreatCirclesPointsOnSphere(list_normals)


global flag_ProjectionModel;
if length(flag_ProjectionModel)==0
    error('flag_ProjectionModel not provided')
end
if flag_ProjectionModel==0 % 0-Barreto Model, 1-Mei model, 2-ladybug linear model
%error('not done because of xi')
elseif flag_ProjectionModel==1
    global xi;
    ListInfoCalibration.xi=xi;
elseif flag_ProjectionModel==2
    % nothing
else
    error('wrong case')
end

ListInfoCalibration.flag_ProjectionModel=flag_ProjectionModel;


%   - ListInfoCalibration. cf Fn_Parameters_and_Initialization_SequenceInformation
%           - flag_ProjectionModel:
%                 - 0-Barreto Model. It requires the global variables Hc and Phi
%                 - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%                 - 2-Ladybug linear model. It requires the global variables: height_image and width_image
%                 - 10-Stereo-omni by Jang Gijeong
%           - Hc Phi xi kc; % Hc is for Barreto and Mei's model. Phi is for Barreto. xi and kc are for Mei.
%           - height_image, width_image; % for ladybug linear projection

[Array_GreatCirclePts] = Fn_ComputeGreatCirclesPointsOnSphere_Local(list_normals,ListInfoCalibration);
