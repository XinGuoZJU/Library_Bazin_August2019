function ListInfoCalibration=Fn_Build_ListInfoCalibration(flag_ProjectionModel,Hc,Phi,xi,kc, HeightImage, WidthImage)

% used to avoid too many input arguments in:
%   - Fn_MotionEstimation_Omni_Simple_MAIN

%   - ListInfoCalibration. cf Fn_Parameters_and_Initialization_SequenceInformation
%           - flag_ProjectionModel:
%                 - 0-Barreto Model. It requires the global variables Hc and Phi
%                 - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%                 - 2-Ladybug linear model. It requires the global variables: height_image and width_image
%                 - 10-Stereo-omni by Jang Gijeong
%           - Hc Phi xi kc; % Hc is for Barreto and Mei's model. Phi is for Barreto. xi and kc are for Mei.
%           - height_image, width_image; % for ladybug linear projection

% if xi=0 for Mei then perspective.

% related to Fn_Build_ListInfoCalibration_Unpack

% read input
ListInfoCalibration.flag_ProjectionModel=flag_ProjectionModel;
ListInfoCalibration.Hc=Hc;
ListInfoCalibration.Phi=Phi;
ListInfoCalibration.xi=xi;
if length(kc)==0; kc=zeros(1,5); end
ListInfoCalibration.kc=kc;
ListInfoCalibration.height_image=HeightImage;
ListInfoCalibration.width_image=WidthImage;