
% Fn_Projection_On_Image_From_Sphere

% version 20/11/2009
% modifications: 03/2009
%   - only Phi and Hc as global variables
%   - Fn_Check_Arguments to verify the format of input
%   - another formula for projection
% modifications: 15/11/2009
%   - flag_ProjectionModel to use different projection models
%   - explanations about inputs, outputs and procedure
%   - vectorized version so faster
% modifications: 20/11/2009
%   - some notes about normalization
%   - verify the global variables
% modifications: 03/02/2010
%   - projection for ladybug linear model (flag_ProjectionModel==2)
% modifications: 04/07/2010
%   - extension to the HxWx3 case
% modifications: 05/07/2010
%   - call Fn_Projection_OnImageFromSphere_Local (it avoids to modify several similar codes)

% inputs:
%   - my_list_points_sphere: Nx3, the coordinates in the sphere, in x,y,z format
%
% output:
%   - my_list_points: Nx2 the coordinates in the image, in x,y DOUBLE format (i.e. not integer)
%
% global variables:
%   flag_ProjectionModel:
%       - 0-Barreto Model. It requires the global variables Hc and Phi
%       - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc. 

% Note: I think the points must have a unit norm TO CONFIRM!


function [my_image_point_list]=Fn_Projection_OnImageFromSphere(my_point_sphere_list) % x,y, z and returns x,y


%ListInfoCalibration=Fn_BuildListInfoCalibration();
global ListInfoCalibration; % see Fn_Build_ListInfoCalibration();
if length(ListInfoCalibration)==0, error('undefined'); end
[my_image_point_list]=Fn_Projection_OnImageFromSphere_Local(my_point_sphere_list,ListInfoCalibration);
return;
