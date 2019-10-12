
% Fn_Projection_OnSphere

% version 15/11/2009
% modifications:
%   - only Phi and Hc as global variables
%   - Fn_Check_Arguments to verify the format of input
%   - a code slighlyt optimized for projection
% modifications: 15/11/2009
%   - flag_ProjectionModel to use different projection models
%   - explanations about inputs, outputs and procedure
%   - vectorized version
%   - the inputs can accept the whole image pixels so the format of input and output data is changed
% modifications: 01/07/2010: if no distortion for Mei, then do not remove distortions
% modifications: 01/07/2010: call Fn_Projection_OnSphere_Local (it avoids to modify several similar codes)

% inputs:
%   - my_list_points:
%       - Nx2 the coordinates in the image, in x,y format
%       - Or HxW for the whole image
% output:
%   - my_list_points_sphere:
%         - Nx3, the coordinates in the sphere, in x,y,z format
%         - Or HxWx3, the coordinates in the sphere, in x,y,z format

% global variables:
%   flag_ProjectionModel:
%       - 0-Barreto Model. It requires the global variables Hc and Phi
%       - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%       - 2-Ladybug linear model. It requires the global variables: height_image and width_image

function my_list_points_sphere=Fn_Projection_OnSphere(my_list_points)


ListInfoCalibration=Fn_BuildListInfoCalibration();
my_list_points_sphere=Fn_Projection_OnSphere_Local(my_list_points,ListInfoCalibration);
return

