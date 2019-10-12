% Author: Jean-Charles Bazin
% Version: April 2016
% References:
%   - "Motion estimation by decoupling rotation and translation in catadioptric vision", by Jean-Charles Bazin, Cedric Demonceaux, Pascal Vasseur and Inso Kweon, CVIU, 2010
%   - "Rotation Estimation and Vanishing Point Extraction by Omnidirectional Vision in Urban Environment", by Jean-Charles Bazin, Cedric Demonceaux, Pascal Vasseur and Inso Kweon, IJRR, 2012
%
% This code is for extracting lines in omnidirectional image acquired by a Ladybug camera. By setting the calibration parameters (see below), it can also be applied to other central images (catadioptric, fisheye, perspective, etc).
%
% Inputs:
%   - an omnidirectional (Ladybug) image
%   - an associated binary mask. optional
%   - the intrinsic calibration parameters

% Outputs:
%   - list_normals: contains the normals of the detected lines in the equivalent sphere. Size is Nx3 (xyz) where N is the number of detected lines
%   - list_LineImagePoints
%   - list_LineSphPoints

clear all; close all;
%restoredefaultpath % optional

% set the filename of the image and the binary mask
image_filename='LadybugIndoor.png'
mask_filename='LadybugIndoor_Mask.bmp' % if no mask, just write '' or []



% do not change the following lines (initialization and calibration parameters):
% initialization
global flag_ProjectionModel Hc Phi xi kc; % Hc is for Barreto and Mei's model. Phi is for Barreto. xi and kc (size is 1x5) are for Mei.
global height_image width_image; % for ladybug linear projection
global Mask;
global ListInfoCalibration;
% calibration
flag_ProjectionModel=2; % set the projection model: 0-Barreto, 1-Mei, 2-Ladybug, 3-Scaramuzza model, 10-Stereo-omni by Jang Gijeong

% read the image and the mask
ImageGray=rgb2gray(imread(image_filename));
if isempty(mask_filename)==1
    Mask=ones(size(ImageGray,1),size(ImageGray,2)); % same size as the input image
else
    Mask=imread(mask_filename);
end

% if the image is way too big, let's downsize it (processing is much faster and the result is almost the same)
target_H=1000;
if size(ImageGray,1)>target_H
    ImageGray=imresize(ImageGray,[target_H nan],'bicubic');
    Mask=imresize(Mask,[target_H nan],'nearest'); % binary
end

% build the calibration information
height_image=size(ImageGray,1);
width_image=size(ImageGray,2);
ListInfoCalibration=Fn_Build_ListInfoCalibration(flag_ProjectionModel,Hc,Phi,xi,kc, height_image, width_image);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%  START - extract the lines  %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[list_normals, list_LineImagePoints, list_LineSphPoints]=Fn_Detection_OmniLine(ImageGray); % returns a list of normal vectors (z-coord is positive)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%   END - extract the lines   %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% start - display the lines %%%%%%%%
Image=Fn_Draw_Lines(ImageGray, list_normals, list_LineImagePoints, [255 0 0], 1, 0);
figure('name','display the final result'); imshow(Image);
imwrite(Image,'results_final.png');
%%%%%%%  end - display the lines  %%%%%%%%


