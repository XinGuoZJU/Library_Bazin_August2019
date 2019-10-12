% author: Jean-Charles Bazin
% date: 2012/08/10
% references: 
%   - "Motion estimation by decoupling rotation and translation in catadioptric vision", by Jean-Charles Bazin, Cedric Demonceaux, Pascal Vasseur and Inso Kweon, CVIU, 2010
%   - "Rotation Estimation and Vanishing Point Extraction by Omnidirectional Vision in Urban Environment", by Jean-Charles Bazin, Cedric Demonceaux, Pascal Vasseur and Inso Kweon, IJRR, 2012

% inputs:
%   - a perspective image
%   - its calibration parameters

% outputs:
%   - list_normals: contains the normals of the detected lines in the equivalent sphere. Size is Nx3 (xyz) where N is the number of detected lines 
%   - list_LineImagePoints
%   - list_LineSphPoints

clear all; close all;

% set the filename of the image
image_filename='AMRO_65000.png'



% read the image
ImageGray=rgb2gray(imread(image_filename));

% % build the calibration information
% height_image=size(ImageGray,1);
% width_image=size(ImageGray,2);
% ListInfoCalibration=Fn_Build_ListInfoCalibration(flag_ProjectionModel,Hc,Phi,xi,kc, height_image, width_image);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%  START - extract the lines  %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% [list_normals, list_LineImagePoints, list_LineSphPoints]=Fn_Detection_OmniLine(ImageGray); 

[list_normals, list_LineImagePoints]=Fn_Detection_PerspecLine(ImageGray); % returns a list of normal vectors (z-coord is positive)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%   END - extract the lines   %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% start - display the lines %%%%%%%%
Image=Fn_Draw_Lines_Perspec(ImageGray, list_normals, list_LineImagePoints, 1);
    
figure('name','final'); imshow(Image/255);
imwrite(Image,'result_final.png');
%%%%%%%  end - display the lines  %%%%%%%%


