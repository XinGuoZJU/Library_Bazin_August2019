% IROS 2012: "3-line RANSAC for Orthogonal Vanishing Point Detection", by Jean-Charles Bazin and Marc Pollefeys
% version: April 2016 (from 22/04/2014)


% Set the following parameters:
%    - filenames and paths: see beginning of Fn_Demo_Ladybug_MAIN
%    - line detection: see Fn_Detection_OmniLine
%    - line clustering: see Nb_VPs and InlierThreshold and flag_InlierDistanceDefinition below
%    - RANSAC: see Fn_Find_Rotation_by_MultiRansac (nb of iterations)

clear all; close all; clc;

% get code for line clustering
addpath('..\ACCV2012_LineClustering_ByBnB')

% get some data
addpath('..\CVIU2009_LineExtraction_Ladybug')
Fn_Demo_Ladybug_MAIN %will provide list_normals and ImageGray



clear MyData; % contains the data to process
MyData.SetNormals=list_normals; % the list of the normals on the equivalent sphere, obtained by Fn_Demo_Ladybug_MAIN
MyData.Nb_VPs=3; % set the number of vanishing points here (usually 3)
InlierThreshold=deg2rad(2); % set the inlier threshold according to the distance definition (see flag_InlierDistanceDefinition). It is an important parameter: it controls when a line is considered an inlier.
flag_InlierDistanceDefinition=0; % see Fn_ComputeVPDist
%       - 0 or []: geodesic (angular) distance on the sphere
%       - 1: distance in the image between the VP and the lines
%       - 2: distance in the image between the VP and the middle of the lines
AprioriSolution=[]

% apply RANSAC
profile on
[MyAngles, ListExtraInfo, ExtraOutput]...
    =Fn_Find_Rotation_by_MultiRansac(MyData.SetNormals,InlierThreshold, MyData.Nb_VPs, flag_InlierDistanceDefinition);
profile viewer



% decode the results
dominant_directions=ListExtraInfo.dominant_directions;
List_ClusteredNormals=ListExtraInfo.List_ClusteredNormals;
%List_ClusteredNormalsIndices=ListExtraInfo.List_ClusteredNormalsIndices;


%%%%%%% start - display the clustered lines and the VPS %%%%%%%%
Icol=Fn_Draw_Lines_and_VPs(ImageGray, List_ClusteredNormals, [], [], dominant_directions, [1 1 0]);
figure; imshow(Icol);
imwrite(Icol,'VP-extraction.png');
%%%%%%%  end - display the clustered lines and the VPS  %%%%%%%%
