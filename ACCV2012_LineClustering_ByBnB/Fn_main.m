% ACCV 2012: "Globally optimal consensus set maximization through rotation search" by Jean-Charles Bazin, Yongduek Seo, and Marc Pollefeys, ACCV, 2012.
% version: April 2016 


% Set the following parameters:
%    - filenames and paths: see beginning of Fn_Demo_Ladybug_MAIN
%    - line detection: see Fn_Detection_OmniLine
%    - line clustering: see Nb_VPs and InlierThreshold and flag_InlierDistanceDefinition below
%    - BnB: see Fn_BnB_InlierOptimization (FlagDepthOrBreadthSearch and flag_DFS_SelectionMethod)


clear all; close all;
%restoredefaultpath % optional

% get some data
addpath('..\CVIU2009_LineExtraction_Ladybug')
Fn_Demo_Ladybug_MAIN %will provide list_normals and ImageGray

clear MyData; % contains the data to process
MyData.SetNormals=list_normals; % the list of the normals on the equivalent sphere, obtained by Fn_Demo_Ladybug_MAIN
MyData.Nb_VPs=3; % set the number of vanishing points here (usually 3)
MyData.InlierThreshold=deg2rad(2); % set the inlier threshold according to the distance definition (see flag_InlierDistanceDefinition). It is an important parameter: it controls when a line is considered an inlier.
MyData.FlagDistanceDefinition=0; % see Fn_ComputeVPDist
%       - 0 or []: geodesic (angular) distance on the sphere
%       - 1: distance in the image between the VP and the lines
%       - 2: distance in the image between the VP and the middle of the lines
AprioriSolution=[]

flag_SynthesizedOrRealData=1; % 0 for synthesized data, 1 for real data (for display or debug/check)

% apply BnB
profile viewer
flag_application=10; % do not change (10 for line clustering)
[R_best,ListPostInfo,ListExtraInfo]=Fn_BnB_InlierOptimization(flag_application,MyData,AprioriSolution)


% display the BnB monitoring results
Fn_BnB_InlierOptimization_DisplayResult


% decode the results
dominant_directions=ListExtraInfo.dominant_directions;
List_ClusteredNormals=ListExtraInfo.List_ClusteredNormals;
%List_ClusteredNormalsIndices=ListExtraInfo.List_ClusteredNormalsIndices;
%MyAngles_ByBounds=ListExtraInfo.RotationAngles;


%%%%%%% start - display the clustered lines and the VPS %%%%%%%%
Icol=Fn_Draw_Lines_and_VPs(ImageGray, List_ClusteredNormals, [], [], dominant_directions, [1 1 0]);
figure; imshow(Icol);
imwrite(Icol,'VP-extraction.png');
%%%%%%%  end - display the clustered lines and the VPS  %%%%%%%%
