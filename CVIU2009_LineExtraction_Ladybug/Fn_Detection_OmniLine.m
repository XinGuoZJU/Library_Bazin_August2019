function [ListNormals, list_ClusteredImagePoints, list_ClusteredSphPoints]=Fn_Detection_OmniLine(Image)
%
% version April 2016 (from 04/07/2010)
% Inputs:
%   - Image: RGB image
%
% Outputs:
%   - ListNormals: contains the normals of the detected lines in the equivalent sphere. Size is Nx3 (xyz) where N is the number of detected lines
%   - list_ClusteredImagePoints: list_ClusteredImagePoints{i} contains the 2D points (Mx2 for x,y coordinates) of the i-th line. The nb of points varies for each line. Contains the same number of lines N as ListNormals
%   - list_ClusteredSphPoints: list_ClusteredSphPoints{i} contains the 3D spherical points (Mx3 for x,y,z coordinates) of the i-th line. The nb of points varies for each line. Contains the same number of lines N as ListNormals
%
% Acknowledgement:
% This code has been implemented by Jean-Charles BAZIN in collaboration with Cedric DEMONCEAUX and Pascal VASSEUR, in the CAVIAR project framework.
% One may refer to the following paper:
% "Fast Central Catadioptric Line Extraction" by Jean Charles Bazin, Cedric Demonceaux, Pascal Vasseur
% 3rd Iberian Conference on Pattern Recognition and Image Analysis (IbPRIA'07), June 2007, Girona, Spain.
%
% This current code contains some important details that the user may want to know:
%      - in splitting step, the normal is computed by the first and the last points of the chain
%      - in merging step, the normal is computed using all the points, by SVD
%      - in merging step, graph theory is used to merge the splitting conics. There exists another version that uses sequential scanning to merge the splitting conics.
%
% modifications:
%   - update of the informations
%   - repair an error on Normal/Normal2 when computing and saving the new merged normal
%   - function Fn_Read_Image
%
% modifications: 01/11/2009
%   - defined as a function
%   - global Mask
%   - remove global Normal; Image
%   - explanations input and output
%
% modifications: 15/11/2009
%   - global Hc is not needed in this function so removed
%
% modifications: 20/11/2009
%   - flag_ComputeOrRead_edge=0; % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
%           - 0:traditional procedure: apply edge extraction, chaining and splitting
%           - 1: do not perform neither edge nor chain. For the splitting, force the fitting, i.e. do not split
%
% modifications: 26/12/2009
%   - write edge and chain in a file
% modifications: 04/07/2010
%   - Image is uint8 for memory efficiency
%
disp('start Fn_Detection_OmniLine_Special')

global Mask;
global nth_frame;
global my_dir_saving;
close all;

%%%%%%%%%%% start  - parameters and options %%%%%%%%%%%%%%%
%Fn_Parameters_and_Initialization_ForLineExtraction;
% set the parameters here:
flag_LineCostFunction=1; %0-algebraic cost, 1-geometric cost
local_length=0.08*size(Image,1); % as a percentage of the image size
min_length_edge_link=local_length;%set the minimal authorized length in edge chaining (if smaller, the chain will be removed)
ThresholdPixel =local_length;%set the minimal authorized length in splitting step (if smaller, the chain will be removed)
ThresholdError = deg2rad(0.5);%in algebraic cost or geometric radians, depending on flag_LineCostFunction %set the maximum error distance (if higher, then split again)
ThresholdMergingAngle = 0%2;  %if smaller, then the 2 normals will be merged
ThresholdLengthAfterMerging=local_length;

flag_ComputeOrRead_edge=0; % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
%%%%%%%%%%% end  - parameters and options %%%%%%%%%%%%%%%

% start - check inputs %
if length(Image)==0;  error('no image'); end
if length(Mask)==0;   error('no Mask'); end
if isa(Image,'uint8')==0; class(Image), error('wrong type'); end % we prefer uint8 because it greatly helps the memory

% end - check inputs %


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                               %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       START - EDGE STEP       %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                               %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% start  - step 0 read image %%%%%%%%%%%%%%%
%Image=Fn_Read_Image(image_filename,1);
if ndims(Image)==3
    Image=rgb2gray(Image);
end
%figure('Name','original image in gray color'); imshow(Image);
height=size(Image,1);
width=size(Image,2);
%%%%%%%%%%% end  - step 0 read image %%%%%%%%%%%%%%%

%%%%%%%%%%% start  - step 1 edge detection %%%%%%%%%%%%%%%
disp('start - edge')
if flag_ComputeOrRead_edge==0 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
    Image_edge = edge(Image,'canny');
end

flag_save_current_figure=1;
if flag_save_current_figure==1 && flag_ComputeOrRead_edge==0 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
    figure('Name','edge image'); imshow(Image_edge);
    imwrite(Image_edge,'results_0-edge.png');
end
disp('end - edge')
%%%%%%%%%%% end  - step 1 edge detection %%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                               %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       END - EDGE STEP         %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                               %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      START - CHAINING STEP        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('start MAKE EDGE LIST')

%%%%%%%%%%% start  - step 1-bis apply mask (optional) %%%%%%%%%%%%%%%
disp ('start - apply mask');
if flag_ComputeOrRead_edge==0 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
    %Mask=imread(strcat(my_dir,image_filename_mask));
    Image_edge=Fn_ApplyMask(Mask, Image_edge);
    figure('Name','edge image with mask'); imshow(Image_edge);
    clear Mask;
end
disp ('end - apply mask');
%%%%%%%%%%% end  - step 1-bis apply mask (optional) %%%%%%%%%%%%%%%


%%%%%%%%%%% start  - step 2 chain extraction and small chain removing %%%%%%%%%%%%%%%
disp ('start - chaining');
if flag_ComputeOrRead_edge==0 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
    [edgelist edgeim] = Fn_Edge_Link(Image_edge, min_length_edge_link);
    disp ('end - chaining');
    
    clear Image_edge;
    clear edgeim;
end
%%%%%%%%%%% end  - step 2 chain extraction and small chain removing %%%%%%%%%%%%%%%


%%%%%%%%%%% start  - display %%%%%%%%%%%%%%%
flag_save_current_figure=1;
if flag_save_current_figure==1
    disp('start - display')
    temp_image=Fn_Convert_rgb2gray_3dim(Image); % this image contains all the edge links
    size_points=1;
    for p = 1 : size(edgelist,2)
        my_color=round(rand(1,3)*255);
        temp_image=Fn_aux_draw_points(temp_image, edgelist{p}(:,[2 1]), size_points, my_color,0);
    end
    
    %figure('Name','edge linking'); imshow(temp_image);
    imwrite(temp_image,'results_1-chain.png');
    disp('end - display')
    clear temp_image;
end
%%%%%%%%%%% end  - display %%%%%%%%%%%%%%%


disp('end MAKE EDGE LIST')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       END - CHAINING STEP         %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%           start step 3 - SPLITTING STEP           %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('start SPLITTING STEP')
% initialize the structured list that will contain :the normals (in .vect) and the associated sphere points (in .points) and the associated 2D image points (in . coords for display)
Normal.vect =[];
Normal.points =[];
Normal.coords=[];


%for all the edge links
for p = 1 : size(edgelist,2)
    
    
    nb_points=size(edgelist{p},1);
    Edge_Link=zeros(nb_points,5);% format: x, y, xs, ys, zs
    
    Edge_Link(:,1:2)=edgelist{p}(:,[2 1]);% fomat: x, y
    Edge_Link(:,3:5)=Fn_Projection_OnSphere(Edge_Link(:,1:2));% format: xs, ys, zs
    
    
    Normal = Fn_Detection_OmniLine_Split(Edge_Link, 1, nb_points, Normal, ThresholdPixel, ThresholdError,flag_LineCostFunction);
    
    
    %     %%% START - WRITE IN A FILE for post processing
    %     my_filename_data=strcat(my_dir_saving,'my_data_nth-chain-',int2str(p),'.txt');
    %     my_fid = fopen(my_filename_data,'w');
    %      fprintf(my_fid,'%d \n',size(edgelist{p},1));
    %     for j = 1 : size(edgelist{p},1)
    %         fprintf(my_fid,'%d %d %f %f %f \n', Edge_Link(j).X, Edge_Link(j).Y, Edge_Link(j).Xs, Edge_Link(j).Ys, Edge_Link(j).Zs);
    %     end
    %     fclose(my_fid);
    %     %%% END - READ IN A FILE for post processing
    
    
end



save_Normal=Normal;% simply used for making code testing easier
disp('end SPLITTING STEP')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%        end step 3 -  SPLITTING STEP          %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                start  DISPLAY                      %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
flag_save_current_figure=1;
if flag_save_current_figure==1
    disp('start DISPLAY')
    %%%%%%%%%%% display all the conics on one image and the associated points%%%%%%%%%%%%
    Icol=Fn_Draw_Lines(Image, Normal.vect, Normal.coords, [],[], 1);
    
    
    figure('Name','detected line and associates pixels - before merging'); imshow(Icol);
    imwrite(Icol,'results_2-before-merging.png');
    clear Icol;
    disp('end DISPLAY')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                 end  DISPLAY                       %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                 START MERGING STEP                %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Fn_Detection_OmniLine_Merge;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%             END MERGING STEP                       %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                start  DISPLAY                      %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
flag_save_current_figure=1;
if flag_save_current_figure==1
    disp('start drawing')
    %%%%%%%%%%% display all the conics on one image and the associated points%%%%%%%%%%%%
    Icol=Fn_Draw_Lines(Image, Normal.vect, Normal.coords,[],[], 1);
    
    
    figure('Name','detected line and associates pixels - after merging'); imshow(Icol);
    imwrite(Icol,'results_3-after-merging.png');
    
    clear Icol;
    disp('end drawing')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                 end  DISPLAY                       %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% output
ListNormals=Normal.vect;
list_ClusteredImagePoints= Normal.coords;
list_ClusteredSphPoints= Normal.points;

