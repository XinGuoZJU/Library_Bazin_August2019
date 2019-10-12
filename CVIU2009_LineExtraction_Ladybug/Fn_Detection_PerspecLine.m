function [list_normals, list_ClusteredImagePoints]=Fn_Detection_PerspecLine(Image,Params)

%
% version 26/12/2009
% inputs:
%   - Image: RGB (HxWx3) or gray scale (HxW) image

% output:
%   - list_normals: Nx3 [my_normal 0];
%   - list_ClusteredImagePoints{nth_vect}: list of [x y], the image points TO CHECK

disp('start Fn_Detection_PerspecLine')

close all;




%%%%%%%%%%% start  - parameters and options %%%%%%%%%%%%%%%
%Fn_Parameters_and_Initialization_ForLineExtraction;
% set the parameters here:
if exist('Params')==0 || length(Params)==0
flag_LineCostFunction=1; %0-algebraic cost, 1-geometric cost    

min_length_edge_link = 70;          % minimum edge length of interest
ThresholdPixel = 70;                % minimum length of an edge that can be splitted
ThresholdError = 1;              %set the maximum error distance (if higher, then split again) in PIXELS for perspective images
ThresholdMergingAngle = 0.0;          %if smaller, then the 2 normals will be merged
ThresholdLengthAfterMerging = 70;
else
    flag_LineCostFunction=Params.flag_LineCostFunction;

min_length_edge_link = Params.min_length_edge_link;
ThresholdPixel = Params.ThresholdPixel;
ThresholdError = Params.ThresholdError;
ThresholdMergingAngle = Params.ThresholdMergingAngle;
ThresholdLengthAfterMerging = Params.ThresholdLengthAfterMerging;
end
%%%%%%%%%%% end  - parameters and options %%%%%%%%%%%%%%%


flag_ComputeOrRead_edge=0;

% start - check inputs %
if length(Image)==0
    error('no image');
end
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
figure('Name','original image in gray color'); imshow(Image);
height=size(Image,1);
width=size(Image,2);
%%%%%%%%%%% end  - step 0 read image %%%%%%%%%%%%%%%

%%%%%%%%%%% start  - step 1 edge detection %%%%%%%%%%%%%%%
disp('start - edge')
if flag_ComputeOrRead_edge==0 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
    Image_edge = edge(Image,'canny');
end

flag_save_current_figure=0;
if flag_save_current_figure==1 && flag_ComputeOrRead_edge==0 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
    figure('Name','edge image'); imshow(Image_edge);
    imwrite(Image_edge,strcat('result_0-edge','.png'));
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
    if flag_ComputeOrRead_edge==0 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
        disp('start - display')
        temp_image=zeros(height, width, 3); % this image contains all the edge links
        temp_image(:,:,1)=Image;   temp_image(:,:,2)=Image;    temp_image(:,:,3)=Image;
        size_points=1;
        for p = 1 : size(edgelist,2)
            my_color=round(rand(1,3)*255);
            temp_image=Fn_aux_draw_points(temp_image, edgelist{p}(:,[2 1]), size_points, my_color,0);
        end
        
        figure('Name','edge linking'); imshow(temp_image/255);
        imwrite(temp_image/255,strcat('result_1-chain','.png'));
        disp('end - display')
        clear temp_image;
    end
end
%%%%%%%%%%% end  - display %%%%%%%%%%%%%%%



%best_index=Fn_aux_retrieve_line(edgelist)

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
% initialize the structured list that will contain :the normals (in .vect) and the associated sphere points (in .points) and the associated 2D image points (in . coords for display)
Normal.vect =[];
Normal.coords=[];


%for all the edge links
if flag_ComputeOrRead_edge==0 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
    for p = 1 : size(edgelist,2)
        
        
        nb_points=size(edgelist{p},1);
        Edge_Link=zeros(nb_points,2);% format: x, y
        
        Edge_Link(:,1:2)=edgelist{p}(:,[2 1]);% fomat: x, y
        
        
        Normal = Fn_Detection_PerspecLine_Split(Edge_Link, 1, nb_points, Normal, ThresholdPixel, ThresholdError);
        
        
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
elseif flag_ComputeOrRead_edge==1 % 0-compute (apply) edge detection, 1-direclty get the list of points from a file (for synthesized data)
    
    % IT IS A MODIFIED VERSION FROM THE OMNI VERSION!!!!
    
    %    my_color_edge=1;
    %    [rows cols]=find(Image==1);
    %    Edge_Link(:,1:2)=[cols rows]; % x y
    %   Normal = Fn_Detection_PerspecLine_Split(Edge_Link, 1, size(Edge_Link,1), Normal, ThresholdPixel, ThresholdError);
    
    [edgelist edgeim] = Fn_Edge_Link(Image, min_length_edge_link);
    
    %%%%%%%%%%%%%% START _ TEMP %%%%%%%%%%%%%%%%%
    disp('start - display')
    temp_image=zeros(height, width, 3); % this image contains all the edge links
    temp_image(:,:,1)=Image;   temp_image(:,:,2)=Image;    temp_image(:,:,3)=Image;
    size_points=1;
    for p = 1 : size(edgelist,2)
        my_color=round(rand(1,3)*255);
        temp_image=Fn_aux_draw_points(temp_image, edgelist{p}(:,[2 1]), size_points, my_color,0);
    end
    figure('Name','edge linking TEMP'); imshow(temp_image/255);
    %%%%%%%%%%%%%% END _ TEMP %%%%%%%%%%%%%%%%%
    
    for p = 1 : size(edgelist,2)
        nb_points=size(edgelist{p},1);
        Edge_Link=zeros(nb_points,2);% format: x, y
        Edge_Link(:,1:2)=edgelist{p}(:,[2 1]);% fomat: x, y
        Normal = Fn_Detection_PerspecLine_Split(Edge_Link, 1, nb_points, Normal, ThresholdPixel, ThresholdError);
    end
    
else
    error('wrong case')
end

save_Normal=Normal;% simply used for making code testing easier
disp('end SPLITTING STEP')
%Normal, Normal.coords, input('sdfsdf')
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
    Icol=Fn_Draw_Lines_Perspec(Image, Normal.vect, Normal.coords, 0);
    
    
    figure('Name','detected line and associates pixels - before merging'); imshow(uint8(Icol));
    imwrite(Icol,'result_2-beforemerging.png')
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
    Icol=Fn_Draw_Lines_Perspec(Image, Normal.vect, Normal.coords, 1);
    
    figure('Name','detected line and associates pixels - after merging'); imshow(uint8(Icol));
    imwrite(Icol,'result_3-aftermerging.png')
    
    clear Icol;
    disp('end drawing')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%                 end  DISPLAY                       %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%% start - write in file for post-processing and test %%%%%%%%%%%%%%%
flag_WriteInFile=0;
if flag_WriteInFile==1
    % FOR POST PROCESSING, save the Normal matrix in txt file
    
    fid = fopen(strcat('file_normal-extraction.txt'),'w');
    fprintf(fid,'%d \n', size(Normal.vect,1)); % i.e. the number of normals
    for i = 1 : size(Normal.vect,1)
        my_normal = Normal.vect(i,:);
        fprintf(fid,'%g %g %g \n', my_normal(1),my_normal(2),my_normal(3));
    end
    fclose(fid);
    
    % % FOR POST PROCESSING, save the Normal matrix in matlab file.
    % % It is also used in Fn_Draw_Lines to draw the points
    % my_string1=Fn_aux_create_string_from_integer(nth_frame,4);
    % save(strcat(my_dir_saving,'file_line-info_nth-',my_string1,'.mat'),'Normal');
end
%%%%%%%%%%%%%%%% end - write in file for post-processing and test %%%%%%%%%%%%%%%



% output
list_normals=Normal.vect;
list_ClusteredImagePoints= Normal.coords;


