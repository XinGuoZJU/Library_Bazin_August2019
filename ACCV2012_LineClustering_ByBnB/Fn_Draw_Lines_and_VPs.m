function Icol=Fn_Draw_Lines_and_VPs(Image, list_normals_clustered, list_points, list_ClusteredPoints, list_VP, extra_flags_draw)


% version 21/11/2009

% modifications 25/11/2008
%   - function style
%   - Hc as global

% modifications 27/08/2008:
%   - color for VPs
%   - points associated to lines
% it needs:
%   - my_dir, my_d, nth_frame
%   - vanishing_directions_normals_list
%   - vanishing_directions_points_list if we want to display the points
%
% modifications 01/11/2009
%   - list_points is NxMx2: list_points(i,j+1,:) is the jth point of the ith line
%   - list_ClusteredPoints is KxNxMx2 list_ClusteredPoints(k,j+1,i+1,:) is the ith point of the jth line of the kth VP
%
% modifications 06/11/2009
%   - add flags for drawing conics and lines
%   - explanations about inputs
%
% modifications: 21/11/2009
%   - fast version using the temporary global image TempGlobalImage

% inputs:
%   - Image: rgb or gray image (it will be converted in gray scale image)
%   - list_normals_clustered. access by list_normals_clustered{nth_VP,nth_line}
%   - list_points
%   - list_ClusteredPoints: list_ClusteredPoints(kth_VP).lines(kth_line).points as size Nx2
%   - list_VP
%   - extra_flags_draw: [flag_draw_VPs flag_draw_conics flag_draw_points]
%       - the flags can have the values (0,1,2)
%               - 0: dont draw
%               - 1: draw only the points (by Fn_aux_draw_points)
%               - 2:draw the entire line (by '.-')
%       - flag_draw_VPs
%       - flag_draw_conics; % if we draw the conics and the points with the same color, then we cannot distinguish them
%       - flag_draw_points

disp('start Fn_Draw_Lines_and_VPs')

if nargin~=6, error('wrong inputs'); end
if length(extra_flags_draw)~=3. , error('wrong inputs'); end

flag_FastGlobalDrawing=1;

flag_draw_VPs=extra_flags_draw(1);
flag_draw_conics=extra_flags_draw(2);
flag_draw_points=extra_flags_draw(3);

color_list=[255 0 0; 0 255 0; 0 255 255]; %red, green, blue (cyan clear)
%color_list_VP=[255 0 0; 255 0 255; 0 255 0; 255 255 0; 0 0 255; 0 255 255];% red, pink, green, yellow, blue, cyan
color_list_VP=[255 255 0; 255 255 0; 255 255 0; 255 255 0; 255 255 0; 255 255 0];
% figure; hold on;
% for k=1:6
%     plot(k,1,'.','Color',color_list_VP(k,:)/255,'MarkerSize',30)
% end
% hold off
height=size(Image,1);
width=size(Image,2);

%Icol=Image; % keep the color image
Icol=Fn_Convert_rgb2gray_3dim(Image);
clear Image;

% define the temporary global image TempGlobalImage
if flag_FastGlobalDrawing==1
    global TempGlobalImage;
    TempGlobalImage=Icol;
    clear Icol;
    %figure('Name','INITIAL TempGlobalImage in Fn_Draw_Lines_and_VPs'); imshow(TempGlobalImage/255);
end

%%%%%%%%%%% START - display all the conics belonging to the 3 detected directions %%%%%%%%%%%%
if flag_draw_conics==1
    my_line_width=0;%1;
    for nth_VP=1:size(list_normals_clustered,1) % the N dominant directions
        %for each computed normal. in other words, for each detected part of line
        nb_associated_normals= size(list_normals_clustered,2); % TO IMPROVE: NOT THE TRUE SIZE
        for nth_line=1:nb_associated_normals
            %OLD my_normal =  squeeze(list_normals_clustered(nth_VP,nth_line,:))';
            my_normal =  list_normals_clustered{nth_VP,nth_line};
            if length(my_normal)==0; break; end % i.e. end of the real normals
            my_color=color_list(nth_VP,:);

            %project the great circle on the image. in other words, project each detected part of line
            if flag_FastGlobalDrawing==0
                [ConicPts_temp,GreatCirclePts_temp,Icol]=Fn_PlotConic(my_normal,Icol,my_color,my_line_width,flag_FastGlobalDrawing);
            else
                Fn_PlotConic(my_normal,[],my_color,my_line_width,flag_FastGlobalDrawing);
            end

            %figure('Name',strcat('line nth-',int2str(nth_line-1))); imshow(Icol/255);
        end
    end
end
%%%%%%%%%%%% END - display  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% START - display the points associated to the lines %%%%%%%%%%%%
if flag_draw_points==1
    size_point=2;
    my_color=[255 0 255];
    for nth_line=1:size(list_points,1)% nth line

        %OLD nb_points=list_points(nth_line,1,1);
        %OLD my_points_image=squeeze(list_points(nth_line,1+1:1+nb_points,:)); % image coordinates
        my_points_image=list_points{nth_line}; % image coordinates
        if flag_FastGlobalDrawing==0
            Icol=Fn_aux_draw_points(Icol, my_points_image, size_point, my_color, flag_FastGlobalDrawing);
           
        else
            Fn_aux_draw_points([], my_points_image, size_point, my_color, flag_FastGlobalDrawing);
        end
    end
end
%%%%%%%%%%%% END - display  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% START - display the CLUSTERED points associated to the lines and VPs %%%%%%%%%%%%
if flag_draw_points==1 || flag_draw_points==2
    size_point=3;
    for nth_VP=1:length(list_ClusteredPoints) % nth VP
        %JC nb_lines=list_ClusteredPoints(nth_VP,1,1,1);
        nb_lines=length(list_ClusteredPoints(nth_VP).lines) % NOT SURE
        my_color=color_list(nth_VP,:); % [255 0 255]
        for nth_line=1:nb_lines % nth line

            %JC nb_points=list_ClusteredPoints(nth_VP,nth_line+1,1,1); % the nb of points of the current line of the current VP
            %JC my_points_image=squeeze(list_ClusteredPoints(nth_VP,nth_line+1,1+1:1+nb_points,:)); % image coordinates
            my_points_image=list_ClusteredPoints(nth_VP).lines(nth_line).points; % image coordinates
            if flag_draw_points==1
                if flag_FastGlobalDrawing==0
                    Icol=Fn_aux_draw_points(Icol, my_points_image, size_point, my_color, flag_FastGlobalDrawing);
                else
                    Fn_aux_draw_points([], my_points_image, size_point, my_color, flag_FastGlobalDrawing);
                end
            elseif flag_draw_points==2
                nb_initial_points=size(my_points_image,1);%usually there are only 2 points
                if flag_FastGlobalDrawing==0
                [cx,cy,c] = improfile(Icol,my_points_image([1 nb_initial_points],1), my_points_image([1 nb_initial_points],2)); % x endpoints and yendpoints
                else
                    [cx,cy,c] = improfile(TempGlobalImage,my_points_image([1 nb_initial_points],1), my_points_image([1 nb_initial_points],2)); % x endpoints and yendpoints
                end
                my_points_image_new=round([cx cy]);
                if flag_FastGlobalDrawing==0
                    Icol=Fn_aux_draw_points(Icol, my_points_image_new, size_point, my_color, flag_FastGlobalDrawing);
                else
                    Fn_aux_draw_points([], my_points_image_new, size_point, my_color, flag_FastGlobalDrawing);
                end
            end

        end
    end
end
%%%%%%%%%%%% END - display  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%% START - display the intersection point %%%%%%%%%%%%
if flag_draw_VPs==1
    size_point=8;
    for nth_VP=1:size(list_VP,1) % the N dominant directions
        my_direction=list_VP(nth_VP,1:3);

        % let's back-project the vanishing points on the image, in both direction (i.e. +VP and -VP)
        % draw one direction
        my_image_point=floor(Fn_Projection_OnImageFromSphere(my_direction));
        my_color=color_list_VP(2*nth_VP-1,:);
        if flag_FastGlobalDrawing==0
            Icol=Fn_aux_draw_points(Icol, my_image_point, size_point, my_color,flag_FastGlobalDrawing);
        else
            Fn_aux_draw_points([], my_image_point, size_point, my_color,flag_FastGlobalDrawing);
        end
        % draw the other direction
        my_image_point=floor(Fn_Projection_OnImageFromSphere(-my_direction));
        my_color=[255 255 0];
        if flag_FastGlobalDrawing==0
            Icol=Fn_aux_draw_points(Icol, my_image_point, size_point, my_color,flag_FastGlobalDrawing);
        else
            Fn_aux_draw_points([], my_image_point, size_point, my_color,flag_FastGlobalDrawing);
        end
    end
end
%%%%%%%%%%%% END - display  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_FastGlobalDrawing==1
    Icol=TempGlobalImage;
    % figure('Name','TempGlobalImage in Fn_Draw_Lines_and_VPs AFTER ALL'); imshow(TempGlobalImage/255);

    clear TempGlobalImage;
end


disp('end Fn_Draw_Lines_and_VPs')

% %%%%%%%%%%% START - display the points associated to the lines %%%%%%%%%%%%
% size_point=1;
% for nth_dir=1:3% nth direction
%     nb_lines=vanishing_directions_points_list(nth_dir,1,1,1);%nb of lines
%     for nth_line = 1 : nb_lines% nth line
%         nb_of_coords=vanishing_directions_points_list(nth_dir,1+nth_line,1,1);
%         points_list=squeeze(vanishing_directions_points_list(nth_dir,1+nth_line,1+1:nb_of_coords+1,:));% list x-y
%         Icol=Fn_aux_draw_points(Icol, points_list, size_point, [0 255 255]);
%     end
% end
% %%%%%%%%%%%% END - display  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%
% figure; imshow(Icol/255);
% my_string1=Fn_aux_create_string_from_integer(nth_frame,4);
% imwrite(Icol/255,strcat(my_dir_saving,'final_directions_angle-',int2str(normal_similarity_angle_threshold),'_all_frame-',my_string1,'.jpg'));

