
% version 20/11/2009


function Icol=Fn_Draw_Lines_Perspec(Image, list_normals, list_points, flag_draw_points)

% inputs:
%   - Image: a gray image HxW
%   - list_normals: Nx3
%   - list_points{nth_vect}: list of [x y], the image points

flag_FastGlobalDrawing=0;


Fn_Check_Arguments(Image, list_normals, list_points, flag_draw_points);

disp('start DISPLAY')

height=size(Image,1);
width=size(Image,2);

Icol=zeros(height, width, 3);

Icol(:,:,1)=Image;
Icol(:,:,2)=Image;
Icol(:,:,3)=Image;


% define the temporary global image TempGlobalImage
if flag_FastGlobalDrawing==1
    global TempGlobalImage;
    TempGlobalImage=Icol;
    %figure('Name','INITIAL TempGlobalImage in Fn_Draw_Lines'); imshow(TempGlobalImage/255);
end



%for each computed normal. in other words, for each detected part of line
my_color=[255 0 0];
my_line_width=0;
for nth_normal = 1 : size(list_normals,1)
    my_normal = list_normals(nth_normal,:);
    if flag_FastGlobalDrawing==0
        % OMNI: [Array_ConicPts_temp,Array_GreatCirclePts_temp,Icol]=Fn_Plot_conic_curve(my_normal,Icol,my_color,my_line_width,flag_FastGlobalDrawing);
        [list_PointsOnTheLine,LineLength]=Fn_GenerateLinePointsFromNormal(my_normal,[height width], 1000,0);
    %list_PointsOnTheLine, input('sdfsdf')
    Icol=Fn_aux_draw_points(Icol, list_PointsOnTheLine, 1, my_color, 0);
    
    else
        error('not done yet in perspec')
        Fn_Plot_conic_curve(my_normal,[],my_color,my_line_width,flag_FastGlobalDrawing);
%        figure('Name','TempGlobalImage in Fn_Draw_Lines'); imshow(TempGlobalImage/255);

    end
end
   
% test
% if flag_FastGlobalDrawing==1
%     figure('Name','TempGlobalImage in Fn_Draw_Lines AFTER ALL NORMALS'); imshow(TempGlobalImage/255);
% end

if flag_draw_points==1
    size_point=3;
    for nth_line = 1 : length(list_points)
        my_color=round(rand(1,3)*255);
        coords_list=list_points{nth_line};% list x-y
         if flag_FastGlobalDrawing==0
        Icol=Fn_aux_draw_points(Icol, coords_list, size_point, my_color, flag_FastGlobalDrawing);
         else
            Fn_aux_draw_points([], coords_list, size_point, my_color,flag_FastGlobalDrawing);
         end
    end
end

if flag_FastGlobalDrawing==1
    Icol=TempGlobalImage;
    % figure('Name','TempGlobalImage in Fn_Draw_Lines AFTER ALL NORMALS AND POINTS'); imshow(TempGlobalImage/255);

    clear TempGlobalImage;
end

disp('end DISPLAY')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Fn_Check_Arguments(Image, list_normals, list_points, flag_draw_points)

if ndims(Image)~=2
    error('image is not in gray')
end

if size(list_normals,2)~=3
    size(list_normals)
    error('wrong format for lines')
end

% if ~(length(list_points)==0 || size(list_points,3)==2) % must be Nx2
%     size(list_points)
%     error('wrong format for points')
% end



