
% version 30/06/2010
% modifications: 01/11/2009
%   - explanations about the inputs
%   - format of ListPoints has changed
% modifications: 21/11/2009
%   - fast version using the temporary global image TempGlobalImage
% modifications: 30/06/2010
%   - extra inputs: color and line width
%   - error if not correct inputs

function Icol=Fn_Draw_Lines(Image, ListNormals, ListPoints, my_color, linewidth, FlagDrawPoints)

% inputs:
%   - Image: a color or gray image. will be converted to grayscale
%   - ListNormals: contains the normals of the detected lines in the equivalent sphere. Size is Nx3 (xyz) where N is the number of detected lines
%   - ListPoints: ListPoints{i} contains the 2D points (Nx2 for x,y coordinates) of the ith line. Contains the same number of lines as ListNormals
%   - my_color: the color of the points. 1x3. can be [] then red
%   - linewidth. can be [] then 0


% % check error: nb arguments
if nargin~=6;  nargin, error('wrong nb of inputs'); end
if nargout~=1;  nargout, error('wrong nb of outputs'); end

if length(my_color)==0; my_color=[255 0 0]; end
if length(linewidth)==0; linewidth=0; end


flag_FastGlobalDrawing=1; %0-traditional, 1-fast drawing using a global image


Fn_Check_Arguments(Image, ListNormals, ListPoints, FlagDrawPoints);

disp('start DISPLAY')

height=size(Image,1)
width=size(Image,2)

Icol=Fn_Convert_rgb2gray_3dim(Image);
clear Image;
% if ndims(Image)==3 % i.e. convert from color image to gray image
%     Image=rgb2gray(Image);
% end
% 
% % initialize the output picture
% % Icol=zeros(height, width, 3);
% % Icol(:,:,1)=Image;  Icol(:,:,2)=Image;  Icol(:,:,3)=Image;
% Icol=double(Image);  clear Image;
% Icol=repmat(Icol,[1 1 3]);


%figure; imshow(Icol);
%sdfsdf

% define the temporary global image TempGlobalImage
if flag_FastGlobalDrawing==1
    global TempGlobalImage;
    TempGlobalImage=Icol; % the rgb color image (mandatory?)
    %figure('Name','INITIAL TempGlobalImage in Fn_Draw_Lines'); imshow(TempGlobalImage/255);
    clear Icol; % for memory
end



%for each computed normal. in other words, for each detected part of line
for nth_normal = 1 : size(ListNormals,1)
    my_normal = ListNormals(nth_normal,:);
    if flag_FastGlobalDrawing==0
        [Array_ConicPts_temp,Array_GreatCirclePts_temp,Icol]=Fn_PlotConic(my_normal,Icol,my_color,linewidth,flag_FastGlobalDrawing);
        
    else
        Fn_PlotConic(my_normal,[],my_color,linewidth,flag_FastGlobalDrawing);
%        figure('Name','TempGlobalImage in Fn_Draw_Lines'); imshow(TempGlobalImage/255);

    end
end
   
   
% test
% if flag_FastGlobalDrawing==1
%     figure('Name','TempGlobalImage in Fn_Draw_Lines AFTER ALL NORMALS'); imshow(TempGlobalImage/255);
% end

if FlagDrawPoints==1
    size_point=1;%10;
    for nth_line = 1 : length(ListPoints) %JC size(ListPoints,1)
        my_color=round(rand(1,3)*255);
        coords_list=ListPoints{nth_line}; % list x-y
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

    clear TempGlobalImage; % for memory
end

disp('end DISPLAY')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Fn_Check_Arguments(Image, ListNormals, ListPoints, FlagDrawPoints)

% if ndims(Image)~=2
%     error('image is not in gray')
% end

if size(ListNormals,2)~=3
    size(ListNormals)
    error('wrong format for lines')
end

if ~(isa(ListPoints,'double') && (length(ListPoints)==0 || size(ListPoints,3)==2)) && ~(isa(ListPoints,'cell') && (length(ListPoints)==0 || size(ListPoints{1},2)==2)) % not an array, nor a cell
    size(ListPoints)
    class(ListPoints)
        size(ListPoints{1},2)
    error('wrong format for points')
end



