function [list_CornerPoints,list_ValidPoints,nb_valid_points]=Fn_ComputeLineBoundariesInImage(my_line_normal,image_size)

% version: 24/09/2009
% version: 04/06/2012
%   - image_size or list_XY_limits
% usually run within Fn_GenerateLinePointsFromNormal
% inputs:
%   - my_line_normal: in the format ax+by+c=0
%   - image_size:
%       - either [height width]
%       - or list_XY_limits=[Xmin_effective Ymin_effective; Xmax_effective Ymax_effective];

% outputs:
%   - list_CornerPoints
%   - list_ValidPoints
%   - nb_valid_points

if length(image_size(:))==2
    
    MinX=0;   MinY=0;
    MaxX=image_size(2);   MaxY=image_size(1);
else
    MinX=image_size(1,1);   MinY=image_size(1,2);
    MaxX=image_size(2,1);   MaxY=image_size(2,2);
end
%image_size, MinX,  MinY, MaxX, MaxY

list_CornerPoints=zeros(4,2);
list_ValidPoints=zeros(2,2); % may be too big
nb_valid_points=0;

%if abs(my_line_normal(1))<10^-7, error('probably unstable') , end
MyEps=10^-7;

if abs(my_line_normal(1))>MyEps
    % for top
    x=(-my_line_normal(2)*MinY-my_line_normal(3))/my_line_normal(1);
    %[x MinY]
    if MinX<x & x<MaxX
        list_CornerPoints(1,:)=[x MinY];
        nb_valid_points=nb_valid_points+1;
        list_ValidPoints(nb_valid_points,:)=[x MinY];
    end
    % for bottom
    x=(-my_line_normal(2)*MaxY-my_line_normal(3))/my_line_normal(1);
    %[x MaxY]
    if MinX<x & x<MaxX
        list_CornerPoints(2,:)=[x MaxY];
        nb_valid_points=nb_valid_points+1;
        list_ValidPoints(nb_valid_points,:)=[x MaxY];
    end
end
if abs(my_line_normal(2))>MyEps
    % for left
    y=(-my_line_normal(1)*MinX-my_line_normal(3))/my_line_normal(2);
    %[y MinY]
    if MinY<y & y<MaxY
        list_CornerPoints(3,:)=[MinX y];
        nb_valid_points=nb_valid_points+1;
        list_ValidPoints(nb_valid_points,:)=[MinX y];
    end
    % for right
    y=(-my_line_normal(1)*MaxX-my_line_normal(3))/my_line_normal(2);
    %[y MaxY]
    if MinY<y & y<MaxY
        list_CornerPoints(4,:)=[MaxX y];
        nb_valid_points=nb_valid_points+1;
        list_ValidPoints(nb_valid_points,:)=[MaxX y];
    end
end


if nb_valid_points==0
    list_ValidPoints=[];
else
    list_ValidPoints=unique(list_ValidPoints(1:nb_valid_points,:),'rows');
    nb_valid_points=size(list_ValidPoints,1);
    % list_ValidPoints=list_ValidPoints(1:nb_valid_points,:);
end


