function [list_PointsOnTheLine,LineLength]=Fn_GenerateLinePointsFromNormal(my_line_normal_InImage,image_size, nb_PointsToDisplay,flag_UniformOrRandom)

% version: 25/10/2009
% update: 04/06/2012
%   - image_size: [height width] or list_XY_limits
% inputs:
%   - image_size:
%       - either [height width]
%       - or list_XY_limits=[Xmin_effective Ymin_effective; Xmax_effective Ymax_effective];
%   - flag_UniformOrRandom: 0-uniform sampling, 1-random sampling

% outputs:
%   - list_PointsOnTheLine: Nx2 [x y]
% modifications:
%   - 25/10/2009: example of call
% modifications:
%   - 04/06/2012: add flag_UniformOrRandom

% call example:
%     my_line_normal_InImage=[1 1 -80];
%     image_size=[200 300];
%     [list_PointsOnTheLine,LineLength]=Fn_GenerateLinePointsFromNormal(my_line_normal_InImage,image_size, 10);
%     figure; imshow(zeros(image_size(1), image_size(2))); hold on; plot(list_PointsOnTheLine(:,1),list_PointsOnTheLine(:,2),'.r'); hold off;

if nargin~=4, nargin, error('wrong nb of inputs'), end;
if nargout~=2, nargout, error('wrong nb of outputs'), end;

[list_CornerPoints,list_ValidPoints,nb_valid_points]=Fn_ComputeLineBoundariesInImage(my_line_normal_InImage,image_size);

if nb_valid_points~=2
        my_line_normal_InImage, my_line_normal_InImage(1)
        image_size
        list_CornerPoints
        list_ValidPoints
        nb_valid_points
    list_PointsOnTheLine=[];
    LineLength=[];
    %error('uncorrect nb of points') i.e. the line is not in the image
    %since it does not intersect the image border at two locations
        
    return;
end

list_PointsOnTheLine=zeros(nb_PointsToDisplay,2);
my_direction=list_ValidPoints(2,:)-list_ValidPoints(1,:);
my_UnitDirection=my_direction/norm(my_direction);
dist=norm(list_ValidPoints(2,:)-list_ValidPoints(1,:));
if flag_UniformOrRandom==0
    my_DistStep=dist/(nb_PointsToDisplay-1);
    for kth_point=1:nb_PointsToDisplay
        my_P=list_ValidPoints(1,:)+(kth_point-1)*my_DistStep*my_UnitDirection;
        list_PointsOnTheLine(kth_point,:)=my_P;
    end
    
else % i.e. flag_UniformOrRandom==1
    if length(image_size(:))==2
        dfgdf
        MinX=0;   MinY=0;
        MaxX=image_size(2);   MaxY=image_size(1);
    else
        MinX=image_size(1,1);   MinY=image_size(1,2);
        MaxX=image_size(2,1);   MaxY=image_size(2,2);
    end
    

    
    MAXiter=1000;
    my_iter=0;
    for kth_point=1:nb_PointsToDisplay
        while 1
            MyRandNumber=Fn_RandomList_BoundedLimit(1,0,1);
            my_P=list_ValidPoints(1,:)+MyRandNumber*my_direction;
            
            my_iter=my_iter+1;
            if MinX<my_P(1) && my_P(1)<MaxX & MinY<my_P(2) && my_P(2)<MaxY
                break; % success
            elseif my_iter==MAXiter
                error('cannot synthesized')
            else
                % not correct, so let's try again
            end
        end
        list_PointsOnTheLine(kth_point,:)=my_P;
    end
end


LineLength=dist;

flag_display=0;
if flag_display==1
    figure;
    plot(list_PointsOnTheLine(:,1),list_PointsOnTheLine(:,2),'.b');
    
    if length(image_size)==2
        MinX=0;   MinY=0;
        MaxX=image_size(2);   MaxY=image_size(1);
    else
        MinX=image_size(1,1);   MinY=image_size(1,2);
        MaxX=image_size(2,1);   MaxY=image_size(2,2);
    end
    
    xlim([MinX MaxX]); ylim([MinY MaxY]); axis equal;
    my_line_normal_InImage, input('sdfsdfsd')
end

