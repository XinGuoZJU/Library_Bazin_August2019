function Normal = Fn_Detection_PerspecLine_Split(Edge_Link, starting_point, finishing_point, Normal,ThresholdPixel,ThresholdError)

% version 28/12/2009
% input:
% output:
%   - Normal
%       - Normal.vect(nth_vect,1:3)
%       - Normal.coords{nth_vect}: list of [x y], the image points

if nargin~=6; nargin, error('wrong nb of inputs'); end
if nargout~=1; nargin, error('wrong nb of outputs'); end

if (finishing_point - starting_point) > ThresholdPixel


    %compute the direction of the line (directly the start and end points, i.e. no explicit fitting)
    point_start=Edge_Link(starting_point,1:2);
    point_end=Edge_Link(finishing_point,1:2);
    my_normal=cross([point_start 1], [point_end 1]);% like in Hartley's book: the line passing through the start and end points % point_start-point_end;
    my_normal=my_normal/norm(my_normal);% normalisation
    if my_normal(3)<0, my_normal=-my_normal; end % for merging?
    
    % compute the distance from each point of Edge_Link to the line
    % if you change the cost function, please change also the splitting part at the end of the function
    % cf http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    nb_of_points=finishing_point - starting_point + 1;
    distance_list=zeros(nb_of_points, 1);
    index=0;
    for k=starting_point:finishing_point
        index=index+1;
        my_point=Edge_Link(k,:);
        %distance_list(index)=norm( (point_end(1)-point_start(1))*(point_start(2)-my_point(2)) - (point_start(1)-my_point(1))*(point_end(2)-point_start(2)) );
        %distance_list(index)=distance_list(index) / sqrt( (point_end(1)-point_start(1))^2 + (point_end(2)-point_start(2))^2); 
        %     distance_list(index)=norm(det([point_2-point_1 point_1-point_0])) / point_2-point_1 );
        %distance_list(index)=Fn_ComputeDistance_2DpointTo2Dline_FromLineEndPoints(point_start,point_end,my_point);
        distance_list(index)=Fn_ComputeDistance_2DpointTo2Dline_FromNormal(my_normal,my_point);
    end
    [my_cost my_index]=max(distance_list);
    %distance_list, my_cost, ThresholdError, size(distance_list), finishing_point-starting_point


    % if the max distance is low then this can be a part of a line. Thus add its normal to the list of normals, and add the associated points
    if my_cost < ThresholdError
        

       nth_vect=size(Normal.vect,1)+1;
       nb_points=finishing_point-starting_point+1;
       Normal.vect(nth_vect,1:3)=my_normal;
       Normal.coords{nth_vect}=Edge_Link(starting_point:finishing_point,:); % the image points

       
    % otherwise split
    else

        %find the point of Edge_Link which is the furthest from the plane and then run recursively on the split Edge_Link
        % if you change the splitting part, please change also the cost function at the middle of the function
        if my_index==1%the first point
            Normal = Fn_Detection_PerspecLine_Split(Edge_Link, starting_point+1, finishing_point, Normal,ThresholdPixel,ThresholdError);
        elseif my_index==(finishing_point-starting_point+1)%the last point
            Normal = Fn_Detection_PerspecLine_Split(Edge_Link, starting_point, finishing_point-1, Normal,ThresholdPixel,ThresholdError);
        else
        Normal = Fn_Detection_PerspecLine_Split(Edge_Link, starting_point, starting_point+my_index-1, Normal,ThresholdPixel,ThresholdError);
        Normal = Fn_Detection_PerspecLine_Split(Edge_Link, starting_point+my_index,finishing_point, Normal,ThresholdPixel,ThresholdError);
        end


    end
end
