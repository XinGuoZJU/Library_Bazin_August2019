function Normal = Fn_Detection_OmniLine_Split(Edge_Link, starting_point, finishing_point, Normal,ThresholdPixel,ThresholdError,flag_LineCostFunction)



% version 15/11/2009
% modifications:
%   - improve the the splitting part to take care of special case (split at first or last point)
% modifications 21/10/2009
%   - the format of Normal has been changed from (nth_point+1,nth_line,:) towards (nth_line, nth_point+1,:)
% modifications 03/11/2010
%   - compute algebraic or geometric cost

% inputs:
%   - Edge_Link
%   - starting_point, finishing_point
%   - Normal
%   - ThresholdPixel
%   - ThresholdError
%   - flag_LineCostFunction; %0-algebraic cost, 1-geometric cost

if nargin~=7; nargin, error('wrong nb of inputs'); end

if (finishing_point - starting_point) > ThresholdPixel



    %compute the vector normal to the plane defined by the sphere center and the two points (starting_point and finishing_point): crossproduct( (sphere_center,finishing_point) ; (sphere_center,starting_point)  )
    normal= cross(Edge_Link(starting_point,3:5), Edge_Link(finishing_point,3:5));
    normal=normal'/norm(normal);% normalisation


    % compute the max distance from each point of Edge_Link to the plane
    if flag_LineCostFunction==0 %0-algebraic cost, 1-geometric cost
        [my_cost my_index]=max(abs(Edge_Link(starting_point:finishing_point,3:5)*normal));

    elseif flag_LineCostFunction==1
        [my_cost my_index]=max(abs( acos(Edge_Link(starting_point:finishing_point,3:5)*normal) - pi/2));
    else
        error('wrong case');
    end

    flag_LineCostFunction, ThresholdError
    % if the max distance is low then this can be a part of a line. Thus add its normal to the list of normals, and add the associated points
    if my_cost < ThresholdError

        if normal(3)<0
            normal=-normal;
        end


        nth_vect=size(Normal.vect,1)+1;
        nb_points=finishing_point-starting_point+1;
        Normal.vect(nth_vect,1:3)=normal;
        %JC Normal.points(nth_vect,1:nb_points+1,1:3)=[[nb_points 0 0]; Edge_Link(starting_point:finishing_point,3:5)]; % the image points
        %JC Normal.coords(nth_vect,1:nb_points+1,1:2)=[[nb_points 0]; Edge_Link(starting_point:finishing_point,1:2)]; % the spherical points
        Normal.points{nth_vect}=Edge_Link(starting_point:finishing_point,3:5); % the image points
        Normal.coords{nth_vect}=Edge_Link(starting_point:finishing_point,1:2); % the spherical points


        % otherwise split
    else

        if my_index==1%the first point % usually impossible since the model (i.e. the line passes through the first and last points)
            Normal = Fn_Detection_OmniLine_Split(Edge_Link, starting_point+1, finishing_point, Normal,ThresholdPixel,ThresholdError,flag_LineCostFunction);
        elseif my_index==(finishing_point-starting_point+1)%the last point % usually impossible since the model (i.e. the line passes through the first and last points)
            Normal = Fn_Detection_OmniLine_Split(Edge_Link, starting_point, finishing_point-1, Normal,ThresholdPixel,ThresholdError,flag_LineCostFunction);
            % Fn_Detection_OmniLine_Split_Original
        else
            % split the first part
            Normal = Fn_Detection_OmniLine_Split(Edge_Link, starting_point, starting_point+my_index-1, Normal,ThresholdPixel,ThresholdError,flag_LineCostFunction);
            % split the second part
            Normal = Fn_Detection_OmniLine_Split(Edge_Link, starting_point+my_index,finishing_point, Normal,ThresholdPixel,ThresholdError,flag_LineCostFunction);
        end


    end
end
