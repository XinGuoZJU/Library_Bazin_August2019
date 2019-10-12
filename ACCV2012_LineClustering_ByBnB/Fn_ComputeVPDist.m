function ListCost=Fn_ComputeVPDist(list_normals,my_direction,flag_cost,K)

% inputs:
%   - list_normals: Nx3 must be unit norm because of acos and dot product
%   - my_direction: 3x1 or 3x1  must be unit norm because of acos and dot product
%       - can also be an interval
%   - flag_cost: can be []
%       - 0 or []: geodesic (angular) distance on the sphere (in radians)
%       - 1: distance in the image between the VP and the lines
%       - 2: distance in the image between the VP and the middle of the lines
%   - K: the calibration matrix. can be []. used only for flag_cost==1 and flag_cost==2
% output:
%   - ListCost: costs in radians. Nx1

if nargin~=4, error('wrong nb of inputs'), end

if ~(size(list_normals,2)==3 && length(my_direction)==3)
    size(list_normals), my_direction, error('wrong size')
end

if (size(my_direction,1)==1 && size(my_direction,2)==3), my_direction=my_direction'; end % from horizontal to vertical

% simply check the first line and the direction have a unit norm. could check all the lines but is computationnaly expensive
if abs(sum(list_normals(1,:).^2)-1)>10^-5, list_normals(1,:), error('not unit norm'), end
if abs(sum(my_direction.^2)-1)>10^-5, my_direction, error('not unit norm'), end

if flag_cost==0 || length(flag_cost)==0
    ListCost=abs(acos(list_normals*my_direction)-pi/2);
elseif flag_cost==1
    % compute the normals in the image
    %global K; % load; defined in main file
    if length(K)==0, error('undefined'), end
    %list_normals_InImage=(K*list_normals')';
     list_normals_InImage=Fn_Projection_OnImageFromSphere_LinePerspec(list_normals,K);

    
    % get the VP in the image
    MyVP_InImage=(K*my_direction);
    MyVP_InImage=MyVP_InImage(1:2)/MyVP_InImage(3); % vertical
    
    % compute the distance between the lines and the VP in the image
    NbLines=size(list_normals_InImage,1);
    ListCost=zeros(NbLines,1);
    for kth_line=1:NbLines
        MyDist=Fn_ComputeDistance_2DpointTo2Dline(MyVP_InImage',list_normals_InImage(kth_line,:));
        ListCost(kth_line)=MyDist;
    end
elseif flag_cost==2
    error('to do')
else
    error('to do')
end

%list_normals, my_direction, ListCost