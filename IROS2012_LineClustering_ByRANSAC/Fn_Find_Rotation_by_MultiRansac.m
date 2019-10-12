function [MyAngles_ByMultiRansac, ListExtraInfo, ExtraOutput]...
    =Fn_Find_Rotation_by_MultiRansac(list_normals, ThreshInlier, Nb_VPs, flag_InlierDistanceDefinition)

% Goal: identify the inlier lines and estimate the rotation (and vanishing points) by a 3-line RANSAC. The N orthogonal VPs are estimated simultaneously
%
% Reference: "3-line RANSAC for Orthogonal Vanishing Point Detection", by Jean-Charles Bazin and Marc Pollefeys

% Version Arpil 2016 (from 04/04/2011)

% Inputs:
%   - list_normals: contains the normals of the detected lines in the equivalent sphere. Size is Nx3 (xyz) where N is the number of detected lines. list_normals(i,:) is the i-th normal (i.e. the i-th extracted line)
%   - ThreshInlier: the inlier threshold: it controls when a line is considered an inlier. Must be set in accordance with the distance definition set by flag_InlierDistanceDefinition
%   - flag_InlierDistanceDefinition: see definition in Fn_ComputeVPDist
%       - 0 or []: geodesic (angular) distance on the sphere (in radians)
%       - 1: distance in the image between the VP and the lines
%       - 2: distance in the image between the VP and the middle of the lines
%   - Nb_VPs: the number of vanishing points to detect (and cluster the lines with respect to). Generally, it should be set to 3
%

% Outputs:
%   - MyAngles_ByMultiRansac: 1x3 roll, pitch and yaw
%   - ListExtraInfo: structure
%       - dominant_directions: the list of VPs Kx3, where K=3  dominant_directions(i,:) is the ith VP
%       - List_ClusteredNormals{nth_direction,nth_normal} contains the 1x3 normals associated to each main VP
%       - List_ClusteredNormalsIndices{nth_direction} contains the indices of the normals associated to each main VP
%   - ExtraOutput: structure:
%           - BestSupportingIndices
%           - best_distances: nb of lines passing through VP1, VP2, VP3 and the total

%%%%% start - parameters %%%%%
%NbRansacIterations=50000; % hard coded version
my_prob=0.99; RatioOutlier=0.8; NbMinPoints=3; % computed version
NbRansacIterations=Fn_RANSAC_ComputeNbIterations(my_prob,RatioOutlier,NbMinPoints)
%%%%% end - parameters %%%%%

close all;

if ~(nargin==4 || nargin==0), nargin, error('wrong nb of inputs'); end
if nargout~=3; nargout, error('wrong nb of outputs'); end

ProximityThresh=deg2rad(3);

local_iter=0;
best_sum=0;
FlagSolutionFound=0;
NbLines=size(list_normals,1);
for kth_RansacIter=1:NbRansacIterations
    
    
    if mod(kth_RansacIter,50000)==0
        kth_RansacIter, NbRansacIterations
    end
    
    
    
    % randomly select 2 lines and compute VP_1
    MaxLocal=1000;
    iter_local=0;
    while 1
        iter_local=iter_local+1;
        if iter_local>MaxLocal; iter_local,MaxLocal, error('cannot synthesize'); end
        MySupportIndices_set1=round(Fn_RandomList_BoundedLimit(2, 1, NbLines));
        if MySupportIndices_set1(1)~=MySupportIndices_set1(2)
            break;
        end
    end
    % MySupportIndices_set1=[17 279];
    if abs(acos(dot(list_normals(MySupportIndices_set1(1),:),list_normals(MySupportIndices_set1(2),:))))<ProximityThresh
        continue;
    end
    VP1=cross(list_normals(MySupportIndices_set1(1),:),list_normals(MySupportIndices_set1(2),:));
    VP1=VP1/norm(VP1); %1x3
    
    % randomly select a third line and compute VP_2
    MaxLocal=1000;
    iter_local=0;
    while 1
        iter_local=iter_local+1;
        if iter_local>MaxLocal; iter_local,MaxLocal, error('cannot synthesize'); end
        MySupportIndex_set2=round(Fn_RandomList_BoundedLimit(1, 1, NbLines));
        if MySupportIndex_set2~=MySupportIndices_set1(1) && MySupportIndex_set2~=MySupportIndices_set1(2) % must be not a line selected for VP1
            break;
        end
    end
    VP2=cross(VP1,list_normals(MySupportIndex_set2,:));
    VP2=VP2/norm(VP2); %1x3
    
    if abs(acos(dot(VP1,list_normals(MySupportIndex_set2,:)))-pi/2)<ProximityThresh
        continue;
    end
    
    % compute VP_3
    VP3=cross(VP1,VP2);
    VP3=VP3/norm(VP3); %1x3
    
    % get the rotation angles (by orthonormalization)
    my_rotation_angles=Fn_Rotation_Between_2_Coord_Syst_angles([1 0 0; 0 1 0; 0 0 1],[VP1; VP2; VP3]);
    
    
    
    
    % apply algorithm
    [directions_temp1, temp2, temp3, distance_results_curr,temp4]=...
        Fn_GetLineClustering_GivenRotation(list_normals,my_rotation_angles,ThreshInlier, Nb_VPs,0,flag_InlierDistanceDefinition);
    
    
    if distance_results_curr(4)>=best_sum % this is the best result so far
        best_sum=distance_results_curr(4);
        best_rotation_angles=my_rotation_angles;
        BestSupportingIndices=[MySupportIndices_set1 MySupportIndex_set2];
        
        FlagSolutionFound=1;
    end
    
end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%     START - GET THE FINAL RESULTS    %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% we need:
%     - dominant_directions
%     - best_rotation_angles (already computed)
%     - best_distances (already computed)
%     - normals_dominant_directions

if FlagSolutionFound==1
    % compute the clustered normals
    MyAngles_ByMultiRansac=best_rotation_angles;
    [dominant_directions, List_ClusteredNormals, List_ClusteredNormalsIndices,best_distances,UnclusteredIndices_temp]=...
        Fn_GetLineClustering_GivenRotation(list_normals,MyAngles_ByMultiRansac,ThreshInlier, Nb_VPs,1,flag_InlierDistanceDefinition);
    
    ExtraOutput.BestSupportingIndices=BestSupportingIndices;
    ExtraOutput.best_distances=best_distances;

else
    % example of reasons:
    %   - no input lines (LengthThresh is too high)
    %   - "proximity constraint": see ProximityThresh
    
    
    MyAngles_ByMultiRansac=[NaN NaN NaN];
    dominant_directions=[NaN NaN NaN; NaN NaN NaN; NaN NaN NaN];
    List_ClusteredNormals=[];
    best_distances=[NaN NaN NaN NaN];
    List_ClusteredNormalsIndices=[];
    ExtraOutput=[];
   
end


ListExtraInfo.dominant_directions=dominant_directions;
ListExtraInfo.List_ClusteredNormals=List_ClusteredNormals;
ListExtraInfo.List_ClusteredNormalsIndices=List_ClusteredNormalsIndices;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%     END - GET THE FINAL RESULTS    %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


disp('finish - Fn_Find_Rotation_by_MultiRansac')


flag_display=0;
if flag_display==1
    Fn_DisplayClusteredGreatCirclesOnSphere([],List_ClusteredNormals,dominant_directions,' by MultiRansac')
end

