function [dominant_directions, List_ClusteredNormals, List_ClusteredNormalsIndices, ListNbClusteredIndices,UnclusteredIndices]=...
    Fn_GetLineClustering_GivenRotation(list_normals,MyRotAnglesOrRotMatrix,ThreshInlier, Nb_VPs, Flag_ComputeOutputs,flag_InlierDistanceDefinition)

% version: 24/04/2011

% inputs:
%   - list_normals: Nx3: list_normals(k,:) is the k-th normal
%   - MyRotAnglesOrRotMatrix
%       - either 1x3 in degrees for rotation angles
%       - or 3x3 in degrees for rotation angles
%   - ThreshInlier
%   - Nb_VPs
%   - Flag_ComputeOutputs: 0 or 1
%       - if 0: returns only ListNbClusteredIndices
%   - flag_InlierDistanceDefinition: cf Fn_ComputeVPDist

% outputs:
%   - dominant_directions: dominant_directions(i,:) contains the i-th VP: Nb_VPsx3
%   - List_ClusteredNormals: List_ClusteredNormals{nth_direction,nth_normal} contains the 1x3 normal
%   - List_ClusteredNormalsIndices: List_ClusteredNormalsIndices{i} contains the list of the indices of the ith VP
%   - ListNbClusteredIndices: the nb of lines clustered in the 1st, 2nd, 3rd VPs and the total

if nargin~=6; nargin, error('wrong nb of inputs'); end
if nargout~=5; nargout, error('wrong nb of outputs'); end

if length(MyRotAnglesOrRotMatrix(:))==3 % input is rotation angles
    MyRotationAngles=MyRotAnglesOrRotMatrix;
elseif length(MyRotAnglesOrRotMatrix(:))==9 % input is rotation matrix
    MyRotationAngles=Fn_Rotation_Matrix_values_from_matrix_Vect(MyRotAnglesOrRotMatrix,1);
else
    MyRotAnglesOrRotMatrix, error('wrong format'),
end
clear MyRotAnglesOrRotMatrix;

if Flag_ComputeOutputs==0
    dominant_directions=[];
    List_ClusteredNormals=[];
    List_ClusteredNormalsIndices=[];
end

flag_AtMostOneMatching=1;


dominant_directions=Fn_GetVPs_GivenRotation(MyRotationAngles);
dominant_directions=dominant_directions(1:Nb_VPs,:); % returns only the nb of VPs requested

%dominant_directions, input('sdfsdf 2')




%%%%%% START - create the list of normals indices %%%%%%
List_ClusteredNormalsIndices=cell(Nb_VPs,1); %List_ClusteredNormalsIndices=zeros(Nb_VPs,1); % better initializaiton: zeros(3,max(best_distances(1:3))+1);
%start - test the 3 directions
ListNbClusteredIndices=zeros(1,4);
ValidIndices=1:size(list_normals,1);

% initialization
if Flag_ComputeOutputs==1
    for i=1:Nb_VPs
        List_ClusteredNormalsIndices{i}=[]; % maybe useless actually since it is already intializaed above
    end
end

for i=1:Nb_VPs
    %     if i==1
    %         my_direction=my_x;
    %     elseif i==2
    %         my_direction=my_y;
    %     elseif i==3
    %         my_direction=my_z;
    %     end
    if length(ValidIndices)==0
        break; % all the normals have been clustered so Fn_ComputeVPDist is useless
    end
    
    my_direction=dominant_directions(i,:);
    indices=find(Fn_ComputeVPDist(list_normals(ValidIndices,:),my_direction,flag_InlierDistanceDefinition,[])<ThreshInlier);
    %i, my_direction, Fn_ComputeVPDist(list_normals(ValidIndices,:),my_direction), ThreshInlier, indices
    
    ListNbClusteredIndices(i)=length(indices);
    
    if Flag_ComputeOutputs==1
        %List_ClusteredNormalsIndices(i,1:length(indices)+1)=[length(indices); indices];
        List_ClusteredNormalsIndices{i}=ValidIndices(indices);
    end
    
    % to avoid the same line to be clustered to several VPs
    if flag_AtMostOneMatching==1
        ValidIndices=setdiff(ValidIndices,ValidIndices(indices));
    end
    
    %     %start - test all the normals
    %     for j=1:size(list_normals,1)
    %         my_normal=list_normals(j,:);
    %         %if abs(acos(dot(my_direction,my_normal))/pi*180-90)<normal_similarity_angle_threshold
    %         %JC 09/2010 for cvpr line clustering  (change also at the end)
    %          if abs(my_normal*my_direction) <normal_similarity_angle_threshold
    %             List_ClusteredNormalsIndices(i,1)=List_ClusteredNormalsIndices(i,1)+1; % the nb of correct normals
    %             List_ClusteredNormalsIndices(i,List_ClusteredNormalsIndices(i,1)+1)=j; % the indices of the correct normals
    %         end
    %     end
    %     %finish - test all the normals
end
%finish - test the 3 directions
if flag_AtMostOneMatching==1
    UnclusteredIndices=ValidIndices;
else
    ClusteredIndices_all=[List_ClusteredNormalsIndices{:}];
    UnclusteredIndices=setdiff(1:size(list_normals,1),ClusteredIndices_all);
end
ListNbClusteredIndices(4)=sum(ListNbClusteredIndices(1:Nb_VPs));
%%%%%% END - create the list of normals indices %%%%%%

if Flag_ComputeOutputs==1
    % normals_dominant_directions contains the normals of the 3 directions
    % for i=1:3
    % my_indices=similar_normals_indices_results(index,i,1+1:similar_normals_indices_results(index,i,1)+1,:);
    % normals_dominant_directions(i,1:length(my_indices)+1,:)=[[length(my_indices) 0 0]; list_normals(my_indices,:)];
    % end
    for nth_direction=1:Nb_VPs
        %my_indices=List_ClusteredNormalsIndices(nth_direction,1+1:List_ClusteredNormalsIndices(nth_direction,1)+1,:);
        my_indices=List_ClusteredNormalsIndices{nth_direction};
        % array format: List_ClusteredNormals(i,1:length(my_indices)+1,:)=[[length(my_indices) 0 0]; list_normals(my_indices,:)];
        if length(my_indices)~=0
            for nth_normal=1:length(my_indices) % COULD BE IMPROVED TO ASSIGN DIRECTLY THE WHOLE LIST AND DEFINED THE SIZE AT THE BEGINNING
                List_ClusteredNormals{nth_direction,nth_normal}=list_normals(my_indices(nth_normal),:); %list_normals(my_indices,:);
            end
        else
            % is that an error??
            List_ClusteredNormals{nth_direction,1}=[];
        end
    end
    
    % for security, check a line is not clustered more than once
    if flag_AtMostOneMatching==1
        for i=1:Nb_VPs
            for j=i+1:Nb_VPs
                ListIndices1=List_ClusteredNormalsIndices{i};
                ListIndices2=List_ClusteredNormalsIndices{j};
                if length(intersect(ListIndices1, ListIndices2))~=0
                    for k=1:Nb_VPs, List_ClusteredNormalsIndices{k}, end
                    i,j
                    error('some intersection!!!')
                end
            end
        end
    end
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%     END - GET THE FINAL RESULTS    %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
