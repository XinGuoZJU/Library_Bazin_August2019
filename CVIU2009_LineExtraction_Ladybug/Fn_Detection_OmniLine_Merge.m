

% version 01/11/2009

% modifications: 01/11/2009
%   - the format of Normal has been changed from (nth_point+1,nth_line,:) towards (nth_line, nth_point+1,:)

% note: this current merging is performed by graph
disp('start merging')

if isfield(Normal,'points')==1
FlagOmniOrPersp=0;
else
    FlagOmniOrPersp=1;
end

Normal=save_Normal;
nb_of_normals_before=size(Normal.vect,1);

Normal2.vect =[];
if FlagOmniOrPersp==0
Normal2.points =[];
end
Normal2.coords=[];

%initialize connected components of the graph. Each line is connected to itsself
CC=zeros(1,nb_of_normals_before);
for i=1:nb_of_normals_before
    CC(i)=i;
end

%fill in the graph
% if we check 1-3 then it is same than checking 3-1 so do from i+1
for i=1:nb_of_normals_before
    for j=i+1:nb_of_normals_before % j=1:nb_of_normals_before OR maybe only j=i+1:nb_of_normals_before (i.e. starting from i+1 instead of 1)
        val_i=CC(i);
        val_j=CC(j);


        if val_i==val_j % if they are already connected then go to next one (i.e. no need to check if they are connected and no need to connect them)
            continue;
        end

        U = Normal.vect(i,:);
        V = Normal.vect(j,:);
        if rad2deg(abs(acos(U*V')))<ThresholdMergingAngle

            % i,j,U,V,rad2deg(abs(acos(U*V'))), input('test')
            
            CC(j)=val_i; % if lower than threshold, then j get connected to i
            for k=1:nb_of_normals_before % and for each normal that is connected to j, we now connect it to i
                if CC(k)==val_j
                    CC(k)=val_i;
                end
            end
        end
    end
end

% get points connected by graph
index=0;% corresponds to the number of lines after the merging
list_length_segments=[];
for i=1:nb_of_normals_before
    flag_new=0;
    for j=1:nb_of_normals_before % given i, we search all the lines connected to i and we create the list of associated normals
        if CC(j)==i
            if flag_new==0
                flag_new=1;
                index=index+1;
                nb_of_points_1=0;
                nb_of_coords_1=0;
            else
                %JC nb_of_points_1=Normal2.points(index,1,1);
                %JC nb_of_coords_1=Normal2.coords(index,1,1);
                if FlagOmniOrPersp==0
                nb_of_points_1=size(Normal2.points{index},1);
                end
                nb_of_coords_1=size(Normal2.coords{index},1);
            end
            
            % get the elements to add in the list
%        JC     nb_of_points_2=Normal.points(j,1,1);
%        JC     nb_of_coords_2=Normal.coords(j,1,1);
%        JC     points_list_2=squeeze(Normal.points(j,1+1:1+nb_of_points_2,:));
%        JC     coords_list_2=squeeze(Normal.coords(j,1+1:1+nb_of_coords_2,:));
if FlagOmniOrPersp==0
nb_of_points_2=size(Normal.points{j},1);
points_list_2=Normal.points{j};
end
            nb_of_coords_2=size(Normal.coords{j},1);
            coords_list_2=Normal.coords{j};
            % vect=Normal.vect(:,j);

            % add the elements in the list
%     JC        Normal2.points(index,1,1:3)=[nb_of_points_1+nb_of_points_2 0 0];
%     JC        Normal2.points(index,nb_of_points_1+2:nb_of_points_1+nb_of_points_2+1,1:3)=points_list_2;
%     JC        Normal2.coords(index,1,1:2)=[nb_of_coords_1+nb_of_coords_2 0];
%     JC        Normal2.coords(index,nb_of_coords_1+2:nb_of_coords_1+nb_of_coords_2+1,1:2)=coords_list_2;

if index>length(Normal2.coords)% i.e. Normal2.points{index} is not created yet
    if FlagOmniOrPersp==0
             Normal2.points{index}=points_list_2;
    end
            Normal2.coords{index}=coords_list_2;
else
    if FlagOmniOrPersp==0
            Normal2.points{index}=[Normal2.points{index}; points_list_2];
    end
            Normal2.coords{index}=[Normal2.coords{index}; coords_list_2];
end        
            % note that we will compute the merged normal later
            
            list_length_segments(index)=size(Normal2.coords{index},1); %JC Normal2.points(index,1,1);
            
        end
    end
end
% up to this point, the results are saved in Normal2

% check length threshold
indices=find(list_length_segments>ThresholdLengthAfterMerging);
%JC Normal.points=Normal2.points(indices,:,:);
%JC Normal.coords=Normal2.coords(indices,:,:);
% Normal.points=Normal2.points{indices}; NOT POSSIBLE DIRECTLY
% Normal.coords=Normal2.coords{indices}; NOT POSSIBLE DIRECTLY
for index=indices
    if FlagOmniOrPersp==0
Normal.points{index}=Normal2.points{index};
    end
Normal.coords{index}=Normal2.coords{index};
end
Normal.vect=zeros(length(indices),3);


% compute the new normal associated to the connected points
nb_of_normals_after=index;
for i = 1 : length(Normal.coords) %JC size(Normal.points,1)
    %JC nb_of_points=Normal.points(i,1,1);

    % the new normal is computed by SVD on all the points
    %JC points_list=squeeze(Normal.points(i,1+1:nb_of_points+1,:)); % the size is Nx3
    if FlagOmniOrPersp==0
    points_list=Normal.points{i}; % the size is Nx3
    else
        points_list=[Normal.coords{i} ones(size(Normal.coords{i},1),1)]; % the size is Nx3
    end
    
    [U,S,V] = svd(points_list);
    new_normal=V(:,end);
    if new_normal(3)<0
        new_normal=-new_normal;
    end

    Normal.vect(i,:)=new_normal; %save the normal
end

save_Normal=Normal;% simply used for making code testing easier

clear Normal2;

disp('end merging')

