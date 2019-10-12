
% version 21/11/2009

% modifications: 21/11/2009
%   - fast version using the temporary global image TempGlobalImage
%   - faster version: use "find", one single loop over the points
%   - Note; the drawing can still be improved
% modifications 04/03/2010
%   - flag_SquareOrCircle
%   - apply rounding to deal with non integr coordinates


% inputs:
%   - my_image: a rgb or gray image
%   - points_list: Nx2 for [x,y]
%   - point_size: the size of the points. 0: only 1 point, 1: 3x3 patch, 2: 5x5 patch, etc...
%   - my_color: a 3-vector
%   - flag_FastGlobalDrawing: 0 (no global) or 1 (global - faster)



function my_image2=Fn_aux_draw_points(my_image, points_list, point_size, my_color, flag_FastGlobalDrawing)

if nargin~=5, nargin, error('wrong nb of inputs'); end
if nargout~=1 && nargout~=0, nargout, error('wrong nb of outputs'); end % there can have 1 or 0 output (global image)

flag_SquareOrCircle=1; %0-draw squares, 1-draw circles

if flag_FastGlobalDrawing==0
    
    height=size(my_image,1);
    width=size(my_image,2);
    
    if ndims(my_image)==2
        my_image2(:,:,1)=my_image;
        my_image2(:,:,2)=my_image;
        my_image2(:,:,3)=my_image;
    else
        my_image2=my_image;
    end
    
else
    global TempGlobalImage; % load
    if length(TempGlobalImage)==0; error('no global image');  end
    if ndims(TempGlobalImage)~=3, size(TempGlobalImage), error('wrong format');   end
    height=size(TempGlobalImage,1);
    width=size(TempGlobalImage,2);
    
    % the returned value
    my_image2=[];
end

if size(points_list,2)~=2; size(points_list), error('wrong size'); end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     START - NEW VERSION    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global i_debug;
if point_size==0
    indices=find(0<points_list(:,1) & points_list(:,1)<=width & 0<points_list(:,2) & points_list(:,2)<=height);
    % this part could be improved by "block or vector-based operations"
    if flag_FastGlobalDrawing==0
        for i=1:length(indices)
            my_image2(points_list(indices(i),2),points_list(indices(i),1),:)=my_color;
        end
        % IndicesImage=sub2ind(size(my_image2), points_list(indices,2), points_list(indices,1)); % rows and cols
    else
        for i=1:length(indices)
            i_debug=i;
%             if i==1
%                 size(indices), size(points_list), size(TempGlobalImage), points_list(indices(i),:)
%             end
            TempGlobalImage(points_list(indices(i),2),points_list(indices(i),1),:)=my_color;
        end
        %figure('Name','TempGlobalImage in Fn_aux_draw_points'); imshow(TempGlobalImage/255)
    end
    
    
else
    % convert the neighborhood from [-D:+D]x[-D:+D] into [(2D+1)^2 x 2]
    % the current technique could be improved
    nb_neighbors=(2*point_size+1)^2; % might be updated for circles
    list_neighbor_local=zeros(nb_neighbors,2); % might be too big for circle so will be cut
    nth_point=0;
    for m=-point_size:point_size
        for n=-point_size:point_size
            %if m^2+n^2<=point_size^2 % not necessarly, in that case, you MUST "cut" list_neighbor_local=list_neighbor_local(1:nth_point,:);
            if flag_SquareOrCircle==0 || (flag_SquareOrCircle==1 && m^2+n^2<=point_size^2)
                nth_point=nth_point+1;
                list_neighbor_local(nth_point,:)=[m n];
            end
        end
    end
    nb_neighbors=nth_point;
    list_neighbor_local=list_neighbor_local(1:nth_point,:); % cut
    %[nth_point  nb_neighbors] % they should be same
    %size(list_neighbor_local)
    %nb_neighbors
    
    % build the neighborhoods for all the current points, i.e. from NbNeighborsx2 to (NbPoints*NbNeighbors)x2
    nb_points=size(points_list,1);
    list_neighbor_global=repmat(list_neighbor_local,nb_points,1); % we must apply the local neighbor (list_neighbor_local) to all the original points (there are nb_points), so we repeat it nb_points times
    
    %convert from [x1 y1; x2 y2] into [x1 y1; ....; x1 y1; x2 y2; ... x2 y2], i.e. repeat the points several times
    
    points_list_global=transpose(repmat(points_list,1,nb_neighbors)); %from [x1 y1; x2 y2] into [x1 y1 ... x1 y1; x2 y2 ... x2 y2] and then into [x1 x2; y1 y2; ... x1 x2; y1 y2]
    %disp('hi'), size(points_list), nb_neighbors, size(points_list_global)
    points_list_global=transpose(reshape(points_list_global,2,[])); % from [x1 x2; y1 y2; ... x1 x2; y1 y2]  into [x1 ... x1 x2 ... x2; y1 ... y1 y2 ... y2]  and then into [x1 y1; ....; x1 y1; x2 y2; ... x2 y2]
    %disp('hello'), size(points_list), nb_neighbors, size(points_list_global)
    %nb_neighbors, nb_points, size(points_list), size(list_neighbor_global), size(points_list_global), size(list_neighbor_local)
    % add the offsets to draw the patch for each each point
    points_list_global=points_list_global+list_neighbor_global;
    %extract points not in the border
    points_list_global_rounded=round(points_list_global);
    indices1=find(0<points_list_global_rounded(:,1) & points_list_global_rounded(:,1)<=width & 0<points_list_global_rounded(:,2) & points_list_global_rounded(:,2)<height);
    %remove "doublons"
    points_list_global_rounded=unique(points_list_global_rounded(indices1,:),'rows');
    
    
    % draw the points. this part could be improved by "block or vector-based operations"
    if flag_FastGlobalDrawing==0
        for i=1:size(points_list_global_rounded,1)
            my_image2(points_list_global_rounded(i,2),points_list_global_rounded(i,1),:)=my_color;
        end
    else
        for i=1:size(points_list_global_rounded,1)
            TempGlobalImage(points_list_global_rounded(i,2),points_list_global_rounded(i,1),:)=my_color;
        end
        
    end
end
% TEST:
%   - syms x1 y1 x2 y2 x3 y3 real; points_list=[x1 y1; x2 y2; x3 y3]; point_size=1;
%   - Fn_aux_draw_points([1], points_list, point_size, [],0);
%   - check the value of points_list_global before adding the nighborhood
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     END - NEW VERSION    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% START - OLD VERSION %%%%%%%
% for i=1:size(points_list,1)
%     for m=-point_size:point_size
%         for n=-point_size:point_size
%             my_y=points_list(i,2)+m;
%             my_x=points_list(i,1)+n;
%             if 0<my_y && my_y<=height  && 0<my_x && my_x<=width
%                 my_image2(my_y,my_x,:)=my_color;
%             end
% %             my_image2(points_list(i,2)+m,points_list(i,1)+n,:)=[255 0 0];
%         end
%     end
% end
%
% % figure; imshow(my_image2);
%%%%%%%% END - OLD VERSION %%%%%%%

