    
function ListCubes=Fn_BnB_RemoveCubesWithSameBounds(ListCubes)

% if several intervals with the same equal lower and upper, keep only one

%disp('start cleaning')
TimerCleanCubes=tic;

IndexFlag=4;
IndicesBounds=[5 6];
% inputs:  ListCubesInfo(nth_cube,:)=[Rx Ry Rz flag MyLowerBoundOutliers MyUpperBoundOutliers];

NbRemovedCubes=0;

UniqueLowerBoundsValue=unique(ListCubes(:,IndicesBounds(1))); % the unique values of the lower bounds (upper would be also ok)

% if length(find(UniqueLowerBoundsValue==-1))~=0
%     error('some cubes with unknown bounds are being deleted')
% end

for MyLowerBound=UniqueLowerBoundsValue'
    % just security
    if length(MyLowerBound)~=1; error('wrong format'); end % check that UniqueLowerBoundsValue is horizontal
    
%     % get the cubes that have equal lower and upper bounds, whose value is MyLowerBound
%     indices=find(ListCubes(:,IndicesBounds(1))==MyLowerBound & ListCubes(:,IndicesBounds(2))==MyLowerBound);
    % get the VALID cubes that have equal lower and upper bounds, whose value is MyLowerBound, and known bounds
    indices=find(ListCubes(:,IndicesBounds(1))==MyLowerBound & ListCubes(:,IndicesBounds(2))==MyLowerBound & ListCubes(:,IndexFlag)==1  & ListCubes(:,IndicesBounds(1))~=-1);
    
    % if more than 1 cube like this
    if length(indices)>1
        for my_index=indices(2:end)' % remove all except the first one (set the flag to 0)
           % just security
    if length(my_index)~=1; error('wrong format'); end % check that indices is horizontal
    
   
            ListCubes(my_index,IndexFlag)=0; % i.e. make it infeasible
            NbRemovedCubes=NbRemovedCubes+1;
        end
    end
end

TimerCleanCubes_Duration=toc(TimerCleanCubes);
%sprintf('TimerCleanCubes_Duration=%f',TimerCleanCubes_Duration)
%NbRemovedCubes

%IndicesEqualBounds=find((ListCubesInfo(:,1)==ListCubesInfo(:,2)));
%UniqueBoundsValue=unique(ListCubesInfo(IndicesEqualBounds,1));
