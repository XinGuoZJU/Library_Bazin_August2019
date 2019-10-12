function [NewListCubes, LevelSubdivision]=Fn_BnB_CubeSubdivision(ListCubes, LevelSubdivision, MyIndex, DimModel,DimRadius, FlagSubdividingMethod, FlagRotationBall)

% Comment: why "-HalfRadiusCube" ( (norm(NewListCubes(kth_new_cube,1:3))-HalfRadiusCube)) when we check if it is inside the cube

% goal: subdivide the list of rotation cubes
% at each iteration, the radius cube is divided by 2

% inputs:
%   - ListCubes: NxDim, where Dim is the number of elements
%       - exemple of Dim: 3+1+2; % 3 for rotation, 1 for the flag, 2 for the bounds
%   - LevelSubdivision
%   - MyIndex: if [], then subdivide all; otherwise subdivide only this index cube. Used for depth-first search
%   - DimModel
%   - DimRadius: usually 1 for rotation application
% %   - RadiusCube: the radius of the cube
%   - FlagSubdividingMethod: 0-subdivide into 8 cubes, 1-subdivide along the longest dimension
%   - FlagRotationBall: 0-for non rotation, 1-for rotation, 2-for partial rotation (used when reordering of R is also a solution, i.e. ambiguous case), 3-R and f

% outputs:
%   - NewListCubes
%   - RadiusCube
%   - LevelSubdivision

if nargin~=7; nargin, error('wrong nb of inputs'); end
if nargout~=2; nargin, error('wrong nb of ouputs'); end

%disp('start subdivision')
TimerSubdivision=tic;

NbCubes=size(ListCubes,1);
if (NbCubes<1)
    NewListCubes=[];
    RadiusCube=[];
    return;
end

% if DimModel==3 % 3 for rotation
%     flag_PureRotationModel_OrRandFocal=0;
% elseif DimModel==4
%     flag_PureRotationModel_OrRandFocal=1;
% else
%     DimModel
%     error('wrong case')
% end


%ListCubes, input('before subdividing')

LevelSubdivision=LevelSubdivision+1; % for post processing
if FlagSubdividingMethod==0
    if DimModel==3
    ListDelta=[1 1 1; 1 1 -1; 1 -1 1; 1 -1 -1; -1 1 1; -1 1 -1; -1 -1 1; -1 -1 -1; 0 0 0]; % the last one is to keep its original locaton if it is feasible. THE LAST ON IS NOT USED??
    elseif DimModel==2
        ListDelta=[1 1; 1 -1; -1 1; -1 -1; 0 0]; % the last one is to keep its original locaton if it is feasible. THE LAST ON IS NOT USED??
    elseif DimModel==4
        A=[-1 1]; B=[-1 1]; C=[-1 1]; D=[-1 1];
[AA BB CC DD]=ndgrid(A,B,C,D);
ListDelta= [AA(:) BB(:) CC(:) DD(:)];
    else
                % general formula for 2 dims
A=[-1 1]; B=[-1 1];
[AA BB]=ndgrid(A,B)
COMB= [AA(:) BB(:)]

        % general formula for 4 dims
A=[-1 1]; B=[-1 1]; C=[-1 1]; D=[-1 1];
[AA BB CC DD]=ndgrid(A,B,C,D)
COMB= [AA(:) BB(:) CC(:) DD(:)]
        error('to test')
    end
    Nd=size(ListDelta,1)-1;%8; % nb of split
elseif FlagSubdividingMethod==1
    ListDelta=[];%???
    Nd=2; % nb of split. one extra
else
    error('wrong value')
end

% compute the number of new cubes
[IndexFlag,IndicesBounds,IndexRadius]=Fn_BnB_CubeInitialization_Dim(DimModel,DimRadius);

if length(MyIndex)==0
    NewNbCubes=sum(ListCubes(:,IndexFlag))*Nd; % each feasible cube gives Nd new cubes
else
    % nb of valid cubes (without counting the current cube so must be substracted) plus the nb of new cubes (if the current cube is valid)
    NewNbCubes=sum(ListCubes(:,IndexFlag))-ListCubes(MyIndex,IndexFlag) + ListCubes(MyIndex,IndexFlag)*Nd; % each feasible cube gives 8 new cubes
end
DimCube=size(ListCubes,2);
NewListCubes=zeros(NewNbCubes,DimCube);
NewListCubes(:,IndicesBounds(1))=-1; % bounds not calculated yet
NewListCubes(:,IndicesBounds(2))=-1;

kth_new_cube=0;
for my_IndexCube=1:NbCubes % for all the cubes
    if ListCubes(my_IndexCube,IndexFlag)==1 % if the cube is feasible
        
        if ListCubes(my_IndexCube,IndicesBounds(1))~=-1 && ListCubes(my_IndexCube,IndicesBounds(2))~=-1 &  ListCubes(my_IndexCube,IndicesBounds(1))==ListCubes(my_IndexCube,IndicesBounds(2)) % if its bounds are known and same (i.e. known point interval)
            % do not subdvide it
            kth_new_cube=kth_new_cube+1;
            NewListCubes(kth_new_cube,:)=ListCubes(my_IndexCube,:); % copy the entire information
            continue;
        end
        
        if length(MyIndex)==0 || (length(MyIndex)~=0 && MyIndex==my_IndexCube) % i.e. subivide all the cubes (if applied) or subdivide only the interesting cube
            
            % test for debugging
            %             if length(MyIndex)~=0
            %             my_IndexCube, MyIndex,
            %             ListCubes(my_IndexCube,:)
            %             end
            
            if FlagSubdividingMethod==0 % 0-subdivide into N (N=8 orXX) cubes, 1-subdivide along the longest dimension
                RadiusCube=ListCubes(my_IndexCube,IndexRadius)/2;
                HalfRadiusCube=RadiusCube;
                
                %HalfRadiusCube
                HalfRadiusCubeRotation=HalfRadiusCube(1);
%                 if FlagRotationBall==3
%                 HalfRadiusCubeFocal=HalfRadiusCube(2);
%                 end
            elseif FlagSubdividingMethod==1
                if FlagRotationBall~=0, error('possible??'), end
                % get the longest dim
                [val MaxDim]=max(ListCubes(my_IndexCube,IndexRadius));
                %RadiusCube=ListCubes(my_IndexCube,IndexRadius(MaxDim))/2;
                % copy and replace the value
                RadiusCube=ListCubes(my_IndexCube,IndexRadius);
                RadiusCube(MaxDim)=val/2;
                HalfRadiusCube=val/2;%RadiusCube;
                
                %MaxDim, ListCubes(my_IndexCube,IndexRadius),  input('ok to cut?')
                ListDelta=zeros(2,length(IndexRadius));
                ListDelta(1,MaxDim)=-1;
                ListDelta(2,MaxDim)=1;
               % ListDelta, input('sdfsdf')
            end
            
            for kth_split=1:Nd % split it into 8 small cubes (some might be infeasible)
                kth_new_cube=kth_new_cube+1;
                
                %ListDelta, ListDelta(kth_split,:), RadiusCube, FlagSubdividingMethod, DimModel
                
                if FlagSubdividingMethod==0 % 0-subdivide into 8 cubes, 1-subdivide along the longest dimension
                    % move the cube center by delta
                    %DimModel, RadiusCube
                    if FlagRotationBall~=3
                NewListCubes(kth_new_cube,1:DimModel)=ListCubes(my_IndexCube,1:DimModel) + ListDelta(kth_split,:).*RadiusCube;
                NewListCubes(kth_new_cube,IndexRadius)=HalfRadiusCube;
                    else
                        NewListCubes(kth_new_cube,1:DimModel)=ListCubes(my_IndexCube,1:DimModel) + ListDelta(kth_split,:).*[RadiusCube(1) RadiusCube(1) RadiusCube(1) RadiusCube(2)]; % 3 for R and 1 for focal
                NewListCubes(kth_new_cube,IndexRadius)=HalfRadiusCube;
                    end
                elseif FlagSubdividingMethod==1
                    NewListCubes(kth_new_cube,1:DimModel)=ListCubes(my_IndexCube,1:DimModel) + ListDelta(kth_split,:).*RadiusCube;
                NewListCubes(kth_new_cube,IndexRadius)=ListCubes(my_IndexCube,IndexRadius);
                NewListCubes(kth_new_cube,IndexRadius(MaxDim))=HalfRadiusCube;
                end
                
                % check if it is inside the ball
                %C++ mag=sum((NewListCubes(kth_new_cube,1:3)).^2) % its squared norm: norm(NewListCubes(kth_new_cube,1:3))^2
                %C++ if (sqrt(mag)-HalfRadiusCube) > pi % it must be inside the ball of radius PI
                if (FlagRotationBall==1 || FlagRotationBall==2  || FlagRotationBall==3) && (norm(NewListCubes(kth_new_cube,1:3))-sqrt(2)*HalfRadiusCubeRotation) > pi % it must be inside the ball of radius PI, at least partially
                    NewListCubes(kth_new_cube,IndexFlag) = 0; % make it infeasible (the cube is completely outside)
                else
                    NewListCubes(kth_new_cube,IndexFlag) = 1; % make it feasible (the cube is at least partially inside the rotation ball)
                    % disp('feasible')
                end
%                 % version: before June 2013. let's use half of the ball
%                 if FlagRotationBall==2 && (NewListCubes(kth_new_cube,3)<0)
%                     NewListCubes(kth_new_cube,IndexFlag) = 0; % make it infeasible (the cube is completely outside)
%                 end
                % version: before June 2013. let's use 1/8 of the ball
                if FlagRotationBall==2 && (NewListCubes(kth_new_cube,1)+HalfRadiusCubeRotation<0 || NewListCubes(kth_new_cube,2)+HalfRadiusCubeRotation<0 || NewListCubes(kth_new_cube,3)+HalfRadiusCubeRotation<0)
                    NewListCubes(kth_new_cube,IndexFlag) = 0; % make it infeasible (the cube is completely outside)
                end
                
            end
        else % i.e. a feasible cube to not subdivide
            kth_new_cube=kth_new_cube+1;
            NewListCubes(kth_new_cube,:)=ListCubes(my_IndexCube,:); % copy the entire information
        end
    end
end

NewListCubes=NewListCubes(1:kth_new_cube,:); % cut
%if kth_new_cube~=NewNbCubes, kth_new_cube, NewNbCubes, error('wrong size'); end

% if length(MyIndex)~=0
%     if ListCubes(my_IndexCube,IndexFlag)==1 % if the cube is feasible
%     ListCubes(MyIndex,:)=ListCubes(end,:); %put the last one at the current index to remove it
%     NewListCubes=[NewListCubes;  ListCubes(1:end-1,:)];
%     % check size
%     if ~(size(NewListCubes,1)==NbCubes+8-1), size(NewListCubes,1), NbCubes, error('wrong size'); end
%     else
%         ListCubes(MyIndex,:)=ListCubes(end,:); %put the last one at the current index to remove it
%         NewListCubes=ListCubes(1:end-1,:);
%     end
% end

% check is there any overlapping due to bad subdivision
flag_test=0;
if flag_test==1
NbCubes=size(NewListCubes,1);
for kth_cube=1:NbCubes
    MyCenter=NewListCubes(kth_cube,1:DimModel);
    MyRadius=NewListCubes(kth_cube,IndexRadius);
    MyLower=MyCenter-MyRadius;
    MyUpper=MyCenter+MyRadius;
    for kth_cube2=kth_cube+1:NbCubes
        MyCenter2=NewListCubes(kth_cube2,1:DimModel);
        
    flag=Fn_BnB_GetIntervalIntersection(MyCenter2,MyLower,MyUpper);
    if flag==1
        NewListCubes
        IndexRadius
        MyCenter, MyRadius, MyLower, MyUpper, MyCenter2
        kth_cube, kth_cube2
        (MyLower<MyCenter2),(MyCenter2<MyUpper)
        sdfsdf
    end
    end
end
end

TimerSubdivision_Duration=toc(TimerSubdivision); % display the elapsed time
%sprintf('TimerSubdivision_Duration=%f',TimerSubdivision_Duration)
%disp('end subdivision')


