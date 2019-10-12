function [NewListCubes, RadiusCube, LevelSubdivision]=Fn_BnB_CubeSubdivision_OneCube(ListCubes, RadiusCube, LevelSubdivision)

% Comment: why "-HalfRadiusCube" ( (norm(NewListCubes(kth_new_cube,1:3))-HalfRadiusCube)) when we check if it is inside the cube

% goal: subdivide the list of rotation cubes
% at each iteration, the radius cube is divided by 2

% inputs:
%   - ListCubes: NxDim, where Dim is the number of elements
%       - exemple of Dim: 3+1+2; % 3 for rotation, 1 for the flag, 2 for the bounds
%   - RadiusCube: the radius of the cube
%   - LevelSubdivision


% outputs:
%   - NewListCubes
%   - RadiusCube
%   - LevelSubdivision

disp('start subdivision')
tic

NbCubes=size(ListCubes,1);
if (NbCubes<1)
    NewListCubes=[];
    RadiusCube=[];
    return;
end


LevelSubdivision=LevelSubdivision+1; % for post processing
RadiusCube=RadiusCube/2;
ListDelta=[1 1 1; 1 1 -1; 1 -1 1; 1 -1 -1; -1 1 1; -1 1 -1; -1 -1 1; -1 -1 -1; 0 0 0]; % the last one is to keep its original locaton if it is feasible. THE LAST ON IS NOT USED??
Nd=8; % nb of split

% compute the number of new cubes
IndexFlag=4;
IndicesBounds=[5 6];
NewNbCubes=sum(ListCubes(:,IndexFlag))*Nd; % each feasible cube gives 8 new cubes
DimCube=size(ListCubes,2);
NewListCubes=zeros(NewNbCubes,DimCube);
NewListCubes(:,IndicesBounds(1))=-1; % bounds not calculated yet
NewListCubes(:,IndicesBounds(2))=-1;

kth_new_cube=0;
for kth_cube=1:NbCubes % for each cube of the list
    if ListCubes(kth_cube,IndexFlag)==1 % if the cube is feasible
        HalfRadiusCube=RadiusCube;
        for kth_split=1:Nd % split it into 8 small cubes (some might be infeasible)
            kth_new_cube=kth_new_cube+1;

            % move the cube center by delta
            NewListCubes(kth_new_cube,1:3)=ListCubes(kth_cube,1:3) + ListDelta(kth_split,:)*RadiusCube;

            % check if it is inside the ball
            %C++ mag=sum((NewListCubes(kth_new_cube,1:3)).^2) % its squared norm: norm(NewListCubes(kth_new_cube,1:3))^2
            %C++ if (sqrt(mag)-HalfRadiusCube) > pi % it must be inside the ball of radius PI
            if (norm(NewListCubes(kth_new_cube,1:3))-sqrt(2)*HalfRadiusCube) > pi % it must be inside the ball of radius PI
                NewListCubes(kth_new_cube,IndexFlag) = 0; % make it infeasible
            else
                NewListCubes(kth_new_cube,IndexFlag) = 1; % make it feasible
            end

        end

    end
end

HalfRadiusCube
disp('end subdivision')
toc

