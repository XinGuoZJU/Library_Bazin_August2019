function [ListCubes, RadiusCube, LevelSubdivision]=Fn_BnB_CubeInitialization(DimModel,CubeCenter,RadiusCube)

% goal: initialize the list of cubes ListCubes, which will contains only one element (the box containing the rotation ball
% of radius pi, so the box has a height/width of 2*pi)
% This element contains all the possible rotations, so contains the optimal solution

% inputs:
%   - Dim: the dimension (i.e. the number of elements to save information). can be []
%   - CubeCenter:
%   - DimModel: the dimension of the model
% RadiusCube: can be a list

% outputs:
%   - ListCubes: fixed 1xDim
%       - and also: ListCubes(1,IndexFlag)=1; % feasible cube
%   - RadiusCube: fixed pi. a cube of radius pi. it is the half length of the cube
%   - LevelSubdivision: fixed 1

% call example: [ListCubes, RadiusCube, LevelSubdivision]=Fn_BnB_CubeInitialization(DimModel,zeros(1,3),[]);

NbCubes=1;
LevelSubdivision=0;

%RadiusCube=pi;
if length(RadiusCube)==0
    RadiusCube=pi;
end
DimRadius=length(RadiusCube);

[IndexFlag,IndicesBounds,IndexRadius]=Fn_BnB_CubeInitialization_Dim(DimModel,DimRadius);

ListCubes(1,1:DimModel)=CubeCenter; %ListCubes(1,1:3)=zeros(1,3);
ListCubes(1,IndexFlag)=1; % feasible cube
ListCubes(1,IndicesBounds)=[-1 -1]; % bounds not calculated yet
ListCubes(1,IndexRadius)=RadiusCube;


