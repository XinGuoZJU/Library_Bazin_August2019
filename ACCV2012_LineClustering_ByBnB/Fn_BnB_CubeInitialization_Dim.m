function [IndexFlag,IndicesBounds,IndexRadius]=Fn_BnB_CubeInitialization_Dim(DimModel,DimRadius)


if length(DimRadius)==0
DimRadius=3;
end

% not used for the moment.......
Dim=DimModel+1+2+DimRadius; % DimModel for the model, 1 for the flag, 2 for the bounds, DimRadius for the radius 
% e.g. DimRadius can be 1 for rotation, but 2 for rotation-focal


IndexFlag=DimModel+1;%4
IndicesBounds=IndexFlag+[1 2];%[5 6];
IndexRadius=IndicesBounds(end)+[1:DimRadius];


