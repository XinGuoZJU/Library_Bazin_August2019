
function [R_best,ListPostInfo,ListExtraInfo]=Fn_BnB_InlierOptimization(flag_application,MyData,AprioriSolution)

% Goal: maximize the nb of inliers for several applications (cf flag_application) using a branch-and-bound approach
%
% Version April 2016
% Version v2: maximize the nb of outliers
%
% Inputs:
%   - flag_application: 0-pure rotation, 1-camera resectioning (find R,C given 2d and 3d points), 2-two view reconstruction (find R, C, 3D points Xi, given 2D points),  3-3D/3D registration (find R,T given 3D points), 4-uncalibrated SfM (get F (or K, R ,T) from a set of 2D correspondences), 10-pure rotation (lines/VPs), 12-pure rotation (surface 3D normals for plane segmentation/clustering) 20-pure rotation and unknown pairing, 30-uncalibrated Hinfinity: find R and focal from a set of 2D correspondences
%   - MyData: contains the data to process
%           - if flag_application==0
%                   - MyData.SetLeft=SpherePointsSetLeft;
%                   - MyData.SetRight=SpherePointsSetRight;
%                   - MyData.InlierThreshold=InlierThreshold;
%           - if flag_application==1
%                   - MyData.Set2DPoints=Data_2DPoints;
%                   - MyData.Set3DPoints=Data_3DPoints;
%                   - MyData.InlierThreshold=InlierThreshold;
%           - if flag_application==2
%                   - MyData.SetLeft=SetPointsLeft;
%                   - MyData.SetRight=SetPointsRight;
%                   - MyData.Set3DPoints=Set3DPoints;
%                   - MyData.InlierThreshold=InlierThreshold;
%           - if flag_application==5
%                   - MyData.SetLeft=SetPointsLeft;
%                   - MyData.SetRight=SetPointsRight;
%                   - MyData.InlierThreshold=InlierThreshold;
%           - if flag_application==10
%                   - MyData.SetNormals=ListNormals;
%                   - MyData.Nb_VPs=Nb_VPs;
%                   - MyData.InlierThreshold=InlierThreshold;
%                   - MyData.FlagDistanceDefinition cf Fn_ComputeVPDist
%           - if flag_application==12
%                   - MyData.SetSurfNormals=ListSurfNormals;
%                   - MyData.Nb_Orientations=Nb_Orientations;
%                   - MyData.InlierThreshold=InlierThreshold;
%           - if flag_application==20
%                   - cf flag_application==0
%           - if flag_application==30
%                   - MyData.SetLeft: 2D points in the left image
%                   - MyData.SetRight: 2D points in the left image
%                   - MyData.CamCenter: the camera center [Cx Cy]
%                   - MyData.InlierThreshold: inlier threshold in pixels (image error) or radians (angular error)
%                   - MyData.BnB_fcenter: the center of the focal length interval
%                   - MyData.BnB_fradius: the radius of the focal length interval
%                   - MyData.BnB_Rotcenter: cube coordinates/parameters, 1x3
%                   - MyData.BnB_Rotradius=0.3 % pi
%   - AprioriSolution: can be [] TO DO
%       - Bound: nb of inliers obtained by any other method
%       - RotAngles: the associated rotation angles (in degrees)
%
%
%
% Outputs:
%   - R_best
%   - ListPostInfo: info about the cubes
%   - ListExtraInfo: struct
%       - Flag_BnBsuccess
%       - IndexFlag
%       - RotationAngles in degrees
%       - CardinalityResult: the nb of detected inliers
%       - NbPoints
%       - IndicesInliers:
%           - which format for lines/VPs? if at-most-one, then easy Nx1 where N is the nb of lines. If no constraint, then NxK where K is the nb of VPs??
%       - IndicesOutliers
%       - when flag_application==10 (lines/VPs)
%             ExtraOutput.dominant_directions=dominant_directions_temp;
%             ExtraOutput.List_ClusteredNormalsIndices=List_ClusteredNormalsIndices_temp;


FlagDepthOrBreadthSearch=1; %0-depth, 1-breadth
FlagMethod_BnB=0; % 0-zero-th order approximation, 1-first-order approximation, 10-MISOCP-zeroth-order, 11-??
Flag_WorkOnCommonInliersOutliers=0;


if FlagDepthOrBreadthSearch==1
    FlagAllowEarlyStop=1; %0-, 1-. only when FlagAllowEarlyStop==1
end

StartTimerFct=tic;

%%%%%%%%%%%%%%%  start - read the data  %%%%%%%%%%%%%%%%%%%
if flag_application==0 %0-pure rotation, 1-camera resectioning (find R,C given 2d and 3d points), 2-two view reconstruction (find R, C, 3D points Xi, given 2D points),  3-3D/3D registration (find R,T given 3D points), 4-uncalibrated SfM (get F (or K, R ,T) from a set of 2D correspondences)
    SpherePointsSetLeft=MyData.SetLeft;
    SpherePointsSetRight=MyData.SetRight;
    NbPoints=size(SpherePointsSetLeft,1);
    
elseif flag_application==1
    Data_2DPoints=MyData.Set2DPoints;
    Data_3DPoints=MyData.Set3DPoints;
    NbPoints=size(Data_2DPoints,1);
    
elseif flag_application==2
    SetPointsLeft=MyData.SetLeft;
    SetPointsRight=MyData.SetRight;
    Set3DPoints=MyData.Set3DPoints;
    NbPoints=size(SetPointsLeft,1);
    
elseif flag_application==10
    ListNormals=MyData.SetNormals;
    Nb_VPs=MyData.Nb_VPs;
    NbPoints=size(ListNormals,1);
    InlierThreshold=MyData.InlierThreshold;
    
elseif flag_application==12
    ListNormals=MyData.SetSurfNormals;
    Nb_Orientations=MyData.Nb_Orientations;
    NbPoints=size(ListNormals,1);
    InlierThreshold=MyData.InlierThreshold;
    
elseif flag_application==20
    SpherePointsSetLeft=MyData.SetLeft;
    SpherePointsSetRight=MyData.SetRight;
    NbPoints=[size(SpherePointsSetLeft,1) size(SpherePointsSetRight,1)];
    
    % compute kd-tree (to avoid computing N-to-N matching). Will be combined with rangesearch
    global kdtreeNS_GLOBAL;
    kdtreeNS_GLOBAL = KDTreeSearcher(SpherePointsSetRight,'Distance','euclidean')
    
elseif flag_application==30
    SetPointsLeft=MyData.SetLeft;
    SetPointsRight=MyData.SetRight;
    NbPoints=size(SetPointsLeft,1);
    
    fcenter_initial=MyData.BnB_fcenter;
    fradius_initial=MyData.BnB_fradius;
    Rotradius_initial=MyData.BnB_Rotradius; % pi; % the rotation ball
    
    InlierThreshold=MyData.InlierThreshold;
    
else
    flag_application
    error('wrong case or not done')
end
ListExtraInfo.NbPoints=NbPoints;
%%%%%%%%%%%%%%%  end - read the data  %%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%  START - Initialization of the cube list  %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_application~=30
    DimModel=3 % 3 for rotation
    DimRadius=1 % the radius of the rotation cube
else % i.e. R and focal
    DimModel=3+1 % 3 for rotation and 1 for focal length
    DimRadius=1+1 % the radius of the rotation cube + the radius of the focal length cube
end
Dim=DimModel+1+2+DimRadius; % 1 for the flag, 2 for the bounds

IndexFlag=DimModel+1; % e.g. usually 4
IndicesBounds=[IndexFlag+1 IndexFlag+2]  % e.g. usually [5 6]
if flag_application~=30
    if exist('Rotradius_initial')==0
        Rotradius_initial=[];
    end
    
    IndexModelRadius=IndicesBounds(2)+1; % e.g. usually 7
    ModelInitialization=zeros(1,3); % R: center of the rotation ball
    ModelRadius=Rotradius_initial;
    if flag_application==10 || flag_application==12 % for VPs/lines, and for surface normals (i.e plane clustering)
        FlagRotationBall=2; % cf values/meaning in Fn_BnB_CubeSubdivision
    else
        FlagRotationBall=1;%1; % cf values/meaning in Fn_BnB_CubeSubdivision
    end
    
else
    IndexModelRadius=IndicesBounds(2)+[1 2] % two radius: one for R and one for f
    %ModelInitialization=[zeros(1,3) fcenter_initial]; % R: center of the rotation ball, and Fcenter
    %ModelRadius=[pi fradius_initial]; % the rotation ball and the focal
    global TrueR_GLOBAL; % for debugging. defined in Fn_GenerateData_RandFocal
    if length(TrueR_GLOBAL)==0, error('undefined'), end
    TRUE_RotComponents=Fn_BnB_GetCubeFromRotation(TrueR_GLOBAL);
    ModelInitialization=[TRUE_RotComponents fcenter_initial]; % R: center of the rotation ball, and Fcenter
    ModelRadius=[Rotradius_initial fradius_initial]; % the rotation ball and the focal
    FlagRotationBall=3;%1; % cf values/meaning in Fn_BnB_CubeSubdivision
end
[ListCubes, RadiusCube, LevelSubdivision]=Fn_BnB_CubeInitialization(DimModel,ModelInitialization,ModelRadius);
% subdivision
FlagSubdividingMethod=0;
if FlagRotationBall==2
    % FlagRotationBall, input('are you sure to study the ball partially?')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%   END - Initialization of the cube list   %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:1
    k,
    [ListCubes, LevelSubdivision]=Fn_BnB_CubeSubdivision(ListCubes, LevelSubdivision,[],DimModel,DimRadius, FlagSubdividingMethod, FlagRotationBall);
end
size(ListCubes), LevelSubdivision

ListExtraInfo.IndexFlag=IndexFlag;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% START - TEST THE SYNTHESIZED MODEL %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Goal: test/compute the bounds for the synthesized model
flag_TestSynthSolution=0;
if flag_TestSynthSolution==1
    Fn_Test_SynthesizedSolution
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%  END - TEST THE SYNTHESIZED MODEL  %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% max outlier MinUpperBound=99999;
MaxLowerBound=-1;
R_best=[];
%Best_LowerBound=-1; % the lower bound of the cube providing MinUpperBound
% max outlier MinLowerBound=-1; % anything
MinUpperBound=77777; % anything
BestAssociated_UpperBound=77777; % anything
MaxUpperBound=9999; % anything
CardinalityResult=[];
Flag_BnBsuccess=1;

nth_iteration=0;
MAXiterations=20000;
MAXcubes=9999999999999;%200000;%1000000;
ListPostInfo=zeros(MAXiterations,6); % contains various information for post-processing and the paper. too long but will eb cut. 2 (for the bounds) +??
%ListPostInfo_DFS=zeros(MAXiterations,7); % contains various information for post-processing and the paper. too long but will eb cut. 2 (for the bounds) +??
ListPostInfo_DFS=zeros(MAXiterations,size(ListCubes,2)); % contains various information for post-processing and the paper. too long but will eb cut. 2 (for the bounds) +??
disp('START OPTIMIZATION LOOP ##############'), %input('start?')
while 1 % stop when no solutions, or small cube size ()
    
    close all;
    
    if FlagDepthOrBreadthSearch==1 || (FlagDepthOrBreadthSearch==0 && mod(nth_iteration,1000)==1) %0-depth, 1-breadth
        nth_iteration
    end
    %%%%%%% START - stop conditions %%%%%%%%
    if RadiusCube<deg2rad(0.5); disp('small cube so stop'); Flag_BnBsuccess=0; break; end % maybe not valid any more since the DFS allows to have several different radius
    if nth_iteration>MAXiterations; disp('max iterations so stop'); Flag_BnBsuccess=0; break; end
    %if length(IndicesValid)>MAXcubes; disp('max iterations so stop'); break; end % too many cubes, we could keep going but it will take time
    if size(ListCubes,1)>MAXcubes; disp('max cubes so stop'); Flag_BnBsuccess=0; break; end % too many cubes, we could keep going but it will take time
    
    if nth_iteration~=0 && FlagDepthOrBreadthSearch==0 && length(IndexBest_DFS)==0, nth_iteration, IndexBest_DFS, disp('all cubes with same upper bound'); Flag_BnBsuccess=1; break; end % NOT SURE
    %min outliers: if MinUpperBound==MinLowerBound; MinUpperBound, MinLowerBound, CardinalityResult=MinUpperBound; disp('same values so can be stopped'); break; end
    if MaxUpperBound==MaxLowerBound; MaxUpperBound, MaxLowerBound, CardinalityResult=MaxLowerBound; disp('same values so can be stopped'); break; end
    nth_iteration=nth_iteration+1;
    %%%%%%% END - stop conditions %%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%  START - octal subdivision  %%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if FlagDepthOrBreadthSearch==0 && nth_iteration==1 %0-depth-first search, 1-breadth-first search
        Index_Subdivision=[]; % if DFS and very first iteration, then sub-divide all
    elseif FlagDepthOrBreadthSearch==0  % if DFS and not the very first iteration (i.e. nth_iteration~=1), then sub-divide only the best cube
        Index_Subdivision=IndexBest_DFS;
    else % i.e. FlagDepthOrBreadthSearch==1
        Index_Subdivision=[]; % if BFS, then sub-divide all
    end
    ListCubes_Original=ListCubes; % for debug
    % if length(find(ListCubes_Original(:,IndexModelRadius)==0))~=0; error('some null radius'); end
    [ListCubes, LevelSubdivision]=Fn_BnB_CubeSubdivision(ListCubes, LevelSubdivision,Index_Subdivision,DimModel,DimRadius, FlagSubdividingMethod, FlagRotationBall);
    % ListCubes, input('temp')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%   END - octal subdivision   %%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if nth_iteration==1; disp('analyzing all the PRE-sub-divided cubes (i.e. the original K first sub-division). might take time...'), end
    
    if length(ListCubes)==0, error('the list is empty'), end % reasons: the cubes were outside the rotation ball, no solution, etc...
    IndicesValid=find(ListCubes(:,IndexFlag)==1);
    if length(IndicesValid)==0, size(ListCubes), error('not valid cubes'), end % reasons: ??
    if FlagDepthOrBreadthSearch==0%0-depth-first search, 1-breadth-first search
        %         [MinUpperBound_DFS,IndexBest_DFS]=min(ListCubes(IndicesValid,IndicesBounds(2))); % the minimum upper bound among the valid cubes
        %         IndicesToAnalyze=IndicesValid(IndexBest_DFS); % i.e. the cube index with the best cost
        %         MinUpperBound_DFS
        % get the valid cubes with unknown bounds, basically it is the new 8 sons
        IndicesToAnalyze=find(ListCubes(:,IndexFlag)==1 & ListCubes(:,IndicesBounds(1))==-1 & ListCubes(:,IndicesBounds(2))==-1);
        IndicesToAnalyze=IndicesToAnalyze';% must be horizontal to be used in a "for loop"
    elseif FlagDepthOrBreadthSearch==1
        IndicesToAnalyze=IndicesValid'; % must be horizontal to be used in a "for loop"
        
    else
        error('wrong case')
    end
    MaxLowerBound_CurrIter_ForDebug=-1; % initialization
    
    
    if Flag_WorkOnCommonInliersOutliers==1
        ArrayInliers_Debug=zeros(NbPoints,length(IndicesToAnalyze));
    end
    kth_cube_local=0;
    for MyIndex=IndicesToAnalyze % for each valid cube
        clear ExtraInputs_ForBnB; % for security
        
        CurrRadiusCube=ListCubes(MyIndex,IndexModelRadius);
        if flag_application~=30
            CurrRadiusCube_Rotation=CurrRadiusCube;
        else
            CurrRadiusCube_Rotation=CurrRadiusCube(1);
        end
        MyOffset=sqrt(3)*CurrRadiusCube_Rotation; % the offset is sqrt(3)*sigma, where sigma is the half-side length of the cube
        ExtraInputs_ForBnB.RadiusCube=CurrRadiusCube;
        ExtraInputs_ForBnB.InlierThreshold=InlierThreshold;
        ExtraInputs_ForBnB.MyOffset=MyOffset;
        
        kth_cube_local=kth_cube_local+1;
        if (FlagDepthOrBreadthSearch==1 && mod(kth_cube_local,1000)==0) || (FlagDepthOrBreadthSearch==0 && mod(kth_cube_local,10)==0) %0-depth, 1-breadth %  mod(kth_cube_local,1)==1
            sprintf('%d-th out of %d',kth_cube_local,length(IndicesToAnalyze))
            sprintf('nth_iteration=%d , MinUpperBound=%d , BestAssociated_UpperBound=%d , MaxLowerBound=%d , MaxUpperBound=%d, MaxLowerBound_CurrIter=%d',nth_iteration,MinUpperBound,BestAssociated_UpperBound,MaxLowerBound,MaxUpperBound,MaxLowerBound_CurrIter_ForDebug)
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%  START - COMPUTE THE BOUNDS FOR THE CURRENT CUBE  %%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %get the boundsExtraInputs_ForBnB
        RotComponents=ListCubes(MyIndex,1:3); % the cube center
        %         %%%%%%%%%%  START - TEST THE TRUE SOLUTION %%%%%%%%%%%
        %         RotComponents=Fn_BnB_GetCubeFromRotation(TrueR)
        %         CurrRadiusCube=0.001; % the cube radius
        %           MyOffset=sqrt(3)*CurrRadiusCube; % the offset is sqrt(3)*sigma, where sigma is the half-side length of the cube
        %         ExtraInputs_ForBnB.RadiusCube=CurrRadiusCube;
        %         ExtraInputs_ForBnB.InlierThreshold=InlierThreshold;
        %         ExtraInputs_ForBnB.MyOffset=MyOffset;
        %         %%%%%%%%%% END - TEST THE TRUE SOLUTION %%%%%%%%%%%
        
        if flag_application==0 || flag_application==20% 0-pure rotation, 1-camera resectioning (find R,C given 2d and 3d points), 2-two view reconstruction (find R, C, 3D points Xi, given 2D points),  3-3D/3D registration (find R,T given 3D points), 4-uncalibrated SfM (get F (or K, R ,T) from a set of 2D correspondences), 5-uncalibrated Hinfinity: find R and focal from a set of 2D correspondences, 10-pure rotation (lines/VPs), 20-pure rotation and unknown pairing
            if flag_application==0
                Flag_PointsOrCorrespondences=1; % Flag_PointsOrCorrespondences=0 for points, 1 for correspondences
            elseif flag_application==20
                Flag_PointsOrCorrespondences=0;
            end
            
            [MyLowerBoundInliers,MyUpperBoundInliers,Rbar_BnB,IndicesInliers]=Fn_BnB_ComputeBounds_PureRotation(RotComponents,SpherePointsSetLeft,SpherePointsSetRight,Flag_PointsOrCorrespondences,ExtraInputs_ForBnB,FlagMethod_BnB);
        
        elseif flag_application==1
            [MyLowerBoundOutliers,MyUpperBoundOutliers,Rbar_BnB,MyC_BnB]=Fn_BnB_ComputeBounds_CameraResectioning(RotComponents,Data_2DPoints,Data_3DPoints,ExtraInputs_ForBnB,FlagMethod_BnB);
        
        elseif flag_application==2
            [MyLowerBoundOutliers,MyUpperBoundOutliers,Rbar_BnB,MyC_BnB,My3DPoints_BnB]=Fn_BnB_ComputeBounds_TwoViewReconstruction(RotComponents,SetPointsLeft,SetPointsRight,Set3DPoints,ExtraInputs_ForBnB,FlagMethod_BnB);
            
        elseif flag_application==10
            FlagDistanceDefinition=MyData.FlagDistanceDefinition;
            [MyLowerBoundInliers,MyUpperBoundInliers,Rbar_BnB,IndicesInliers]=Fn_BnB_ComputeBounds_LinesVPs(RotComponents,ListNormals,Nb_VPs,ExtraInputs_ForBnB, FlagMethod_BnB,FlagDistanceDefinition);
        
        elseif flag_application==12
            [MyLowerBoundInliers,MyUpperBoundInliers,Rbar_BnB,IndicesInliers,IndicesOutliers]=Fn_BnB_ComputeBounds_SurfNormals(RotComponents,ListNormals,Nb_Orientations,ExtraInputs_ForBnB, FlagMethod_BnB);
        
        elseif flag_application==30
            
            Flag_BNBMETHODFORFOCALLENGTH=2; % 0-analytical approach, 1- geometric approach (object (circles, curves,etc..) intersection), 2-by Interval Analysis (recommended for the moment)
            
            if Flag_BNBMETHODFORFOCALLENGTH==0
                % analytical approach??
                [MyLowerBoundInliers,MyUpperBoundInliers,Rbar_BnB,IndicesInliers]=Fn_BnB_ComputeBounds_RotAndFocal(RotComponents,SpherePointsSetLeft,SpherePointsSetRight,Flag_PointsOrCorrespondences,ExtraInputs_ForBnB,FlagMethod_BnB);
                
            elseif Flag_BNBMETHODFORFOCALLENGTH==1
                
                % geometric approach by Richard Hartley
                MyFocalCenter=ListCubes(MyIndex,4);
                % MyFocalCenter, ExtraInputs_ForBnB
                
                %                         %%%%%%%%%%  START - TEST THE TRUE SOLUTION %%%%%%%%%%%
                %                         global TrueR_GLOBAL; % for debugging. defined in Fn_GenerateData_RandFocal
                %                         RotComponents=Fn_BnB_GetCubeFromRotation(TrueR_GLOBAL)
                %                         global f_true; % defined in Fn_GenerateData_RandFocal
                %                         MyFocalCenter=f_true
                %                         ExtraInputs_ForBnB.RadiusCube=[0.00001 0.00001]; %[0.0001 50];%
                %                         ExtraInputs_ForBnB.InlierThreshold=InlierThreshold;
                %                         ExtraInputs_ForBnB.MyOffset=sqrt(3)*ExtraInputs_ForBnB.RadiusCube(1); %0.00001; % the offset is sqrt(3)*sigma, where sigma is the half-side length of the cube
                %                         input('using true results')
                %                         %%%%%%%%%% END - TEST THE TRUE SOLUTION %%%%%%%%%%%
                
                
                [MyLowerBoundInliers,MyUpperBoundInliers,IndicesInliers]=Fn_BnB_ComputeBounds_RotAndFocal_ByGeoApproach(RotComponents,MyFocalCenter,SetPointsLeft,SetPointsRight,ExtraInputs_ForBnB);
                
            elseif Flag_BNBMETHODFORFOCALLENGTH==2
                
                asdasdas
                % write in xml file
                CamCenter=MyData.CamCenter% the given/true camera center [Cx Cy]
                NbTotalPoints=size(SetPointsLeft,1);
                SetPointsLeft_Centered=SetPointsLeft-repmat(CamCenter,[NbTotalPoints 1])
                SetPointsRight_Centered=SetPointsRight-repmat(CamCenter,[NbTotalPoints 1])
                theta=MyData.ExtraInfo.theta;  phi=MyData.ExtraInfo.phi;  alpha=MyData.ExtraInfo.alpha;  focal=MyData.BnB_fcenter; %focal=TrueCameraParams.f;
                InlierBinaryMask=zeros(1,NbTotalPoints); InlierBinaryMask(1:MyData.ExtraInfo.NbInliers)=1;
                data_filenameXML='C:\Users\jebazin\Desktop\SVN_JCresearch\SVN_JCresearch\JCresearch\trunk\JC_CODE\CVPR2014_FocalLength\TestJC\test.xml'
                NbMinInliers=floor(MyData.ExtraInfo.NbInliers*0.7)
                Fn_WriteXML_data(data_filenameXML,SetPointsLeft_Centered,SetPointsRight_Centered,theta,phi,alpha,focal,InlierThreshold,InlierBinaryMask,NbMinInliers)
                
                % call BnB
                
                
                
                clear ExtraInputs;
                ExtraInputs.my_dir_cpp='';
                ExtraInputs.my_execname_cpp='';
                ExtraInputs.my_filename_inputdata=data_filenameXML;
                ExtraInputs.my_filename_outputdata='C:\Users\jebazin\Desktop\SVN_JCresearch\SVN_JCresearch\JCresearch\trunk\JC_CODE\CVPR2014_FocalLength\TestJC\output_data.txt';%'';
                
                AprioriSolution=[];
                MyData=[];
                
                [R,flength,my_angles_FromExternalProg,NbInliers,ExecTime]=Fn_RunBnB_FocalAndRotation_ByIA_CPP(MyData,InlierThreshold,AprioriSolution,ExtraInputs)
                
                MyLowerBoundInliers=NbInliers;
                MyUpperBoundInliers=NbInliers; % NOT SURE
                
            else
                error('wrong method case');
            end
            % MyLowerBoundInliers, MyUpperBoundInliers, input('bound results of the current cube')
            
        else
            error('wrong case or not done')
        end
        
        % save the bounds
        ListCubes(MyIndex,IndicesBounds)=[MyLowerBoundInliers MyUpperBoundInliers];
        
        if MyLowerBoundInliers>MaxLowerBound_CurrIter_ForDebug
            MaxLowerBound_CurrIter_ForDebug=MyLowerBoundInliers;
        end
        
        if Flag_WorkOnCommonInliersOutliers==1
            
            LocalBinaryInliers=zeros(NbPoints,1);
            LocalBinaryInliers(IndicesInliers(:,1))=1;
            LocalBinaryOutliers=zeros(NbPoints,1);
            LocalBinaryOutliers(IndicesOutliers)=1;
            if kth_cube_local==1
                CommonBinaryInliers=LocalBinaryInliers;
                CommonBinaryOutliers=LocalBinaryOutliers;
            else
                %CommonBinaryInliers=(CommonBinaryInliers==1 && CommonBinaryInliers==LocalBinaryInliers);
                % test: a=[1 3 4 6 1 82 23]', b=[8 2 1 9 3 1 9]', min(a,b)
                CommonBinaryInliers=min(CommonBinaryInliers,LocalBinaryInliers);   % definitive inliers are 1, unsure are 0
                CommonBinaryOutliers=min(CommonBinaryOutliers,LocalBinaryOutliers);   % definitive outliers are 1, unsure are 0
                
                
                %                 figure('name','CommonBinaryInliers');
                %                 plot(CommonBinaryInliers,'.b');
                %                 %hold on;
                %                 % plot(LocalBinaryInliers,'.r');
                %                 %hold off;
                %                 title([int2str(sum(CommonBinaryInliers)) 'common inliers out of ' int2str(length(CommonBinaryInliers)) 'data']);
                %                 sprintf('%d-th out of %d cubes',kth_cube_local,length(IndicesToAnalyze))
                %                 nth_iteration
                %                 input(''); close;
                
            end
            ArrayInliers_Debug(:,kth_cube_local)=LocalBinaryInliers;
            %                 %figure('name','CommonBinaryInliers'); imshow(repmat(CommonBinaryInliers,1,100)); input(''); close;
            %                 figure('name','CommonBinaryInliers'); plot(CommonBinaryInliers,'.b');
            %                 title([int2str(sum(CommonBinaryInliers)) 'common inliers out of ' int2str(length(CommonBinaryInliers)) 'data']);
            %                 sprintf('%d-th out of %d cubes',kth_cube_local,length(IndicesToAnalyze))
            %                 nth_iteration
            %                 input(''); close;
        end
        %            MyLowerBoundInliers, MyUpperBoundInliers, Rbar_BnB, input('result current cube')
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%   END - COMPUTE THE BOUNDS FOR THE CURRENT CUBE   %%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        
        %         if MyLowerBoundOutliers>MinUpperBound
        %             %disp('discard')
        %              ListCubes(MyIndex,IndexFlag)=0;% the cube has bad bounds so discard it (make it invalid)
        %         else
        %             if MyUpperBoundOutliers<MinUpperBound
        %                 %disp('better')
        %                 MinUpperBound=MyUpperBoundOutliers;
        %                 R_best=Rbar;
        %                 Best_LowerBound=MyLowerBoundOutliers; % save also the lb (of the cube providing MinUpperBound) for post-processing
        %                 if flag_application==1 %0-pure rotation, 1-camera resectioning (find R,C given 2d and 3d points), 2-two view reconstruction (find R, C, 3D points Xi, given 2D points),  3-3D/3D registration (find R,T given 3D points), 4-uncalibrated SfM (get F (or K, R ,T) from a set of 2D correspondences)
        %                     C_best=MyC;
        %                 elseif flag_application==2
        %                     List3DPoints_best=XXX;
        %                 end
        %             else % i.e. MyUpperBoundOutliers==MinUpperBound
        %                 % nothing to do
        %             end
        %         end
        % %         MinUpperBound, Best_LowerBound
        % %         MyLowerBoundOutliers,MyUpperBoundOutliers,
        % %         input('bounds of the current cube')
        
        
        
        %         if FlagDepthOrBreadthSearch==1 && FlagAllowEarlyStop==1
        %             % MyLowerBoundInliers MyUpperBoundInliers
        %             if MyLowerBoundInliers==MaxUpperBound
        %                   IndicesToAnalyze=IndicesToAnalyze(1:kth_cube_local,:); % because later there is a test "if length(IndicesToAnalyze)~=kth_cube_local then error"
        %                 break; % stop studying the cubes. early stop
        %             end
        %         end
        
    end % end of for each valid cube
    
    if length(IndicesToAnalyze)~=kth_cube_local % probably useless
        length(IndicesToAnalyze), kth_cube_local
        error('is that possible?')
    end
    %%%%%%%%%%%%%%%
    %     [MinUpperBound,IndexBest_temp]=min(ListCubes(IndicesValid,IndicesBounds(2))); % the minimum upper bound among the (valid) cubes
    %     IndexBest_temp=IndicesValid(IndexBest_temp);
    %     BestAssociated_LowerBound=ListCubes(IndexBest_temp,IndicesBounds(1));
    %     [MaxLowerBound,IndexBest_temp2]=max(ListCubes(IndicesValid,IndicesBounds(1))); % the maximum lower bound among the (valid) cubes
    %     [MinLowerBound,IndexBest_temp2]=min(ListCubes(IndicesValid,IndicesBounds(1))); % the maximum lower bound among the (valid) cubes
    %
    
    [MaxLowerBound,IndexBest_temp]=max(ListCubes(IndicesValid,IndicesBounds(1))); % the maximum lower bound among the (valid) cubes
    IndexBest_temp=IndicesValid(IndexBest_temp);
    BestAssociated_UpperBound=ListCubes(IndexBest_temp,IndicesBounds(2));
    [MinUpperBound,IndexBest_temp2]=min(ListCubes(IndicesValid,IndicesBounds(2))); % the maximum lower bound among the (valid) cubes
    [MaxUpperBound,IndexBest_temp2]=max(ListCubes(IndicesValid,IndicesBounds(2))); % the maximum lower bound among the (valid) cubes
    % MaxLowerBound, MinUpperBound, MaxUpperBound, nth_iteration, input('check')
    
    % extra model
    %IndexBest_temp
    R_best=Fn_BnB_GetRotationFromCube(ListCubes(IndexBest_temp,1:3));
    RCubeCenter_best=ListCubes(IndexBest_temp,1:3);
    if flag_application==30
        MyFocalCenter_best=ListCubes(IndexBest_temp,4);
    end
    % R_best, ListCubes(IndexBest_temp,:), ListCubes(IndexBest_temp,IndicesBounds), input('check2')
    
    
    if length(AprioriSolution)~=0
        %MaxLowerBound, AprioriSolution.Bound
        MaxLowerBound_prev=MaxLowerBound;
        MaxLowerBound=max(MaxLowerBound,AprioriSolution.Bound);
        if MaxLowerBound>MaxLowerBound_prev
            R_best_angles=AprioriSolution.RotAngles
            R_best=Fn_Rotation_Matrix_matrix_from_values_Vect(R_best_angles, 1)
            %         [my_vect, my_angle]=Fn_Rotation_Matrix_VectorAndAngle_from_matrix(R_best, 0)
            %         RCubeCenter_best=my_vect/my_angle;
            %         Fn_BnB_GetRotationFromCube(RCubeCenter_best), R_best, input('check conversion')
            RCubeCenter_best=Fn_BnB_GetCubeFromRotation(R_best)
            MyAngles_ByBounds=R_best_angles;% for generality
            
            flag_debug=0;
            if flag_debug==1
                RotComponents_local=RCubeCenter_best;
                ExtraInputs_ForBnB_local=ExtraInputs_ForBnB;
                ExtraInputs_ForBnB_local.RadiusCube=0;
                ExtraInputs_ForBnB_local.MyOffset=0;
                
                [MyLowerBoundInliers_local,MyUpperBoundInliers_local,Rbar_BnB_local,IndicesInliers_local]=Fn_BnB_ComputeBounds_LinesVPs(RotComponents_local,ListNormals,Nb_VPs,ExtraInputs_ForBnB_local, FlagMethod_BnB,FlagDistanceDefinition);
                if MyLowerBoundInliers_local~=MyUpperBoundInliers_local, MyLowerBoundInliers_local, MyUpperBoundInliers_local, error('since radius is 0, then should be same'); end
                if MyLowerBoundInliers_local~=MaxLowerBound, MyLowerBoundInliers_local, MaxLowerBound, error('should be same'); , end
                
                MyLowerBoundInliers_local,MyUpperBoundInliers_local, length(IndicesInliers_local), MaxLowerBound, input('check3')
            end
        end
        %    sdfsdf
    end
    
    indices=find(ListCubes(:,IndicesBounds(2))<MaxLowerBound); % i.e. the cubes such that MyLowerBoundOutliers>MinUpperBound
    % min outliers: indices=find(ListCubes(:,IndicesBounds(1))>MinUpperBound); % i.e. the cubes such that MyLowerBoundOutliers>MinUpperBound
    ListCubes(indices,IndexFlag)=0;% the cube has bad bounds so discard it (make it invalid)
    %     length(indices), length(IndicesValid)
    %     input('continue?')
    
    %     for k=1:size(ListCubes,1)
    %         MyLowerBoundOutliers=ListCubes(k,IndicesBounds(1));
    %         MyUpperBoundOutliers=ListCubes(k,IndicesBounds(2));
    %
    %             if MyLowerBoundOutliers>MinUpperBound
    %             %disp('discard')
    %              ListCubes(k,IndexFlag)=0;% the cube has bad bounds so discard it (make it invalid)
    %         else
    %             if MyUpperBoundOutliers<MinUpperBound
    %                 %disp('better')
    %                 MinUpperBound=MyUpperBoundOutliers;
    %                 R_best=Rbar;
    %                 Best_LowerBound=MyLowerBoundOutliers; % save also the lb (of the cube providing MinUpperBound) for post-processing
    %                 if flag_application==1 %0-pure rotation, 1-camera resectioning (find R,C given 2d and 3d points), 2-two view reconstruction (find R, C, 3D points Xi, given 2D points),  3-3D/3D registration (find R,T given 3D points), 4-uncalibrated SfM (get F (or K, R ,T) from a set of 2D correspondences)
    %                     C_best=MyC;
    %                 elseif flag_application==2
    %                     List3DPoints_best=XXX;
    %                 end
    %             else % i.e. MyUpperBoundOutliers==MinUpperBound
    %                 % nothing to do
    %             end
    %         end
    % %         MinUpperBound, Best_LowerBound
    % %         MyLowerBoundOutliers,MyUpperBoundOutliers,
    % %         input('bounds of the current cube')
    %%%%%%%%%%%%%%%%
    
    
    
    % if many cubes with the same ub/lb, then keep only one
    ListCubes=Fn_BnB_RemoveCubesWithSameBounds(ListCubes);
    
    % compute the volume
    if nth_iteration==1
        if FlagRotationBall~=3
            OriginalVolume=sum( 2*(ListCubes(:,IndexFlag).*ListCubes(:,IndexModelRadius)).^3); % sum of cube volumes % 4/3*pi*pi^3; % we could also use a cube of radius pi
        else
            OriginalVolume=sum( 2*(ListCubes(:,IndexFlag).*sum(ListCubes(:,IndexModelRadius),2)).^4); % sum of cube volumes % 4/3*pi*pi^3; % we could also use a cube of radius pi
        end
    end
    if FlagRotationBall~=3
        CurrVolume=sum( 2*(ListCubes(:,IndexFlag).*ListCubes(:,IndexModelRadius)).^3); % sum of cube volumes
    else
        CurrVolume=sum( 2*(ListCubes(:,IndexFlag).*sum(ListCubes(:,IndexModelRadius),2)).^4); % sum of cube volumes
    end
    PercentageVolume=CurrVolume/OriginalVolume*100;
    
    % For post-processing: save the max lower bound and the min upper bound. They should converge.
    % what should we save here? the bounds of the cube that gives the minimum upper bound
    %MaxLowerBound=-1;
    NbValidCubes_local=sum(ListCubes(:,IndexFlag));
    ListPostInfo(nth_iteration,1:6)=[MaxUpperBound MaxLowerBound BestAssociated_UpperBound CurrVolume PercentageVolume NbValidCubes_local];
    % min outliers ListPostInfo(nth_iteration,1:5)=[MinLowerBound MinUpperBound BestAssociated_LowerBound CurrVolume PercentageVolume];
    
    
    if Flag_WorkOnCommonInliersOutliers==1
        figure('name','CommonBinaryInliers');
        subplot(2,1,1); plot(CommonBinaryInliers,'.b');
        title([int2str(sum(CommonBinaryInliers)) 'common inliers out of ' int2str(length(CommonBinaryInliers)) 'data']);
        subplot(2,1,2); plot(CommonBinaryOutliers,'.b');
        title([int2str(sum(CommonBinaryOutliers)) 'common outliers out of ' int2str(length(CommonBinaryOutliers)) 'data']);
        sprintf('%d-th out of %d cubes',kth_cube_local,length(IndicesToAnalyze))
        nth_iteration
        input('CommonBinaryInliers'); close;
        
        
        figure('name','ArrayInliers_Debug'); imshow(ArrayInliers_Debug);
        size(ArrayInliers_Debug)
        IndicesLocal=find(sum(ArrayInliers_Debug,2)==size(ArrayInliers_Debug,2))
        sum(CommonBinaryInliers)
        sprintf('Nb Common inliers: min is %d and max is %d', min(sum(ArrayInliers_Debug,2)),max(sum(ArrayInliers_Debug,2)))
        
        
        CommonBinaryInliersIndices=find(CommonBinaryInliers);
        sum(ArrayInliers_Debug(CommonBinaryInliersIndices,:),2)'
        if length(IndicesLocal)~=sum(CommonBinaryInliers)
            length(IndicesLocal), sum(CommonBinaryInliers)
            error('should be same')
        end
        input('ArrayInliers_Debug'); close;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%  START - BEST CUBE SELECTION FOR DFS  %%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if FlagDepthOrBreadthSearch==0 %0-depth-first search, 1-breadth-first search
        MinRadiusSelection=10^-7;%10^-3; % sometimes, always the same cube is selected and the radius becomes so small, but because of inaccuracies the bounds do not converge
        
        flag_DFS_SelectionMethod=2; % 0-the min upper bound, 1-the min lower bound, 2-the max upper bound, 3-the max lower bound
        IndexBest_DFS=Fn_BnB_SelectBestCube_ForDFS(ListCubes,IndicesBounds,IndexFlag,IndexModelRadius,MinRadiusSelection,flag_DFS_SelectionMethod);
        
        if length(IndexBest_DFS)==0
            % it means for example all the cubes have the same bound. cf definition in Fn_BnB_SelectBestCube_ForDFS
            ListCubes(:,IndicesBounds), IndexBest_DFS
        else
            % security check: has this cube already been selected:
            ListPostInfo_DFS(nth_iteration,:)=ListCubes(IndexBest_DFS,:);
        end
        if nth_iteration~=1 % degeneracy at the first iteration, and no need to search actually because empty
            %indices=find(isqual(ListPostInfo_DFS(1:nth_iteration-1,[1 2 3 ]), ListCubes(IndexBest_DFS,[1 2 3 7])));
            %             indices=find(sum(abs( ListPostInfo_DFS(1:nth_iteration-1,[1 2 3 IndexModelRadius]) - repmat(ListPostInfo_DFS(nth_iteration,[1 2 3 IndexModelRadius]),nth_iteration-1,1) ),2)<(MinRadiusSelection/2)); % i.e. a little bit less than MinRadiusSelection
            %             if length(indices)~=0;
            %                 ListPostInfo_DFS(indices,:),  ListCubes(IndexBest_DFS,:)
            %                 ListPostInfo_DFS(indices,IndexModelRadius),  ListCubes(IndexBest_DFS,IndexModelRadius)
            %                 error('cube already selected by DFS');
            %             end
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%   END - BEST CUBE SELECTION FOR DFS   %%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%% START - display bounds of the current boxes %%%%%%
    % display "sometimes" the results
    if (FlagDepthOrBreadthSearch==0 && mod(nth_iteration,200)==0) || FlagDepthOrBreadthSearch==1 %0-depth-first search, 1-breadth-first search
        Fn_BnB_DisplayBoundsOfValidCubes(ListCubes,IndexFlag,IndicesBounds)
        % min outliers:  sprintf('nth_iteration=%d , MaxLowerBound=%d , BestAssociated_LowerBound=%d , MinUpperBound=%d , MinLowerBound=%d',nth_iteration,MaxLowerBound,BestAssociated_LowerBound,MinUpperBound,MinLowerBound)
        sprintf('nth_iteration=%d , MinUpperBound=%d , BestAssociated_UpperBound=%d , MaxLowerBound=%d , MaxUpperBound=%d, MaxLowerBound_CurrIter=%d',nth_iteration,MinUpperBound,BestAssociated_UpperBound,MaxLowerBound,MaxUpperBound,MaxLowerBound_CurrIter_ForDebug)
        %nth_iteration, MaxLowerBound, BestAssociated_LowerBound, MinUpperBound, MinLowerBound
        PercentageVolume
        
        if FlagDepthOrBreadthSearch==1
            sprintf(['rotation cube size=' num2str(ListCubes(1,IndexModelRadius))])
            
            %                %ListCubes(1:5,IndexModelRadius)
            %                figure; plot(sort(ListCubes(:,IndexModelRadius)),'.b'); input('radius')
            
            ExtraInputs_ForBnB
            %           - RadiusCube: the half size of the cube
            %           - InlierThreshold: the original inlier threshold (without offset)
            %           - MyOffset
            %input('radius and offset')
        end
        
        IndicesValid=find(ListCubes(:,IndexFlag)==1);
        %     ExtraInfo_DisplayFigure=Fn_Build_ListInfo_DisplayFigure('distribution of radius','radius','distribution');
        %     [my_hist,temp]=Fn_DisplayHist_UniqueBins(ListCubes(IndicesValid,IndexModelRadius),[],1,ExtraInfo_DisplayFigure)
        
        
        % input('bounds after the whole iteration (on all the cubes), ok?')
    end
    %%%%%%% END - display bounds of the current boxes %%%%%%
    
end % end of while

ListExtraInfo.Flag_BnBsuccess=Flag_BnBsuccess;
if Flag_BnBsuccess==0
    R_best=[]; ListPostInfo=[]; % could be modified
    return;
end

ListExtraInfo.CardinalityResult=CardinalityResult;

% the same as the previously displayed window
if Flag_WorkOnCommonInliersOutliers==1
    figure('name','CommonBinaryInliers');
    subplot(2,1,1); plot(CommonBinaryInliers,'.b');
    title([int2str(sum(CommonBinaryInliers)) 'common inliers out of ' int2str(length(CommonBinaryInliers)) 'data']);
    subplot(2,1,2); plot(CommonBinaryOutliers,'.b');
    title([int2str(sum(CommonBinaryOutliers)) 'common outliers out of ' int2str(length(CommonBinaryOutliers)) 'data']);
    input(''); close;
end

toc(StartTimerFct)

% could be improved, i.e. without re-computing the results
if flag_application==0 || flag_application==20 %cf definition at the top of this file
    [MyLowerBoundInliers,MyUpperBoundInliers,Rbar_BnB,IndicesInliers]=Fn_BnB_ComputeBounds_PureRotation(RCubeCenter_best,SpherePointsSetLeft,SpherePointsSetRight,Flag_PointsOrCorrespondences,ExtraInputs_ForBnB,FlagMethod_BnB);

elseif flag_application==1
    [MyLowerBoundOutliers,MyUpperBoundOutliers,Rbar_BnB,MyC_BnB]=Fn_BnB_ComputeBounds_CameraResectioning(RCubeCenter_best,Data_2DPoints,Data_3DPoints,ExtraInputs_ForBnB,FlagMethod_BnB);

elseif flag_application==2
    [MyLowerBoundOutliers,MyUpperBoundOutliers,Rbar_BnB,MyC_BnB,My3DPoints_BnB]=Fn_BnB_ComputeBounds_TwoViewReconstruction(RCubeCenter_best,SetPointsLeft,SetPointsRight,Set3DPoints,ExtraInputs_ForBnB,FlagMethod_BnB);

elseif flag_application==10
    [MyLowerBoundInliers,MyUpperBoundInliers,Rbar_BnB,IndicesInliers]=Fn_BnB_ComputeBounds_LinesVPs(RCubeCenter_best,ListNormals,Nb_VPs,ExtraInputs_ForBnB, FlagMethod_BnB,FlagDistanceDefinition);
    
    %%%%%%% start - compute extra outputs %%%%%%%%
    MyAngles_ByBounds=Fn_Rotation_Matrix_values_from_matrix_Vect(R_best,1);
    [dominant_directions, List_ClusteredNormals, List_ClusteredNormalsIndices,ListNbClusteredIndices_temp, UnclusteredIndices_temp]=...
        Fn_GetLineClustering_GivenRotation(ListNormals,MyAngles_ByBounds,InlierThreshold, Nb_VPs,1,FlagDistanceDefinition);
    
    ListExtraInfo.dominant_directions=dominant_directions;
    ListExtraInfo.List_ClusteredNormalsIndices=List_ClusteredNormalsIndices;
    ListExtraInfo.List_ClusteredNormals=List_ClusteredNormals;
    ListExtraInfo.RotationAngles=MyAngles_ByBounds;
    %%%%%%%  end - compute extra outputs  %%%%%%%%
    
elseif flag_application==30
    [MyLowerBoundInliers,MyUpperBoundInliers,IndicesInliers]=Fn_BnB_ComputeBounds_RotAndFocal_ByGeoApproach(RCubeCenter_best,MyFocalCenter_best,SetPointsLeft,SetPointsRight,ExtraInputs_ForBnB);
    
elseif flag_application==12
    [MyLowerBoundInliers,MyUpperBoundInliers,Rbar_BnB,IndicesInliers,IndicesOutliers]=Fn_BnB_ComputeBounds_SurfNormals(RCubeCenter_best,ListNormals,Nb_Orientations,ExtraInputs_ForBnB, FlagMethod_BnB);
    
else
    error('wrong case or not done')
end
% display
sprintf('nb of inliers= %d, Nb of input data=%d', length(IndicesInliers), ListExtraInfo.NbPoints)
if flag_application==10
    ListNbClusteredIndices_temp, R_best
end

ListExtraInfo.IndicesInliers=IndicesInliers;
if (flag_application==0 || flag_application==20) && Flag_PointsOrCorrespondences==1
    ListExtraInfo.IndicesOutliers=setdiff(1:ListExtraInfo.NbPoints,ListExtraInfo.IndicesInliers);
elseif (flag_application==0 || flag_application==20) && Flag_PointsOrCorrespondences==0
    ListExtraInfo.IndicesOutliers=[]; % not sure what to put. [] looks ok
end
if length(ListExtraInfo.IndicesInliers)~=CardinalityResult, size(ListExtraInfo.IndicesInliers), length(ListExtraInfo.IndicesInliers),CardinalityResult,MaxUpperBound, MaxLowerBound, error('sdfsdf'),end
if MyLowerBoundInliers~=MyUpperBoundInliers, MyLowerBoundInliers, MyUpperBoundInliers, error('Error: should be same'),end
if MyLowerBoundInliers~=CardinalityResult, MyLowerBoundInliers, CardinalityResult, error('Error: should be same'),end


