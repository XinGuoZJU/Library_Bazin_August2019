
% minimize nb of outliers: function [MyLowerBoundOutliers,MyUpperBoundOutliers,R_solution]=Fn_BnB_ComputeBounds_LinesVPs(RotCubeCenter,list_normals,Nb_VPs,ExtraInputs_ForBnB, FlagMethod_BnB)
function [MyLowerBoundInliers,MyUpperBoundInliers,R_solution,IndicesInliers]=Fn_BnB_ComputeBounds_LinesVPs(RotCubeCenter,list_normals,Nb_VPs,ExtraInputs_ForBnB, FlagMethod_BnB, FlagDistanceDefinition)

% Comment: for the line clustering application, find the best solution
% similar to Fn_BnB_ComputeBounds_PureRotation

% inputs:
%   - RotCubeCenter: the cube center (Cx,Cy,Cz) 1x3 or 3x1
%   - list_normals: contains the unclustered normals Nx3
%   - Nb_VPs
%   - ExtraInputs_ForBnB: struct
%           - RadiusCube: the half size of the cube
%           - InlierThreshold: the original inlier threshold (without offset)
%           - MyOffset: the offset to take into account the distance/difference of the "cube center"
%                   -  sqrt(3)*sigma, where sigma is the half-side length of the cube
%   - FlagMethod_BnB:
%       - 0-zero-th order approximation, 1-first-order approximation
%       - 10-MISOCP-zeroth-order, 11-MISOCP-first-order
%   - FlagDistanceDefinition: cf Fn_ComputeVPDist
%
%Kmatrix_ForDistance=[]; % cf Fn_ComputeVPDist

% minimize nb of outliers:    - MyLowerBoundOutliers,MyUpperBoundOutliers: the lower and upper bounds of the nb of outliers, for the current Rbar and the current offset (related to the cube size)

% outputs:
%   - MyLowerBoundInliers,MyUpperBoundInliers: the lower and upper bounds of the nb of inliers, for the current Rbar and the current offset (related to the cube size)
%   - R_solution: a refined solution. same as the input RotCubeCenter if FlagMethod_BnB=0
%   - IndicesInliers: of definite inliers?

if nargin~=6; nargin, error('wrong nb of inputs'); end
if nargout~=4; nargout, error('wrong nb of outputs'); end
if length(RotCubeCenter)~=3; RotCubeCenter, error('wrong size'); end



Rbar=Fn_BnB_GetRotationFromCube(RotCubeCenter);
sigma=ExtraInputs_ForBnB.RadiusCube;
InlierThreshold=ExtraInputs_ForBnB.InlierThreshold;
MyOffset=ExtraInputs_ForBnB.MyOffset;

% %% start - test - interval analyzis
% for k=1:3
% RotCubeCenter_box(k)=midrad(RotCubeCenter(k),sigma);
% end
% Rbar_local=Fn_BnB_GetRotationFromCube(RotCubeCenter);
% Rbar_box=Fn_BnB_GetRotationFromCube(RotCubeCenter_box);
% %RotCubeCenter, RotCubeCenter_box, Rbar_local, Rbar_box
% % cf fucntion Fn_ComputeQuality_ForBnBinterval: TotalCost=Fn_ComputeQuality_ForBnBinterval(Rbox, list_normals, ThreshInlier, Nb_VPs)
% %% end - test

%%%%%%%%%%% START - COMPUTE THE VPS %%%%%%%%%%%%%
list_VPs=Fn_GetVPs_GivenRotationMat(Rbar);
%%%%%%%%%%%  END - COMPUTE THE VPS  %%%%%%%%%%%%%

Flag_LineToAtMostOneVP=0; %0-a line can be (un)-luckily clustered to several VPs, 1-force that a line belongs to at most one VP

if FlagMethod_BnB==0
    
    %FlagMethod_local=1; %0-simple counting (faster and recommended), 1-by integer programming (slow but for additional test, asked by Yongduek)
    for FlagMethod_local=0%1%%0[0 1]
        flag_ComputeExtraInfo=0;
        if flag_ComputeExtraInfo==0
               MyUpperBoundOutliers=[]; MyLowerBoundOutliers=[];
        end
        
        %FlagMethod_local
        if FlagMethod_local==0
            
            % compute the bounds by simply counting the number of outliers
            MyUpperBoundOutliers=0; MyLowerBoundOutliers=0;
            MyUpperBoundInliers=0; MyLowerBoundInliers=0;
            
            NbLines=size(list_normals,1);
            Indices_LbOutliers=zeros(1,NbLines); % for post-processing. 1 means outliers for the lower bound
            Indices_LbInliers=zeros(1,NbLines); % for post-processing. 1 means inliers
            
            flag_submethod=2; % 0-slow version but clear to understand, 1-fast version, 2- from other function
            
            if flag_submethod==0 % 0-slow version
            for nth_line=1:NbLines
                % given Rbar (Rcenter), compute the distance between the line and the VP
                my_N=list_normals(nth_line,:);
                flag_CurrentLineClustered=0;
                for nth_VP=1:Nb_VPs
                    my_VP=list_VPs(nth_VP,:);
                    MyCost=Fn_ComputeVPDist(my_VP,my_N',FlagDistanceDefinition,Kmatrix_ForDistance); % MUST USE THE SAME COST THAN WHEN GENERATING THE DATA IN Fn_GenerateSphPoints_WithRotAndOutliers
                    
                    % compute the LOWER bound of outliers
                    if MyCost<= InlierThreshold+MyOffset % then inliers
                        % nothing. it might be an inlier
                        %if Flag_LineToAtMostOneVP==0 || (Flag_LineToAtMostOneVP==1 && flag_CurrentLineClustered==0)
                         MyUpperBoundInliers=MyUpperBoundInliers+1; % the upper nb of inliers
                         flag_CurrentLineClustered=1;
                        %end
                    else % we are sure it is an outlier
                        if flag_ComputeExtraInfo==1
                        MyLowerBoundOutliers=MyLowerBoundOutliers+1; % the nb of outliers
                        Indices_LbOutliers(nth_line)=1; % 0 for outlier
                        end
                    end
                    
                                        % compute the UPPER bound of outliers
                    if MyCost<= InlierThreshold % then we are sure it is an inlier
                        % nothing, we are sure it is an inlier
                        %if Flag_LineToAtMostOneVP==0 || (Flag_LineToAtMostOneVP==1 && flag_CurrentLineClustered==0)
                        MyLowerBoundInliers=MyLowerBoundInliers+1; % the lower nb of inliers
                        Indices_LbInliers(nth_line)=1; % 0 for inlier
                        %end
                    else % outliers (it might be an outlier)
                        if flag_ComputeExtraInfo==1
                        MyUpperBoundOutliers=MyUpperBoundOutliers+1; % the nb of outliers
                        end
                    end
                    
                    if Flag_LineToAtMostOneVP==1 && flag_CurrentLineClustered==1
                        break;
                    end

                end
            end
            
            % fast version
            elseif flag_submethod==1
                
%                 % TEMP TEST!!!!!!!!!!!!!!!
%                 TotalCost=Fn_ComputeQuality_ForBnBinterval(Rbar_box, list_normals, InlierThreshold, Nb_VPs);
%                 IndicesInliers=[];
%                 
%                 Rbar_box
%                 
%                 for kth_VP=1:Nb_VPs
%     
%     % get the initial axis
%     if kth_VP==1
%         my_axis=[1 0 0]';
%     elseif kth_VP==2
%         my_axis=[0 1 0]';
%     elseif kth_VP==3
%         my_axis=[0 0 1]';
%     end
%     % compute the rotated axis
%     my_rotated_axis=Rbar_box*my_axis; % size is 3x1
%     norm(my_rotated_axis)
%     %my_rotated_axis=Rbox'*my_axis; % size is 3x1
%     % yListCost=Fn_ThresholdInterval(Fn_ComputeVPDist(list_normals,my_rotated_axis),ThreshInlier);
%    ListCost_Interval=Fn_ComputeVPDist(list_normals,my_rotated_axis)
%    MyUpperBoundInliers=length(find(inf(ListCost_Interval)<=InlierThreshold))
%    MyLowerBoundInliers=length(find(sup(ListCost_Interval)<=InlierThreshold))
%                 ddddddddddddddddddddddd
%                 end
%                 %TotalCost
                
                
                ArrayCost=zeros(NbLines,Nb_VPs);
                for nth_VP=1:Nb_VPs
                    my_VP=list_VPs(nth_VP,:);
                 ArrayCost(:,nth_VP)=Fn_ComputeVPDist(list_normals,my_VP,FlagDistanceDefinition,[]); % MUST USE THE SAME COST THAN WHEN GENERATING THE DATA IN Fn_GenerateSphPoints_WithRotAndOutliers
                end
                %indices=find(ArrayCost(:,1)<=InlierThreshold+MyOffset || ArrayCost(:,2)<=InlierThreshold+MyOffset || ArrayCost(:,3)<=InlierThreshold+MyOffset);
                ArrayBinary_ThreshOff=(ArrayCost<=InlierThreshold+MyOffset);
                ArrayBinary_Thresh=(ArrayCost<=InlierThreshold);
                if Flag_LineToAtMostOneVP==1
                    MyUpperBoundInliers=sum(max(ArrayBinary_ThreshOff,[],2));
                    if flag_ComputeExtraInfo==1
                    MyLowerBoundOutliers=sum(~ArrayBinary_ThreshOff(:)); % sum of all the 0
                  %  Indices_LbOutliers(nth_line)=1; % 0 for outlier
                    end
                  
%                   MyLowerBoundInliers=length(find(ArrayBinary_Thresh==1));
%                   MyUpperBoundOutliers=length(find(ArrayBinary_Thresh==0));
                    MyLowerBoundInliers=sum(max(ArrayBinary_Thresh,[],2));
                    IndicesInliers=find(sum(ArrayBinary_Thresh,2)>=1); % same as find(sum(max(ArrayBinary_Thresh,[],2))==1)
                    if flag_ComputeExtraInfo==1
                    MyUpperBoundOutliers=sum(~(ArrayBinary_Thresh(:))); % sum of all the 0
                    end
                else
                    MyUpperBoundInliers=sum(ArrayBinary_ThreshOff(:)); % sum of all the 1
                    if flag_ComputeExtraInfo==1
                    MyLowerBoundOutliers=sum(~ArrayBinary_ThreshOff(:)); % sum of all the 0
                    end
                    
                    MyLowerBoundInliers=sum(ArrayBinary_Thresh(:)); % sum of all the 1
                    if flag_ComputeExtraInfo==1
                    MyUpperBoundOutliers=sum(~(ArrayBinary_Thresh(:))); % sum of all the 0
                    end
                    %IndicesInliers=find(sum(ArrayBinary_Thresh,2)>=1); % TO CHECK. same as find(sum(max(ArrayBinary_Thresh,[],2))==1)
                    IndicesInliers=find(ArrayBinary_Thresh>=1); % NxK TO CHECK. same as find(sum(max(ArrayBinary_Thresh,[],2))==1)
                end
                
                
            elseif flag_submethod==2
                
                 % apply algorithm
    [directions_temp1, temp2, List_ClusteredNormalsIndices_Definite, ListNbClusteredIndices_Definite,UnclusteredIndices_Definite]=...
        Fn_GetLineClustering_GivenRotation(list_normals,Rbar,InlierThreshold, Nb_VPs,1,FlagDistanceDefinition);
     [directions_temp1, temp2, List_ClusteredNormalsIndices_Potential, ListNbClusteredIndices_Potential,UnclusteredIndices_Potential]=...
        Fn_GetLineClustering_GivenRotation(list_normals,Rbar,InlierThreshold+MyOffset, Nb_VPs,1,FlagDistanceDefinition);
    
    %UnclusteredIndices_Definite, UnclusteredIndices_Potential, ListNbClusteredIndices_Definite, ListNbClusteredIndices_Potential, List_ClusteredNormalsIndices_Definite
    IndicesInliers=[List_ClusteredNormalsIndices_Definite{:}];
    MyLowerBoundInliers=ListNbClusteredIndices_Definite(4);
    MyUpperBoundInliers=ListNbClusteredIndices_Potential(4);
            else
                error('wrong case')
            end
            
            %MyLowerBoundOutliers, MyUpperBoundOutliers
            MyLowerBoundOutliers_COUNTING=MyLowerBoundOutliers;
            MyUpperBoundOutliers_COUNTING=MyUpperBoundOutliers;
            MyLowerBoundInliers_COUNTING=MyLowerBoundInliers;
            MyUpperBoundInliers_COUNTING=MyUpperBoundInliers;
            %MyLowerBoundOutliers_COUNTING, MyUpperBoundOutliers_COUNTING, MyLowerBoundInliers_COUNTING, MyUpperBoundInliers_COUNTING
            
            
        elseif FlagMethod_local==1
            error('to do. cf Fn_BnB_ComputeBounds_PureRotation')
            
        else
            error('wrong case');
        end
    end
    R_solution=Rbar;
    
    %         if ~(MyLowerBoundOutliers_COUNTING==MyLowerBoundOutliers_BINTPROG && MyUpperBoundOutliers_COUNTING==MyUpperBoundOutliers_BINTPROG)
    %             MyLowerBoundOutliers_COUNTING, MyLowerBoundOutliers_BINTPROG
    %             MyUpperBoundOutliers_COUNTING, MyUpperBoundOutliers_BINTPROG
    %             x, Indices_LbOutliers
    %             [A_matrix*x, b_vect x]
    %             DiffIndices=(x~=Indices_LbOutliers')
    %             B=A_matrix*x;
    %             B(DiffIndices), b_vect(DiffIndices)
    %             error('counting and bintprog give different results')
    %         end
    
    %input('all right?')
    
    
    
elseif FlagMethod_BnB==1
    error('to do. cf Fn_BnB_ComputeBounds_PureRotation')
    
elseif FlagMethod_BnB==10
    error('to do. cf Fn_BnB_ComputeBounds_PureRotation')
    
elseif FlagMethod_BnB==11
    error('to do. cf Fn_BnB_ComputeBounds_PureRotation')
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%   START - UNPACK RESULTS OF FIRST-ORDER   %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if FlagMethod_BnB==1 ||  FlagMethod_BnB==11
    error('to do. cf Fn_BnB_ComputeBounds_PureRotation')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%    END - UNPACK RESULTS OF FIRST-ORDER    %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % simple check on the bounds
% if MyUpperBoundOutliers<MyLowerBoundOutliers;
%     MyUpperBoundOutliers, MyLowerBoundOutliers,
%     my_solution_MIP
%     MyClustering_MIP
%     ListCost_DEBUG,IndicesBinaryOutliers_DEBUG, InlierThreshold
%     input('ERROR: lower is higher than upper');
% end
% simple check on the bounds
if MyUpperBoundInliers<MyLowerBoundInliers;
    MyUpperBoundInliers, MyLowerBoundInliers,
    %my_solution_MIP
    %MyClustering_MIP
    %ListCost_DEBUG,IndicesBinaryOutliers_DEBUG, 
    InlierThreshold
    error('ERROR: lower is higher than upper'); % input('ERROR: lower is higher than upper');
end


return;