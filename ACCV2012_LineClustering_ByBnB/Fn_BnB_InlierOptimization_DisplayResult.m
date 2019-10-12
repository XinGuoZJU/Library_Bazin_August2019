
% name: Fn_BnB_InlierOptimization_DisplayResult
% used in Fn_MaximizationByBnB_Rotation_MAIN, associated with Fn_BnB_InlierOptimization

% returned values:
R_best % the rotation
%MinUpperBound, MaxLowerBound % its lower and upper bound
ListExtraInfo.CardinalityResult
ListExtraInfo.NbPoints




if flag_application==0 % 0-pure rotation, 1-camera resectioning (find R,C given 2d and 3d points), 2-two view reconstruction (find R, C, 3D points Xi, given 2D points),  3-3D/3D registration (find R,T given 3D points), 4-uncalibrated SfM (get F (or K, R ,T) from a set of 2D correspondences)
end

%Fn_BnB_DisplayBoundsOfValidCubes(ListCubes,IndexFlag,IndicesBounds)

nth_iteration=find(ListPostInfo(:,1)==0 & ListPostInfo(:,2)==0 & ListPostInfo(:,3)==0 & ListPostInfo(:,4)==0 & ListPostInfo(:,5)==0,1,'first'); nth_iteration=nth_iteration-1; % CAN BE IMPROVED by cutting in the main fucntion
if length(nth_iteration)==0, nth_iteration=size(ListPostInfo,1); end
ListPostInfo=ListPostInfo(1:nth_iteration,:); % cut
figure('name','Min or max UpperBound evolution')
plot(ListPostInfo(:,2),'.-g'); % upper bound

    Fn_BnB_PlotConvergence_Bounds(ListPostInfo(:,2),ListPostInfo(:,1)) % lower and upper bounds


% if ~(isequal(sort(ListPostInfo(:,2),'descend'), ListPostInfo(:,2))), error('not continuously decreasing'), end
% if ~(isequal(sort(ListPostInfo(:,1)), ListPostInfo(:,1))), error('not continuously increasing'), end
if ~(isequal(sort(ListPostInfo(:,1),'descend'), ListPostInfo(:,1))), error('not continuously decreasing'), end
if ~(isequal(sort(ListPostInfo(:,2)), ListPostInfo(:,2))), error('not continuously increasing'), end
ListPostInfo(:,2), ListPostInfo(:,1)
 if flag_SynthesizedOrRealData==0
NbInliersTrue=ExtraInfo.NbInliers
if ListPostInfo(end,1)~=NbInliersTrue,error('the nb of detected inliers does not match the nb of synthesized inliers'),end
 end
 
figure('name','Volume convergence'), plot(ListPostInfo(1:nth_iteration,4),'.-b'); title('volume in absolute')
% volume in percent
Fn_BnB_PlotConvergence_Volume(ListPostInfo(1:nth_iteration,5))

Fn_BnB_PlotConvergence_NbCubes(ListPostInfo(:,6))
