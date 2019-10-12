
% version 04/11/2009
% original version by Sang 11/2009
% modified by J.C. Bazin

% goal: given normals and calibration parameters, get (1) points of the great circles in the sphere and (2) points of the conics in the image
% note: similar to PlotConic of Barreto but can work for perspective images also (i.e. xi=0)

% modifications: 04/11/2009:
%       - add explanations about inputs. outputs and code
%       - avoid interatively increasing an array size: use the method "too long but cut"
%       - compute only once the Xs
%       - fast version to check the points above the stereographic point

% inputs:
%   - normals: 3xN
%   - Array_GreatCirclePts. can be []
%   - ListInfoCalibration. cf Fn_Parameters_and_Initialization_SequenceInformation
%           - flag_ProjectionModel:
%                 - 0-Barreto Model. It requires the global variables Hc and Phi
%                 - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%                 - 2-Ladybug linear model. It requires the global variables: height_image and width_image
%                 - 3-Scaramuzza model
%                 - 10-Stereo-omni by Jang Gijeong
%           - Hc Phi xi kc; % Hc is for Barreto and Mei's model. Phi is for Barreto. xi and kc are for Mei.
%           - height_image, width_image; % for ladybug linear projection
%
% outputs:
%   - GreatCirclePts: MxNx3: GreatCirclePts(i,j+1,:) is the jth point of the ith line in the sphere
%   - ConicPts: MxNx2: ConicPts(i,j+1,:) is the jth point of the ith line in the image


function [Array_GreatCirclePts,Array_ConicPts] = Fn_PlotConic_Mei_Local(list_normals, Array_GreatCirclePts,ListInfoCalibration)


if nargin~=3, nargin, error('wrong nb of inputs'); end
if nargout~=2, nargout, error('wrong nb of outputs'); end 

if length(Array_GreatCirclePts)==0
    [Array_GreatCirclePts] = Fn_ComputeGreatCirclesPointsOnSphere_Local(list_normals,ListInfoCalibration);
end



% size initialization. The number of points in the conic is the same than the number of points on the great circle
Array_ConicPts = zeros(size(Array_GreatCirclePts,1),size(Array_GreatCirclePts,2),2);

% for all the lines
for nth_line = 1 : size(Array_GreatCirclePts,1)

    % get the spherical points
    NbValidPoints=Array_GreatCirclePts(nth_line,1,1);
    ListSphericalPoints=squeeze(Array_GreatCirclePts(nth_line,1+1:1+NbValidPoints,:));
    
    
    
    % project in the image
    ListImagePoints=Fn_Projection_OnImageFromSphere_Local(ListSphericalPoints,ListInfoCalibration);
    
    
    % store
    Array_ConicPts(nth_line,1,1) = NbValidPoints;
    Array_ConicPts(nth_line,1,2)=Array_GreatCirclePts(nth_line,1,2) ; % nb of points in part1 and part2. MAYBE USELESS. nb of points in part 2= nb total - nb of points in part 1
    Array_ConicPts(nth_line,1+1:NbValidPoints+1,1:2) = ListImagePoints; % x y

end


