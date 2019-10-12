
% version 04/11/2009
% original version by Sang 11/2009
% modified by J.C. Bazin

% goal: given normals and calibration parameters, get (1) points of the great circles in the sphere and (2) points of the conics in the image
% note: similar to PlotConic of Barreto but can work for perspective images also (i.e. xi=0)

% inputs:
%   - normals: 3xN
%   - Array_GreatCirclePts. can be []
%
% outputs:
%   - GreatCirclePts: MxNx3: GreatCirclePts(i,j+1,:) is the jth point of the ith line in the sphere
%   - ConicPts: MxNx2: ConicPts(i,j+1,:) is the jth point of the ith line in the image


function [Array_GreatCirclePts,Array_ConicPts] = Fn_PlotConic_Mei(list_normals, Array_GreatCirclePts)



if nargin~=2, nargin, error('wrong nb of inputs'); end
if nargout~=2, nargout, error('wrong nb of outputs'); end 

%   - ListInfoCalibration. cf Fn_Parameters_and_Initialization_SequenceInformation
%           - flag_ProjectionModel:
%                 - 0-Barreto Model. It requires the global variables Hc and Phi
%                 - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%                 - 2-Ladybug linear model. It requires the global variables: height_image and width_image
%                 - 3-Scaramuzza model 
%                 - 10-Stereo-omni by Jang Gijeong
%           - Hc Phi xi kc; % Hc is for Barreto and Mei's model. Phi is for Barreto. xi and kc are for Mei.
%           - height_image, width_image; % for ladybug linear projection

ListInfoCalibration=Fn_BuildListInfoCalibration();


[Array_GreatCirclePts,Array_ConicPts] = Fn_PlotConic_Mei_Local(list_normals, Array_GreatCirclePts, ListInfoCalibration);


