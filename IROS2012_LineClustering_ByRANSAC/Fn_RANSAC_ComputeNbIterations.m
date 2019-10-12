function N=Fn_RANSAC_ComputeNbIterations(my_prob,RatioOutlier,NbMinPoints)

% inputs
%   - my_prob: alo called p, probability that at least one of the random samples of s points is free from outliers
%   - RatioOutlier: also called epsilon, the ration/proportion of outliers
%   - NbMinPoints: also called s, the minimal nb of points to compute the model (e.g. 4 for homography)

% output:
%   - N: number of samples

% reference: p119 of Multiple View Geometry, 2nd edition

% e.g. 
% my_prob=0.99, RatioOutlier=0.2, NbMinPoints=6, Fn_RANSAC_ComputeNbIterations(my_prob,RatioOutlier,NbMinPoints)

% Note if RatioOutlier=0, then N=0 ecause all inliers. We set N=1 due to at least one iteration


if RatioOutlier==0
    N=1;
else
N=ceil(log(1-my_prob)./log(1-(1-RatioOutlier).^NbMinPoints));
end