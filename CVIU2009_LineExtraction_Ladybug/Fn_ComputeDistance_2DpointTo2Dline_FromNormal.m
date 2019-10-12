function ListDist=Fn_ComputeDistance_2DpointTo2Dline_FromNormal(ListNormals,my_point)

% reference: 2D case: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html

% inputs:
%   - ListNormals: Nx3 (with N>=1): [a, b, c] where ax+by+c=0. 
%   - my_point: 1x2 or 2x1 or 1x3 or 3x1
%
% output:
%   - ListDist: geometric distance from the point to the line

% test call:
%   - Fn_ComputeDistance_2DpointTo2Dline_FromNormal([0 1 3],[1 2]) : line y=-3 and point [1 2] should return 5
%   - Fn_ComputeDistance_2DpointTo2Dline_FromNormal([1 0 -7],[1 3]) : line x=7 and point [1 3] should return 6

if ~(size(ListNormals,2)==3), size(ListNormals), error('wrong size'), end

NbLines=size(ListNormals,1);

% % slow version
% ListDist=zeros(NbLines,1);
% for nth_line=1:NbLines
%     my_normal=ListNormals(nth_line,:);
% my_dist=abs(my_normal(1)*my_point(1) + my_normal(2)*my_point(2) + my_normal(3)*1) / sqrt(my_normal(1)^2+my_normal(2)^2);
% ListDist(nth_line)=my_dist;
% end

% faster (vectorized) version
if size(my_point,2)==2; my_point=[my_point 1]; end %from 1x2 to 3x1
if size(my_point,2)==3; my_point=my_point'; end %from 1x3 to 3x1
ListDist=abs(ListNormals*my_point); %Nx1
ListDist=ListDist./sqrt(sum(ListNormals(:,1).^2+ListNormals(:,2).^2,2)); % Nx1


