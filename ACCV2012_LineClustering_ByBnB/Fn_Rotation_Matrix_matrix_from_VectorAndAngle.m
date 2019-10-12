
function R=Fn_Rotation_Matrix_matrix_from_VectorAndAngle(my_vect, my_angle, flag_RadOrDeg,flag_method)

% version 16/02/2011
% update 16/02/2011: 
%   - wikipedia version

% associated with Fn_Rotation_Matrix_VectorAndAngle_from_matrix

% inputs:
%   - my_vect: the axis of rotation 1x3 or 3x1(unit norm)
%   - my_angle: the amplitude of the rotation, in degrees or radians (cf flag_RadOrDeg) 
%   - flag_RadOrDeg: 0 in radians, 1 in degrees (cf my_angle)
%   - flag_method: 0 (from Kanatani's book) or 1 (from wiki)
%           - both methods provide the same results
%           - flag_method=1 is recommended: probably faster and simpler to write


% output
%   - R: 3x3

%%% start - example #1:
% my_vect=randn(1,3) my_vect=my_vect/norm(my_vect);
% my_angle=randn*50 % random between -50 and +50 degrees
% R=Fn_Rotation_Matrix_matrix_from_VectorAndAngle(my_vect, my_angle, 0,0)  
%%% finish - example #1:


if nargin~=4, nargin, error('wrong nb of inputs'), end
if abs(norm(my_vect)-1)>0.1, my_vect, error('the vector must have a unit norm'), end
if flag_RadOrDeg==1 % convert from degrees to rad
    my_angle=my_angle/180*pi;
end


if flag_method==0
    % FROM: from "Geometric computation for machine vision", by Kenichi Kanatani, p102

    R=zeros(3,3);
    
% pre-compute some values for faster execution
my_cos=cos(my_angle); 
my_sin=sin(my_angle);

R(1,1) = my_cos+my_vect(1)^2*(1-my_cos);
R(1,2) = my_vect(1)*my_vect(2)*(1-my_cos)-my_vect(3)*my_sin;
R(1,3) = my_vect(1)*my_vect(3)*(1-my_cos)+my_vect(2)*my_sin;

R(2,1) = my_vect(1)*my_vect(2)*(1-my_cos)+my_vect(3)*my_sin;
R(2,2) = my_cos+my_vect(2)^2*(1-my_cos);
R(2,3) = my_vect(2)*my_vect(3)*(1-my_cos)-my_vect(1)*my_sin;

R(3,1) = my_vect(1)*my_vect(3)*(1-my_cos)-my_vect(2)*my_sin;
R(3,2) = my_vect(3)*my_vect(2)*(1-my_cos)+my_vect(1)*my_sin;
R(3,3) = my_cos+my_vect(3)^2*(1-my_cos);

elseif flag_method==1
    % FROM: http://en.wikipedia.org/wiki/Axis_angle#Exponential_map_from_so.283.29_to_SO.283.29
    % same in [Seo ICVV'09]
    w_skew=skew_symetric_v(my_vect);
    R=eye(3)+sin(my_angle)*w_skew + w_skew*w_skew*(1-cos(my_angle));

end

