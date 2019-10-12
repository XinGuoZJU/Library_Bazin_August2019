%be carefull: the order of the arguments are different than in Fn_Rotation_Matrix_values_from_matrix

% related functions: Fn_Rotation_Matrix_values_from_matrix and Fn_Rotation_Matrix_matrix_from_values_Vect

% flag_type:
%   - 0: double rad values
%   - 1: double deg values
%   - 2: interval rad values
%   - 3: interval rad values

function R=Fn_Rotation_Matrix_matrix_from_values(roll_angle, pitch_angle, yaw_angle, flag_type)

% if usual double, then initialize the size. it is not possible with
% interval values
if flag_type==0 || flag_type==1
    R=zeros(3,3);
end

if flag_type==1 || flag_type==3% convert from degrees to rad
    yaw_angle=yaw_angle/180*pi;
roll_angle=roll_angle/180*pi;
pitch_angle=pitch_angle/180*pi;
end

alpha=yaw_angle;
beta=pitch_angle;
gamma=roll_angle;

% first row
R(1,1) = cos(alpha)*cos(beta);
R(1,2) = -sin(alpha)*cos(gamma) + cos(alpha)*sin(beta)*sin(gamma);
R(1,3) = sin(alpha)*sin(gamma) + cos(alpha)*sin(beta)*cos(gamma);

% second row
R(2,1) = sin(alpha)*cos(beta);
R(2,2) = cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma);
R(2,3) = -cos(alpha)*sin(gamma) + sin(alpha)*sin(beta)*cos(gamma);

% thrid row
R(3,1) = -sin(beta);
R(3,2) = cos(beta)*sin(gamma);
R(3,3) = cos(beta)*cos(gamma);

% R
% R'
% inv(R)
% 
% from MTx user manual
% alpha=roll_angle;
% beta=pitch_angle;
% gamma=yaw_angle;
% 
% R(1,1) = cos(gamma)*cos(beta);
% R(1,2) = -cos(alpha)*sin(gamma) + sin(alpha)*sin(beta)*cos(gamma);
% R(1,3) = sin(alpha)*sin(gamma) + cos(alpha)*sin(beta)*cos(gamma);
% 
% R(2,1) = sin(gamma)*cos(beta);
% R(2,2) = cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma);
% R(2,3) = -sin(alpha)*cos(gamma) + cos(alpha)*sin(beta)*sin(gamma);
% 
% R(3,1) = -sin(beta);
% R(3,2) = cos(beta)*sin(alpha);
% R(3,3) = cos(beta)*cos(alpha);
% 
% R
% R'
% inv(R)
% 
flag_test=0;
if flag_test==1
    roll_angle=(rand*2-1)*180; pitch_angle=(rand*2-1)*180; yaw_angle=(rand*2-1)*180;
    for SignRoll=[-1 1]
            for SignPitch=[-1 1]
                    for SignYaw=[-1 1]
                        disp('new')
                        roll_angle2=roll_angle+SignRoll*180; pitch_angle2=SignPitch*180-pitch_angle; yaw_angle2=yaw_angle+SignYaw*180; 
[roll_angle2 pitch_angle2 yaw_angle2]
R2=Fn_Rotation_Matrix_matrix_from_values(roll_angle2, pitch_angle2, yaw_angle2, 1)

                        % disp('original')
                        [roll_angle pitch_angle yaw_angle]
R=Fn_Rotation_Matrix_matrix_from_values(roll_angle, pitch_angle, yaw_angle, 1)

if max(max(abs(R2-R)))>0.0001; R, R2, error('diff rot'); end

                    end
            end
    end
end

