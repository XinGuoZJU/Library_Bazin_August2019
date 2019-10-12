%be carefull: the order of the arguments are different than in Fn_Rotation_Matrix_matrix_from_values

% flag_type:
%   - 0: double rad values
%   - 1: double deg values

function RotationAngles=Fn_Rotation_Matrix_values_from_matrix_Vect(R,flag_type)

yaw_angle=atan2(R(2,1),R(1,1));
roll_angle=atan2(R(3,2),R(3,3));
pitch_angle=atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));

if flag_type==1 % convert from rad to deg
yaw_angle=yaw_angle/pi*180;
roll_angle=roll_angle/pi*180;
pitch_angle=pitch_angle/pi*180;
end


RotationAngles=[roll_angle pitch_angle yaw_angle];