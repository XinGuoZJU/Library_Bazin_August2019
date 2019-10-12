
% version 14/11/2011

function rotation_angles=Fn_Rotation_Between_2_Coord_Syst_angles(coord_syst_left,coord_syst_right)
%each row of coord_syst_left is a direction (ex: vanishing point, etc...)
%each row of coord_syst_right is a direction (ex: vanishing point, etc...)

R=Fn_Rotation_Between_2_Coord_Syst(coord_syst_left,coord_syst_right,[]);

%R, input('sdfsdf')

% [yaw_angle roll_angle pitch_angle]=Fn_Rotation_Matrix_values_from_matrix(R,1);
% rotation_angles=[roll_angle, pitch_angle, yaw_angle];


rotation_angles=Fn_Rotation_Matrix_values_from_matrix_Vect(R,1);