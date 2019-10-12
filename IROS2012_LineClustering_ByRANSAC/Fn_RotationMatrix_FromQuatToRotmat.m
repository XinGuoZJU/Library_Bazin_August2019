function my_R=Fn_RotationMatrix_FromQuatToRotmat(my_q)

% from http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Conversion_to_and_from_the_matrix_representation

%be careull, there is an error in HOrn's paper or R_32 (page 641)!!! you can compare with http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
%if you use the original formulation, you do not obtain norm(col)=1 and R' is different than inv(R)

% input: 
%   - my_q: 1x4 or 4x1

% test 
%   syms q0 qx qy qz real;
%   my_R=Fn_RotationMatrix_FromQuatToRotmat([q0 qx qy qz])

% test 
%   syms q0 qx qy real;
%   qz=sqrt(1-q0^2-qx^2-qy^2)
%   my_R=Fn_RotationMatrix_FromQuatToRotmat([q0 qx qy qz])

% CHECK OUT Fn_Rotation_Between_2_Coord_Syst
q0=my_q(1);
qx=my_q(2);
qy=my_q(3);
qz=my_q(4);

R_11 = q0^2 + qx^2 - qy^2 - qz^2;
R_22 = q0^2 - qx^2 + qy^2 - qz^2;
R_33 = q0^2 - qx^2 - qy^2 + qz^2;
R_12 = 2* (qx*qy - q0*qz);
R_21 = 2* (qx*qy + q0*qz);
R_13 = 2* (qx*qz + q0*qy);
R_31= 2* (qx*qz - q0*qy);
R_23 = 2* (qy*qz - q0*qx);
R_32 = 2* (qy*qz + q0*qx);

my_R=[R_11 R_12 R_13;
    R_21 R_22 R_23;
    R_31 R_32 R_33];