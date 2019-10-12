function ListVPs=Fn_GetVPs_GivenRotation(MyRotationAngles)

R=Fn_Rotation_Matrix_matrix_from_values(MyRotationAngles(1), MyRotationAngles(2), MyRotationAngles(3), 1);
% dominant_directions contains the computed orthogonal directions
%my_x=R'*[1 0 0]';   my_y=R'*[0 1 0]';   my_z=R'*[0 0 1]';
my_x=R*[1 0 0]';       my_y=R*[0 1 0]';       my_z=R*[0 0 1]'; %JC 09/2010 for cvpr line clustering (change also at the beginning of the code)


ListVPs=zeros(3,3);
ListVPs(1,:)=my_x;
ListVPs(2,:)=my_y;
ListVPs(3,:)=my_z;
