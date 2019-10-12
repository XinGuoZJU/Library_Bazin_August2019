
% version 27/08/2008

% modifications:
%   - 20/03/2009: checking if it is a direct coordinate system

% from Horn's paper: "Closed-form solution of absolute orientation using unit quaternions"
% related function: Fn_RegisterTwo3Dsets

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%  START - TEST#1  %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MyRotationAngles=[10 15 20];%Fn_RandomList_BoundedLimit(3, -20, 20);
% Rtrue_local=Fn_Rotation_Matrix_matrix_from_values(MyRotationAngles(1), MyRotationAngles(2), MyRotationAngles(3), 1)
% 
% NbPoints=5;
% ListPointsLeft=randn(NbPoints,3); ListPointsLeft=Fn_NormalizeToUnitNorm(ListPointsLeft);
% ListPointsRight=(Rtrue_local*ListPointsLeft')';
% flag_method=0
% Rest=Fn_Rotation_Between_2_Coord_Syst(ListPointsLeft,ListPointsRight,flag_method)
% Rest, Rtrue_local
% (Rest*ListPointsLeft')', ListPointsRight
% 
% flag_method=1
% Rest=Fn_Rotation_Between_2_Coord_Syst(ListPointsLeft,ListPointsRight,flag_method)
% Rest, Rtrue_local
% (Rest*ListPointsLeft')', ListPointsRight
% 
% [s2 R2 T2] = Fn_RegisterTwo3Dsets( ListPointsLeft', ListPointsRight')
% R2, Rtrue_local
% (R2*ListPointsLeft')', ListPointsRight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%   END - TEST#1  %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%  START - TEST#2  %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this one does not work (when pure random data for left and right sets). of course because the "angle" between the vectors is not the same
% NbPoints=2;
% ListPointsLeft=randn(NbPoints,3); 
% ListPointsRight=randn(NbPoints,3);
% % ListPointsLeft=Fn_NormalizeToUnitNorm(ListPointsLeft);
% % ListPointsRight=Fn_NormalizeToUnitNorm(ListPointsRight);
% FlagMethod=0;
% Rest=Fn_Rotation_Between_2_Coord_Syst(ListPointsLeft,ListPointsRight,FlagMethod)
% (Rest*ListPointsLeft')', ListPointsRight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%   END - TEST#2   %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%  START - TEST#2  %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check if right-handed frame and determinant is 1
% for k=1:100000
% k
% NbPoints=2;
% ListPointsLeft=randn(NbPoints,3); 
% ListPointsRight=randn(NbPoints,3);
% ListPointsLeft=Fn_NormalizeToUnitNorm(ListPointsLeft);
% ListPointsRight=Fn_NormalizeToUnitNorm(ListPointsRight);
% FlagMethod=0; % any value is ok since the right-handed test is performed before the rotation estimation
% Rest=Fn_Rotation_Between_2_Coord_Syst(ListPointsLeft,ListPointsRight,FlagMethod);
% (Rest*ListPointsLeft')', ListPointsRight
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%   END - TEST#2   %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function R=Fn_Rotation_Between_2_Coord_Syst(coord_syst_left,coord_syst_right,flag_method)
%each row of coord_syst_left is a direction (ex: vanishing point, etc...)
%each row of coord_syst_right is a direction (ex: vanishing point, etc...)

if nargin~=3, error('wrong nb of inputs'), end
if isempty(flag_method)==1
    flag_method=0;
end

% they must have the same size
if ~(size(coord_syst_left,1)==size(coord_syst_right))
    size(coord_syst_left,1), size(coord_syst_right)
    coord_syst_left, coord_syst_right
error('must have same size')
end

% if only two directions, then we create a third one by cross-product
if size(coord_syst_left,1)==2
    v3_left=cross(coord_syst_left(1,:),coord_syst_left(2,:));
    v3_left=v3_left/norm(v3_left); %if v1 and v2 are not orthogonal and unit, then the cross-product will not provide a unit v3 so we have to normalize
    coord_syst_left(3,:)=v3_left;
    
    v3_right=cross(coord_syst_right(1,:),coord_syst_right(2,:));
    v3_right=v3_right/norm(v3_right); %if v1 and v2 are not orthogonal and unit, then the cross-product will not provide a unit v3 so we have to normalize
    coord_syst_right(3,:)=v3_right;
end


% test from http://www.am.sanken.osaka-u.ac.jp/~mukaigaw/papers/ICRA2007-CameraMotion.pdf
% R=[coord_syst_right']*inv(coord_syst_left')

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% test from Fn_RegisterTwo3Dsets
% An=coord_syst_left';
% Bn=coord_syst_right';
% Na=3;
% 
% %Compute the quaternions
% M = zeros(4,4);
% for i=1:Na 
%     %Shortcuts
%     a = [0;An(:,i)];
%     b = [0;Bn(:,i)];    
%     %Crossproducts
%     Ma = [  a(1) -a(2) -a(3) -a(4) ; 
%             a(2)  a(1)  a(4) -a(3) ; 
%             a(3) -a(4)  a(1)  a(2) ; 
%             a(4)  a(3) -a(2)  a(1)  ];
%     Mb = [  b(1) -b(2) -b(3) -b(4) ; 
%             b(2)  b(1) -b(4)  b(3) ; 
%             b(3)  b(4)  b(1) -b(2) ; 
%             b(4) -b(3)  b(2)  b(1)  ];
%     %Add up
%     M = M + Ma'*Mb;
% end
% 
% %Compute eigenvalues
% [E D] = eig(M);
% 
% %Compute the rotation matrix
% e = E(:,4);
% M1 = [  e(1) -e(2) -e(3) -e(4) ; 
%         e(2)  e(1)  e(4) -e(3) ; 
%         e(3) -e(4)  e(1)  e(2) ; 
%         e(4)  e(3) -e(2)  e(1)  ];
% M2 = [  e(1) -e(2) -e(3) -e(4) ; 
%         e(2)  e(1) -e(4)  e(3) ; 
%         e(3)  e(4)  e(1) -e(2) ; 
%         e(4) -e(3)  e(2)  e(1)  ];
% 
% 
% R = M1'*M2;
% 
% %Retrieve the 3x3 rotation matrix
% R = R(2:4,2:4);
%  (R*coord_syst_left')', coord_syst_right, R*R' % sdfsd
%  R=R/norm(R(:,1))
%  (R*coord_syst_left')', coord_syst_right, R*R', sdfsd
 
% return;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ndims(coord_syst_left)==2 %i.e. one frame with its vanishing directions
    if size(coord_syst_left,1)==3 % square matrix to compute the det

        % note: a wrong determinant might happen because some input vectors are almost the same. the "volume " is almost 0. It can occur when the input points are very close to each other in the image
        
        if det(coord_syst_left)<0
                    coord_syst_left, coord_syst_right, det(coord_syst_left)
%            error('wrong det')
        end
        if det(coord_syst_right)<0
                    coord_syst_left, coord_syst_right, det(coord_syst_right)
   %         error('wrong det')
        end
    end
end


if ndims(coord_syst_left)==3 %i.e. several frames, with several vanishing directions
    nb_images=size(coord_syst_left,1)
    size(coord_syst_left)
    size(coord_syst_right)

% convert coord_syst_left to Nx3 where column1 contains all the x, column2 all the y, column3 all the z
    temp1=reshape(squeeze(coord_syst_left(:,:,1))',[],1);% v1x, v2x, v3x of the first frame, v1x, v2x, v3x of the second frame, etc...
    temp2=reshape(squeeze(coord_syst_left(:,:,2))',[],1);% v1y, v2y, v3y of the first frame, v1y, v2y, v3y of the second frame, etc...
    temp3=reshape(squeeze(coord_syst_left(:,:,3))',[],1);% v1z, v2z, v3z of the first frame, v1z, v2z, v3z of the second frame, etc...
    coord_syst_left=zeros(3*nb_images,3);
    coord_syst_left(:,1)=temp1;             coord_syst_left(:,2)=temp2;  coord_syst_left(:,3)=temp3;

    temp1=reshape(squeeze(coord_syst_right(:,:,1))',[],1);
    temp2=reshape(squeeze(coord_syst_right(:,:,2))',[],1);
    temp3=reshape(squeeze(coord_syst_right(:,:,3))',[],1);
    coord_syst_right=zeros(3*nb_images,3);
    coord_syst_right(:,1)=temp1;             coord_syst_right(:,2)=temp2;  coord_syst_right(:,3)=temp3;
end


if flag_method==0

Sxx=sum(coord_syst_left(:,1).*coord_syst_right(:,1));
Sxy=sum(coord_syst_left(:,1).*coord_syst_right(:,2));
Sxz=sum(coord_syst_left(:,1).*coord_syst_right(:,3));
Syx=sum(coord_syst_left(:,2).*coord_syst_right(:,1));
Syy=sum(coord_syst_left(:,2).*coord_syst_right(:,2));
Syz=sum(coord_syst_left(:,2).*coord_syst_right(:,3));
Szx=sum(coord_syst_left(:,3).*coord_syst_right(:,1));
Szy=sum(coord_syst_left(:,3).*coord_syst_right(:,2));
Szz=sum(coord_syst_left(:,3).*coord_syst_right(:,3));


N_11=Sxx+Syy+Szz;
N_12=Syz-Szy;
N_13=Szx-Sxz;
N_14=Sxy-Syx;
N_22=Sxx-Syy-Szz;
N_23=Sxy+Syx;
N_24=Szx+Sxz;
N_33=-Sxx+Syy-Szz;
N_34=Syz+Szy;
N_44=-Sxx-Syy+Szz;

N=[N_11 N_12 N_13 N_14;
    N_12 N_22 N_23 N_24;
    N_13 N_23 N_33 N_34;
    N_14 N_24 N_34 N_44] ;

[U,S,V]=svd(N);

q=V(:,1);

% U,S,V
% S
% %q=V(:,4) % don't use that line
% q'*N*q % interesting value
% N
% %norm(q)

%disp('result1')
R=Fn_RotationMatrix_FromQuatToRotmat(q);

% disp('result2')
% q0=q(1);
% qx=q(2);
% qy=q(3);
% qz=q(4);
% 
% R_11 = q0^2 + qx^2 - qy^2 - qz^2;
% R_22 = q0^2 - qx^2 + qy^2 - qz^2;
% R_33 = q0^2 - qx^2 - qy^2 + qz^2;
% R_12 = 2* (qx*qy - q0*qz);
% R_21 = 2* (qx*qy + q0*qz);
% R_13 = 2* (qx*qz + q0*qy);
% R_31= 2* (qx*qz - q0*qy);
% R_23 = 2* (qy*qz - q0*qx);
% R_32 = 2* (qy*qz + q0*qx);
% 
% 
% R=[R_11 R_12 R_13;
%     R_21 R_22 R_23;
%     R_31 R_32 R_33]

elseif flag_method==1
    NbPoints=size(coord_syst_left,1);
C=zeros(3,3);
for i=1:NbPoints
    C=C+coord_syst_right(i,:)'*coord_syst_left(i,:);
end
[U,S,V]=svd(C);
s=sign(det(U*V'));
R=U*[1 0 0; 0 1 0; 0 0 s]*V';

elseif flag_method==2

[s2_temp R2 T2_temp] = Fn_RegisterTwo3Dsets( coord_syst_left', coord_syst_right');
R=R2;

end

return;


% making some experiments on rotation matrix reconstruction from quaternions
% q=randn(4,1);q=q/norm(q)
% q0=q(1);
% qx=q(2);
% qy=q(3);
% qz=q(4);
% % R_11 = q0^2 + qx^2 - qy^2 - qz^2;
% % R_12 = 2* (qx*qy - q0*qz);
% % R_13 = 2* (qx*qz + q0*qy);
% % R_21 = 2* (qy*qx + q0*qz);
% % R_22 = q0^2 - qx^2 + qy^2 - qz^2;
% % R_23 = 2* (qy*qz - q0*qx);
% % R_31= 2* (qz*qx - q0*qy);
% % R_32= 2* (qz*qy + q0*qz);
% % R_33 = q0^2 - qx^2 - qy^2 + qz^2;
% R_11 = q0^2 + qx^2 - qy^2 - qz^2;
% R_12 = 2* (qx*qy - q0*qz);
% R_13 = 2* (qx*qz + q0*qy);
% R_21 = 2* (qy*qx + q0*qz);
% R_22 = q0^2 - qx^2 + qy^2 - qz^2;
% R_23 = 2* (qy*qz - q0*qx);
% R_31= 2* (qz*qx - q0*qy);
% R_32= 2* (qz*qy + q0*qx);
% R_33 = q0^2 - qx^2 - qy^2 + qz^2;
% R=[R_11 R_12 R_13;
%     R_21 R_22 R_23;
%     R_31 R_32 R_33];
% R'
% inv(R)
% norm([R_12 R_22 R_32 ])
% norm([R_21 R_22 R_23 ])


% test
% vect1=rand(1,3); vect1=vect1/norm(vect1);
% vect2=rand(1,3); vect2=vect2/norm(vect2);
% coord_syst_left=[vect1; vect2]
% % coord_syst_left=[1 0 0; 0 1 0; 0 0 1]
% R=Fn_Rotation_Matrix_matrix_from_values(30, 10, 50, 0)
% coord_syst_right=(R*coord_syst_left')'
% R2=Fn_Rotation_Between_2_Coord_Syst(coord_syst_left,coord_syst_right,1)
% [yaw_angle roll_angle pitch_angle]=Fn_Rotation_Matrix_values_from_matrix(R2,1)

% vect3=cross(vect1,vect2)
% R*vect3'
% vect1_bis=coord_syst_right(1,:)
% vect2_bis=coord_syst_right(2,:)
% vect3_bis=cross(vect1_bis,vect2_bis)




% HORN ORIGINAL METHOD ON 2 or 3 VECTORS
%
% %         if v3_left(3)<0%added
% %             v3_left=-v3_left;
% %         end
% %         if v3_right(3)<0%added
% %             v3_right=-v3_right;
% %         end
%
%         x_left=v2_left-v1_left;
%         x_left=x_left/norm(x_left);
%         y_left=(v3_left-v1_left) - ((v3_left-v1_left)*x_left')*x_left;
%         y_left=y_left/norm(y_left);
%         z_left=cross(x_left,y_left);
%
%         x_right=v2_right-v1_right;
%         x_right=x_right/norm(x_right);
%         y_right=(v3_right-v1_right) - ((v3_right-v1_right)*x_right')*x_right;
%         y_right=y_right/norm(y_right);
%         z_right=cross(x_right,y_right);
%         %
% %         if z_left(3)<0%added
% %             z_left=-z_left;
% %         end
% %         if z_right(3)<0%added
% %             z_right=-z_right;
% %         end
%
%         M_left=[x_left' y_left' z_left'];
%         M_right=[x_right' y_right' z_right'];
%         R=M_right*M_left';



% temp synthesized data
my_R_IMU_Cam=Fn_Rotation_Matrix_matrix_from_values(10,15,20,1);
orginal_directions=[1 0 0; 0 1 0; 0 0 1];
nb_frames=200;
list_directions_IMU=zeros(nb_frames,3,3);
list_directions_camera=zeros(nb_frames,3,3);
for nth_frame=1:nb_frames

my_angles=randn(1,3)*20;
    R_IMU=Fn_Rotation_Matrix_matrix_from_values(my_angles(1),my_angles(2),my_angles(3),1);
    
    my_list=transpose(R_IMU'*orginal_directions');% 3 rows: v1, v2, v3
    list_directions_IMU(nth_frame,1:3,1:3)=my_list;
   
    list_directions_camera(nth_frame,1:3,1:3)=transpose(my_R_IMU_Cam'*my_list');
end
 figure('Name','comparison of directions - IMU vs camera')
    Fn_aux_draw_subplot_3_3(list_directions_IMU,list_directions_camera);
    
R=Fn_Rotation_Between_2_Coord_Syst(list_directions_IMU,list_directions_camera)
[yaw_angle roll_angle pitch_angle]=Fn_Rotation_Matrix_values_from_matrix(R',1)

list_directions_camera_new=zeros(nb_frames,3,3);
for nth_frame=1:nb_frames
list_directions_camera_new(nth_frame,:,:)=transpose(R'*squeeze(list_directions_camera(nth_frame,:,:))');
end

 figure('Name','comparison of directions - IMU vs camera - calibrated 1')
    Fn_aux_draw_subplot_3_3(list_directions_IMU,list_directions_camera_new);
    figure('Name','comparison of directions - IMU vs camera - calibrated 2')
    Fn_aux_draw_subplot_3_3(list_directions_camera_new,list_directions_IMU);
    
       figure; 
for i=1:3
    subplot(3,1,i);
    plot(list_directions_IMU(:,1,i),'r')
end

   figure; 
for i=1:3
    subplot(3,1,i);
    plot(list_directions_camera_new(:,1,i),'b')
end