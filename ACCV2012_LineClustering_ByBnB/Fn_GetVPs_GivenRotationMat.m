function ListVPs=Fn_GetVPs_GivenRotationMat(MyRotMatrix)


% dominant_directions contains the computed orthogonal directions
%my_x=MyRotMatrix'*[1 0 0]';   my_y=MyRotMatrix'*[0 1 0]';   my_z=MyRotMatrix'*[0 0 1]';
my_x=MyRotMatrix*[1 0 0]';       my_y=MyRotMatrix*[0 1 0]';       my_z=MyRotMatrix*[0 0 1]'; %JC 09/2010 for cvpr line clustering (change also at the beginning of the code)


ListVPs=zeros(3,3);
ListVPs(1,:)=my_x;
ListVPs(2,:)=my_y;
ListVPs(3,:)=my_z;

% % like the following
% list_axis(1,:)=[1 0 0]; % Ox
% list_axis(2,:)=[0 1 0]; % Oy
% list_axis(3,:)=[0 0 1]; % Oz
% 
% list_VPs=zeros(Nb_VPs,3);
% for kth_VP=1:Nb_VPs
%     my_axis=list_axis(kth_VP,:);
%     list_VPs(kth_VP,:)=(Rbar*my_axis');
% end