
% Fn_Projection_On_Sphere_Local

% version 01/07/2010
% modifications:
%   - 01/07/2010: if no distortion for Mei, then do not remove distortions


% inputs:
%   - my_list_points:
%       - Nx2 (or Nx3) the coordinates in the image, in x,y format
%       - Or HxWx2 for the whole image
%   - ListInfoCalibration. cf Fn_Parameters_and_Initialization_SequenceInformation
%           - flag_ProjectionModel:
%                 - 0-Barreto Model. It requires the global variables Hc and Phi
%                 - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%                 - 2-Ladybug linear model. It requires the global variables: height_image and width_image
%                 - 3-scaramuzza
%                 - 10-Stereo-omni by Jang Gijeong
%           - Hc Phi xi kc; % Hc is for Barreto and Mei's model. Phi is for Barreto. xi and kc are for Mei.
%           - height_image, width_image; % for ladybug linear projection
%
% output:
%   - my_list_points_sphere:
%         - Nx3, the coordinates in the sphere, in x,y,z format
%         - Or HxWx3, the coordinates in the sphere, in x,y,z format

% global variables:
%   flag_ProjectionModel:
%       - 0-Barreto Model. It requires the global variables Hc and Phi
%       - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%       - 2-Ladybug linear model. It requires the global variables: height_image and width_image

% related to: Fn_Projection_OnSphere

function my_list_points_sphere=Fn_Projection_OnSphere_Local(my_list_points,ListInfoCalibration)

Fn_Check_Arguments(my_list_points)

% read calibration parameters
if length(ListInfoCalibration)==0; error('undefined'); end
flag_ProjectionModel=ListInfoCalibration.flag_ProjectionModel;
%[flag_ProjectionModel,Hc,Phi,xi,kc, HeightImage, WidthImage]=Fn_Build_ListInfoCalibration_Unpack(ListInfoCalibration);

if ndims(my_list_points)==2 % i.e. list is Nx2 or Nx3
    Listx=my_list_points(:,1);
    Listy=my_list_points(:,2);
elseif ndims(my_list_points)==3 % i.e. list is HxWx2
    Listx=squeeze(my_list_points(:,:,1));
    Listy=squeeze(my_list_points(:,:,2));
else
    error('wrong format')
end

if flag_ProjectionModel==0 || flag_ProjectionModel==1 % 0-Barreto Model, 1-Mei model, 2-ladybug linear model
    Hc=ListInfoCalibration.Hc;
    %frame conversion
    ListX=(Listx-Hc(1,3))/Hc(1,1);
    ListY=(Listy-Hc(2,3))/Hc(2,2);
end

if flag_ProjectionModel==0 % 0-Barreto Model, 1-Mei model, 2-ladybug linear model
    %     %projection on the sphere
    %      temp1=1/((ListX.*ListX)+(ListY.*ListY)+Phi^2+2*Phi+1);
    %         temp2=2*(Phi+1);
    %         Xs=temp2*temp1*ListX;
    %         Ys=temp2*temp1*ListY;
    %         Zs=-((ListX.*ListX)+(ListY.*ListY)-Phi^2-2*Phi-1)*temp1;
    %         [ListXs ListYs ListZs]
    
    % read calibration parameters
    Hc=ListInfoCalibration.Hc;
    Phi=ListInfoCalibration.Phi;
    
    % complete formula:
    ListXs=2*(Phi+1)./((ListX.*ListX)+(ListY.*ListY)+Phi^2+2*Phi+1).*ListX;
    ListYs=2*(Phi+1)./((ListX.*ListX)+(ListY.*ListY)+Phi^2+2*Phi+1).*ListY;
    ListZs=-((ListX.*ListX)+(ListY.*ListY)-Phi^2-2*Phi-1)./((ListX.*ListX)+(ListY.*ListY)+Phi^2+2*Phi+1);
    % [Xs Ys Zs]
    
elseif flag_ProjectionModel==1
    
    % read calibration parameters
    xi=ListInfoCalibration.xi;
    kc=ListInfoCalibration.kc;
    
    % remove distortions by iterations
    if length(kc)==0 || isequal(kc,zeros(1,5))==1 % i.e. if no distortion
        Listxu = ListX;
        Listyu = ListY;
    else
        [DistortX,DistortY] = FuncDistort(ListX,ListY,kc(1:4));
        for i = 1:50
            [DistortX,DistortY] = FuncDistort(ListX-DistortX,ListY-DistortY,kc(1:4));
        end
        Listxu = ListX - DistortX;
        Listyu = ListY - DistortY;
    end
    
    % from the normalized plane onto the sphere (by Sang, symbolic programming)
    a = (xi + sqrt(1 + (1-xi^2)*(Listxu.^2+Listyu.^2)))./(Listxu.^2+Listyu.^2+1);
    ListXs = a.*Listxu;
    ListYs = a.*Listyu;
    ListZs = a - xi;
    
    % if perspective (i.e. if xi=0), then equivalent up-to-scale to:
    %my_left_ray=(Hc_inv*[Pi_left 1]')'
    %my_right_ray=(Hc_inv*[Pi_right 1]')'
    
    
elseif flag_ProjectionModel==2
    
    % read calibration parameters
    height_image=ListInfoCalibration.height_image;
    width_image=ListInfoCalibration.width_image;
    
    
    % if you change here, change also in Fn_Projection_On_Image_From_Sphere
    min_x=0;    max_x=width_image;
    min_y=0;    max_y=height_image;
    
    flag_PanoToSphereConvention=0; %0-Matlab, 1-Banno
    if flag_PanoToSphereConvention==0
        min_phi=-pi/2;    max_phi=pi/2;
        min_theta=-pi;    max_theta=pi; % not 0~2pi because of cart2sph
        
        min_x=0;    max_x=width_image;
        min_y=0;    max_y=height_image;
        %     mapping:
        %         min_x --> min_theta
        %         max_x --> max_theta
        %         min_y --> max_phi (the first row of the image is mapped to the top of the sphere)
        %         max_y --> min_phi (the last row of the image is mapped to the bottom of the sphere)
        
        my_a_theta=(max_theta-min_theta)/(max_x-min_x);
        my_b_theta=min_theta-my_a_theta*min_x;
        my_a_phi=(max_phi-min_phi)/(min_y-max_y);
        my_b_phi=min_phi-my_a_phi*max_y;
        
    elseif flag_PanoToSphereConvention==1
        min_phi=0;    max_phi=pi;
        min_theta=0;    max_theta=2*pi;
        
        %     mapping:
        %         min_x --> max_theta
        %         max_x --> min_theta
        %         min_y --> min_phi (the first row of the image is mapped to the top of the sphere)
        %         max_y --> max_phi (the last row of the image is mapped to the bottom of the sphere)
        
        my_a_theta=(min_theta-max_theta)/(max_x-min_x);
        my_b_theta=max_theta-my_a_theta*min_x;
        my_a_phi=(min_phi-max_phi)/(min_y-max_y);
        my_b_phi=max_phi-my_a_phi*max_y;
        
    else
        error('wrong case')
    end
    
    my_list_theta=Listx*my_a_theta + my_b_theta;
    my_list_phi=Listy*my_a_phi + my_b_phi;
    my_list_r=ones(size(my_list_theta,1),size(my_list_theta,2)); % general form to handle Nx2 and HxWx2
    
    [ListXs, ListYs, ListZs]=Fn_sph2cart(my_list_theta, my_list_phi, my_list_r, flag_PanoToSphereConvention);
    
elseif flag_ProjectionModel==3
    global ocam_model,
    my_list_points_sphere=cam2world(my_list_points(:,[2 1])', ocam_model); % input: y,x in 2xN format. output: x,y,z in 3xN format
    ListXs=my_list_points_sphere(1,:)';
    ListYs=my_list_points_sphere(2,:)';
    ListZs=my_list_points_sphere(3,:)';
    %my_list_points_sphere=my_list_points_sphere';
    
else
    error('wrong case')
    
end

if ndims(my_list_points)==2
    my_list_points_sphere=[ListXs ListYs ListZs];
elseif ndims(my_list_points)==3
    my_list_points_sphere(:,:,1)=ListXs; clear ListXs;
    my_list_points_sphere(:,:,2)=ListYs; clear ListYs;
    my_list_points_sphere(:,:,3)=ListZs; clear ListZs;
else
    error('wrong format')
end


function Fn_Check_Arguments(my_list_points)

if length(my_list_points)==0
    my_list_points
    size(my_list_points)
    error('wrong size of arguments')
end

%%% test
% [my_point_sphere]=Fn_Projection_On_Sphere([100 200])
% my_point=Fn_Projection_On_Image_From_Sphere(my_point_sphere)


