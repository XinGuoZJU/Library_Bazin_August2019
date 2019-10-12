
% Function name: Fn_Projection_OnImageFromSphere_Local
% authors: Jean-Charles Bazin

% version 07/07/2010
% modifications: 03/2009
%   - only Phi and Hc as global variables
%   - Fn_Check_Arguments to verify the format of input
%   - another formula for projection
% modifications: 15/11/2009
%   - flag_ProjectionModel to use different projection models
%   - explanations about inputs, outputs and procedure
%   - vectorized version so faster
% modifications: 20/11/2009
%   - some notes about normalization
%   - verify the global variables
% modifications: 03/02/2010
%   - projection for ladybug linear model (flag_ProjectionModel==2)
% modifications: 04/07/2010
%   - extension to the HxWx3 case


% inputs:
%   - my_list_points_sphere: the coordinates (x,y,z) in the sphere. either Nx3 or HxWx3
%   - ListInfoCalibration. cf Fn_Parameters_and_Initialization_SequenceInformation
%           - flag_ProjectionModel:
%                 - 0-Barreto Model. It requires the global variables Hc and Phi
%                 - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%                 - 2-Ladybug linear model. It requires the global variables: height_image and width_image
%                 - 10-Stereo-omni by Jang Gijeong
%           - Hc Phi xi kc; % Hc is for Barreto and Mei's model. Phi is for Barreto. xi and kc are for Mei.
%           - height_image, width_image; % for ladybug linear projection
%
%
% output:
%   - my_list_points: the coordinates (x,y) in the image, in x,y DOUBLE format (i.e. not integer). either Nx3 or HxWx3


% Note: I think the points must have a unit norm TO CONFIRM!



function [my_image_point_list]=Fn_Projection_OnImageFromSphere_Local(my_point_sphere_list,ListInfoCalibration) % x,y, z and returns x,y




Fn_Check_Arguments(my_point_sphere_list);



% read calibration parameters
flag_ProjectionModel=ListInfoCalibration.flag_ProjectionModel;


if ndims(my_point_sphere_list)==2 % i.e. list is Nx3
    if size(my_point_sphere_list,2)~=3; error('wrong format'); end
    ListXs=my_point_sphere_list(:,1);
    ListYs=my_point_sphere_list(:,2);
    ListZs=my_point_sphere_list(:,3);
else % i.e. list is HxWx3
    if size(my_point_sphere_list,3)~=3; error('wrong format'); end
    ListXs=squeeze(my_point_sphere_list(:,:,1));
    ListYs=squeeze(my_point_sphere_list(:,:,2));
    ListZs=squeeze(my_point_sphere_list(:,:,3));
end


if flag_ProjectionModel==0 % 0-Barreto Model, 1-Mei model, 2-ladybug linear model
    %     X=sqrt(  (Phi^2+2*Phi+1)*(1-Zs) / ( (1+Zs)*(1+(Ys/Xs)^2) ) );
    %     %X=sqrt(  (Xs^2)*(Phi^2+2*Phi+1)*(1-Zs) / ( (1+Zs)*(Xs^2+(Ys)^2) ) );
    %     if Xs<0 % the sqrt has two solutions but we don't know which one is good
    %         X=-X;
    %     end
    %     Y=X*Ys/Xs;
    %     x=Hc(1,1)*X+Hc(1,3);
    %     y=Hc(2,2)*Y+Hc(2,3);
    
    % read calibration parameters
    Hc=ListInfoCalibration.Hc;
    Phi=ListInfoCalibration.Phi;
    
    % project
    Listx = ListXs.*(1+Phi)./(1+ListZs)*Hc(1,1)+Hc(1,3);
    Listy = ListYs.*(1+Phi)./(1+ListZs)*Hc(2,2)+Hc(2,3);
    
elseif flag_ProjectionModel==1
    
    % read calibration parameters
    Hc=ListInfoCalibration.Hc;
    xi=ListInfoCalibration.xi;
    kc=ListInfoCalibration.kc;
    
    % center
    xu = ListXs./(ListZs+xi); % if xi=0, then it is traditional perspective projection, i.e. divide by z
    yu = ListYs./(ListZs+xi);
    
    % add distortions
    if isequal(kc,zeros(1,5))==1 || length(kc)==0 % i.e. no distortion
        xd = xu;
        yd = yu;
    else
        [DistortX,DistortY] = FuncDistort(xu,yu,kc(1:4));
        xd = xu + DistortX;
        yd = yu + DistortY;
    end
    
    ku=Hc(1,1);
    kv=Hc(2,2);
    u0=Hc(1,3);
    v0=Hc(2,3);
    Listx = xd*ku+u0;
    Listy = yd*kv+v0;
    
elseif flag_ProjectionModel==2
    % read calibration parameters
    height_image=ListInfoCalibration.height_image;
    width_image=ListInfoCalibration.width_image;
    
    % if you change here, change also in Fn_Projection_OnSphere
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
        
        my_a_x=(max_x-min_x)/(max_theta-min_theta);
        my_b_x=min_x-my_a_x*min_theta;
        my_a_y=(min_y-max_y)/(max_phi-min_phi);
        my_b_y=max_y-my_a_y*min_phi;
        
    elseif flag_PanoToSphereConvention==1
        min_phi=0;    max_phi=pi;
        min_theta=0;    max_theta=2*pi;
        
        %     mapping:
        %         min_x --> max_theta
        %         max_x --> min_theta
        %         min_y --> min_phi (the first row of the image is mapped to the top of the sphere)
        %         max_y --> max_phi (the last row of the image is mapped to the bottom of the sphere)
        
        my_a_x=(min_x-max_x)/(max_theta-min_theta);
        my_b_x=max_x-my_a_x*min_theta;
        my_a_y=(max_y-min_y)/(max_phi-min_phi);
        my_b_y=min_y-my_a_y*min_phi;
        
    else
        error('wrong case')
        
    end
    
    [ListTheta, ListPhi, ListR]=Fn_cart2sph(ListXs, ListYs, ListZs,flag_PanoToSphereConvention);
    
    Listx=ListTheta*my_a_x + my_b_x;
    Listy=ListPhi*my_a_y + my_b_y;
    
elseif flag_ProjectionModel==3
    global ocam_model;
    
    my_image_point_list=world2cam(my_point_sphere_list', ocam_model); % input: x,y,z in 3xN format. output: y,x in 2xN format.
    %my_image_point_list=my_image_point_list([2 1],:)';
    Listx=my_image_point_list(2,:)';
    Listy=my_image_point_list(1,:)';
    
else
    error('wrong case')
end

% temporary bug fixing. sometimes I have imaginary value when nx=0 and persp
Listx=real(Listx);
Listy=real(Listy);

if ndims(my_point_sphere_list)==2
    my_image_point_list=[Listx Listy];
else
    my_image_point_list(:,:,1)=Listx; clear Listx;
    my_image_point_list(:,:,2)=Listy; clear Listy;
end



function Fn_Check_Arguments(my_point_sphere_list)


if ndims(my_point_sphere_list)==2 && size(my_point_sphere_list,2)==3
    % ok
elseif ndims(my_point_sphere_list)==3 && size(my_point_sphere_list,3)==3
    % ok
    
else
    size(my_point_sphere_list)
    error('wrong size of arguments')
end

%%% test
% [my_point_sphere]=Fn_Projection_On_Sphere([100 200])
% my_point=Fn_Projection_On_Image_From_Sphere(my_point_sphere)

%%% normalization
% my_point_sphere_list=my_point_sphere_list./repmat(sqrt(sum(my_point_sphere_list.^2,2)),1,3);
