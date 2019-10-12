
% version 04/11/2009
% original version by Sang 11/2009
% modified by J.C. Bazin

% goal: given normals and calibration parameters, get the points of the great circles in the sphere
% It can be used to:
%   - draw the great circle in the spherical image
%   - draw points in the image in the function Fn_Plot_conic_curve_Mei


% modifications: 04/11/2009:
%       - add explanations about inputs. outputs and code
%       - avoid interatively increasing an array size: use the method "too long but cut"
%       - compute only once the Xs
%       - fast version to check the points above the stereographic point
% modifications: 15/11/2009
%       - scan also along y
%       - Array_GreatCirclePts(i,1,1:3)=[NbValidPoints NbValidPointsPart1 NbValidPointsPart2];

% inputs:
%   - list_normals: Nx3 
%   - ListInfoCalibration. cf Fn_Parameters_and_Initialization_SequenceInformation
%           - flag_ProjectionModel:
%                 - 0-Barreto Model. It requires the global variables Hc and Phi
%                 - 1-Mei model. It works also for perspective image and takes into account distortion. It requires the global variables: Hc, xi and kc
%                 - 2-Ladybug linear model. It requires the global variables: height_image and width_image
%                 - 3-Scaramuzza model
%                 - 10-Stereo-omni by Jang Gijeong
%           - Hc Phi xi kc; % Hc is for Barreto and Mei's model. Phi is for Barreto. xi and kc are for Mei.
%           - height_image, width_image; % for ladybug linear projection
%
% outputs:
%   - GreatCirclePts: MxNx3: GreatCirclePts(i,j+1,:) is the jth point of the ith line in the sphere
%


% explanations
%   - 2 constraints:
%       - n_x x + n_y y + n_z z=0, i.e. the points belong to the great circle
%       - x^2 + y^2 + z^2=1, i.e. the points belong to the sphere
%   - from these 2 constraints, we can 
%       - extract z: z=-(n_x x + n_y y)/n_z
%       - and thus obtain x^2 + y^2 + ((n_x x + n_y y)/n_z)^2 -1=0
%       - it is a 2nd degree polynomial with these 2 roots (can be found by symbolic programming 'simple(solve(x^2 + y^2 + (-(n_x*x + n_y*y)/n_z)^2 -1,y))') syms n_x n_y n_z x y z real;
%           -      (-n_x*x*n_y+n_z*(-x^2*n_z^2+n_z^2-n_x^2*x^2-n_y^2*x^2+n_y^2)^(1/2))/(n_z^2+n_y^2)
%           -      (-n_x*x*n_y-n_z*(-x^2*n_z^2+n_z^2-n_x^2*x^2-n_y^2*x^2+n_y^2)^(1/2))/(n_z^2+n_y^2)
%       - since we know that n_x^2+n_y^2+n_z^2=1, these 2 roots can be simplified
%           -      (-n_x*x*n_y+n_z*(-x^2+n_z^2+n_y^2)^(1/2))/(n_z^2+n_y^2)
%           -      (-n_x*x*n_y-n_z*(-x^2+n_z^2+n_y^2)^(1/2))/(n_z^2+n_y^2)
%   - because of the square root, x must be such that -x^2+n_z^2+n_y^2>=0, i.e. n_z^2+n_y^2>=x^2
%
%   - do the same for y. i.e.  by symbolic programming 'simple(solve(x^2 + y^2 + (-(n_x*x + n_y*y)/n_z)^2 -1,x))') syms n_x n_y n_z x y z real;
%         -       (-n_x*n_y*y+n_z*(-n_y^2*y^2-y^2*n_z^2+n_z^2-n_x^2*y^2+n_x^2)^(1/2))/(n_z^2+n_x^2)
%         -       (-n_x*n_y*y-n_z*(-n_y^2*y^2-y^2*n_z^2+n_z^2-n_x^2*y^2+n_x^2)^(1/2))/(n_z^2+n_x^2)
%       - since we know that n_x^2+n_y^2+n_z^2=1, these 2 roots can be simplified
%           -      (-n_x*y*n_y+n_z*(-y^2+n_z^2+n_x^2)^(1/2))/(n_z^2+n_x^2)
%           -      (-n_x*y*n_y-n_z*(-y^2+n_z^2+n_x^2)^(1/2))/(n_z^2+n_x^2)
%   - because of the square root, y must be such that -y^2+n_z^2+n_x^2>=0, i.e. n_z^2+n_x^2>=y^2

function [Array_GreatCirclePts] = Fn_ComputeGreatCirclesPointsOnSphere_Local(list_normals,ListInfoCalibration)

% read calibration parameters
flag_ProjectionModel=ListInfoCalibration.flag_ProjectionModel;

% set the array size
list_sampling=[-1:0.001:1];
Array_GreatCirclePts = zeros(size(list_normals,1),1+2*length(list_sampling)*0.5,3); % too big but will be cut
Max_Nb_Points=0;


% for all the lines
for i = 1 : size(list_normals,1)

    % initialization
   XsTotal=[]; YsTotal=[]; ZsTotal=[];
   
    % get the normal
    nx = list_normals(i,1);     ny = list_normals(i,2);     nz = list_normals(i,3);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     START - SCAN ALONG X     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % find the Xs that verify the square root 
    XsP = zeros(1,length(list_sampling)); % too long but will be cut
    nth_aux=0;
    for x_val = list_sampling
        if ny^2+nz^2>=x_val^2 % does it verify the square root
            nth_aux=nth_aux+1;
            XsP(nth_aux)=x_val;
        end
    end
    XsP=XsP(1:nth_aux);% cut
    XsN=-XsP;


    % from the Xs obtained above
    %   - compute the Ys (the 2 roots of the 2nd degree polynomial as explained in the introduction)
    %   - and finally compute Zs from n_x x^2+ n_y y^2+ n_z z^2=0 which implies
    YsP = (-nx*XsP*ny + nz*sqrt(-XsP.^2 + nz^2 + ny^2) )/(nz^2 + ny^2); % the second roots
    YsN = (-nx*XsN*ny - nz*sqrt(-XsN.^2 + nz^2 + ny^2))/(nz^2 + ny^2); % the first roots
    ZsP = (-nx/nz)*XsP + (-ny/nz)*YsP;
    ZsN = (-nx/nz)*XsN + (-ny/nz)*YsN;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % concat the positive and negative solutions into a single column vector
    XsTemp = [XsP XsN]';    YsTemp = [YsP YsN]';    ZsTemp = [ZsP ZsN]';

    if flag_ProjectionModel==0 || flag_ProjectionModel==2 || flag_ProjectionModel==3 % 0-Barreto Model, 1-Mei model, 2-ladybug linear model, 3-Scaramuzza
         % concatenate the positive and negative solutions into a single row vector
        Xs = XsTemp;    Ys = YsTemp;     Zs = ZsTemp;
        
    elseif flag_ProjectionModel==1
        global xi; if length(xi)==0; error('undefined'); end
    if xi == 0      % perspective case
        % concatenate the positive and negative solutions into a single row vector
        Xs = XsTemp;    Ys = YsTemp;     Zs = ZsTemp;
    else % general case
        % keep only the points above the stereographic point.
        % explanations: the great circle and the stereographic point form a cone that (1) gives a conic in the image but 
        %               also (2) gives a small circle in the sphere above the stereo point. we must remove this small circle
        % new version (faster)
        indices=find(ZsTemp>=(1-xi)); % extract the point indices above the stereographic point
        Xs=XsTemp(indices);    Ys=YsTemp(indices);    Zs=ZsTemp(indices);
        % old version
        %         for k = 1:length(ZsTemp)
        %             if ZsTemp(k)>=1-xi
        %                 Zs = [Zs; ZsTemp(k)];
        %                 Ys = [Ys; YsTemp(k)];
        %                 Xs = [Xs; XsTemp(k)];
        %             end
        %         end
    end
    
    else
        error('wrong case')
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    NbValidPointsPart1=length(Xs);
       XsTotal=[XsTotal; Xs];
        YsTotal=[YsTotal; Ys];
            ZsTotal=[ZsTotal; Zs];
           

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      END - SCAN ALONG X      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %if NbValidPointsPart1<length(list_sampling*0.6)
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     START - SCAN ALONG Y     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 clear XsP XsN Xs YsP YsN Ys ZsP ZsN Zs; % for security 
    % find the Ys that verify the square root 
    YsP = zeros(1,length(list_sampling)); % too long but will be cut
    nth_aux=0;
    for y_val = list_sampling
        if nz^2+nx^2>=y_val^2 % does it verify the square root
            nth_aux=nth_aux+1;
            YsP(nth_aux)=y_val;
        end
    end
    YsP=YsP(1:nth_aux);% cut
    YsN=-YsP;


    % from the Ys obtained above
    %   - compute the XXs (the 2 roots of the 2nd degree polynomial as explained in the introduction)
    %   - and finally compute Zs from n_x x^2+ n_y y^2+ n_z z^2=0 which implies
    XsP = (-nx*YsP*ny + nz*sqrt(-YsP.^2 + nz^2 + nx^2) )/(nz^2 + nx^2); % the second roots
    XsN = (-nx*YsN*ny - nz*sqrt(-YsN.^2 + nz^2 + nx^2 ) )/(nz^2 + nx^2); % the first roots
    ZsP = (-nx/nz)*XsP + (-ny/nz)*YsP;
    ZsN = (-nx/nz)*XsN + (-ny/nz)*YsN;



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % concat the positive and negative solutions into a single column vector
    XsTemp = [XsP XsN]';    YsTemp = [YsP YsN]';    ZsTemp = [ZsP ZsN]';

     if flag_ProjectionModel==0 || flag_ProjectionModel==2 || flag_ProjectionModel==3 % 0-Barreto Model, 1-Mei model, 2-ladybug linear, 3-Scaramuzza
         % concatenate the positive and negative solutions into a single row vector
        Xs = XsTemp;    Ys = YsTemp;     Zs = ZsTemp;
     elseif flag_ProjectionModel==1
    if xi == 0      % perspective case
        % concatenate the positive and negative solutions into a single row vector
        Xs = XsTemp;    Ys = YsTemp;     Zs = ZsTemp;
    else % general case
        indices=find(ZsTemp>=(1-xi)); % extract the point indices above the stereographic point
        Xs=XsTemp(indices);    Ys=YsTemp(indices);    Zs=ZsTemp(indices);
    end
     else
         error('wrong case')
     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        NbValidPointsPart2=length(Xs);
    XsTotal=[XsTotal; Xs];
        YsTotal=[YsTotal; Ys];
            ZsTotal=[ZsTotal; Zs];
%     else
%         NbValidPointsPart2=0;
%     end
           
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      END - SCAN ALONG Y      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%    START - STORE THE RESULTS    %%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    NbValidPoints=length(XsTotal);
    Array_GreatCirclePts(i,1,1:3)=[NbValidPoints NbValidPointsPart1 NbValidPointsPart2];
    Array_GreatCirclePts(i,2:NbValidPoints+1,1:3)=[XsTotal YsTotal ZsTotal];


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%     END - STORE THE RESULTS     %%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if NbValidPoints>Max_Nb_Points
        Max_Nb_Points=NbValidPoints; % used to cut the array
    end

end
Array_GreatCirclePts=Array_GreatCirclePts(:,1:1+Max_Nb_Points,:); % cut


