% Fn_PlotConic

% version 21/11/2009

% modifications: 21/11/2009
%   - fast version using the temporary global image TempGlobalImage
%   - draw the lines using Fn_aux_draw_points
% modifications: 22/11/2009
%   - use an auxiliary function to get the point coordinates. This auxiliary function, will be used by other functions

% note: similar to PlotConic of Barreto but can work for perspective images also (i.e. xi=0)

% inputs:
%   - my_normal
%   - img
%   - my_color
%   - line_width

% outputs:
%   - ConicPts: MxNx2: ConicPts(j+1,i,:) is the jth point of the ith line in the image
%   - GreatCirclePts
%   - img

function [ConicPts,GreatCirclePts,img]=Fn_PlotConic(my_normal,img,my_color,line_width, flag_FastGlobalDrawing)

if flag_FastGlobalDrawing==0
        height_image=size(img,1);
    width_image=size(img,2);
else
        global TempGlobalImage;
        if length(TempGlobalImage)==0
            error('no global image');
        end
        if ndims(TempGlobalImage)~=3
            size(TempGlobalImage)
            error('wrong format');
        end
           height_image=size(TempGlobalImage,1);
    width_image=size(TempGlobalImage,2);
    
    %figure('Name','INIT TempGlobalImage in Fn_Plot_conic_curve'); imshow(TempGlobalImage/255);

end


    image_size=[height_image width_image];
    [ConicPts,GreatCirclePts]=Fn_PlotConic_Aux(my_normal,image_size);
    
%flag_FastGlobalDrawing, my_normal, image_size


% NEW VERSION
if flag_FastGlobalDrawing==0
img=Fn_aux_draw_points(img, round(ConicPts), line_width, my_color,flag_FastGlobalDrawing);
else
    Fn_aux_draw_points([], round(ConicPts), line_width, my_color,flag_FastGlobalDrawing);
end

% %%%%%%%% START - OLD VERSION %%%%%%%%%
% if line_width~=0
%     % make list of neighbors
%     size_neighborhood=line_width;
%     correct_neighborhood_list=[];
%     for n_temp=-size_neighborhood:size_neighborhood %y
%         for m_temp=-size_neighborhood:size_neighborhood %x
%             if m_temp^2+n_temp^2<=line_width^2
%                 correct_neighborhood_list=[correct_neighborhood_list; [m_temp n_temp]];
%             end
%         end
%     end
% 
%     % fill the image
%     [NbPoints,temp]=size(ConicPts);
%     for i=1:1:NbPoints %nb of original points
%         for k=1:size(correct_neighborhood_list,1) %for all correct neighbors
%             my_new_point=[ConicPts(i,1)+correct_neighborhood_list(k,1) ConicPts(i,2)+correct_neighborhood_list(k,2)];% x y real
%             my_new_point_round=round(my_new_point);
%             if 0<my_new_point_round(1) && my_new_point_round(1)<=width_image && 0<my_new_point_round(2) && my_new_point_round(2)<=height_image
%                 img(my_new_point_round(2),my_new_point_round(1),1:3)=my_color;
%             end
%         end
%     end
% 
% else
% 
%     [NbPoints,temp]=size(ConicPts);
%     for nth_point=1:1:NbPoints
%         my_x=round(ConicPts(nth_point,1));
%         my_y=round(ConicPts(nth_point,2));
%         if 0<my_x && my_x<width_image && 0<my_y && my_y<height_image
%         img(my_y,my_x,:)=my_color;
%         end
%     end
% 
% end
% %%%%%%%% END - OLD VERSION %%%%%%%%%
