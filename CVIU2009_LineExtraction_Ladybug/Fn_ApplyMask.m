
% This code has been implemented by Jean-Charles Bazin (RCV Lab, KAIST, South Korea) in collaboration with Cedric Demonceaux (MIS, UPJV, France) and Pascal Vasseur (MIS, UPJV, France).
% Goal: apply a mask in order to keep only the inner part of the catadioptric image

% version 01/11/2009

% inputs:
%   - Mask: 
%       - HxW matrix. The zero elements will remove the corresponding pixels in the image
%       - OR a single scalar=1. in that case, the whole image will be selected
%   - my_image: HxW a gray scale image
%
% outputs:
%   - my_image: HxW gray scale image where the outer part of the image has been removed 

function my_image=Fn_ApplyMask(Mask, my_image)

if ndims(my_image)~=2
    size(my_image)
    error('wrong size image')
end

% FASTER METHOD
[rows cols]=find(Mask==0);
indices=sub2ind(size(my_image), rows, cols);
my_image(indices)=0;

% % BASIC METHOD
% for i =1:height
%     for j = 1:width
%         if Mask(i,j)==0
%             Image_edge(i,j)=0;
%         end
%     end
% end