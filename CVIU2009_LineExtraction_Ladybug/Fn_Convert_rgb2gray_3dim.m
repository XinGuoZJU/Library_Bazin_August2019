function myImage_gray_3Dim=Fn_Convert_rgb2gray_3dim(myRGBImage)

% version 07/07/2010
% modifications 07/07/2010
%   - info on output: uint8 
%   - and more memory efficient



% traditional method
% myImage_gray=rgb2gray(myRGBImage);
% myImage_gray_3Dim=zeros(size(myRGBImage));
% for k=1:3
% myImage_gray_3Dim(:,:,k)=myImage_gray;
% end

if ndims(myRGBImage)==3 % rgb image
    myImage_gray_3Dim=rgb2gray(myRGBImage);
else % gray color
myImage_gray_3Dim=myRGBImage; % memory efficient by "lazy copying"
end
clear myRGBImage;
myImage_gray_3Dim=repmat(myImage_gray_3Dim,[1 1 3]);

if isa(myImage_gray_3Dim,'uint8')==0; class(myImage_gray_3Dim), error('not uint8. mandatory??'); end