% Importamt Prepare you lane line image 
% scale it so make 10FT = 33 pixels ( 3.3meter =33 pixel  , 1 pixel=0.1 meter )
% this script will make the lane line reduce to 1 pixel to match the witdh
% in realworld.

% reading image

filename='igvc2013basic.png';

%read image and convert to BW , line is in black
BW = im2bw(imread(filename));

% invert the image so line is in white for thinning
BW3 = bwmorph(~BW,'thin',Inf);

% convert back to balck line for viewing
figure, imshow(~BW3)

imout=~BW3;
% save
imwrite(imout,['bw_thin_' filename])