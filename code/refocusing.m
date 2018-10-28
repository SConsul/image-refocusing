tic();
disp0_1 = parsePfm('../data/disp0.pfm');
picture_left = im2double(imread('../data/im0.png'));
disp0_1(disp0_1 == Inf) = 0;

%%Parameters using camera calib
f = 3962.004;
doffs = 107.911;
baseline = 237.604;
mmperpixel = 0.005196629; %Pixel size in mm
f_mm = f*mmperpixel;%Focal length in mm
K = 0.3;

depth_map = zeros(size(disp0_1));
depth_map = (baseline*f)./(disp0_1 + doffs); %Converting disparities to depth in mm
% figure()
% imshow(mat2gray(depth_map));
% figure();
% imshow(mat2gray(disp0_1));
picture_left = imresize(picture_left,1/3);
depth_map = imresize(depth_map,1/3);
[height,width,c] = size(picture_left);

imshow(picture_left);
h = imfreehand; 
mask = h.createMask();
[y,x] = find(mask);
in_focus = depth_map(mask);
Z_nearest = min(in_focus);
Z_farthest = max(in_focus);

Z_u = (Z_farthest - Z_nearest)/3 + Z_nearest;%Image plane chosen by the user
Z_u_behind = f_mm*Z_u/(Z_u - f_mm);

Z_p_behind =  zeros(size(disp0_1));
Z_p_behind = (f_mm*depth_map)./(depth_map - f_mm);

C_p = abs((f_mm*(Z_p_behind - Z_u_behind))./Z_p_behind);%Assumed N to be 1

refocussed_image =zeros(size(picture_left));
hsize = 3*floor(K*max(max(abs(C_p)))/mmperpixel);
I = (depth_map>Z_nearest) & (depth_map<Z_farthest);
picture_left_copy = picture_left;
% picture_left(I) = 0;
input1 = padarray(picture_left, [hsize hsize], 0);
for i = 1+hsize:height+hsize
        for j = 1+hsize:width+hsize
                if( depth_map(i-hsize,j-hsize)> Z_nearest && depth_map(i-hsize,j-hsize)<Z_farthest)
                   refocussed_image(i-hsize,j-hsize,:) = input1(i,j,:);
                else
                    windowR = input1( i-hsize:i+hsize, j-hsize:j+hsize,1);
                    windowG = input1( i-hsize:i+hsize, j-hsize:j+hsize,2);
                    windowB = input1( i-hsize:i+hsize, j-hsize:j+hsize,3);
                    sigma = K*abs(C_p(i-hsize,j-hsize))/mmperpixel;
                    gaussian_space = fspecial('gaussian',[2*hsize+1,2*hsize+1],sigma);
                    refocussed_image(i-hsize,j-hsize,1) = sum(sum(gaussian_space.*windowR));
                    refocussed_image(i-hsize,j-hsize,2) = sum(sum(gaussian_space.*windowG));
                    refocussed_image(i-hsize,j-hsize,3) = sum(sum(gaussian_space.*windowB));
                
                end 
                    
        end
end
figure();
% refocussed_image(I) = picture_left_copy(I);
imshow(refocussed_image);
figure;
imshow(mat2gray(abs(C_p)));
figure;
imshow(picture_left);
toc();