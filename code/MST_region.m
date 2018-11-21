close all
% tic;
%% Compute Superpixels of Input RGB Image
% Read image into the workspace.
img0 = imread('../../middlebury_dataset/im0.png'); %left POV - shifted to right 
img0 = imresize(img0,0.25);

img1 = imread('../../middlebury_dataset/im1.png'); %right POV - shifted to left
img1 = imresize(img1,0.25);

dX = fspecial('sobel')';
dX_img0 = imfilter(img0,dX,'replicate');
dX_img1 = imfilter(img1,dX,'replicate');
% Disparity Calculation; dmin=1, dmax=15
dmin=0;
dmax=15;

diff_I = zeros(dmax,size(img1,1),size(img1,2)-dmax,size(img1,3));
diff_dX = zeros(dmax,size(img1,1),size(img1,2)-dmax,size(img1,3));
for i=dmin:dmax
    diff_I(i+1,:,:,:) = abs(img0(:,i+1:size(img1,2)-dmax+i,:) - img1(:,1:size(img1,2)-dmax,:));
    diff_dX(i+1,:,:,:) = abs(dX_img0(:,i+1:size(img1,2)-dmax+i,:) - dX_img1(:,1:size(img1,2)-dmax,:));
end
beta = 0.11;
img = img1(:,1:size(img1,2)-dmax,:);
Ti = 8*double(ones(size(diff_I),'like',diff_I));
Tg = 2*double(ones(size(diff_dX),'like',diff_dX));
disp = beta*min(diff_I,Ti) + (1-beta)*min(diff_dX,Tg);
% Calculate superpixels of the images
[L,N] = superpixels(img,1000);
% [L1,N1] = superpixels(img1,1000);

% Display the superpixel boundaries overlaid on the original image.
figure;
BW = boundarymask(L);
imshow(imoverlay(img,BW,'cyan'))
% img0 = imgaussfilt(img0,1);
% figure;
% imshow(imoverlay(img0,BW0,'cyan'))

% % Set the color of each pixel in the output image to the mode RGB color of the superpixel region.
modeImg = zeros(size(img),'like',img);
idx = label2idx(L);
m = size(img,1);
n = size(img,2);

img = double(img);
disparity = zeros(N,1);

for labelVal = 1:N
    redIdx = idx{labelVal};
    greenIdx = idx{labelVal}+m*n;
    blueIdx = idx{labelVal}+2*m*n;
    seq = 256*256*img(redIdx) + 256*img(greenIdx) + img(blueIdx);
    [rgb,f] = mode(seq);
    b = mod(rgb,256);
    rgb = (rgb - b)/256;
    g = mod(rgb,256);
    rgb = (rgb - g)/256;
    r = mod(rgb,256);
    modeImg(redIdx) = r;
    modeImg(greenIdx) = g;
    modeImg(blueIdx) = b;
%     freq_img(redIdx) = f;
%     region_mode(labelVal,:) = [r g b];
    disparity(labelVal) = sum(disp(redIdx));
end  
figure;
imshow(modeImg);
modeImg = double(modeImg);


g = graph();
g = addnode(g,N);

% nodenums = reshape(1:size(L,1)*size(L,2),size(L));
% ind = unique(nodenums.*BW);
ind = find(BW==1);
[ind_r, ind_c ]= ind2sub([m,n],ind);
dx = [1,0,1,1];
dy = [0,1,1,-1];
sigma = 2.0;
for i = 1:4
    nbr_r = ind_r - dx(i);
    nbr_c = ind_c - dy(i);
    nbr_rr = nbr_r(nbr_r>0 & nbr_c>0 & nbr_r<=m & nbr_c<=n);
    nbr_cc = nbr_c(nbr_r>0 & nbr_c>0 & nbr_r<=m & nbr_c<=n);
    ind_c = ind_c(nbr_r>0 & nbr_c>0 & nbr_r<=m & nbr_c<=n);
    ind_r = ind_r(nbr_r>0 & nbr_c>0 & nbr_r<=m & nbr_c<=n);
    a = m*(ind_c-1) + ind_r;
    b = m*(nbr_cc-1) + nbr_rr;
    r = findedge(g, L(a),L(b));
    a = a(r==0);
    b = b(r==0);
    
    weights = max([abs((modeImg(a)-modeImg(b))'),abs((modeImg(a+m*n)-modeImg(b+m*n))'),abs((modeImg(a+2*m*n)-modeImg(b+2*m*n))')]);
    list = [L(a)' ; L(b)'];

    list = sort(list,1);
    list = unique(list','rows');
%     weights = exp(weights/sigma);
    g = addedge(g,list(:,1),list(:,2),weights);
end
g = rmedge(g, 1:numnodes(g), 1:numnodes(g));
[T,pred] = minspantree(g);
figure(2)
p = plot(g,'EdgeLabel',g.Edges.Weight);
highlight(p,T);
figure(3)
plot(T,'EdgeLabel',T.Edges.Weight)