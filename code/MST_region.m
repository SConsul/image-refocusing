close all
% tic;
%% Compute Superpixels of Input RGB Image
% Read image into the workspace.
img = imread('../../middlebury_dataset/im0.png');
img = imresize(img,0.25);

% Calculate superpixels of the image.
[L,N] = superpixels(img,1000);
[m,n] = size(L);
% Display the superpixel boundaries overlaid on the original image.
figure;
BW = boundarymask(L);
imshow(imoverlay(img,BW,'cyan'))
RGB = label2rgb(L);
% img = imgaussfilt(img,1);
figure;
imshow(imoverlay(img,BW,'cyan'))
% % Set the color of each pixel in the output image to the mean RGB color of the superpixel region.
modeImg = zeros(size(img),'like',img);
idx = label2idx(L);
numRows = size(img,1);
numCols = size(img,2);
% f= zeros(1,N);
img = double(img);
% freq_img = zeros(size(img,1),size(img,2));
% region_mode = zeros(N,3);
for labelVal = 1:N
    redIdx = idx{labelVal};
    greenIdx = idx{labelVal}+numRows*numCols;
    blueIdx = idx{labelVal}+2*numRows*numCols;
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
end  
figure;
imshow(modeImg);
modeImg = double(modeImg);
% figure;
% imshow(freq_img, [min(min(freq_img)), max(max(freq_img))]);
% toc;


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
    
    % weights = norm((modeImg(a)-modeImg(b)),2);
    list = [L(a)' ; L(b)'];

    list = sort(list,1);
    list = unique(list','rows');
    weights = exp(max(abs(modeImg(list(:,1))' - modeImg(list(:,1))'))/sigma);
    g = addedge(g,list(:,1),list(:,2),weights);
end
g = rmedge(g, 1:numnodes(g), 1:numnodes(g));
[T,pred] = minspantree(g);
% figure(2)
% p = plot(g,'EdgeLabel',g.Edges.Weight);
% highlight(p,T);
% figure(3)
% plot(T,'EdgeLabel',T.Edges.Weight)