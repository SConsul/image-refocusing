%% Compute Superpixels of Input RGB Image
% Read image into the workspace.
img = double(imread('middlebury_dataset/im0.png'));
img = imresize(img,0.25);

% Calculate superpixels of the image.
[L,N] = superpixels(img,150);
[m,n] = size(L);
% Display the superpixel boundaries overlaid on the original image.
% figure(1)
BW = boundarymask(L);
% imshow(imoverlay(img/255,BW,'cyan'))

% % Set the color of each pixel in the output image to the mean RGB color of the superpixel region.
% meanImg = zeros(size(img),'like',img);
% idx = label2idx(L);
% numRows = size(img,1);
% numCols = size(img,2);
% for labelVal = 1:N
%     redIdx = idx{labelVal};
%     greenIdx = idx{labelVal}+numRows*numCols;
%     blueIdx = idx{labelVal}+2*numRows*numCols;
%     meanImg(redIdx) = mean(img(redIdx));
%     meanImg(greenIdx) = mean(img(greenIdx));
%     meanImg(blueIdx) = mean(img(blueIdx));
% end  


% % Set the color of each pixel in the dominant color of the superpixel region.
% domC = zeros(size(img),'like',img);
% hsvImage = rgb2hsv(rgbImage);
% hImage = hsvImage(:, :, 1);
% counts = histcounts(hImage);
% [binHeight, index] = max(counts);
idx = label2idx(L);
numRows = size(img,1);
numCols = size(img,2);
for labelVal = 1:N
    redI+
    dx = idx{labelVal};
    greenIdx = idx{labelVal}+numRows*numCols;
    blueIdx = idx{labelVal}+2*numRows*numCols;
    domC(redIdx) = mean(img(redIdx));
    domC(greenIdx) = mean(img(greenIdx));
    domC(blueIdx) = mean(img(blueIdx));
end 

g = graph();
g = addnode(g,N);

% nodenums = reshape(1:size(L,1)*size(L,2),size(L));
% ind = unique(nodenums.*BW);
ind = find(BW==1);
[ind_r, ind_c ]= ind2sub([m,n],ind);
dx = [1,0,1,1];
dy = [0,1,1,-1];

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
    % weights = norm((meanImg(a)-meanImg(b)),2);
    list = [L(a)' ; L(b)'];
    list = sort(list,1);
    list = unique(list','rows');
    g = addedge(g,list(:,1),list(:,2));
end
g = rmedge(g, 1:numnodes(g), 1:numnodes(g));
[T,pred] = minspantree(g);
% figure(2)
% p = plot(g,'EdgeLabel',g.Edges.Weight);
% highlight(p,T);
% figure(3)
% plot(T,'EdgeLabel',T.Edges.Weight)