close all
clear all
clc
%% Read image into the workspace.
img0 = imread('../../middlebury_dataset/im0.png'); %left POV - shifted to right 
img0 = imresize(img0,1/12);

img1 = imread('../../middlebury_dataset/im1.png'); %right POV - shifted to left
img1 = imresize(img1,1/12);

dX = fspecial('sobel')';
dX_img0 = imfilter(img0,dX,'replicate');
dX_img1 = imfilter(img1,dX,'replicate');

%% Parameters
dmin=0;
dmax=15;
beta = 0.11;
Ti = 8;
Tg = 2;
sigma = 0.1;

%% Disparity Calculation

diff_I0 = zeros(size(img0,1),size(img0,2),dmax-dmin+1);
diff_dX0 = zeros(size(img0,1),size(img0,2),dmax-dmin+1);
diff_I1 = zeros(size(img1,1),size(img1,2),dmax-dmin+1);
diff_dX1 = zeros(size(img1,1),size(img1,2),dmax-dmin+1);
for i=dmin:dmax
    diff_I0(:,i+1:size(img0,2), i+1) = sum((img0(:,i+1:size(img1,2),:) - img1(:,1:size(img1,2)-i,:)).^2,3);
    diff_dX0(:,i+1:size(img0,2), i+1) = sum((dX_img0(:,i+1:size(img1,2),:) - dX_img1(:,1:size(img1,2)-i,:)).^2,3);
    
    diff_I1(:,1:size(img1,2)-i, i+1) = sum((img0(:,i+1:size(img1,2),:) - img1(:,1:size(img1,2)-i,:)).^2,3);
    diff_dX1(:,1:size(img1,2)-i, i+1) = sum((dX_img0(:,i+1:size(img1,2),:) - dX_img1(:,1:size(img1,2)-i,:)).^2,3);
end

pixel_disp0 = beta*min(diff_I0,Ti) + (1-beta)*min(diff_dX0,Tg);
pixel_disp1 = beta*min(diff_I1,Ti) + (1-beta)*min(diff_dX1,Tg);

numRows = size(img1,1);
numCols = size(img1,2);

%% Pixel Level MST
pixel_mst0 = gen_pixel_mst(img0);
aggr_pixel_cost0 = graph_traversal(pixel_mst0, pixel_disp0, [numRows, numCols], sigma); 

pixel_mst1 = gen_pixel_mst(img1);
aggr_pixel_cost1 = graph_traversal(pixel_mst1, pixel_disp1, [numRows, numCols], sigma); 

%% Region Level MST
[region_mst1,region_disp1,L1,N1] = gen_region_mst(img1);
aggr_region_cost1 = graph_traversal(region_mst1, region_disp1, N1, sigma);

[region_mst0,region_disp0,L0,N0] = gen_region_mst(img0);
aggr_region_cost0 = graph_traversal(region_mst0, region_disp0, N0, sigma);

%% Combined Pixel Costs
idx0 = label2idx(L0);
idx1 = label2idx(L1);
edges0 = edge(img0(:,:,1), 'Canny');
edges1 = edge(img1(:,:,1), 'Canny');
% edge_pixels1 = edges1(edges1==1);
imshow(edges1);
e0 = label2idx(int8(edges0)+1);
e1 = label2idx(int8(edges1)+1);
alpha0 = zeros(1,N0);
alpha1 = zeros(1,N1);
combined_cost0 = zeros(numRows, numCols);
combined_cost1 = zeros(numRows, numCols);

for labelVal1 = 1:N1
    alpha1(1,labelVal1) = length(intersect(idx1{labelVal1},e1{2}))/length(idx1{labelVal1});
    combined_cost1(idx1{labelVal1}) = (1-alpha1(1,labelVal1))*aggr_region_cost1(labelVal1) + alpha1(1,labelVal1)*aggr_pixel_cost1(idx1{labelVal1}); 
end

for labelVal0 = 1:N0
    alpha0(1,labelVal0) = length(intersect(idx0{labelVal0},e0{2}))/length(idx0{labelVal0});
    combined_cost0(idx0{labelVal0}) = (1-alpha0(1,labelVal0))*aggr_region_cost0(labelVal0) + alpha0(1,labelVal0)*aggr_pixel_cost0(idx0{labelVal0}); 
end

%% WTA: Winner Take All
disparity0 = min(combined_cost0,[],3);
disparity1 = min(combined_cost1,[],3);

%% Non Local Refinement
% Check for Stable points
cost_new = zeros(numRows,numCols, dmax-dmin+1);
for y=1:numRows
    for x=1:numCols
        if disparity0(x,y) == disparity1(x-disparity0(x,y),y) 
            cost_new(x,y,:) = [dmin:dmax]' - disparity0(x,y);
        end
    end
end

aggr_pixel_cost = graph_traversal(pixel_mst0, cost_new, [numRows, numCols], sigma); 

disparity = min(aggr_pixel_cost,[],3);

