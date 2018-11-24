close all
clc
tic;
%% Read image into the workspace.
img0 = double(imread('../../middlebury_dataset/Motorcycle/im0.png')); %left POV - shifted to right 
img0 = imresize(img0,1/5);
%%
img1 = double(imread('../../middlebury_dataset/Motorcycle/im1.png')); %right POV - shifted to left
img1 = imresize(img1,1/5);

img0 = imgaussfilt(img0,0.2);
img1 = imgaussfilt(img1,0.2);

dX = fspecial('sobel')';

dX_img0 = imfilter(img0,dX,'replicate');
dX_img1 = imfilter(img1,dX,'replicate');

%% Parameters
dmin=0;
dmax=60;
beta = 0.11;
Ti = 8;
Tg = 2;
sigma = 2;
slic_seed = floor(size(img0,1)*size(img0,1)/150);
threshold = 1;
%% Disparity Calculation

diff_I0 = zeros(size(img0,1),size(img0,2),dmax-dmin+1);
diff_dX0 = zeros(size(img0,1),size(img0,2),dmax-dmin+1);
diff_I1 = zeros(size(img1,1),size(img1,2),dmax-dmin+1);
diff_dX1 = zeros(size(img1,1),size(img1,2),dmax-dmin+1);

for i=dmin:dmax
    diff_I0(:,i+1:size(img0,2), i+1) = sum(abs(img0(:,i+1:size(img1,2),:) - img1(:,1:(size(img1,2)-i),:)),3);
    diff_dX0(:,i+1:size(img0,2), i+1) = sum(abs(dX_img0(:,i+1:size(img1,2),:) - dX_img1(:,1:size(img1,2)-i,:)),3);
    
    diff_I1(:,1:size(img1,2)-i, i+1) = sum(abs(img0(:,i+1:size(img1,2),:) - img1(:,1:size(img1,2)-i,:)),3);
    diff_dX1(:,1:size(img1,2)-i, i+1) = sum(abs(dX_img0(:,i+1:size(img1,2),:) - dX_img1(:,1:size(img1,2)-i,:)),3);
end

pixel_disp0 = beta*min(diff_I0,Ti) + (1-beta)*min(diff_dX0,Tg);
pixel_disp1 = beta*min(diff_I1,Ti) + (1-beta)*min(diff_dX1,Tg);

numRows = size(img1,1);
numCols = size(img1,2);

%% Pixel Level MST
pixel_mst0 = gen_pixel_mst(img0);
pixel_mst1 = gen_pixel_mst(img1);

aggr_pixel_cost0 = graph_traversal(pixel_mst0, reshape(pixel_disp0,numRows*numCols,[]), sigma); 
aggr_pixel_cost1 = graph_traversal(pixel_mst1, reshape(pixel_disp1,numRows*numCols,[]), sigma); 

%% Region Level MST
[region_mst1,region_disp1,L1,N1] = gen_region_mst(img1, pixel_disp1, slic_seed);
aggr_region_cost1 = graph_traversal(region_mst1, region_disp1, sigma);

[region_mst0,region_disp0,L0,N0] = gen_region_mst(img0, pixel_disp0, slic_seed);
aggr_region_cost0 = graph_traversal(region_mst0, region_disp0, sigma);

%% Combined Pixel Costs

combined_cost0 = combined_pixel_cost(img0,L0,N0,aggr_region_cost0,reshape(aggr_pixel_cost0,numRows,numCols,[]));
combined_cost1 = combined_pixel_cost(img1,L1,N1,aggr_region_cost1,reshape(aggr_pixel_cost1,numRows,numCols,[]));

%% WTA: Winner Take All
[~, disparity0] = min(combined_cost0,[],3);
[~, disparity1] = min(combined_cost1,[],3);
disparity0 = int16(disparity0);
disparity1 = int16(disparity1);
%% Non Local Refinement
% Check for Stable points
cost_new = zeros(numRows,numCols, dmax-dmin+1);
unstable_mask = ones(numRows,numCols);
slack=5;
for x=1:numRows
    for y=1:numCols
        if y > disparity0(x,y)
            if abs(disparity0(x,y) - disparity1(x,y-disparity0(x,y)))<=slack  
                cost_new(x,y,:) = abs((dmin:dmax)' - double(disparity0(x,y)));
                unstable_mask(x,y) = 0;
            end
        end
    end
end
aggr_pixel_cost = graph_traversal(pixel_mst0, reshape(cost_new,numRows*numCols,[]), sigma); 
[~,disparity] = min(aggr_pixel_cost,[],2);
disparity = reshape(disparity, numRows,numCols,[]);

%% Cross-Based Local Multi-points Filtering (CLMF-0)
disparity_clmf = CLMF_0(disparity, threshold);

%% RESULT
gt0 = parsePfm('../../middlebury_dataset/Motorcycle/disp0.pfm');
figure
imshow(gt0/255.0)
gt_shrunk0 = imresize(gt0,1/5,'Antialiasing',false);
figure
imshow(gt_shrunk0/255.0)
figure
imshow(disparity_clmf, [min(min(disparity_clmf)),max(max(disparity_clmf))]); colorbar; colormap(jet)
toc;