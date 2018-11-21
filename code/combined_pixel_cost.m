function combined_cost = combined_pixel_cost(img,L,N,region_cost,pixel_cost)

    edges = edge(img(:,:,1), 'Canny');
%     edge_pixels = edges(edges==1);
%     figure;
%     imshow(edges);
    e = label2idx(int8(edges)+1);

    idx = label2idx(L);
    numRows = size(img,1);
    numCols = size(img,2);
    alpha = zeros(1,N);
    combined_cost = zeros(numRows, numCols,size(pixel_cost,3));


    for labelVal = 1:N
        alpha(1,labelVal) = length(intersect(idx{labelVal},e{2}))/length(idx{labelVal});
        combined_cost(idx{labelVal}+size(img,1)*size(img,2)*[0:size(pixel_cost,3)-1]) = repmat((1-alpha(1,labelVal))*region_cost(labelVal,:),size(idx{labelVal})) + alpha(1,labelVal)*pixel_cost(idx{labelVal}+size(img,1)*size(img,2)*[0:size(pixel_cost,3)-1]);
    end
end
 