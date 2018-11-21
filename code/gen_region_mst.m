function [region_mst,region_disp,L,N] = gen_region_mst(img,disp)
    %% Compute Superpixels of Input RGB Image
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
    region_disp = zeros(N,size(disp,3));

    for labelVal = 1:N
        redIdx = idx{labelVal};
        greenIdx = idx{labelVal}+m*n;
        blueIdx = idx{labelVal}+2*m*n;
        seq = 256*256*img(redIdx) + 256*img(greenIdx) + img(blueIdx);
        [rgb,~] = mode(seq);
        b = mod(rgb,256);
        rgb = (rgb - b)/256;
        g = mod(rgb,256);
        rgb = (rgb - g)/256;
        r = mod(rgb,256);
        modeImg(redIdx) = r;
        modeImg(greenIdx) = g;
        modeImg(blueIdx) = b;
        region_disp(labelVal,:) = sum(disp(redIdx+size(img,1)*size(img,2)*[0:size(disp,3)-1]))';
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
    region_mst = minspantree(g);
    % figure(2)
    % p = plot(g,'EdgeLabel',g.Edges.Weight);
    % highlight(p,T);
    % figure(3)
    % plot(T,'EdgeLabel',T.Edges.Weight)
 end