function output = CLMF_0(disparity_img, threshold)

    [top_mat, bottom_mat,left_mat,right_mat] = skeleton(disparity_img,threshold);
    output_num = zeros(size(disparity_img));
    output_dem = zeros(size(disparity_img));
    [height,width] = size(disparity_img);

    for i = 1:height
        for j=1:width
            %Making the local structure and integrating
            points = 0;
            a0 = 0;
            for h = top_mat(i,j):bottom_mat(i,j)
                points = points + right_mat(i+h,j) - left_mat(i+h,j)+1;
                a0 = a0 +  sum(disparity_img(i+h,j+left_mat(i+h,j):j+right_mat(i+h,j)));
            end
            %updating
            for h = top_mat(i,j):bottom_mat(i,j)
                output_num(i+h,j+left_mat(i+h,j):j+right_mat(i+h,j)) = output_num(i+h,j+left_mat(i+h,j):j+right_mat(i+h,j))+ a0;
                output_dem(i+h,j+left_mat(i+h,j):j+right_mat(i+h,j)) = output_dem(i+h,j+left_mat(i+h,j):j+right_mat(i+h,j))+ points;
            end
        end
    end

    output = output_num ./ output_dem;
end