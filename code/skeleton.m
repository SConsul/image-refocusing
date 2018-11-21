 function [top_mat,bottom_mat, left_mat, right_mat] = skeleton(picture,threshold)
% Gives you the skeleton for every pixel in image
% picture = repmat(eye(2),[10,10]);
% threshold = -1;
    [height,width] = size(picture);
    top_mat =zeros(height,width);
    bottom_mat =zeros(height,width);
    left_mat =zeros(height,width);
    right_mat =zeros(height,width);

    for i = 1:height
        for j = 1:width
    %Finding right
    h = 0;
    Ip_h = picture(i,j);
    delta = 1;
    while(delta ~=0)
        if(j+h > width)
            break;
        end
        Ip = picture(i,j+h);
        if max(abs(Ip_h - Ip)) > threshold
            delta = 0;
        end
        h=h+1;
        alpha= 1/(1+h);
        Ip_h = (1-alpha)*Ip_h + alpha*Ip;
    end
    right_mat(i,j) = h-1;
    %Finding left
    h = 0;
    Ip_h = picture(i,j);
    delta = 1;
    while(delta ~=0)
        if (j+h) < 1
            break;
        end
        Ip = picture(i,j+h);
        if max(abs(Ip_h - Ip)) > threshold
        delta = 0;
        end
        h=h-1;
        alpha= 1/(1+abs(h));
        Ip_h = (1-alpha)*Ip_h + alpha*Ip;
    end
    left_mat(i,j)= h+1;
    %Finding bottom
    h = 0;
    Ip_h = picture(i,j);
    delta = 1;
    while(delta ~=0)
        if i+h > height
            break
        end
        Ip = picture(i+h,j);
        if max(abs(Ip_h - Ip)) > threshold
        delta = 0;
        end
        h=h+1;
        alpha= 1/(1+h);
        Ip_h = (1-alpha)*Ip_h + alpha*Ip;
    end
    bottom_mat(i,j)= h-1;
    %Finding top
    h = 0;
    Ip_h = picture(i,j);
    delta = 1;
    while(delta ~=0)
        if i+h < 1
            break
        end
        Ip = picture(i+h,j);
        if max(abs(Ip_h - Ip)) > threshold
        delta = 0;
        end
        h=h-1;
        alpha= 1/(1+abs(h));
        Ip_h = (1-alpha)*Ip_h + alpha*Ip;
    end
    top_mat(i,j)= h+1;

        end
    end
end