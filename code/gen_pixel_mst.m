function [pixel_mst,pixel_graph] = gen_pixel_mst(img)
    pixel_graph = graph();
    pixel_graph = addnode(pixel_graph, size(img,1)*size(img,2));
    node2nums = reshape(1:size(img,1)*size(img,2),size(img));
    dx = [1,0,1,1];
    dy = [0,1,1,-1];
    [m,n] = size(img);
    
    for i = 1:4
        a = node2nums(max(1,1-dx(i)):min(m-dx(i),m),max(1,1-dy(i)):min(n-dy(i),n));
        b = node2nums(max(1,1+dx(i)):min(m,m+dx(i)),max(1+dy(i),1):min(n,n+dy(i)));
        a = a(:);
        b = b(:);
        weights = abs(img(a)-img(b));
        pixel_graph = addedge(pixel_graph,a,b,weights);
    end
    
%     p = plot(g,'EdgeLabel',g.Edges.Weight);
    [pixel_mst,pred] = minspantree(pixel_graph);
%     highlight(p,T);
end