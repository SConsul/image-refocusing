function [pixel_mst,pixel_graph] = gen_pixel_mst(img)
    pixel_graph = graph();
    [m,n,~] = size(img);
    pixel_graph = addnode(pixel_graph, m*n);
    node2nums = reshape(1:m*n,[m,n]);
    dx = [1,0,1,1];
    dy = [0,1,1,-1];
    
    
    for i = 1:4
        a = node2nums(max(1,1-dx(i)):min(m-dx(i),m),max(1,1-dy(i)):min(n-dy(i),n));
        b = node2nums(max(1,1+dx(i)):min(m,m+dx(i)),max(1+dy(i),1):min(n,n+dy(i)));
        a = a(:);
        b = b(:);
        weights = max([abs((img(a)-img(b))'),abs((img(a+m*n)-img(b+m*n))'),abs((img(a+2*m*n)-img(b+2*m*n))')]);
        pixel_graph = addedge(pixel_graph,a,b,weights);
    end
    
%     p = plot(pixel_graph,'EdgeLabel',pixel_graph.Edges.Weight);
    [pixel_mst,~] = minspantree(pixel_graph);
%     highlight(p,T);
end