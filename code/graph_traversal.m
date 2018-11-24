function cost_aggr = graph_traversal(mst,cost,sigma)
    %% obtain leaf nodes     
    deg = int8(degree(mst));
    leaf_nodes = find(deg==1)';
    deg = [];
    
    if isfile('graph_traverse.mexa64') || isfile('graph_traverse.mexmaci64') || isfile('graph_traverse.mexw64')
        cost_aggr = graph_traverse(mst.Edges.EndNodes,mst.Edges.Weight,leaf_nodes,cost,sigma);
    else
        mex -setup C++
        mex graph_traverse.cpp
        cost_aggr = graph_traverse(mst.Edges.EndNodes,mst.Edges.Weight,leaf_nodes,cost,sigma);
    end
    
end