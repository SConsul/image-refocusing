function cost_aggr = graph_traversal(mst,cost,img_size,sigma)
    deg = int8(degree(mst));
    Q = find(deg==1)';
    deg = [];
    
    q = zeros(1,2*size(Q,2));
    start_q = size(q,2)-size(Q,2)+1;
    end_q = size(q,2);
    q(1,start_q:end_q) = Q;
    Q=[];
    
    mst2 = mst;
    central_node = 0;
    cost_aggr = cost;
    % forward pass
    while(1)
%         p = plot(mst,'EdgeLabel',mst.Edges.Weight);
%     highlight(p,mst2,'EdgeColor','r','LineWidth',2);

       if start_q == end_q
%          only 1 node left in the tree
           central_node = q(start_q);
       elseif start_q > end_q 
%          queue is empty
           break;
       elseif start_q == 1
           temp = zeros(1,end_q); 
           q = [temp q(1:end_q)];
           start_q = end_q + 1;
           end_q = 2*end_q;
       end
       
        % If not empty, pop last node 
       node = q(end_q);
       q(end_q) = 0;
       end_q = end_q - 1;

       n = neighbors(mst2,node);

       if size(n,1) ~= 0 && degree(mst2,n) == 2
            start_q = start_q - 1;
            q(start_q) = n;
       end
       [x_n,y_n,~] = ind2sub(img_size,n);
       [x_node,y_node,~] = ind2sub(img_size,node);
       cost_aggr(x_n,y_n,:) = cost_aggr(x_n,y_n,:) + cost_aggr(x_node,y_node,:).*exp(-mst.Edges.Weight(outedges(mst2,node))/sigma);
       mst2 = rmedge(mst2,node,n);

    end

    %% backward pass
    mst2 = mst;
    n = neighbors(mst2,central_node);
    traversal = zeros(1,sum(img_size(1:2)));
    traversal(1) = central_node;
    traversal(2) = n(1);
    end_t = 2;
    size_t = size(traversal,2);
    tic;
    while(1)
    %     traversal
%     p = plot(mst,'EdgeLabel',mst.Edges.Weight);
%     highlight(p,mst2,'EdgeColor','r','LineWidth',2);
       if degree(mst2,central_node) ==0
           break;
       end
       if end_t == size_t
           traversal = [traversal zeros(1,max(img_size(1:2)))];
           size_t = size(traversal,2);
       end

       parent_node = traversal(end_t-1);
       node = traversal(end_t);
       
       [x_parent,y_parent,~] = ind2sub(img_size,parent_node);
       [x_node,y_node,~] = ind2sub(img_size,node);
       
       S = exp(-mst.Edges.Weight(findedge(mst2,parent_node,node))/sigma);
       cost_aggr(x_node,y_node,:) = S*cost_aggr(x_parent,y_parent,:) + (1-S^2)*cost_aggr(x_node,y_node,:);

       if degree(mst,node) == 1
           while(1)
               mst2 = rmedge(mst2,parent_node,node);
               end_t = end_t-1;
               if end_t == 1 && degree(mst2,traversal(1)) >=1
                    n = neighbors(mst2,traversal(1))';
                    end_t = end_t+1;
                    traversal(end_t) = n(1);
                    break
               elseif end_t == 1 && degree(mst2,traversal(1)) == 0
                   break
               end

               parent_node = traversal(end_t-1);
               node = traversal(end_t);
               if degree(mst2,node) > 1
                   n = neighbors(mst2,node)';
                   end_t = end_t+1;
                   if n(1) ~= parent_node
                       traversal(end_t) = n(1);
                   else
                       traversal(end_t) = n(2);
                   end
                   break
               end
           end
       else
           n = neighbors(mst2,node)';
           end_t = end_t+1;
           if n(1) ~= parent_node
               traversal(end_t) = n(1);
           else
               traversal(end_t) = n(2);
           end
       end

    end

end