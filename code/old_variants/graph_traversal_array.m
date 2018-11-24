img = magic(3);
[mst,g] = gen_pixel_mst(img);
% p = plot(g,'EdgeLabel',g.Edges.Weight);
% highlight(p,mst,'EdgeColor','r','LineWidth',2);

cost = zeros(size(img));
deg = degree(mst);
q = find(deg==1)';
aggregated_cost = 0;
visited = zeros(size(img(:)));
mst2 = mst;
traversal = zeros(size(img(:)));
i = 1;
while(1)   
   if size(q(:),1) == 0 || size(q(:),2) == 0
       break
   end
   
   node = q(end);
   if visited(node) == 1
       q(end) = [];
       continue;
   end
   deg(node) = deg(node) -  1;
   traversal(i) = node;
   i = i+1;
   q(end) = [];
   visited(node) = 1;
   cost(node) = cost(node) + aggregated_cost + img(node);
   aggregated_cost = cost(node);
   nb_node = neighbors(mst2,node);
   for n = nb_node'
       if visited(n) == 0 && deg(n)<=2
           q = [q n];
           deg(n) = deg(n) - 1;
           aggregated_cost = cost(node);
       elseif visited(n) == 0
           cost(n) = cost(n) + aggregated_cost;
           deg(n) = deg(n) - 1;
           aggregated_cost = 0;
       end
   end 
   if deg(node) == 0
       mst2 = rmedge(mst2,node,neighbors(mst2,node));
   end
end
    