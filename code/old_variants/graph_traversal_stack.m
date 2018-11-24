img = magic(3);
[mst,g] = gen_pixel_mst(img);
figure(1);
p = plot(g,'EdgeLabel',g.Edges.Weight);
highlight(p,mst,'EdgeColor','r','LineWidth',2);

cost = zeros(size(img));
deg = degree(mst);
q = find(deg==1)';
stack = java.util.Stack();
for i=1:size(q(:))
    stack.push(q(i));
end
aggregated_cost = 0;
visited = zeros(size(img(:)));
mst2 = mst;
traversal1 = zeros(size(img(:)));
i = 1;

while(1)
    cost
    figure(2);
    p2 = plot(g,'EdgeLabel',g.Edges.Weight);
    highlight(p2,mst2,'EdgeColor','r','LineWidth',2);
   if size(stack) == 0
       break
   end
   
   node = stack.pop();
   
   if visited(node) == 1
       continue;
   end
   
   deg(node) = deg(node) -  1;
   traversal1(i) = node;
   i = i+1;
   visited(node) = 1;
   cost(node) = cost(node) + aggregated_cost + img(node);
   aggregated_cost = cost(node);
   nb_node = neighbors(mst2,node);
   for n = nb_node'
       if visited(n) == 0 && deg(n)<=2
           stack.push(n);
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
    