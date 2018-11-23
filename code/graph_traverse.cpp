#include "mex.hpp"
#include "mexAdapter.hpp"
#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <math.h>

struct Node{
  long n;
  double w;
};

void disp_adjList(std::map<long,std::vector<Node>>& n)
{
  for (auto itr = n.begin(); itr != n.end(); ++itr)
  {
      std::cout << itr->first << '\t';
      for(int i=0;i<itr->second.size();i++)
          std::cout<<'{'<<itr->second[i].n<<','<<itr->second[i].w<<"},";
      std::cout <<'\n';
  }
}

void getAdjacencyList(matlab::data::TypedArray<double>& edge_nodes,
  matlab::data::TypedArray<double>& edge_weights,
  long len,
  std::map<long,std::vector<Node>>& adj_list)

   {
    for(long i=0;i<len;i++)
    {
        Node n1;
        n1.n = edge_nodes[i][1];
        n1.w = edge_weights[i];
        adj_list[(long)edge_nodes[i][0]].push_back(n1);
        n1.n = edge_nodes[i][0];
        adj_list[(long)edge_nodes[i][1]].push_back(n1);
    }
}


matlab::data::TypedArray<double> graph_traversal(std::map<long,std::vector<Node>> graph, std::queue<long>& q,matlab::data::TypedArray<double> cost_aggr,int dmax,double sigma)
{
    std::map<long,std::vector<Node>> graph2 = graph;
    unsigned long central_node=0,node,n;
    int i = 0,node_idx,parent_idx;
    double sim;
    int temp;
    while(!q.empty())
    {
      if(q.size() == 1)
        central_node = q.front();

      node = q.front();
      q.pop();

      n = graph[node][0].n;
      if(graph[n].size()==2)
        q.push(n);

      for(node_idx=0;node_idx<graph[n].size();node_idx++){
        if(graph[n][node_idx].n == node)
            break;
      }

      // aggregate cost
      sim=std::exp(-(graph[n][node_idx].w)/sigma);
      for(int i=0;i<dmax;i++)
      {
        cost_aggr[n-1][i] = cost_aggr[n-1][i] + cost_aggr[node-1][i]*sim;
      }

      // remove edge
      if(graph[n].size()>0)
        graph[n].erase(graph[n].begin()+node_idx);

      if(graph[node].size()>0)
          graph[node].erase(graph[node].begin());

    }

    // backward pass

    graph = graph2;
    // disp_adjList(graph);
    std::vector<long> stack;
    stack.push_back(central_node);
    stack.push_back(graph[central_node][0].n);
    unsigned long parent_node;

    while(graph[central_node].size()>0 || stack.size()>1)
    {

      // std::cout<<stack.back()<<",";
      parent_node = stack[stack.size()-2];
      node = stack[stack.size()-1];

      for(node_idx=0;node_idx<graph[parent_node].size();node_idx++)
        if(graph[parent_node][node_idx].n == node) break;

      for(parent_idx=0;parent_idx<graph[node].size();parent_idx++)
        if(graph[node][parent_idx].n == parent_node)break;

      // update cost aggr
      sim=std::exp(-(graph[parent_node][node_idx].w)/sigma);
      for(int i=0;i<dmax;i++){
        cost_aggr[node-1][i] = sim*cost_aggr[parent_node-1][i] + (1-sim*sim)*cost_aggr[node-1][i];
      }

      if(graph[node].size() == 1)
      {
        while (stack.size()>1)
        {
          for(node_idx=0;node_idx<graph[parent_node].size();node_idx++)
            if(graph[parent_node][node_idx].n == node) break;

          for(parent_idx=0;parent_idx<graph[node].size();parent_idx++)
            if(graph[node][parent_idx].n == parent_node)break;

          if(graph[parent_node].size()>0)
            graph[parent_node].erase(graph[parent_node].begin()+node_idx);
          if(graph[node].size()>0)
              graph[node].erase(graph[node].begin()+parent_idx);

          if(stack.size()>0)
            stack.pop_back();

          if(stack.size()==1 && graph[stack.back()].size()>0)
          {
            stack.push_back(graph[central_node][0].n);
            break;
          }
          else if(stack.size()==1 && graph[stack.back()].size()==0)
            break;
          else {
            if(stack.size()>1){
            parent_node = stack[stack.size()-2];
            node = stack[stack.size()-1];
            }
            else
            break;

          if(graph[stack.back()].size() > 1 )
          {
            if(graph[node][0].n == parent_node)
              stack.push_back(graph[node][1].n);
            else
              stack.push_back(graph[node][0].n);

            break;
          }
          }
        }
      }
      else{
        if(graph[node][0].n == parent_node)
          stack.push_back(graph[node][1].n);
        else
          stack.push_back(graph[node][0].n);

      }
    }

    return cost_aggr;
}


class MexFunction : public matlab::mex::Function {
public:
    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        double len = inputs[0].getDimensions()[0];
        len = (long)len;
        matlab::data::TypedArray<double> edge_nodes = std::move(inputs[0]);
        matlab::data::TypedArray<double> edge_weights = std::move(inputs[1]);
        matlab::data::TypedArray<double> leaf_nodes = std::move(inputs[2]);
        matlab::data::TypedArray<double> cost_aggr = std::move(inputs[3]);
        double sigma = inputs[4][0];

        double dmax = cost_aggr.getDimensions()[1];
        dmax = (int)dmax;
        std::map<long,std::vector<Node>> graph;
        getAdjacencyList(edge_nodes,edge_weights,len,graph);
        // disp_adjList(graph);
        std::queue<long> q;
        for(long i=leaf_nodes.getDimensions()[1]-1;i>=0;i--){
            q.push(leaf_nodes[i]);
        }
        // std::cout<<"\n";
        cost_aggr = graph_traversal(graph,q,cost_aggr,dmax,sigma);
        outputs[0] = cost_aggr;

    }
};
