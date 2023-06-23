#include <bits/stdc++.h>

using namespace std;

vector<vector<vector<double>>> prims_mst(int V, vector<vector<vector<double>>> adj){
        int v[V];
        memset(v,-1,sizeof(v));
        set<pair<double,pair<double,double>>> s;
        s.insert({0,{0,-1}});
        vector<vector<vector<double>>> z(V,vector<vector<double>>());
        while(!s.empty()){
            auto it=s.begin();
            s.erase(it);
            auto k = *it;
            auto y = k.second;
            if(v[(int)y.first]==1)continue;
                v[(int)y.first]=1;
               if(y.second!=-1){
                z[y.first].push_back({y.second,k.first});
                z[y.second].push_back({y.first,k.first});
                }
                for(auto x:adj[y.first]){
                   if(v[(int)x[0]]==-1)s.insert({x[1],{x[0],y.first}});
                }
        }
        return z;
}

vector<vector<vector<double>>> randomise(vector<vector<vector<double>>> z){
     for(int i=0;i<z.size();i++){
       auto rng = default_random_engine {};
       shuffle(begin(z[i]), end(z[i]), rng);
     }
     return z;
}

vector<int> dfs_visit(int node,int visited[], vector<vector<vector<double>>> z, vector<int> p){
      visited[node]=1;
      p.push_back(node);
      for(auto x:z[node]){
          if(visited[(int)x[0]]==-1){
            p=dfs_visit(x[0],visited,z,p);
          }
      }
     return p;
}

vector<vector<int>> convert_to_roots(int Depot,vector<int> p, double Demand[],double Capacity){
    vector<vector<int>> Routes;
    vector<int> OneRoute;
    int ResidueCap = Capacity;
    for(auto v:p){
        if(v==Depot)continue;
        if(ResidueCap-Demand[v]>=0){
            OneRoute.push_back(v);
            ResidueCap -= Demand[v];
        }
        else {
            Routes.push_back(OneRoute);
            OneRoute.clear();
            OneRoute.push_back(v);
            ResidueCap = Capacity-Demand[v];
        }
    }
    Routes.push_back(OneRoute);
    return Routes;
}

double distance(pair<int,int>p1,pair<int,int>p2){
    double d;
    d = (double)sqrt((p1.first-p2.first)*(p1.first-p2.first)+(p1.second-p2.second)*(p1.second-p2.second));
    return d;
}

double cost_of_oneroute(vector<int> OneRoute,int node,vector<pair<double,double>> c){
    double cost=0;
    cost+=distance(c[node],c[OneRoute[0]]);
    int previous_node=OneRoute[0];
    for(auto current_node:OneRoute){
            cost+=distance(c[previous_node],c[current_node]);
            previous_node=current_node;
    }
    cost+=distance(c[previous_node],c[node]);
    return cost;
}

double cost(vector<vector<int>> Routes,int node,vector<pair<double,double>> c){
    double cost=0;
    for(auto OneRoute:Routes){
        cost+=cost_of_oneroute(OneRoute,0,c);
    }
    return cost;
}

vector<vector<int>> Nearest_Neighbour(vector<vector<int>> Routes,int V,vector<pair<double,double>> c){
    int visited[V];
    memset(visited,-1,sizeof(visited));
    vector<vector<int>> Modified_Routes;
    for(auto OneRoute:Routes){
        int n=OneRoute.size();
        vector<int> tour;
        int current = 0;
        visited[current]=1;
        while(tour.size()<n){
            int nearest=-1;
            double minDist = numeric_limits<double>::max();
            for(auto i:OneRoute){
                if(visited[i]==-1){
                    double dist = distance(c[i],c[current]);
                    if(dist<minDist){
                        minDist = dist;
                        nearest = i;
                    }
                }
            }
            current = nearest;
            visited[current] = 1;
            tour.push_back(current);
        }
        Modified_Routes.push_back(tour);
    }
    return Modified_Routes;     
}

vector<vector<int>> two_OPT(vector<vector<int>> Routes,int V,vector<pair<double,double>> p){
    vector<vector<int>> Modified_Routes;
    for(auto OneRoute:Routes){
        vector<int> tour,t;
        tour.push_back(0);
        for(auto x:OneRoute)tour.push_back(x);
        int n = tour.size();
        bool improvement = true;
        while (improvement) {
            improvement = false;
            for (int i = 0; i < n - 1; ++i) {
                for (int j = i + 1; j < n; ++j) {
                int a = tour[i];
                int b = tour[(i + 1) % n];
                int c = tour[j];
                int d = tour[(j + 1) % n];
                double distBefore = distance(p[a],p[b]) + distance(p[c], p[d]);
                double distAfter = distance(p[a], p[c]) + distance(p[b], p[d]);
                if (distAfter < distBefore) {
                    reverse(tour.begin()+(i + 1),tour.begin()+j+1);
                    improvement = true;
                }
                }
            }
        }  
        int p;
        for(int i=0;i<tour.size();i++){
            if(tour[i]==0){
                p=i;
                break;
            }
        }
        for(int i=p+1;i<tour.size();i++){
            t.push_back(tour[i]);
        }
        for(int i=0;i<p;i++){
            t.push_back(tour[i]);
        }
        Modified_Routes.push_back(t);
    }
    return Modified_Routes;
}

vector<vector<int>> Refine_Routes(vector<vector<int>> Routes,int V,vector<pair<double,double>> c){
    vector<vector<int>> Modified_Routes,RoutesOne,RoutesTwo,RoutesThree;
    RoutesOne = Nearest_Neighbour(Routes,V,c);
    RoutesTwo = two_OPT(RoutesOne,V,c);
    RoutesThree = two_OPT(Routes,V,c);
    for(int i=0;i<Routes.size();i++){
        if(cost_of_oneroute(RoutesTwo[i],0,c)>=cost_of_oneroute(RoutesThree[i],0,c)){
            Modified_Routes.push_back(RoutesThree[i]);
        }
        else Modified_Routes.push_back(RoutesTwo[i]);
    }
    return Modified_Routes;
}

int main(){
    int V,Capacity;
    cin>>V>>Capacity;
    vector<pair<double,double>> points(V);
    double Demand[V];
    for(int i=0;i<V;i++){
        double demand,x,y;
        cin>>demand>>x>>y;
        Demand[i]=demand;
        points[i]={x,y};
    }
    int E;
    cin>>E;
    vector<vector<vector<double>>> z(V,vector<vector<double>>());
    int i=0;
    while(i++<E){
        int u,v;
        cin>>u>>v;
        double w = distance(points[u],points[v]);
        vector<double> t1,t2;
        t1.push_back(v);
        t1.push_back(w);
        z[u].push_back(t1);
        t2.push_back(u);
        t2.push_back(w);
        z[v].push_back(t2);
    }       
    z = prims_mst(V,z);
    
    double minCost = INT_MAX;
    vector<vector<int>> minRoutes;
    for(int i=1;i<=pow(10,5);i++){
        vector<int> permutation;
        int visited[V];
        memset(visited,-1,sizeof(visited));
        z = randomise(z);
        permutation = dfs_visit(0,visited,z,permutation);
        vector<vector<int>> Routes = convert_to_roots(0,permutation,Demand,Capacity);
        double Cost = cost(Routes,0,points);
        if(Cost<minCost){
            minCost = Cost;
            minRoutes = Routes;
        }   
    }
    minRoutes = Refine_Routes(minRoutes,V,points);
    minCost = cost(minRoutes,0,points);
    cout<<"minimum cost = "<<minCost<<endl;
    cout<<"minimum cost routes from depot:"<<endl;
    for(auto x:minRoutes){
        for(auto y:x){
            cout<<y<<" ";
        }
        cout<<endl;
    }
}
