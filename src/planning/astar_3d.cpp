#include "rescueswarm/planning/astar_3d.hpp"
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
namespace rs {
namespace { struct KeyHash { std::size_t operator()(const Cell& c) const { return (std::hash<int>{}(c.x())*73856093u)^(std::hash<int>{}(c.y())*19349663u)^(std::hash<int>{}(c.z())*83492791u); } }; struct Eq { bool operator()(const Cell&a,const Cell&b)const{return a==b;} }; }
PathResult astar3D(const VoxelGrid& g, const Cell& start, const Cell& goal, bool unknown_free) {
    PathResult r; if(!g.valid(start)||!g.valid(goal)||g.at(goal)==Occupancy::Occupied) return r;
    struct Node { int f,g; Cell c; }; struct Greater { bool operator()(const Node&a,const Node&b)const{return a.f>b.f;} };
    std::priority_queue<Node,std::vector<Node>,Greater> open; std::unordered_map<Cell,int,KeyHash,Eq> score; std::unordered_map<Cell,Cell,KeyHash,Eq> came;
    auto h=[&](const Cell& c){return (c-goal).cwiseAbs().sum();}; open.push({h(start),0,start}); score[start]=0;
    while(!open.empty()) { auto n=open.top(); open.pop(); ++r.expanded; if(n.c==goal){ for(Cell c=goal;;c=came[c]){r.cells.push_back(c);if(c==start)break;} std::reverse(r.cells.begin(),r.cells.end());r.found=true;return r; }
        if(score[n.c]!=n.g) continue; for(const auto& q:g.neighbors(n.c)){auto occ=g.at(q);if(occ==Occupancy::Occupied||(!unknown_free&&occ==Occupancy::Unknown))continue;int ng=n.g+1+(occ==Occupancy::Unknown?2:0);if(!score.contains(q)||ng<score[q]){score[q]=ng;came[q]=n.c;open.push({ng+h(q),ng,q});}}
    } return r;
}
}

