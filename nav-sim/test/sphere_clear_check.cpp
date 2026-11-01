// sphere_clear_check -- the fast sphereClear (per-axis distances, slices and
// rows skipped when out of reach) answers exactly as the cell-by-cell
// reference does, over random balls on a random map, with and without a
// required core. It is the hottest loop of a simulator step and the veto
// every planner is filtered by, so "faster" must mean "the same".
#include <cstdio>
#include <random>
#include "voxel_map.hpp"
#include "voxel_world.hpp"
using namespace sim;
int main(){
  VoxelWorld w; w.init(0.25f,-15,-15,0,120,120,40);
  std::mt19937 g(3); std::uniform_int_distribution<int> X(0,119),Z(0,39);
  for(int i=0;i<20000;i++) w.set(X(g),X(g),Z(g));
  VoxelMapParams mp; VoxelMap m; m.init(mp,0,0,3); m.imprint(w,0,0,3,14,4);
  std::uniform_real_distribution<float> P(-16,16),Pz(-1,8),R(0.1f,1.2f),C(0,1);
  long bad=0,n=0,clear=0;
  for(int i=0;i<300000;i++){ float x=P(g),y=P(g),z=Pz(g),r=R(g),c=C(g)<0.5f?0.f:C(g);
    bool a=m.sphereClear(x,y,z,r,c), b=m.sphereClearSlow(x,y,z,r,c); bad+=(a!=b); ++n; clear+=a; }
  printf("sphereClear fast vs reference: %ld/%ld disagree (%ld clear)\n",bad,n,clear); return bad?1:0;
}
