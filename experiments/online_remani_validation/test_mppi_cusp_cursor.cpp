#include "mppi_cusp_cursor.hpp"
#include <cassert>
#include <iostream>
using namespace mppi_cusp;
int main(){
  const std::vector<Sample>r={{0,0,0,-.1},{-1,0,0,-.05},{-1,0,0,.05},{0,0,0,.1}};
  Cursor c;double d[14]{};
  assert(c.update(r,{0,0,0,0,0}));c.debug(d);assert(d[1]==0&&d[2]==2&&d[5]==-1);
  assert(!c.update(r,{-.5,0,0,0,0}));
  assert(!c.update(r,{-1,0,0,.1,0})); // Must slow down before switching.
  assert(!c.update(r,{-1,0,0,0,.3})); // No switch while rotating rapidly.
  assert(!c.update(r,{-1,0,.4,0,0})); // No switch while misaligned.
  assert(!c.update(r,{-1,.06,0,0,0})); // Must actually reach the cusp.
  assert(!c.update(r,{-1,0,0,0,0}));  // Two actual measurements, not one.
  assert(c.update(r,{-1,0,0,0,0}));c.debug(d);assert(d[1]==1&&d[9]==1&&d[10]==2);
  assert(!c.update(r,{-1,0,0,0,0})); // Final phase cannot repeatedly switch.
  auto changed=r;changed.back().x=.1;
  assert(c.update(changed,{-1,0,0,0,0}));c.debug(d);assert(d[1]==0&&d[9]==0);
  c.reset();c.debug(d);assert(d[0]==0);
  auto zeros=r;for(auto&x:zeros)x.v=0;
  assert(c.update(zeros,{0,0,0,0,0}));c.debug(d);assert(d[2]==1);
  auto tiny=r;tiny[1].v=1e-10;
  c.reset();assert(c.update(tiny,{0,0,0,0,0}));c.debug(d);assert(d[2]==2);
  bool invalid=false;try{c.update(r,{NAN,0,0,0,0});}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
  invalid=false;auto bad=r;bad[1].yaw=NAN;
  try{c.update(bad,{0,0,0,0,0});}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
  // Geometrically close to a later cusp is insufficient if the active path
  // still has a long suffix: no nearest-point leap over a hairpin.
  std::vector<Sample>loop={{0,0,0,-.1},{-1,0,0,-.1},{-1,.02,0,-.1},
    {0,.02,0,-.1},{0,.02,0,.1},{1,.02,0,.1}};
  c.reset();c.update(loop,{0,0,0,0,0});
  for(int i=0;i<5;++i)assert(!c.update(loop,{0,0,0,0,0}));
  std::cout<<"CURSOR_CHECKS_PASSED\n";
}
