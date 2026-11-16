#include <gtest/gtest.h>
#include <cmath>
#include "campus_dynamic_obstacles/bounded_random_motion.hpp"

using campus_dynamic_obstacles::BoundedRandomMotion;
using campus_dynamic_obstacles::Vector;

static double length(Vector v)
{return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}

TEST(BoundedRandomMotion, GroundAndAirAreContinuousNonlinearAndBounded)
{
  for (bool air : {false, true}) {
    for (unsigned int seed : {0U, 1U, 7U, 42U, 123U, 999U}) {
    Vector low{-.65, -1.8, air ? 12. : .8};
    Vector high{.65, 1.8, air ? 18. : .8};
    BoundedRandomMotion controller(low, high, .83, 1.2, seed);
    Vector position{0., 0., air ? 15. : .8}, velocity{};
    Vector minimum=position, maximum=position;
    double turn_area=0.;
    for (int n=0; n<150000; ++n) {
      const auto next=controller.Step(position,velocity,.004);
      EXPECT_LE(length(next),.83+1e-9);
      EXPECT_LE(length({next[0]-velocity[0],next[1]-velocity[1],next[2]-velocity[2]}),1.2*.004+1e-8);
      turn_area+=std::abs(next[0]*velocity[1]-next[1]*velocity[0]);
      for (int i=0; i<3; ++i) {
        position[i]+=next[i]*.004;
        ASSERT_GE(position[i],low[i]-1e-8);
        ASSERT_LE(position[i],high[i]+1e-8);
        minimum[i]=std::min(minimum[i],position[i]);
        maximum[i]=std::max(maximum[i],position[i]);
      }
      velocity=next;
    }
    EXPECT_GT(turn_area,.5);
    EXPECT_GT(maximum[0]-minimum[0],.7);
    EXPECT_GT(maximum[1]-minimum[1],1.5);
    if (air) {EXPECT_GT(maximum[2]-minimum[2],2.);}
    else {EXPECT_DOUBLE_EQ(position[2],.8);}
    }
  }
}

TEST(BoundedRandomMotion, SeedIsRepeatableButDifferentSeedsDiffer)
{
  BoundedRandomMotion a({-3,-3,12},{3,3,18},.8,1.2,42);
  BoundedRandomMotion b({-3,-3,12},{3,3,18},.8,1.2,42);
  BoundedRandomMotion c({-3,-3,12},{3,3,18},.8,1.2,43);
  Vector p{0,0,15},v{},q=p,w{};
  for (int n=0; n<5000; ++n) {
    const auto av=a.Step(p,v,.004), bv=b.Step(p,v,.004);
    EXPECT_EQ(av,bv);
    const auto cv=c.Step(q,w,.004);
    for (int i=0; i<3; ++i) {p[i]+=av[i]*.004; q[i]+=cv[i]*.004;}
    v=av;w=cv;
  }
  EXPECT_GT(length({p[0]-q[0],p[1]-q[1],p[2]-q[2]}),.1);
}

TEST(BoundedRandomMotion, InvalidBoundsAreRejected)
{
  EXPECT_THROW(BoundedRandomMotion({0,0,0},{0,3,3},.8,1.2,42),std::invalid_argument);
  EXPECT_THROW(BoundedRandomMotion({0,0,0},{3,3,3},.8,-1,42),std::invalid_argument);
}
