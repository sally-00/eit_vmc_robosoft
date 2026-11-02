#include "franka_eit_vmc/ball_mechanism.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace franka_eit_vmc;
void check(bool ok, const char* why) { if(!ok) throw std::runtime_error(why); }
int main(int argc,char** argv) {
 try {
  Config c; c.task=Task::Ball;
  auto free=ball_normal_step(c,.3,0.,.3,0.,false,true,.001,.3);
  check(free.position<.3 && free.velocity<0.,"released unloaded carriage must descend");
  auto brake=ball_normal_step(c,.3,0.,.3,0.,true,true,.001,.3);
  check(brake.position==.3 && brake.velocity==0.,"one-way brake must block descent");
  auto lift=ball_normal_step(c,.3,0.,.308,0.,true,true,.001,.3);
  check(lift.position>.3 && lift.velocity>0.,"support rise must yield upward from virtual reaction");
  auto lost=ball_normal_step(c,.3,.01,.308,0.,false,false,.001,.3);
  check(lost.position==.3 && lost.velocity==0.,"invalid data must hold both directions");
  double recovery_z=.3, recovery_v=.01;
  for(int i=0;i<100;++i) {
    const auto step=ball_normal_step(c,recovery_z,recovery_v,.308,0.,false,true,.001,.3);
    check(!step.at_stop,"unloaded recovery unexpectedly hit a carriage stop");
    recovery_z=step.position; recovery_v=step.velocity;
  }
  check(recovery_z<.3 && recovery_v<0.,"unloaded virtual carriage rose instead of searching down");
  auto limit=ball_normal_step(c,.275,-.015,.275,0.,false,true,.001,.3);
  check(limit.at_stop && std::abs(limit.position-.275)<1e-12 && limit.velocity==0.,"downward search exceeded travel bound");
  auto floor=ball_normal_step(c,.0502,-.015,.076,0.,false,true,.02,.07368,.0501);
  check(floor.at_stop && std::abs(floor.position-.0501)<1e-12 && floor.velocity==0.,
        "lost-contact recovery crossed the workspace floor");
  c.normal_force_limit=4.;
  check(ball_normal_step(c,.3,0.,.4,0.,true,true,.001,.3).force==-4.,"hardware force cap was not retained");
  check(ball_drive_force(c,.32,.30,0.,.30,0.,.30,1.,true,0.,.001)>0.,"goal force sign wrong");
  check(ball_drive_force(c,.28,.30,0.,.30,0.,.30,1.,true,0.,.001)<0.,"negative goal force sign wrong");
  check(ball_drive_force(c,.32,.30,0.,.30,0.,.30,1.,false,0.,.001)==0.,"lost-contact hold must not chase goal");
  double paused_force=-5.;
  for(int i=0;i<100;++i)
    paused_force=ball_drive_force(c,.28,.30,0.,.30,-.15,.30,1.,false,paused_force,.001);
  check(paused_force>-.1,"lost-contact lateral drive did not unwind promptly");
  check(ball_drive_force(c,.32,.30,1.,.30,0.,.30,1.,true,0.,.001)<0.,"estimated velocity must damp drive");
  const double proportional=ball_drive_force(c,.31,.30,0.,.30,0.,.30,1.,true,0.,1.);
  const double advanced=ball_drive_force(c,.315,.30,0.,.30,0.,.30,1.,true,0.,1.);
  check(advanced>proportional && advanced<=std::min(c.goal_force_limit,c.force_xy_limit),
        "advanced spring reference did not increase bounded lateral drive");
  check(argc==2,"missing saved simulation oracle fixture");
  std::ifstream in(argv[1]);check(in.good(),"cannot read saved simulation fixture");
  c.normal_force_limit=7.;int n=0;std::string line;
  while(std::getline(in,line)) {
    if(line.empty()||line[0]=='#')continue;
    std::istringstream row(line);
    double z,v,p,pv,brake_value,valid,dt,origin,force,next_z,next_v;
    check(bool(row>>z>>v>>p>>pv>>brake_value>>valid>>dt>>origin>>force>>next_z>>next_v),"malformed oracle row");
    // The saved simulation predates the unloaded recovery correction. Its
    // contacting states still provide an independent normal-carriage oracle.
    if(brake_value<.5)continue;
    const auto result=ball_normal_step(c,z,v,p,pv,true,valid>.5,dt,origin);
    check(std::abs(result.force-force)<1e-10 && std::abs(result.position-next_z)<1e-12 &&
          std::abs(result.velocity-next_v)<1e-12,"C++ normal mechanism differs from saved simulation");++n;
  }
  check(n>100,"insufficient saved simulation states");
  std::cout<<"Direct ball force and recovery checks passed; "<<n<<" contacting simulation states match\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
