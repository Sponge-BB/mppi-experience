#include <plan_manage/planner_manager.h>
#include "collab_factorial.hpp"
#include <iostream>
#include <stdexcept>

void require(bool value,const char* name){if(!value)throw std::runtime_error(name);}
int main(){
  ros::Time::init();
  const double now=ros::Time::now().toSec();
  std::vector<double> packet={1,now,1,2,123,now,now,now,8,6};
  for(int i=0;i<6;++i){double row[]={i*.2,i*.02,0,0,.1,0,.1,0};packet.insert(packet.end(),row,row+8);}
  collab::Seed parsed;
  require(collab::parse(packet,parsed),"valid packet");require(parsed.rows.size()==6,"row count");
  auto malformed=packet;malformed.pop_back();require(!collab::parse(malformed,parsed),"incomplete packet rejected");
  malformed=packet;malformed[10]=NAN;require(!collab::parse(malformed,parsed),"nonfinite rejected");
  malformed=packet;malformed[18]=0;require(!collab::parse(malformed,parsed),"nonmonotonic rejected");
  require(collab::parse(packet,collab::seed()),"seed parse");collab::goalToken()=123;
  Eigen::VectorXd state=Eigen::VectorXd::Zero(8);
  std::vector<std::array<double,8>> rows;std::string reason;
  require(collab::prefixRows(state,0,1,rows,reason),"fresh same task prefix eligible");
  collab::goalToken()=124;require(!collab::prefixRows(state,0,1,rows,reason),"other task rejected");
  collab::goalToken()=123;state(1)=.2;require(!collab::prefixRows(state,0,1,rows,reason),"start mismatch rejected");
  state.setZero();require(!collab::prefixRows(state,0,-1,rows,reason),"opposite gear rejected");
  collab::seed().stamp=now-1;require(!collab::prefixRows(state,0,1,rows,reason),"stale prediction rejected");

  Eigen::MatrixXd head=Eigen::MatrixXd::Zero(8,4),tail=head;head(0,1)=.1;tail(0,0)=.3;tail(0,1)=.1;
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(8,2);points(0,0)=.1;points(0,1)=.2;
  Eigen::VectorXd times=Eigen::VectorXd::Ones(3);
  poly_traj::MinSnapOpt<8> fit;fit.reset(head,tail,3);fit.generate(points,times);
  auto trajectory=fit.getTraj(1);
  require((trajectory.getPos(0)-head.col(0)).norm()<1e-8,"fit original start preserved");
  require((trajectory.getPos(3)-tail.col(0)).norm()<1e-8,"fit original goal preserved");
  require((trajectory.getVel(0)-head.col(1)).norm()<1e-8,"start velocity preserved");
  require((trajectory.getVel(3)-tail.col(1)).norm()<1e-8,"end velocity preserved");
  fit.reset(head,tail,3);fit.generate(points,times); // exercise safe re-create
  std::vector<double> field_packet={20261022,now,now,now,-1,-1,1,3,3,.1,1,1,
    0,0,.2, 0,0,0,0,.2,0};
  field_packet.insert(field_packet.end(),9,2.0);
  std::shared_ptr<const collab::Field> field;
  require(collab::parseField(field_packet,field),"projected field packet");
  require(field->gap(0,0,0,now)<0,"projected obstacle rejects target");
  require(field->gap(.8,.8,0,now)>0,"clear projected target eligible");
  require(field->gap(.8,.8,0,now+1)<0,"expired target snapshot rejected");
  field_packet.pop_back();require(!collab::parseField(field_packet,field),"truncated field rejected");
  std::cout<<"PASS seed/fit/projected-field contract checks\n";
}
