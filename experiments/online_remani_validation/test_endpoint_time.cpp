#include "endpoint_time_replay.hpp"
#include <cassert>
#include <iostream>
struct Result {int dim=2;std::vector<int> pieces{1,2,4};Eigen::VectorXd first_x;};
double virtualTime(double t){return t>1.?std::sqrt(2.*t-1.)-1.:1.-std::sqrt(2./t-1.);}
int main(){
  std::vector<Eigen::VectorXd> input(3);
  input[0]=Eigen::VectorXd::Constant(1,2.);
  input[1]=Eigen::Vector2d(1.,2.);
  input[2]=Eigen::Vector4d(1.,2.,3.,4.);
  const auto unchanged=endpoint_time::initialTimes(input,false);
  const auto adjusted=endpoint_time::initialTimes(input,true);
  for(std::size_t n=0;n<input.size();++n)assert((input[n].array()==unchanged[n].array()).all());
  assert(adjusted[0][0]==3.); // singleton first==last is multiplied once.
  assert(adjusted[1][0]==1.5&&adjusted[1][1]==3.);
  assert(adjusted[2][0]==1.5&&adjusted[2][1]==2.&&adjusted[2][2]==3.&&adjusted[2][3]==6.);
  assert(input[0][0]==2.&&input[2][3]==4.); // caller's arrays are untouched.
  Result original,changed;
  const int inner_count=2*((1-1)+(2-1)+(4-1)),time_count=7,gear_count=3*(3-1);
  original.first_x=Eigen::VectorXd::LinSpaced(inner_count+time_count+gear_count,0.,2.);
  changed.first_x=original.first_x;int at=inner_count;
  for(std::size_t n=0;n<input.size();++n)for(int k=0;k<input[n].size();++k,++at){
    original.first_x[at]=virtualTime(input[n][k]);changed.first_x[at]=virtualTime(adjusted[n][k]);
  }
  assert(endpoint_time::endpointCoordinates(original,changed));
  auto bad=changed;bad.first_x[inner_count+4]=virtualTime(4.);
  assert(!endpoint_time::endpointCoordinates(original,bad)); // untouched middle piece changed.
  bad=changed;bad.first_x[0]+=.01;assert(!endpoint_time::endpointCoordinates(original,bad));
  bool rejected=false;try{auto bad_input=input;bad_input[0][0]=0.;endpoint_time::initialTimes(bad_input,true);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected);
  std::cout<<"PASS endpoint mask, singleton deduplication, immutable input, non-time coordinates, positive durations\n";
}
