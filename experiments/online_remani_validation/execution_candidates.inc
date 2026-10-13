// Export bounded, distinct, already kernel-feasible alternatives. This is NOT
// execution approval: every attempted sequence needs the fresh rollout/brake
// validator, including the original winner at index zero.
extern "C" int online_execution_candidates(void*p,double*out,int capacity,int stride,double*lengths){
  if(!p||!out||!lengths||capacity<1||capacity>16||stride<1||stride>64)return 0;
  auto&m=static_cast<Online*>(p)->m;
  std::vector<const std::vector<V>*> candidates;
  if(!m.warm.empty())candidates.push_back(&m.warm);
  std::vector<const MPPI::SearchTrace*> ranked;
  for(const auto&r:m.search_trace)if(r.feasible&&!r.controls.empty())ranked.push_back(&r);
  std::stable_sort(ranked.begin(),ranked.end(),[](const auto*a,const auto*b){return a->cost<b->cost;});
  for(auto*r:ranked){
    if(int(candidates.size())>=capacity)break;
    if(int(r->controls.size())>stride)continue;
    bool duplicate=false;
    for(auto*prior:candidates){
      if(prior->size()!=r->controls.size())continue;
      double difference=0;
      for(size_t i=0;i<prior->size();++i)difference=std::max(difference,norm((*prior)[i]-r->controls[i]));
      if(difference<1e-5){duplicate=true;break;}
    }
    if(!duplicate)candidates.push_back(&r->controls);
  }
  int n=0;
  for(auto*us:candidates){
    if(int(us->size())>stride)continue;
    lengths[n]=us->size();
    for(size_t k=0;k<us->size();++k){out[2*(n*stride+k)]=(*us)[k].x;out[2*(n*stride+k)+1]=(*us)[k].y;}
    ++n;
  }return n;
}

extern "C" int online_accept_execution_controls(void*p,const double*controls,int count){
  if(!p||count<1||count>64)return 1;
  auto&h=*static_cast<Online*>(p);std::vector<V> checked;
  for(int i=0;i<count;++i){
    V u{controls[2*i],controls[2*i+1]};
    if(!std::isfinite(u.x)||!std::isfinite(u.y)||norm(bound(u,h.c)-u)>1e-9)return 2;
    checked.push_back(u);
  }
  h.m.warm=std::move(checked);return 0;
}
