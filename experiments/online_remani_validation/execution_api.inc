// V10: verify the exact selected controls on the publication-time snapshot.
// Output rows: relative time, x, y, yaw, v, w, target_v, target_w.
extern "C" int online_selected_controls(void*p,double*out,int capacity){
  if(!p)return 0;auto&us=static_cast<Online*>(p)->m.warm;
  if(!out)return us.size();int n=std::min(capacity,int(us.size()));
  for(int i=0;i<n;++i){out[2*i]=us[i].x;out[2*i+1]=us[i].y;}return n;
}
extern "C" void online_reference_changed(void*p){
  if(p){auto&m=static_cast<Online*>(p)->m;m.route.path.clear();m.published_route.clear();}
}
extern "C" int online_validate_execution(void*p,const double*state,
    const double*controls,int count,const double*obs,int no,double age,double state_age,
    const double*field,int width,int height,double gx,double gy,double resolution,
    double*out,int capacity,double*report){
  // report: valid, selected min gap, brake min gap, rows, restoration, brake valid.
  std::fill(report,report+6,0.);
  try{
    if(!p||!field||count<1||count>64||capacity<2||age<0||age>.35||state_age<0||state_age>.2)return 1;
    for(int i=0;i<5;++i)if(!std::isfinite(state[i]))return 2;
    for(int i=0;i<no*6;++i)if(!std::isfinite(obs[i]))return 2;
    auto&h=*static_cast<Online*>(p);auto&c=h.c;
    // Solver time consumes part of the CV horizon. Validate the remaining
    // selected prefix; never extrapolate stale tail points beyond validity.
    if(no>0)count=std::min(count,int(std::floor((c.max_prediction-age-.02)/c.rdt)));
    if(count<1)return 4;
    grid=field;nx=width;ny=height;grid_x0=gx;grid_y0=gy;res=resolution;map_distance=distance;
    S initial{state[0],state[1],state[2],state[3],state[4]};
    // The previous issued command, rather than a reference derivative, bridges
    // the measured state to the current publication time.
    initial=step(initial,h.actual,state_age,c);
    Z z;z.t=age;z.fresh=true;
    for(int i=0;i<no;++i)z.os.push_back({{obs[6*i],obs[6*i+1]},{obs[6*i+2],obs[6*i+3]},obs[6*i+4],obs[6*i+5]>.5});
    double initial_gap=clearance(initial,predict(z,0,c),c,100.,c.margin);
    bool restoration=no==0&&initial_gap<0&&initial_gap+c.margin>1e-6;
    if(initial_gap<0&&!restoration)return 3;
    report[4]=restoration;
    auto check=[&](double d,double&previous,bool&restore){
      if(!std::isfinite(d))return false;
      bool ok=restore ? d+c.margin>1e-6&&d>=previous-1e-9 : d>=-1e-9;
      if(d>=0)restore=false;previous=d;return ok;
    };
    auto row=[&](int n,double t,const S&s,V u){
      if(n>=capacity)return false;
      double values[]={t,s.x,s.y,s.yaw,s.v,s.w,u.x,u.y};
      std::copy(values,values+8,out+8*n);return true;
    };
    S s=initial;double t=0,previous=initial_gap,min_gap=initial_gap;
    bool restore=restoration;int n=0;V first{controls[0],controls[1]};
    if(!row(n++,0,s,first))return 1;
    for(int k=0;k<count;++k){
      V u{controls[2*k],controls[2*k+1]};
      if(!std::isfinite(u.x)||!std::isfinite(u.y)||norm(bound(u,c)-u)>1e-9)return 2;
      for(int j=0;j<10;++j){
        t+=c.rdt/10;if(no>0&&age+t>c.max_prediction-1e-6)return 4;
        s=step(s,u,c.rdt/10,c);double gap=clearance(s,predict(z,t,c),c,100.,c.margin);
        min_gap=std::min(min_gap,gap);
        if(!check(gap,previous,restore)){report[1]=min_gap;return 3;}
        if(!row(n++,t,s,u))return 1;
      }
    }
    if(restore)return 3; // A margin-restoration rollout must restore full margin.
    report[1]=min_gap;report[3]=n;
    // Independently check the fallback available after the next issued cycle:
    // execute first action for the 75ms actuator lease, then brake with the same lag/wheel model.
    s=initial;t=0;previous=initial_gap;restore=restoration;double brake_gap=initial_gap,settled=0;
    for(int j=0;j<200;++j){
      V u=t<.075-1e-8?first:V{};t+=.01;
      if(no>0&&age+t>c.max_prediction-1e-6)break;
      s=step(s,u,.01,c);double d=clearance(s,predict(z,t,c),c,100.,c.margin);
      brake_gap=std::min(brake_gap,d);
      // Static restoration may safely hold its positive physical clearance
      // while braking; it does not require braking back out to full margin.
      bool ok=restoration ? d+c.margin>1e-6 : check(d,previous,restore);
      if(!ok){report[2]=brake_gap;return 5;}
      if(t>=.075&&std::abs(s.v)<.01&&std::abs(s.w)<.03)settled+=.01;else settled=0;
      if(settled>=.10){report[5]=1;break;}
    }
    report[2]=brake_gap;if(!report[5])return 5;
    report[0]=1;return 0;
  }catch(...){return 6;}
}
