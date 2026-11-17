from prototype_unified import Guide,env,np
import json
m=env.Model();k=env.WholeBodyMPPI(m,env.CFG['base_limits'],(m.lower,m.upper,np.asarray(env.CFG['arm_velocity']),np.asarray(env.CFG['arm_acceleration'])),[],enabled=False,horizon=10,step=.1,random_samples=0)
rows=[]
for gear in [-1,1]:
 for y in [-.2,-.06,.06,.2]:
  class P:
   ends=[999.];velocity=[np.array([[0.,0.1],[0.,0.]])]
   def sample(self,phase):return np.array([0.,y]),np.array([gear*.1,0.]),.1,0.,gear
  p=P();g=Guide();plant=env.Plant(m,k.limits,k.vmax,k.amax);plant.state=np.zeros(17);u=np.zeros(2)
  for tick in range(12001):
   t=.005*tick
   if tick%20==0:u=g(plant.state,None,p,0,0,t,k)
   plant.step(u,plant.state[5:11],.005)
   if np.linalg.norm(plant.state[:2]-[0,y])<=.05 and abs(float(env.wrap(plant.state[2])))<=.15:break
  rows.append(dict(gear=gear,y=y,t=t,pose=plant.state[:3].tolist(),passed=t<60,transitions=g.transitions))
print(json.dumps(rows,indent=2))
