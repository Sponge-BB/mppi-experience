import json, sys, hashlib
from pathlib import Path
import roslaunch
x=json.loads(Path('/tmp/remani_phase_capacity_version.json').read_text()); root=Path(x['root']); results={}
for enabled in ['true','false']:
 config=roslaunch.config.ROSLaunchConfig()
 roslaunch.xmlloader.XmlLoader().load(str(root/'src/mm_remani/launch/exp0_real.launch'),config,argv=['use_mppi:='+enabled],verbose=False)
 flag=config.params['/mm_controller_node/controller/use_mppi_execution_limits'].value
 assert flag==(enabled=='true')
 limits={k:config.params['/whole_body_mppi/base_limits/'+k].value for k in ['vmax','wmax','wheel_v','track']} if flag else {}
 if flag: assert all(v>0 for v in limits.values())
 controllers=[{'name':n.name,'package':n.package,'type':n.type} for n in config.nodes if n.name in ['mm_controller_node','whole_body_mppi']]
 results[enabled]={'flag':flag,'limits':limits,'nodes':controllers}
results['no_nodes_started']=True
Path(x['case'],'launch_check.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
