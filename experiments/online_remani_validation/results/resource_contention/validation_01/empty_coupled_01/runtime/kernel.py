"""Generate an auditable adaptation of the already tested MPPI/CBF kernel."""
import hashlib
from pathlib import Path


def generate(build):
    source=Path(__file__).resolve().parents[1]/'base_mppi_cbf_compare/core.cpp'
    text=source.read_text().split('// Common experiment guard for ALL three methods.')[0]
    text=text.replace('namespace experiment {','namespace experiment {\nusing Distance = double (*)(double,double);\nDistance map_distance=nullptr;')
    needle='d=std::min(d,ymax-std::abs(p.y)-b.r-margin);'
    assert text.count(needle)==1
    text=text.replace(needle,needle+'\n    if(map_distance)d=std::min(d,map_distance(p.x,p.y)-b.r-margin);')
    needle='hs.push_back(ymax-p.y-b.r-c.margin);hs.push_back(ymax+p.y-b.r-c.margin);'
    assert text.count(needle)==1
    text=text.replace(needle,needle+'\n    if(map_distance)hs.push_back(map_distance(p.x,p.y)-b.r-c.margin);')
    text+='\n} // namespace experiment\n'
    dest=Path(build)/'source/online_kernel.hpp';dest.write_text(text)
    return {'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'adapted_sha256':hashlib.sha256(text.encode()).hexdigest(),'changes':['drop offline scenarios runner and shared synthetic guard','add measured 2D distance-field collision constraints; no obstacle truth input']}


if __name__=='__main__':
    import sys,json
    print(json.dumps(generate(sys.argv[1]),indent=2))
