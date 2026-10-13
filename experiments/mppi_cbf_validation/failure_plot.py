#!/usr/bin/env python3
"""Show the recorded failed run explicitly; do not select a successful seed."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from run import COLORS, load_trace


def plot(out):
    fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True)
    for mode,color in COLORS.items():
        d=load_trace(out/f'delayed_crossing__{mode}__2.csv')
        for ax,key in zip(axes,['v','command_v','clearance','infeasible']):
            ax.plot(d['t'],d[key],c=color,label=mode,lw=1.4)
    nominal=load_trace(out/'delayed_crossing__mppi_cbf__2.csv')
    failures=np.where(nominal['infeasible']>0)[0]
    collisions=np.where(nominal['clearance']<0)[0]
    first_failure=float(nominal['t'][failures[0]]) if len(failures) else None
    first_collision=float(nominal['t'][collisions[0]]) if len(collisions) else None
    axes[1].plot(nominal['t'],nominal['nominal_v'],c='gray',ls='--',lw=1,label='MPPI nominal before filter')
    for ax,label in zip(axes,['Actual v [m/s]','Velocity target [m/s]','Truth clearance [m]','Filter failure flag']):
        ax.set_ylabel(label);ax.set_xlim(3,7);ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper right')
        ax.axvspan(4,4.45,color='gray',alpha=.12)
        if first_failure is not None:ax.axvline(first_failure,color='red',ls=':',lw=1)
        if first_collision is not None:ax.axvline(first_collision,color='red',ls='--',lw=1)
    axes[2].axhline(0,color='red',lw=1);axes[-1].set_xlabel('Simulation time [s]')
    fig.suptitle(f'Failure diagnostics | delayed_crossing, seed 2\nFirst filter failure: {first_failure} s; first modeled collision: {first_collision} s')
    fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(out/'failure_delayed_crossing_seed2.png',dpi=170);plt.close(fig)
    print(out/'failure_delayed_crossing_seed2.png')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('out',type=Path)
    args=parser.parse_args();plot(args.out.resolve())
