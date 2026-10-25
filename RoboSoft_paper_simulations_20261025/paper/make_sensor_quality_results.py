"""Plot the frozen 36-trial position-observation bias/delay sweep; no simulations."""
from pathlib import Path
import csv, hashlib, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from figure_style import annotation_color

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'evidence/ball_followup/analyzed_metrics.csv'
rows=[r for r in csv.DictReader(SOURCE.open()) if r['group']=='quality']
assert len(rows)==36
biases=[-5,0,5]
delays=[0,100,250,500]
errors=np.zeros((3,4));counts=np.zeros((3,4),dtype=int);cells=[]
for i,bias in enumerate(biases):
    for j,delay in enumerate(delays):
        cell=sorted([r for r in rows if np.isclose(float(r['progress_bias'])*1000,bias)
                     and np.isclose(float(r['progress_delay'])*1000,delay)],key=lambda r:int(r['seed']))
        assert len(cell)==3 and len({r['seed'] for r in cell})==3
        assert all(r['late_window_available']=='True' for r in cell)
        assert all(r['attachment']=='centre' and r['progress_source']=='eit' and r['case']=='forward_4N' for r in cell)
        values=np.array([float(r['late_error_mm']) for r in cell])
        assert np.isfinite(values).all()
        errors[i,j]=values.mean()
        counts[i,j]=sum(r['common_success']=='True' for r in cell)
        cells.append(dict(bias_mm=bias,added_delay_ms=delay,n=3,
                          mean_final_error_mm=float(errors[i,j]),successes=int(counts[i,j]),
                          trial_ids=[int(r['trial_id']) for r in cell],
                          final_error_mm_by_trial=values.tolist()))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7,'pdf.fonttype':42,'svg.fonttype':'none'})
fig=plt.figure(figsize=(3.45,2.05))
axes=[fig.add_axes([.16,.38,.35,.45]),fig.add_axes([.61,.38,.35,.45])]
error_image=axes[0].imshow(errors,aspect='auto',vmin=0,vmax=6,cmap='cividis_r')
count_map=ListedColormap(plt.get_cmap('Blues')([.12,.36,.60,.86]))
count_image=axes[1].imshow(counts,aspect='auto',cmap=count_map,norm=BoundaryNorm([-.5,.5,1.5,2.5,3.5],4))
for k,ax in enumerate(axes):
    ax.set_xticks(range(4),[str(d) for d in delays])
    ax.set_yticks(range(3),['−5','0','+5'] if k==0 else [])
    ax.set_xticks(np.arange(-.5,4,1),minor=True)
    ax.set_yticks(np.arange(-.5,3,1),minor=True)
    ax.grid(which='minor',color='white',linewidth=.9)
    ax.tick_params(which='both',length=0,pad=3,labelsize=6.7)
    for spine in ax.spines.values():spine.set_visible(False)
    for i in range(3):
        for j in range(4):
            ax.text(j,i,f'{errors[i,j]:.2f}' if k==0 else f'{counts[i,j]}/3',
                    ha='center',va='center',fontsize=7,
                    color=annotation_color(error_image.cmap(error_image.norm(errors[i,j])) if k==0 else count_image.cmap(count_image.norm(counts[i,j]))))
axes[0].set_title('(a) Final error (mm)',fontsize=7.3,pad=7)
axes[1].set_title('(b) Task completion',fontsize=7.3,pad=7)
axes[0].set_ylabel('Added position bias (mm)',fontsize=7,labelpad=3)
fig.text(.56,.21,'Added position-feedback delay (ms)',ha='center',fontsize=7)
for bounds,img,ticks in [([.16,.105,.35,.035],error_image,[0,3,6]),([.61,.105,.35,.035],count_image,[0,1,2,3])]:
    cb=fig.colorbar(img,cax=fig.add_axes(bounds),orientation='horizontal',ticks=ticks)
    cb.outline.set_visible(False);cb.ax.tick_params(length=2,labelsize=6,pad=2)
    if img is count_image:cb.ax.set_xticklabels(['0/3','1/3','2/3','3/3'])
for ext in ['pdf','png','svg']:
    fig.savefig(ROOT/'figures'/f'sensor_quality_results.{ext}',dpi=280,bbox_inches='tight',facecolor='white')
plt.close(fig)
provenance={'figure':'sensor_quality_results','simulation_rerun':False,
            'source':str(SOURCE.relative_to(ROOT)),'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
            'error_metric':'late_error_mm: mean absolute rolling-direction target error during final second (7–8 s)',
            'completion_metric':'common_success: all task success criteria, not only position error',
            'cells':cells}
(ROOT/'evidence/sensor_quality_figure_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
print(json.dumps({'mean_error_mm':errors.tolist(),'successes_out_of_three':counts.tolist()},indent=2))
