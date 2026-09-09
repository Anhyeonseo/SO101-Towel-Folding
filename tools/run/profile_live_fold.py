"""Read-only timings around a live fold's step, render, and contact inspection."""
import json,runpy,sys,time,os
import faulthandler,signal
import cProfile
profile=cProfile.Profile()
profile_enabled=os.environ.get('SO101_CPROFILE')=='1'
faulthandler.enable()
faulthandler.register(signal.SIGUSR1,all_threads=True)
faulthandler.dump_traceback_later(90,repeat=True)
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
import tools.lib.simulation_rendering as rendering
original=rendering.decimate_rendering
original_pacing=rendering.pace_live_rendering
out=Path(sys.argv[1]);sys.argv=sys.argv[2:]
def instrument(step,interval,pacing_dt=None):
 sim=step.__self__;wrapped=(original_pacing(step,pacing_dt) if pacing_dt is not None else original(step,interval));render=sim.render
 stats={'steps':0,'renders':0,'step_s':0.,'render_s':0.,'contact_s':0.,'contact_calls':0}
 start=None;last=None;hooked=False;previous_stats=dict(stats)
 def timed_render(*a,**kw):
  t=time.perf_counter()
  try:return render(*a,**kw)
  finally:stats['render_s']+=time.perf_counter()-t;stats['renders']+=1
 sim.render=timed_render
 def timed_step(render=True):
  nonlocal start,last,hooked,previous_stats
  if not hooked:
   namespace=sys._getframe(1).f_globals
   snapshot=namespace.get('newton_soft_contact_snapshot')
   if snapshot is not None:
    def timed_snapshot(*a,**kw):
     t=time.perf_counter()
     try:return snapshot(*a,**kw)
     finally:stats['contact_s']+=time.perf_counter()-t;stats['contact_calls']+=1
    namespace['newton_soft_contact_snapshot']=timed_snapshot;hooked=True
  t=time.perf_counter()
  if start is None:
   start=t;last=t
   if profile_enabled:profile.enable()
  result=wrapped(render=render)
  stats['step_s']+=time.perf_counter()-t;stats['steps']+=1
  now=time.perf_counter()
  if now-last>=10:
   report={**stats,'wall_s':now-start,'simulation_s':stats['steps']*sim.get_physics_dt(),
     'viewport_frames_per_wall_second':stats['renders']/(now-start),
     'simulation_to_wall_ratio':stats['steps']*sim.get_physics_dt()/(now-start),
     'window_viewport_fps':(stats['renders']-previous_stats['renders'])/(now-last),
     'window_simulation_to_wall_ratio':(stats['steps']-previous_stats['steps'])*sim.get_physics_dt()/(now-last),
     'scope':'Wall timings include GPU synchronization at existing call sites; categories can overlap.'}
   (out/'live_timing.json').write_text(json.dumps(report,indent=2)+'\n')
   if profile_enabled:
    profile.disable();profile.dump_stats(str(out/'live_profile_latest.pstats'));profile.clear();profile.enable()
   print('S1_LIVE_TIMING '+json.dumps(report),flush=True);last=now;previous_stats=dict(stats)
  return result
 return timed_step
rendering.decimate_rendering=instrument
rendering.pace_live_rendering=lambda step,dt:instrument(step,1,pacing_dt=dt)
runpy.run_path(sys.argv[0],run_name='__main__')
