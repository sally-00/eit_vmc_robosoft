"""Torque-driven UR3e trials with physical objects and a tactile supervisor."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src'))
import json
import time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from scene import make_scene, observe_contact, LIMITS, R0, FACE_Z
from sensing import Sensor

STATES = {'approach': 0, 'engage': 1, 'interact': 2, 'brake': 3,
          'lift': 4, 'reposition': 5, 'recover': 6, 'done': 7}
TRACE_COLUMNS = ['t', 'force_N', 'rim_force_N', 'theta_rad', 'ball_x', 'ball_y',
                 'ball_rotation_y', 'sensor_x', 'sensor_y', 'valid', 'state',
                 'z', 'zdot', 'progress_estimate', 'palm_x', 'palm_z', 'slip_m',
                 'sensor_norm', 'rhat_x', 'rhat_y', 'task_force_x', 'normal_force_z',
                 'torque_norm', 'spring_energy_J', 'virtual_energy_J', 'omega_y',
                 'bottom_slip_m', 'goal', 'task_work_J', 'ball_vx', 'palm_pitch',
                 'true_contact_x', 'external_force_x', 'raw_progress_x']


def smooth(x):
    x = np.clip(x, 0., 1.)
    return x*x*(3-2*x)


def run_trial(cfg, save_path=None):
    started = time.perf_counter()
    m, d, xml, info = make_scene(cfg)
    dt = m.opt.timestep
    sensor = Sensor(cfg['seed'], cfg)
    rng = np.random.default_rng(cfg['seed']+40000)
    body = m.body('tool').id; site = m.site('sensor').id
    object_id = m.body('object').id
    Jp = np.zeros((3, m.nv)); Jw = np.zeros_like(Jp)
    Jc = np.zeros_like(Jp); Jorigin = np.zeros_like(Jp)
    state = 'approach'; phase_t = 0.; engagement_t = None
    events = []; trace = []; qs = []; mocaps = []
    initial_face = d.site_xpos[site].copy()
    approach_ref = initial_face.copy(); hold_ref = initial_face.copy()
    rhat = np.array([0., 0., FACE_Z])
    h0 = np.zeros(3); normal0 = 0.; z = 0.; zdot = 0.; z0 = 0.
    origin_ball = float(info['initial_object'][0])+rng.normal(0, .0005)
    observed = origin_ball; stroke_origin = observed
    goal_ball = origin_ball+cfg.get('distance', .025)
    goal = 0.; brake_goal = 0.; normal_age = 0.
    strokes = 0; recoveries = 0; desired_stick = 0.
    acquire_times = []; premature = 0; acquisition_delays = []
    first_contact = None; first_stroke_contact = None; angle_rotation = 0.
    peak_force = 0.; peak_rim = 0.; peak_speed = 0.; peak_tau = 0.
    top_slip = 0.; bottom_slip = 0.; torque_saturated = 0
    rebase_error = 0.; update_count = 0; step_force_jump = 0.
    task_work = 0.; max_virtual_energy = 0.; max_elastic_energy = 0.
    failed_guard = ''; completed = False; joystick_evals = []
    command_work_offcontact = 0.; disconnected_distance = 0.
    last_face = initial_face.copy()
    interruption_t = cfg.get('interrupt_at', 2.5) if cfg.get('interrupt', False) else None
    lost_time = None
    recovery_time = None; table_home = d.mocap_pos.copy()
    last_touch = None; coupling_gain = 0.; selected_once = False
    Kt = np.array([700., 700., 0.]); Dt = np.array([30., 30., 0.])
    Kn, Dn = 800., 60.
    horizon = cfg.get('horizon', 16. if cfg['task']=='joystick' else 28.)
    mechanism = cfg.get('mechanism', 'mechanism')
    attachment = cfg.get('attachment', 'eit')
    switching = cfg.get('switching', 'event')
    force_prev = np.zeros(3)
    object_velocity = np.zeros(6)
    location_errors = []; false_valid_frames = 0; valid_frames = 0
    resistance_lock = None
    stop_time = None; stop_dwell = 0.; disconnected_progress = 0.
    step_budget = round(horizon/dt)
    progress_queue = []
    progress_source = cfg.get('progress_source', 'eit')
    raw_progress = origin_ball
    evaluation_start = None
    progress_errors = []
    disturbances = cfg.get('disturbances', [])
    table_shift = cfg.get('table_shift', 0.)

    def change(new_state, t, reason):
        nonlocal state, phase_t
        events.append({'t': round(float(t), 6), 'from': state, 'to': new_state,
                       'reason': reason, 'force_N': float(force),
                       'sensor_valid': bool(sensor.valid), 'progress': float(observed)})
        state, phase_t = new_state, t

    def engage(t, reason):
        nonlocal rhat, h0, normal0, z, zdot, z0, stroke_origin
        nonlocal engagement_t, strokes, premature, selected_once, normal_age, evaluation_start
        if attachment == 'centre':
            chosen = np.array([0., 0., FACE_Z])
        elif attachment == 'oracle' and sensor.oracle is not None:
            chosen = sensor.oracle.copy()
        elif sensor.valid:
            chosen = sensor.location.copy()
        else:
            chosen = np.array([0., 0., FACE_Z])
        if np.linalg.norm(chosen[:2]) > .036:
            chosen[:2] *= .036/np.linalg.norm(chosen[:2])
        rhat = chosen
        pc0 = o+R@rhat
        h0 = pc0.copy(); normal0 = pc0[2]
        z = observed if cfg['task']=='ball' else 0.
        z0 = z; zdot = 0.; stroke_origin = observed
        engagement_t = t; normal_age = t; strokes += 1
        if evaluation_start is None:
            evaluation_start = t
        selected_once = sensor.valid
        acquire_times.append(float(t))
        if force < .05:
            premature += 1
        if first_stroke_contact is not None:
            acquisition_delays.append(float(t-first_stroke_contact))
        change('engage', t, reason)

    for step in range(step_budget):
        t = step*dt
        # A separate fixture moves the table; controller receives no event flag.
        if cfg.get('interrupt', False) and interruption_t is not None:
            age = t-interruption_t
            depth = .02*(smooth(age/.15)-smooth((age-.55)/1.2))
            d.mocap_pos[0, 2] = table_home[0, 2]-depth
        if table_shift and evaluation_start is not None:
            d.mocap_pos[0, 0] = table_home[0, 0]+table_shift*smooth((t-evaluation_start-2.)/1.)
        mujoco.mj_forward(m, d)
        force, local, contact_world, rim, slip = observe_contact(m, d)
        o = d.xpos[body].copy(); R = d.xmat[body].reshape(3, 3).copy()
        face = d.site_xpos[site].copy()
        obj = d.xpos[object_id].copy()
        d.qfrc_applied[:] = 0.; d.xfrc_applied[:] = 0.
        disturbance_age = t-evaluation_start if evaluation_start is not None else -1.
        for pulse in disturbances:
            if pulse['start'] <= disturbance_age < pulse['start']+pulse['duration']:
                d.xfrc_applied[object_id, 0] += pulse['force']
        if cfg.get('resistance', False) and cfg.get('resistance_start', 2.5) <= disturbance_age <= cfg.get('resistance_end', 3.3):
            if resistance_lock is None:
                resistance_lock = float(d.qpos[6] if cfg['task']=='joystick' else obj[0])
            if cfg['task']=='joystick':
                d.qfrc_applied[6] = -np.clip(5*(d.qpos[6]-resistance_lock)+.1*d.qvel[6], -.2, .2)
            else:
                d.xfrc_applied[object_id, 0] = -np.clip(1800*(obj[0]-resistance_lock)+15*d.qvel[6], -3., 3.)
        if force > .05:
            if first_contact is None:
                first_contact = t
            if first_stroke_contact is None:
                first_stroke_contact = t
            last_touch = t
        fresh = sensor.step(t, dt, force, local)
        if fresh and sensor.valid:
            b_est = o+R@sensor.location
            if progress_source == 'oracle' and local is not None:
                raw_progress = float((o+R@local)[0])
            elif progress_source == 'odometry':
                raw_progress = origin_ball+.5*(face[0]-initial_face[0])
            else:
                raw_progress = float(b_est[0])+cfg.get('progress_bias', 0.)
            progress_queue.append((t+cfg.get('progress_delay',0.), raw_progress))
            valid_frames += 1
            if local is None:
                false_valid_frames += 1
            elif force > .2:
                location_errors.append(float(np.linalg.norm(sensor.location[:2]-local[:2])*1000))
        while progress_queue and progress_queue[0][0] <= t+1e-10:
            _, sample = progress_queue.pop(0)
            in_dropout = any(a <= disturbance_age < b for a,b in cfg.get('progress_dropouts', []))
            if not in_dropout:
                observed = .65*observed+.35*sample
        if evaluation_start is not None:
            progress_errors.append(abs(observed-obj[0]))
        peak_force = max(peak_force, force); peak_rim = max(peak_rim, rim)
        peak_speed = max(peak_speed, float(np.max(abs(d.qvel[:6]))))
        top_slip += slip*dt
        if cfg['task']=='ball':
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY,
                                    object_id, object_velocity, 0)
            vx, wy = object_velocity[3], object_velocity[1]
            angle_rotation += wy*dt
            if force > .05:
                bottom_slip += abs(vx-info['radius']*wy)*dt
        if interruption_t is not None and interruption_t <= t < interruption_t+2.0 and force < .05:
            disconnected_distance += np.linalg.norm(face[:2]-last_face[:2])
            if lost_time is None:
                lost_time = t

        # Contact-onset uncertainty changes the event time, not the nominal timer.
        if state in ['approach', 'recover']:
            elapsed = t-phase_t
            ready = sensor.valid if switching == 'event' or state=='recover' else elapsed >= cfg.get('nominal_time', .95)
            if ready:
                if state == 'recover':
                    recoveries += 1
                engage(t, 'persistent_touch' if switching=='event' else 'nominal_timer')
            elif elapsed > 4.0:
                failed_guard = 'acquisition_timeout'; break
        if state == 'engage' and t-phase_t >= .55:
            change('interact', t, 'preload_complete')
        if state in ['engage', 'interact'] and switching == 'event' and not sensor.valid:
            if t-normal_age > .15:
                hold_ref = face.copy(); approach_ref = face.copy()
                if lost_time is not None and recovery_time is None:
                    recovery_time = t
                first_stroke_contact = None
                change('recover', t, 'contact_loss_or_stale_frame')

        # Spatial updates occur at acquired frames. Both ball conditions receive
        # the same location-derived object-position feedback above.
        if state in ['engage', 'interact', 'brake'] and fresh and sensor.valid:
            can_update = (cfg['task']=='ball' and attachment!='latched') or not selected_once
            if can_update and attachment != 'centre':
                selected = sensor.oracle if attachment=='oracle' else sensor.location
                if selected is not None:
                    delta = .45*(selected-rhat)
                    delta[2] = 0.
                    norm = np.linalg.norm(delta)
                    if norm > .0025:
                        delta *= .0025/norm
                    new = rhat+delta
                    if np.linalg.norm(new[:2]) > .036:
                        new[:2] *= .036/np.linalg.norm(new[:2])
                    shift = R@(new-rhat)
                    old_extension = h0-(o+R@rhat)
                    h0 += shift; normal0 += shift[2]
                    rhat = new
                    rebase_error = max(rebase_error, np.linalg.norm(old_extension-(h0-(o+R@rhat))))
                    update_count += 1; selected_once = True
            if cfg['task']=='ball':
                correction = np.clip(.20*(observed-z), -.003, .003)
                z += correction
                h0[0] -= 2*correction  # Preserve extension at state correction.

        pc = o+R@rhat
        mujoco.mj_jac(m, d, Jc, Jw, pc, body)
        vc = Jc@d.qvel; omega = Jw@d.qvel
        fv = np.zeros(3); fc = np.zeros(3); elastic = np.zeros(3)
        normal_force = 0.; virtual_energy = 0.; alpha = 0.
        desired_R = R0
        if state in ['approach', 'recover', 'lift', 'reposition']:
            mujoco.mj_jacSite(m, d, Jp, Jw, site)
            if state in ['approach', 'recover']:
                speed = .008 if state=='approach' else .004
                reference = approach_ref-[0, 0, min(.024 if state=='approach' else .008, speed*(t-phase_t))]
            elif state == 'lift':
                reference = hold_ref+[0, 0, .020*smooth((t-phase_t)/.6)]
                if t-phase_t > .85:
                    # Same between-stroke camera measurement for all conditions.
                    camera_x = obj[0]+rng.normal(0, .0005)
                    next_gap = np.clip(cfg['gap']+rng.uniform(-.003, .003), .002, .018)
                    reposition_start = face.copy()
                    reposition_goal = np.array([camera_x, 0, info['top']+next_gap])
                    change('reposition', t, 'between_stroke_camera')
            else:
                reference = reposition_start+smooth((t-phase_t)/1.2)*(reposition_goal-reposition_start)
                if t-phase_t > 1.4:
                    approach_ref = face.copy(); first_stroke_contact = None
                    change('approach', t, 'next_stroke')
            fv = np.array([1000., 1000., 650.])*(reference-face)-np.array([65., 65., 55.])*(Jp@d.qvel)
            Jforce = Jp
        else:
            age = t-engagement_t
            alpha = float(smooth(age/.4))
            if cfg['task']=='joystick':
                # Shared open-loop input sequence; joystick angle is evaluation only.
                desired_stick = np.deg2rad(cfg.get('angle_deg', 12.))*(1 if age < 4.5 else -1)
                goal = desired_stick
                L = cfg.get('lever_length', .08)+cfg.get('cap_radius', .012)
                href = h0+np.array([L*np.sin(z), 0, L*(np.cos(z)-1)])
                G = np.array([L*np.cos(z), 0., -L*np.sin(z)])
                desired_R = Rotation.from_rotvec([0., z, 0.]).as_matrix()@R0
                mass_v, damping_v, goal_k, effort_limit, vmax = .004, .12, 2., .18, .28
                normal_ref = normal0+L*(np.cos(z)-1)
            else:
                goal = goal_ball
                href = h0+np.array([2*(z-z0), 0., 0.])
                G = np.array([2., 0., 0.])
                mass_v, damping_v, goal_k, effort_limit, vmax = .6, 6., 80., 2., .0125
                normal_ref = normal0
                for goal_step in cfg.get('goal_schedule', []):
                    if disturbance_age >= goal_step['start']:
                        goal = origin_ball+goal_step['distance']
                mass_v = cfg.get('virtual_mass', mass_v)
                damping_v = cfg.get('virtual_damping', damping_v)
                goal_k = cfg.get('goal_k', goal_k)
                if state == 'brake':
                    goal = brake_goal
            elastic = Kt*(href-pc)
            fc = alpha*(elastic+Dt*(G*zdot-vc))
            fc = np.clip(fc, -3., 3.)
            preload = cfg.get('preload', 3.)*alpha
            normal_force = np.clip(Kn*(normal_ref-preload/Kn-pc[2])-Dn*vc[2], -7., 4.)
            fv = fc+np.array([0, 0, normal_force])
            if state in ['engage', 'interact', 'brake']:
                if age < .55:
                    zacc = -12*zdot
                elif mechanism == 'point':
                    desired_speed = np.clip(2*(goal-z), -vmax, vmax)
                    zacc = np.clip((desired_speed-zdot)/.12, -20 if cfg['task']=='joystick' else -.2,
                                   20 if cfg['task']=='joystick' else .2)
                else:
                    goal_effort = np.clip(goal_k*(goal-z), -effort_limit, effort_limit)
                    zacc = (goal_effort-damping_v*zdot-cfg.get('reaction_scale',1.)*(G@fc))/mass_v
                zdot = float(np.clip(zdot+dt*zacc, -vmax, vmax))
                z += dt*zdot
            Jforce = Jc
            virtual_energy = .5*mass_v*zdot**2+.5*goal_k*(goal-z)**2
            if cfg['task']=='joystick':
                theta = d.qpos[6]
                if (3.7 < age < 4.4) or (8.3 < age < 9.0):
                    joystick_evals.append([float(abs(theta-desired_stick)), force>.2])
                if age > 9.1:
                    completed = True; change('done', t, 'sequence_complete')
            elif state == 'interact':
                if not cfg.get('fixed_duration') and abs(observed-goal_ball) < .0025 and abs(zdot)<.004:
                    if 'goal_hold_start' not in locals():
                        goal_hold_start = t
                    if t-goal_hold_start > .6:
                        completed = True; change('done', t, 'estimated_target_hold')
                else:
                    goal_hold_start = t
                boundary = np.linalg.norm(sensor.location[:2]) > cfg.get('boundary', .026)
                if cfg.get('allow_regrasp', True) and (boundary or observed-stroke_origin > .023) and goal_ball-observed > .005:
                    brake_goal = z
                    change('brake', t, 'stroke_boundary')
            elif state == 'brake' and t-phase_t > .45:
                if strokes >= 6:
                    failed_guard = 'stroke_budget'; break
                hold_ref = face.copy()
                change('lift', t, 'planned_release')

        if cfg.get('fixed_duration') and evaluation_start is not None and t-evaluation_start >= cfg['fixed_duration']:
            completed = True; change('done',t,'common_observation_horizon')
        orientation_error = Rotation.from_matrix(desired_R@R.T).as_rotvec()
        moment = np.clip(cfg.get('orientation_k',3.)*orientation_error-cfg.get('orientation_d',1.)*omega, -2., 2.)
        tau = Jforce[:, :6].T@fv+Jw[:, :6].T@moment+d.qfrc_bias[:6]-.2*d.qvel[:6]
        peak_tau = max(peak_tau, float(np.max(abs(tau))))
        torque_saturated += int(np.any(abs(tau)>LIMITS))
        d.ctrl[:] = np.clip(tau, -LIMITS, LIMITS)
        power = float(fv@(Jforce@d.qvel)+moment@omega)
        task_work += power*dt
        if lost_time is not None and t < interruption_t+2.:
            if abs((Jforce@d.qvel)[0]) < .002:
                stop_dwell += dt
                if stop_time is None and stop_dwell >= .05:
                    stop_time = t-.05
            else:
                stop_dwell = 0.
            if force < .05 and state in ['engage','interact']:
                disconnected_progress += abs(zdot)*dt
        if force < .05 and state in ['engage', 'interact']:
            command_work_offcontact += max(0., power)*dt
        elastic_energy = float(.5*np.sum(Kt*(href-pc)**2)) if state in ['engage', 'interact', 'brake'] else 0.
        max_virtual_energy = max(max_virtual_energy, virtual_energy)
        max_elastic_energy = max(max_elastic_energy, elastic_energy)
        step_force_jump = max(step_force_jump, float(np.linalg.norm(fv-force_prev)))
        force_prev = fv.copy()
        if step%10 == 0 or state=='done':
            theta = d.qpos[6] if cfg['task']=='joystick' else 0.
            trace.append([t, force, rim, theta, obj[0], obj[1], angle_rotation,
                          sensor.location[0], sensor.location[1], sensor.valid,
                          STATES[state], z, zdot, observed, face[0], face[2],
                          top_slip, sensor.norm, rhat[0], rhat[1], fc[0], normal_force,
                          np.linalg.norm(tau), elastic_energy, virtual_energy,
                          omega[1], bottom_slip, goal, task_work, object_velocity[3],
                          Rotation.from_matrix(R@R0.T).as_rotvec()[1],
                          local[0] if local is not None else np.nan,
                          d.xfrc_applied[object_id,0],raw_progress])
            qs.append(d.qpos.copy())
            mocaps.append(d.mocap_pos.copy())
        last_face = face.copy()
        if force > 15. or rim > 1.0 or np.max(abs(d.qvel[:6])) > 5.:
            failed_guard = 'force_or_rim_or_speed'; break
        if not np.isfinite(d.qpos).all():
            failed_guard = 'nonfinite'; break
        if state=='done':
            break
        mujoco.mj_step(m, d)

    tr = np.asarray(trace)
    if not completed and not failed_guard and step == step_budget-1:
        failed_guard = 'horizon'
    final_obj = d.xpos[object_id].copy()
    if cfg['task']=='joystick':
        e = np.asarray(joystick_evals)
        error = np.rad2deg(e[:, 0].mean()) if len(e) else 180.
        contact_fraction = float(e[:, 1].mean()) if len(e) else 0.
        task_success = completed and error <= 2. and contact_fraction >= .95
        target_error = error
    else:
        target_error = abs(final_obj[0]-goal_ball)*1000
        error = float(np.max(abs(tr[:, 5]))*1000)
        contact_fraction = float(np.mean(tr[np.isin(tr[:, 10], [1, 2, 3]), 1]>.2)) if np.any(np.isin(tr[:,10],[1,2,3])) else 0.
        rolling_error = abs((final_obj[0]-info['initial_object'][0])-info['radius']*angle_rotation)
        task_success = completed and target_error <= 5. and error <= 5.
    success = bool(task_success and not failed_guard and peak_force <= 12. and peak_rim < .2 and premature==0)
    metrics = dict(cfg, success=success, task_success=bool(task_success), completed=bool(completed),
                   guard=failed_guard, duration_s=float(t), physical_steps=step+1,
                   target_error=target_error, path_error_mm=error if cfg['task']=='ball' else 0.,
                   orientation_error_deg=error if cfg['task']=='joystick' else 0.,
                   contact_fraction=contact_fraction, peak_force_N=peak_force,
                   peak_rim_N=peak_rim, peak_joint_speed=peak_speed,
                   peak_requested_tau=peak_tau, saturated_steps=torque_saturated,
                   first_contact_s=first_contact, first_engagement_s=acquire_times[0] if acquire_times else None,
                   premature_engagements=premature, strokes=strokes, recoveries=recoveries,
                   mean_acquisition_delay_s=float(np.mean(acquisition_delays)) if acquisition_delays else None,
                   top_slip_mm=top_slip*1000, bottom_slip_mm=bottom_slip*1000,
                   final_ball_x=float(final_obj[0]), ball_rotation_rad=angle_rotation,
                   rolling_distance_error_mm=rolling_error*1000 if cfg['task']=='ball' else 0.,
                   disconnected_motion_mm=disconnected_distance*1000,
                   loss_response_s=recovery_time-lost_time if recovery_time is not None and lost_time is not None else None,
                   stopping_latency_s=stop_time-lost_time if stop_time is not None and lost_time is not None else None,
                   disconnected_virtual_progress_mm=disconnected_progress*1000,
                   offcontact_positive_work_J=command_work_offcontact,
                   max_rebase_error_m=rebase_error, attachment_updates=update_count,
                   max_step_force_jump_N=step_force_jump, task_work_J=task_work,
                   max_virtual_energy_J=max_virtual_energy, max_elastic_energy_J=max_elastic_energy,
                   location_median_mm=float(np.median(location_errors)) if location_errors else None,
                   location_p95_mm=float(np.quantile(location_errors,.95)) if location_errors else None,
                   false_valid_frames=false_valid_frames, valid_frames=valid_frames,
                   rolling_success=bool(success and rolling_error*1000 <= max(2.,cfg.get('distance',.025)*100)
                                        and top_slip*1000 <= max(2.,cfg.get('distance',.025)*100)) if cfg['task']=='ball' else None,
                   wall_s=time.perf_counter()-started)
    if cfg['task']=='ball' and evaluation_start is not None:
        end_age = cfg.get('fixed_duration', t-evaluation_start)
        age_trace = tr[:,0]-evaluation_start
        window = (age_trace >= end_age-1.) & (age_trace <= end_age+.02)
        available = bool(window.sum() >= 90)
        errs = (tr[:,4]-tr[:,27])*1000
        late_error = float(np.mean(abs(errs[window]))) if available else None
        late_speed = float(np.mean(abs(tr[window,29]))*1000) if available else None
        after = (age_trace >= 1.)
        active = np.isin(tr[:,10],[1,2,3,7])
        nominal_end = tr[:,27][-1]
        first_cross = np.flatnonzero((tr[:,4]>=nominal_end)&(age_trace>.55))
        settle_mask = (abs(errs)<=cfg.get('score_error_mm',5.)) & (abs(tr[:,29])<=.004)
        settle_t = None
        for j in np.flatnonzero(age_trace>=cfg.get('settle_after',3.3)):
            if t-tr[j,0]>=.5 and np.all(settle_mask[j:]):
                settle_t = float(age_trace[j]); break
        metrics.update(evaluation_start_s=evaluation_start, late_window_available=available,
            late_error_mm=late_error, late_speed_mm_s=late_speed,
            peak_pitch_deg=float(np.max(abs(tr[active,30]))*180/np.pi) if np.any(active) else None,
            mean_pitch_deg=float(np.mean(abs(tr[active,30]))*180/np.pi) if np.any(active) else None,
            progress_mae_mm=float(np.mean(progress_errors)*1000) if progress_errors else None,
            integrated_error_mm_s=float(np.trapezoid(abs(errs[after]),tr[after,0])) if np.any(after) else None,
            maximum_overshoot_mm=float(max(0., np.max((tr[after,4]-nominal_end)*1000))) if np.any(after) else None,
            settled_age_s=settle_t,
            common_success=bool(available and completed and not failed_guard and premature==0 and
                late_error<=cfg.get('score_error_mm',5.) and late_speed<=4. and error<=5. and
                peak_force<=12. and peak_rim<.2))
    else:
        metrics.update(late_window_available=False,late_error_mm=None,late_speed_mm_s=None,
            common_success=False,peak_pitch_deg=None,mean_pitch_deg=None,progress_mae_mm=None,
            integrated_error_mm_s=None,maximum_overshoot_mm=None,settled_age_s=None)
    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(save_path, trace=tr, qpos=np.asarray(qs), mocap=np.asarray(mocaps),
                            columns=np.array(TRACE_COLUMNS), config=json.dumps(cfg),
                            metrics=json.dumps(metrics), events=json.dumps(events), xml=xml)
    return metrics
