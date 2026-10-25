"""Readable PDF/HTML report of the completed frozen task campaign."""
from pathlib import Path
import html
import json
import numpy as np
import pandas as pd
from PIL import Image as PILImage
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Image,
                               Table, TableStyle, PageBreak, KeepTogether)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from scene import ROOT

OUT=ROOT/'results'; REP=ROOT/'report'; FIG=REP/'figures'
S=json.loads((OUT/'summary.json').read_text())
P=json.loads((OUT/'frozen_protocol.json').read_text())
V=json.loads((OUT/'verification.json').read_text())
df=pd.read_csv(OUT/'analyzed_metrics.csv')
font=Path('/usr/share/fonts/truetype/dejavu')
for name,file in [('DV','DejaVuSans.ttf'),('DV-Bold','DejaVuSans-Bold.ttf'),('Mono','DejaVuSansMono.ttf')]:
    pdfmetrics.registerFont(TTFont(name,str(font/file)))
pdfmetrics.registerFontFamily('DV',normal='DV',bold='DV-Bold',italic='DV',boldItalic='DV-Bold')
styles=getSampleStyleSheet()
styles.add(ParagraphStyle(name='BodyX',fontName='DV',fontSize=9.2,leading=13,spaceAfter=8))
styles.add(ParagraphStyle(name='TitleX',fontName='DV-Bold',fontSize=20,leading=25,spaceAfter=12,textColor=colors.HexColor('#17344b')))
styles.add(ParagraphStyle(name='HeadX',fontName='DV-Bold',fontSize=13,leading=17,spaceBefore=6,spaceAfter=9,textColor=colors.HexColor('#17344b'),keepWithNext=True))
styles.add(ParagraphStyle(name='CaptionX',fontName='DV',fontSize=8,leading=10.5,spaceAfter=10,textColor=colors.HexColor('#425761')))
styles.add(ParagraphStyle(name='SmallX',fontName='DV',fontSize=8,leading=10.5,spaceAfter=6))
styles.add(ParagraphStyle(name='EqX',fontName='Mono',fontSize=8.5,leading=12,spaceAfter=10,leftIndent=8))
story=[];web=[]
LABELS={'timed_centre':'Timed / centre','timed_eit':'Timed / EIT',
        'event_centre':'Touch / centre','event_eit':'Touch / EIT',
        'event_oracle':'Touch / known location','event_eit_point':'Touch / EIT / point'}


def p(t,kind='BodyX'):
    story.append(Paragraph(t,styles[kind]))
    webtext=t.replace('<link ','<a ').replace('</link>','</a>')
    web.append(f'<p class="{kind}">{webtext}</p>')


def h(t):p(t,'HeadX')
def page():story.append(PageBreak());web.append('<hr>')
def eq(t):p(t,'EqX')


def fig(name,caption,width=490):
    path=FIG/f'{name}.png'
    with PILImage.open(path) as im:height=width*im.height/im.width
    story.append(KeepTogether([Image(str(path),width=width,height=height),Paragraph(caption,styles['CaptionX'])]))
    web.append(f'<img src="figures/{name}.png" alt="{html.escape(caption)}"><p class="CaptionX">{caption}</p>')


def table(rows,widths=None):
    table=Table([[Paragraph(str(c),styles['SmallX']) for c in r] for r in rows],
                colWidths=widths or [490/len(rows[0])]*len(rows[0]),repeatRows=1,hAlign='LEFT')
    table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e7eff2')),
        ('VALIGN',(0,0),(-1,-1),'TOP'),('LINEBELOW',(0,0),(-1,-1),.25,colors.HexColor('#cbd8df')),
        ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]))
    story.append(KeepTogether([table,Spacer(1,8)]))
    web.append('<table>'+''.join('<tr>'+''.join(f'<td>{c}</td>' for c in r)+'</tr>' for r in rows)+'</table>')


def result_table(summary,task='ball'):
    rows=[['Condition','Episode pass','Physical task pass','Error','Mean peak load']]
    for key in LABELS:
        if key not in summary:continue
        s=summary[key]
        error=s['mean_endpoint_error_mm'] if task=='ball' else s['mean_scored_orientation_error_deg']
        text=f'{error:.2f} '+('mm' if task=='ball' else 'deg') if error is not None else 'Unavailable'
        rows.append([LABELS[key],f"{s['passes']}/{s['n']}",f"{s['physical_task_passes']}/{s['n']}",text,f"{s['mean_peak_force_N']:.2f} N"])
    table(rows,[150,75,90,80,95])


def subsection(title,purpose,procedure):
    h(title);p('<b>Purpose.</b> '+purpose);p('<b>What is done.</b> '+procedure)


def loss_latencies(s):
    response='none' if s['mean_loss_response_s'] is None else f"{s['mean_loss_response_s']:.3f} s"
    stop='not observed' if s['mean_stopping_latency_s'] is None else f"{s['mean_stopping_latency_s']:.3f} s"
    return f"Response {response}; stop {stop} ({s['stopped_in_window']}/{s['n']})"


def build():
    p('Anticipating Joystick and Ball Experiments with Tactile VMC','TitleX')
    p('UR3e MuJoCo simulations of virtual task mechanisms, contact attachment and model switching','HeadX')
    p('Prepared for Yi Zhang | 21 October 2026 | Synthetic simulation evidence','CaptionX')
    j=S['primary']['joystick'];b=S['primary']['ball']
    p(f'This report executes the one-axis joystick and straight ball-rolling protocols proposed in RoboSoft_EIT_VMC_Draft/manuscript.tex, using assumed object dimensions. It contains {S["total_trials"]} frozen evaluation trials and {S["physical_steps"]:,} actual physics steps. A physical hinged joystick and a freely rotating sphere replace the stationary support in the earlier report. Synthetic EIT voltages pass through the reconstructed original learned estimator; its force output is unused by the controllers.')
    table([['Main comparison','Joystick episode pass','Short ball-roll episode pass'],
        ['Touch switching / centre attachment',f"{j['event_centre']['passes']}/{j['event_centre']['n']}",f"{b['event_centre']['passes']}/{b['event_centre']['n']}"],
        ['Touch switching / EIT attachment',f"{j['event_eit']['passes']}/{j['event_eit']['n']}",f"{b['event_eit']['passes']}/{b['event_eit']['n']}"],
        ['Touch switching / EIT / point spring',f"{j['event_eit_point']['passes']}/{j['event_eit_point']['n']}",f"{b['event_eit_point']['passes']}/{b['event_eit_point']['n']}"],
        ['Nominal timing / EIT attachment',f"{j['timed_eit']['passes']}/{j['timed_eit']['n']}",f"{b['timed_eit']['passes']}/{b['timed_eit']['n']}"]],[240,125,125])
    p('These are conditional outcomes of the specified models. Their purpose is to expose likely mechanisms, failure cases and useful measurements before purchasing objects and running hardware trials. They do not predict a hardware success rate. The sensor constitutive law, friction, joystick mechanics and bracket recess still need measurement.')
    p('The main result is task dependent. Relocating the attachment helps joystick operation; the short ball roll succeeds with either attachment. The point controller passes every nominal EIT/touch trial and has smaller joystick error than the virtual lever. Touch-based reconnection and loss handling provide a separate benefit, accompanied by greater load on recovery. These results support testing the controller decisions separately on hardware.')
    h('1. Questions and evidence status')
    p('The study asks four questions: does relocating the virtual attachment improve joystick or ball manipulation; does a virtual mechanism with reaction dynamics improve on a point spring; do tactile events improve engagement and reconnection timing; and does loss detection reduce motion after an unexpected interruption? Each comparison below identifies the changed controller input and the shared task conditions.')
    p('The earlier reduced-model and supported-tip experiments remain in ../ur3e_sim/report. Their 50/50 versus 40/50 result is prior evidence for a different task and is excluded from the counts here. The paper draft remains a prospective hardware document. All joystick and ball results in this report are newly executed simulations.')
    page()
    h('2. Physical setup and assumed dimensions')
    p('The six-joint UR3e uses the same official link transforms, masses and inertia tensors as the earlier project, with simplified arm visuals. Joint torques drive the full arm in MuJoCo. The disk contacts the joystick cap or ball through native unilateral frictional contact; no weld, kinematic rolling constraint or object-position reset binds the object to the palm.')
    table([['Quantity','Assumption'],['Sensor and housing','90 mm active diameter; 130 mm housing diameter; 33.5 mm face offset; 0.35 kg tool'],
        ['Recessed rim','32 collision boxes approximate an annulus with 47 mm inner radius; front lip 3 mm beyond pad (0 and 6 mm also tested)'],
        ['Joystick','80 mm pivot-to-cap-centre length; 12 mm cap radius; 65 g moving body; y-axis hinge; +/-20 degrees travel'],
        ['Joystick return mechanics','0.15 N m/rad stiffness; 0.025 N m s/rad damping; stiffness 0.05 and 0.4 also tested'],
        ['Ball','50 mm diameter rigid sphere; uniform-density mass 72.0 g at 1100 kg/m3; 40 and 60 mm also tested'],
        ['Friction and compliance','Sliding coefficient 0.7 nominal, 0.1/0.4/0.8 in size sweep; native contact time constant 8 ms'],
        ['Physics and actuation','1 ms step, implicitfast; exact model bias compensation; torque limits [54,54,28,9,9,9] N m']],[150,340])
    p('The recessed rim is a stated geometric assumption, not a recovered CAD model of the real bracket. The sphere has rigid-body rotation and compliant numerical contact, without a measured rubber deformation law. Arm self-collision and full fixture clearances are outside this simplified scene.')
    fig('setup_joystick','Figure 1. Actual MuJoCo joystick scene, reconstructed from a saved evaluation trajectory. The orange cap is attached to a hinged lever with a return spring. The close view shows the recessed palm and physical contact.',width=490)
    fig('setup_ball','Figure 2. Actual MuJoCo ball scene. A free sphere is squeezed between the palm and table; friction determines whether it rolls or slips. The non-colliding colour markers rotate with the sphere.',width=490)
    page()
    h('3. What VMC model is implemented?')
    p('A virtual mechanism supplies a moving output point h(z). A spring and damper couple that output to a selected material point p_c on the palm. The reaction of the coupling slows the virtual mechanism when the robot encounters resistance. A separate normal spring introduces a nominal 3 N preload, and an orientation spring and damper accommodate the requested tool orientation.')
    fig('mechanisms','Figure 3. Task mechanism schematic. The joystick output follows the cap arc about its assumed pivot. The ball carriage uses a 2:1 virtual transmission. The labelled coupling contains a spring and damper acting in parallel. Physical object motion is still determined by MuJoCo contact.',width=490)
    eq('p_c = p_E + R_E c_hat;  v_c = J_c q_dot')
    eq('f_c = P_task [K_c(h(z)-p_c) + D_c(G z_dot-v_c)]')
    eq('M_v z_ddot = clip(k_g(z_g-z), +/-u_max) - d_v z_dot - G^T f_c')
    eq('tau* = J_c^T(f_c+f_normal) + J_w^T m_v + bias - 0.2 q_dot')
    p('P_task selects horizontal task forces, with stiffness 700 N/m and damping 30 N s/m in x and y. The normal spring uses 800 N/m and 60 N s/m, with a 0.4 s preload ramp. Task-force components are limited to +/-3 N. Orientation stiffness is 3 N m/rad and damping 1 N m s/rad. Joint torque limits are applied after the force-to-torque mapping.')
    table([['Mechanism','Coordinate and output','Internal dynamics'],
        ['Virtual lever','z=theta; h follows an arc of radius 92 mm; palm orientation follows the virtual lever angle','I_v=0.004 kg m2; d_v=0.12 N m s/rad; k_g=2 N m/rad; goal torque <=0.18 N m; speed <=0.28 rad/s'],
        ['Virtual carriage','z=s; h=h0+2t(s-s0); G=2t, where t is the path direction','M_v=0.6 kg; d_v=6 N s/m; k_g=80 N/m; goal effort <=2 N; speed <=12.5 mm/s']],[95,190,205])
    p('The carriage transmission represents the ideal relation between palm travel and sphere-centre travel under rolling on a stationary table. Its factor two appears in both output velocity and reaction force. Actual rotation and slip are evaluated independently; the simulator never enforces that relation as a constraint.')
    p('The joystick axis and lever radius are supplied as assumed geometry. The local arc output is initialized at the selected palm point on engagement, making the initial coupling extension zero. An offset attachment therefore shifts the local arc origin. The study does not identify a joystick pivot from tactile data or test pivot-calibration error.')
    p('<b>Point-spring comparison.</b> The point controller follows the same arc or carriage geometry, with the same force limits, attachment, goals and tactile feedback. Its progress reference follows a bounded kinematic update toward the goal rather than the reaction-driven internal dynamics. This matched comparison tests the usefulness of reaction dynamics; it does not give the mechanism an easier task.')
    page()
    h('3.1 How EIT and switching enter the controller')
    p('The sensor model generates 40 normalized electrical channels from current physical contact position and load. The existing 100-tree ExtraTrees estimator predicts x, y and force. Only contact validity and position reach the task controller. MuJoCo contact truth supplies synthetic electrical labels, independent evaluation, the explicitly labelled known-location reference and common protective load limits.')
    p('The copied checkpoint was truncated and failed to load. Re-fitting the original training arrays with the saved settings reconstructed the original manifest checksum exactly. All saved train, validation, test and material-shift predictions agree exactly. results/estimator_restoration.json records this recovery. The electrical forward mesh is 4.8 mm with an angular offset; the original training mesh was 4.0 mm. Material response lag is 80 ms, and nominal acquisition is 20 Hz.')
    p('<b>Contact events.</b> Three frames above a normalized voltage norm of 0.008 establish touch. Two frames below 0.005 invalidate contact. A stale timestamp also invalidates it. Touch switching uses this event to engage; nominal switching engages 1.00 s after approach begins, fixed from separate development trials. Both begin with a bounded 8 mm/s approach. Truth never vetoes a false tactile event.')
    p('After complete detachment, the forward model returns to its no-contact voltage baseline. Residual electrical signatures after release are not modeled; the 80 ms lag acts on load while a contact is active. The measured loss-response latency therefore describes this assumed 20 Hz detector and dwell rule, and requires independent validation on the physical pad.')
    p('<b>Attachment.</b> Centre conditions act at the disk centre. Joystick EIT conditions latch the selected coordinate; a timed adaptive condition that engages before valid sensing initially retains the common centre estimate and updates when a valid estimate arrives. Ball EIT conditions filter and update the coordinate each frame, with at most 2.5 mm change per frame and a 36 mm radial clamp. Updating the coordinate shifts the reference by R_E delta_c to preserve spring extension.')
    p('<b>Shared ball feedback.</b> Every ball controller, including the centre controller, receives the same estimate b_hat_xy=(p_E+R_E c_hat)_xy and the same filter and internal-state correction rule. Both see an initial camera position with 0.5 mm Gaussian noise. Vision is withheld during rolling; a camera position is supplied only between planned strokes under the same policy. Ball rotation, true position and force remain evaluation signals. Thus the spatial comparison does not confound attachment adaptation with access to tactile object-position feedback.')
    p('The global ball target is the camera-based initial position plus the requested translation. Endpoint error is measured against that commanded target. The CSV also reports translation-distance error relative to the true initial sphere position, so the initial camera error remains visible rather than being treated as perfect object localization.')
    p('State correction and reference rebasing preserve instantaneous spring extension, but the Jacobian, damping and moment arm also change. Speed limits, force limits, measured transients and energy records are included. These checks do not establish passivity of the hybrid controller.')
    h('4. Frozen design and scoring')
    p(f'The evaluation contains {P["count"]} trials with a maximum budget of {P["max_physical_steps"]:,} physics steps. Source hashes, trial settings and thresholds were written before execution. Development seeds 100-104 are excluded. Most paired cells contain three independent synthetic sensor sessions; repeated trajectories do not represent independently built hardware sensors.')
    table([['Group','Question / manipulation','Trials']]+[[g,text,S['groups'][g]['n']] for g,text in [
        ('J1','Joystick: switching x attachment; landing offset and initial gap'),('J2','Matched joystick point-spring comparison'),
        ('J3','Joystick return-stiffness variation'),('B1','Short ball roll: switching x attachment'),
        ('B2','Matched ball point-spring comparison'),('B3','80 mm rolling with planned release and reconnection'),
        ('B4','60 mm rolling with hidden support interruption'),('B5','Ball diameter x friction'),('B6','Recess depth'),
        ('S1','Acquisition rate and material variation'),('S2','High voltage-noise diagnostic'),('D1','Temporary mechanical resistance')]], [45,390,55])
    p('Joystick trials request +12 degrees, then -12 degrees; mean angle error must be <=2 degrees and contact occupancy >=95% in the specified hold windows. Ball tasks require true endpoint error <=5 mm and cross-path error <=5 mm. Full episode pass additionally requires completed motion, zero premature engagements, peak pad force <=12 N, peak rim force <0.2 N and no protective termination. Physical task pass omits the timing and loading conditions so that the two outcomes remain visible. Joystick windows missing after termination are reported as unavailable, while the trial stays in every pass-count denominator.')
    p('The crossed switching comparison exercises complete supervisors: touch switching also handles unexpected loss, while the timed policy continues its scheduled interaction. The first-contact timing interpretation uses the paired EIT-attachment conditions, which retain contact after successful acquisition in the primary tasks. Centre-attachment comparisons can also expose subsequent contact-loss behaviour.')
    p('Common termination limits are 15 N pad force, 1 N rim force and 5 rad/s joint speed. Joystick horizon is 16 s; ball horizons are 28-40 s, with at most six engagements. A separate rolling check requires rotation-distance consistency and accumulated top slip within max(2 mm, 10% of commanded distance). All figures and tables use trial summaries rather than treating trace samples as independent repeats.')
    page()
    subsection('5. Joystick: where should the virtual forces act?',
       'Test whether the robot can produce left-right joystick inputs from the contact it encounters, and whether tactile timing or attachment adaptation explains the result.',
       'Approach the rounded cap at landing offsets -25, 0 and +25 mm and initial gaps 2, 6 and 14 mm. Cross nominal-time versus touch engagement with centre versus EIT attachment. Include a known-location touch reference. For each geometry and timing condition, repeat three sensor sessions. The cap angle is measured for evaluation and is excluded from controller feedback.')
    fig('motion_joystick','Figure 4. Saved joystick states show approach and the positive and negative input phases. Labels report actual cap angle and normal load, rather than the requested trajectory.',width=490)
    result_table(S['primary']['joystick'],'joystick')
    p(f"<b>Result and interpretation.</b> With touch switching, the EIT and known-location attachments each pass {j['event_eit']['passes']}/{j['event_eit']['n']} episodes, while the centre attachment passes {j['event_centre']['passes']}/{j['event_centre']['n']}. The centre succeeds at central landings and degrades at off-centre landings. Nominal timing with EIT passes {j['timed_eit']['passes']}/{j['timed_eit']['n']}; it engages before contact in {j['timed_eit']['premature_episodes']} episodes. Location and engagement timing therefore address different problems in this setup.")
    p('Twelve touch/centre joystick trials terminate on the assumed rim-contact limit. This is a predicted clearance consequence of the complete task and bracket model, and should be checked against measured bracket geometry. The joystick comparison therefore includes contact retention and clearance consequences as well as angle tracking.')
    p('Errors in this table average available hold-window angle errors; episodes terminated before a scoring window remain failed episodes and have no fabricated angle score. The companion CSV includes scored-window counts. Loads include the entire approach and interaction.')
    fig('joystick_results','Figure 5. Joystick episode pass versus gap and landing offset, with load shown alongside it. Touch and timing comparisons share mechanics and gains. A late physical contact can occur after the nominal engagement time.',width=490)
    fig('joystick_traces','Figure 6. A paired +25 mm landing. The traces expose cap-angle tracking, contact load and tactile validity. Centre attachment can produce contact loss or incomplete input even if nominal task references are unchanged.',width=490)
    jp=S['paired_mechanism_vs_point']['joystick']
    h('5.1 Does the virtual lever outperform the point spring?')
    p('<b>Purpose and procedure.</b> Compare the reaction-driven lever against the point spring on the same geometries and goals. The point controller receives the same EIT attachment and switching event, so this isolates the extra mechanism dynamics. The return-stiffness and temporary-resistance tests below examine whether the outcome changes with object resistance.')
    p(f'There are {jp["n_pairs"]} matched joystick pairs: {jp["both_pass"]} pass in both conditions, {jp["mechanism_only_passes"]} pass only with the mechanism, and {jp["point_only_passes"]} pass only with the point controller. Mean task error is {jp["mean_error_mechanism"]:.2f} degrees for the lever and {jp["mean_error_point"]:.2f} degrees for the point spring. A smaller error for the point spring is retained as an unfavorable result for a general mechanism-superiority claim.')
    fig('mechanism_comparison','Figure 7. Mechanism versus point spring under shared nominal geometry and sensing. Panels (a,b) show nominal-gap error by landing offset. Panel (c) shows actual normal load during a temporary resistance test. These metrics test different benefits and should not be combined into a blanket ranking.',width=490)
    page()
    subsection('6. Ball rolling with vision withheld during contact',
       'Test whether tactile location and robot pose can guide a ball to a target, whether moving the virtual attachment helps, and whether the ball actually rolls.',
       'Move the free sphere 25 mm along the table. Use landing offsets -15, 0 and +15 mm, gaps 2, 6 and 14 mm, and the same crossed switching/attachment conditions as the joystick. All conditions receive the same tactile progress feedback. Planned lift and reacquisition are allowed if a landing near the pad edge leaves insufficient stroke length.')
    result_table(S['primary']['ball'])
    p(f"<b>Result and interpretation.</b> Both centre and EIT attachments with touch switching pass {b['event_eit']['passes']}/{b['event_eit']['n']} trials. Mean endpoint errors are {b['event_centre']['mean_endpoint_error_mm']:.2f} mm for the centre and {b['event_eit']['mean_endpoint_error_mm']:.2f} mm for EIT. This straight, nearly horizontal task does not demonstrate a spatial-attachment advantage. Tactile progress feedback and correctly timed engagement are sufficient for both. The lower centre error is retained in the comparison.")
    p(f"Errors relative to a true 25 mm translation from the actual starting position are {b['event_centre']['mean_translation_distance_error_mm']:.2f} mm for the centre and {b['event_eit']['mean_translation_distance_error_mm']:.2f} mm for EIT. This additional diagnostic includes the initial camera-position error.")
    fig('ball_results','Figure 8. Short-roll pass versus initial gap and landing offset. The centre controller receives the same tactile estimate of ball progress as the adaptive controller. Equivalent success here is evidence that attachment adaptation is not necessary for every rolling condition.',width=490)
    page()
    h('6.1 Rolling verification and the carriage comparison')
    p('<b>Purpose and measurement.</b> Check rotation and slip directly from the saved free-sphere states. Compare the carriage and point spring with the same tactile progress input to determine whether the extra mechanism state changes endpoint accuracy or episode pass.')
    fig('ball_traces','Figure 9. Position, rotation-distance consistency and contact migration from saved sphere states. The dotted position estimates are produced from EIT location and robot pose; solid curves are independent simulation truth. The rotation residual tests rolling instead of inferring it from translation alone.',width=490)
    bp=S['paired_mechanism_vs_point']['ball']
    p(f'The mechanism and point comparison has {bp["n_pairs"]} paired ball trials: {bp["both_pass"]} pass in both, {bp["mechanism_only_passes"]} pass only with the carriage and {bp["point_only_passes"]} pass only with the point spring. Mean endpoint error over all attempted trials is {bp["mean_error_mechanism"]:.2f} mm versus {bp["mean_error_point"]:.2f} mm. Endpoint error remains separate from the force, premature-engagement and rolling checks.')
    table([['Touch condition','Rolling check passed / trials','Largest rotation-distance residual']]+[[LABELS[k],f"{s['rolling_passes']}/{s['n']}",f"{s['max_rolling_distance_error_mm']:.2f} mm"] for k,s in S['primary']['ball'].items() if k.startswith('event')],[210,140,140])
    page()
    subsection('7. Long rolling: release and reconnection',
       'Test whether tactile engagement restarts interaction reliably when a long path requires several strokes, while retaining the global target.',
       'Roll the ball 80 mm with initial offsets -10 or +10 mm. A contact-radius boundary of 26 mm or estimated local progress above 23 mm triggers braking, lift, repositioning and a new approach. Between-stroke camera feedback is shared. New approach gap varies by +/-3 mm. Cross switching and attachment policies, and include the adaptive point controller. Compare full episode pass, premature restarts, loading and elapsed time.')
    fig('motion_multistroke','Figure 10. Saved states from an 80 mm run show the actual lift, repositioning and subsequent rolling stroke. Local coupling is reinitialized at each engagement while the global target is retained.',width=490)
    result_table(S['multistroke'])
    p(f"<b>Result and interpretation.</b> Touch/EIT passes {S['multistroke']['event_eit']['passes']}/{S['multistroke']['event_eit']['n']} long rolls. Timed/EIT physically completes {S['multistroke']['timed_eit']['physical_task_passes']}/{S['multistroke']['timed_eit']['n']}, but only {S['multistroke']['timed_eit']['passes']}/{S['multistroke']['timed_eit']['n']} meet the complete episode criteria because some restarts precede contact. These results support measuring reconnection transients in addition to final object error.")
    page()
    h('7.1 What happens at each reconnection?')
    p('The global target remains fixed during planned release. Each new engagement initializes a local coupling at the current palm point. The following counts and traces expose early restarts and force transients that a final-position measurement alone would miss.')
    fig('multistroke_results','Figure 11. Long-roll outcomes include time and premature engagement count. Premature events contribute to episode failure even when the ball eventually reaches its target; the table also shows physical task completion.',width=490)
    table([['Condition','Reconnections','Premature reconnections','Mean engagements']]+[[LABELS[k],s['total_reconnections'],s['false_reconnections'],f"{s['mean_strokes']:.2f}"] for k,s in S['multistroke'].items()],[210,90,110,80])
    fig('multistroke_traces','Long-roll trace from paired EIT-attachment episodes. Solid position curves are truth and dotted curves are tactile estimates. Supervisor phase and actual contact force show the repeated release, reposition and restart sequence.',width=490)
    page()
    subsection('8. Proposed switching task: rolling over a retractable support',
       'Isolate unexpected contact loss from planned stroke transitions. Test whether a tactile supervisor stops motion while disconnected, how quickly it responds, and what load it produces on recovery.',
       'Request a 60 mm roll. At 2.50 s, an independent fixture lowers the table 20 mm over 0.15 s, holds it down for 0.40 s, then returns it over 1.20 s. The controller receives no interruption timestamp. Compare a supervisor driven by tactile loss and reacquisition with one that keeps its scheduled interaction active. Cross centre and EIT attachments; include the EIT point controller. Paired trials have identical fixture motion, object geometry and sensor session.')
    p('This models an independently actuated support fixture, not an accidental numerical contact deletion. The ball remains a free dynamic body. A preliminary 0.15 s return caused a large fixture impact in both conditions; the final 1.20 s return was selected during development and frozen before evaluation. The fast-return development traces are retained and excluded from evaluation.')
    fig('motion_interruption','Figure 12. Saved states show physical support withdrawal and return. The table pose is reconstructed along with the robot and sphere; the controller receives only its ordinary tactile observations.',width=490)
    table([['Condition','Pass','Disconnected travel','Response / stopping latency','Mean peak load']]+[[LABELS[k],f"{s['passes']}/{s['n']}",f"{s['mean_disconnected_motion_mm']:.2f} mm",loss_latencies(s),f"{s['mean_peak_force_N']:.2f} N"] for k,s in S['loss'].items()],[140,50,90,125,85])
    le=S['loss']['event_eit'];lt=S['loss']['timed_eit']
    p(f"<b>Result and tradeoff.</b> Touch/EIT reduces mean disconnected travel from {lt['mean_disconnected_motion_mm']:.1f} to {le['mean_disconnected_motion_mm']:.1f} mm. Its supervisor responds after {le['mean_loss_response_s']:.3f} s and actual motion meets the sustained-stop criterion after {le['mean_stopping_latency_s']:.3f} s on average. Episode pass is {le['passes']}/{le['n']} versus {lt['passes']}/{lt['n']}. Mean peak load rises from {lt['mean_peak_force_N']:.2f} to {le['mean_peak_force_N']:.2f} N; the stop-and-reacquire policy therefore needs hardware loading measurements and further transient design.")
    p('Six timed-policy interruption episodes terminate on rim contact when the support returns. All other interruption outcomes and premature restarts remain in the table. This connects the disconnected-motion metric to a concrete clearance consequence in the assumed geometry.')
    p('Disconnected travel integrates horizontal palm motion while pad contact is absent during the two seconds after fixture withdrawal starts. Response latency measures the supervisor transition after physical contact loss. Stopping latency requires horizontal material-point speed below 2 mm/s for 50 ms; it is reported only when observed in that window. An eventual target pass alone does not establish satisfactory loss handling.')
    fig('loss_results','Figure 13. The loss test separates unwanted disconnected motion, load cost and eventual task completion. A benefit in stopping can coexist with a larger reconnection load.',width=490)
    fig('loss_traces','Figure 14. Paired EIT-attachment traces. Grey shading marks physical support motion from 2.50 to 4.25 s. Supervisor phase, actual palm travel and true force show how contact loss affects the two policies.',width=490)
    page()
    subsection('9. Robustness: size, friction, recess and sensing',
       'Identify which unmeasured hardware properties could change the conclusions, and expose conditions where the nominal controller is insufficient.',
       'Test 40, 50 and 60 mm balls at friction coefficients 0.1, 0.4 and 0.8 during a 40 mm roll. Separately test recess depths 0, 3 and 6 mm at an initial 20 mm offset. Sweep 5, 10 and 20 Hz acquisition on both tasks, increase material variability, and add a deliberately large normalized voltage-noise level of 0.01 as a failure diagnostic. All controller gains remain frozen.')
    fig('ball_sizes','Figure 15. Physical sphere sizes in the actual MuJoCo scene. Density stays fixed, so size changes both geometry and inertia. These are candidate size tests, not validated models of particular purchased balls.',width=490)
    fig('size_friction','Figure 16. Full episode pass by ball size and assumed sliding friction, with three sessions in each cell. Nominal success over this grid does not identify the friction of an actual rubber or silicone product.',width=490)
    p('All 54 size/friction trials and all 18 recess trials pass under the gentle tested motions. This grid establishes feasibility within the assumed contact family; it does not select an optimal ball diameter or validate a purchased material. The electrical footprint model is independent of sphere radius, so this size sweep tests mechanical geometry and inertia rather than a measured curvature-dependent electrical response.')
    recess=df[df.group=='B6']
    table([['Recess / attachment','Pass','Maximum rim load','Mean endpoint error']]+[[f'{r*1000:.0f} mm / {a}',f'{int(s.success.sum())}/{len(s)}',f'{s.peak_rim_N.max():.3f} N',f'{s.target_error.mean():.2f} mm'] for (r,a),s in recess.groupby(['recess','attachment'])],[160,70,120,140])
    fig('sensing_results','Figure 17. Acquisition rate affects when engagement occurs and the load accumulated while waiting for it. These plots report the full approach, not only the eventual task phase.',width=490)
    sens=df[(df.group=='S1')&(df.sensor_shift)]
    noise=df[df.group=='S2']
    table([['Diagnostic','Task','Pass','Premature episodes','Max load']]+[[label,t,f'{int(s.success.sum())}/{len(s)}',int((s.premature_engagements>0).sum()),f'{s.peak_force_N.max():.2f} N'] for label,sub in [('Material variation',sens),('High voltage noise',noise)] for t,s in sub.groupby('task')],[150,75,65,110,90])
    p('Every tested 5-20 Hz and wider-material-variation trial completes, while all six high-noise diagnostics fail through premature electrical engagement. At 5 Hz the ball approach accumulates about 3.04 N before engagement, compared with 1.14 N at 20 Hz. Successful eventual rolling therefore does not remove the need to characterize acquisition latency and the no-contact noise floor.')
    h('9.1 Object resistance and mechanism dynamics')
    p('<b>Purpose and procedure.</b> Vary joystick return stiffness from the nominal 0.15 to 0.05 or 0.4 N m/rad. Separately apply a hidden temporary resisting fixture from 2.50 to 3.30 s: a bounded joystick angular brake or ball translation brake. Compare the mechanism and adaptive point spring with matched input goals. True object position is read only by the external fixture and evaluator, not by the task controller.')
    table([['Test / mechanism','Pass','Mean task error','Mean peak load']]+[[f'Joystick k={k:g} / {mech}',f'{int(s.success.sum())}/{len(s)}',f'{s.scored_orientation_error_deg.mean():.2f} deg',f'{s.peak_force_N.mean():.2f} N'] for (k,mech),s in df[df.group=='J3'].groupby(['stick_k','mechanism'])]+[[f'Resistance: {task} / {mech}',f'{int(s.success.sum())}/{len(s)}',f'{s.target_error.mean():.2f} '+('deg' if task=='joystick' else 'mm'),f'{s.peak_force_N.mean():.2f} N'] for (task,mech),s in df[df.group=='D1'].groupby(['task','mechanism'])],[215,65,110,100])
    p('At the lower joystick return stiffness of 0.05 N m/rad, the lever passes 0/9 while the point spring passes 9/9; their mean errors are 2.35 and 0.59 degrees. At 0.4 N m/rad both pass 9/9. All temporary-resistance trials pass with either mechanism. The current virtual lever therefore has a demonstrated stiffness sensitivity, and these experiments do not establish a general benefit from the extra mechanism state.')
    p('In the central landing at stiffness 0.05 N m/rad (seed 500), positive-hold cap/virtual angles average 14.36/13.64 degrees for a 12-degree goal. With the point reference at 12.00 degrees, the cap averages 12.63 degrees. These states are consistent with the finite goal spring yielding under sustained coupling load.','SmallX')
    page()
    h('10. Numerical checks and evidence limits')
    table([['Check','Observed result'],
       ['Contact-frame Jacobian identity',f"Maximum component difference {V['contact_frame_jacobian_max_difference']:.2g}"],
       ['Equivalent contact-frame torque mapping',f"Maximum difference {V['equivalent_contact_frame_torque_max_difference_Nm']:.2g} N m"],
       ['2:1 transmission power consistency',f"Residual {V['virtual_transmission_power_residual_W']:.2g} W"],
       ['Electrical mesh sensitivity',f"4.8 versus 2.8 mm mesh: {100*V['forward_4p8_vs_2p8mm_relative_contact_signal_difference']:.2f}% difference in one contact-induced voltage probe"],
       ['Reference rebasing in executed trials',f"Maximum spring-extension residual {df.max_rebase_error_m.max():.2g} m"]],[230,260])
    for r in V['time_step_refinement']:
        p(f"Halving the physics step from 1 to 0.5 ms in a representative {r['task']} trial preserves both pass outcomes. Target-error change is {r['target_error_difference']:.5f} {r['target_error_units']}; peak-force change is {r['peak_force_difference_N']:.5f} N.")
    p('The equivalence check confirms that the same contact-frame Jacobian produces the same joint torque. It supplies no advantage over a conventional controller implementing the same equations. The code also saves virtual spring energy, mechanism energy, command work and largest per-step force changes. These records support transient inspection but are not a passivity certificate.')
    p('Across all 540 evaluation trials there are no joint-torque saturation steps. The 18 protective terminations are rim-contact events: 12 in the centre-attachment joystick task and six in the timed interruption task. The largest pad normal force is 7.84 N. Horizon-limited and inaccurate episodes are also retained as failed outcomes.')
    p('The sensor forward law remains an assumed local conductance perturbation in a flat sheet. It is not a calibrated EIT model for a curved cap or a rolling contact patch, and contact area, shear and material deformation are not solved. The original joint position-force decoder used force-labelled synthetic training examples; the present controller uses no EIT force feedback. Exact robot pose and model bias compensation are also idealizations.')
    p('The initial camera observation and between-stroke camera measurements are modeled as noisy positions, not processed camera images. No visual state is supplied during rolling. The three sensor seeds per cell explore synthetic sessions rather than manufacturing variability; confidence bounds over trace samples would be misleading. The trial-level CSV includes every attempted trial, guard and incomplete outcome.')
    page()
    h('11. Ball to purchase and first hardware measurements')
    p('<b>Recommended starting specification.</b> A smooth, solid 50 mm silicone or rubber sphere, nominal Shore A 50-70, with known diameter and mass and a trimmed mould seam, is a practical first object. Start with three identical balls from one batch. The nominal simulation uses a 50 mm, 72 g uniform sphere. Hardness is a purchasing target only: these simulations do not map Shore A hardness to friction or deformation, and do not establish that one material is optimal.')
    p('A firm, nearly spherical ball makes centre location and rotation measurable while retaining useful friction at light preload. Choose a surface that can carry small contrasting ink marks for independent rotation tracking. Before final trials, measure indentation under the intended load, static/sliding friction on the pad and table, mass, diameter and seam height. Large deformation or a seam comparable with the desired position accuracy requires updating the model.')
    p('Supplier examples checked on 21 October 2026: <link href="https://kiritsubo.shop-pro.jp/?cbid=2987098&amp;csid=5&amp;mode=cate&amp;page=2&amp;sort=n" color="#168c85">KIRITSUBO lists 50 mm silicone balls (ball-sil-0500)</link>; <link href="https://www.arubis.be/products/screen-cleaning-balls/" color="#168c85">Arubis lists silicone, polyurethane and rubber screening-ball materials and custom sizes</link>. These establish procurement options; their product pages do not establish compatibility with this tactile pad. Request the actual hardness, finish and small-quantity availability before choosing a product.')
    p('For a spherical ball and horizontal lip, a useful geometric estimate is c_max = r_open - sqrt(2 R d - d^2), where d is recess and R ball radius. The assumed 47 mm rim opening radius, 25 mm ball radius and 3 mm recess give c_max about 35 mm before a margin for indentation, footprint and palm tilt. The controller stops a stroke at 26 mm. Measure the actual bracket geometry and verify clearance throughout the stroke; this calculation describes the assumed geometry.')
    p('Begin hardware validation with slow 25 mm straight rolls and independent ball rotation tracking. Calibrate the location estimator on the purchased sphere and joystick cap. Then cross switching and attachment under paired landing offsets. Add repeated reconnection and the retractable-support test only after ordinary release is characterized. The numerical gains and load thresholds here are experiment-design assumptions, not a verified physical UR3e deployment configuration.')
    p('The two-axis joystick gimbal is identified in the draft as a later extension. This report evaluates the defined one-axis protocol and straight rolling; it supplies no result for a two-axis gimbal or arbitrary planar rolling paths.')
    page()
    h('12. Reproducibility and artifact guide')
    table([['Artifact','Contents'],['results/frozen_protocol.json','540 scheduled trials, source hashes, limits, maximum physics-step budget'],
       ['results/trials/trial_*.npz','Full joint and object positions at 100 Hz, table poses, trace columns, events, config and metrics'],
       ['results/analyzed_metrics.csv','One row per evaluation trial, including failure and scoring availability'],
       ['results/summary.json','Group and controller counts; paired mechanism comparisons'],
       ['results/estimator_restoration.json','Truncated-source checkpoint audit; exact restored checksum and prediction comparisons'],
       ['results/verification.json','Jacobian, equivalent torque, transmission, electrical mesh and physics-step checks'],
       ['development/','Excluded development episodes, including the initially excessive fixture return speed'],
       ['src/scene.py; src/trial.py','Physical objects, torque controller, supervisor, bounded trials and trace logging'],
       ['src/electrical.py; src/sensing.py','FEM electrical forward model and learned tactile observations'],
       ['report/figures/','MuJoCo scene images and data plots as PNG and PDF']],[210,280])
    p('This project reuses the original official robot parameter files and saved EIT training arrays in ../ur3e_sim. The electrical solver is copied with an explicit two-dimensional determinant replacing the removed NumPy 2-D cross-product operation; its numerical model is otherwise the same. requirements.txt and README.md describe the runtime and reproduction commands.')
    p(f'The campaign completed {S["total_trials"]}/{P["count"]} trials, consuming {S["physical_steps"]:,} of the maximum {P["max_physical_steps"]:,} physics steps. Physical time within each trial runs sequentially. Four independent worker processes execute separate episodes. results/STATUS.json records completion.')
    p('AI assistance: OpenAI Codex implemented and executed the new simulations, reconstructed the damaged checkpoint, rendered the MuJoCo scenes, generated figures from saved numerical data and prepared this report. All task results are simulation evidence. No joystick, ball or robot hardware measurements are presented.','CaptionX')
    REP.mkdir(exist_ok=True)
    doc=SimpleDocTemplate(str(REP/'Joystick_Ball_EIT_VMC_Simulation_Report.pdf'),pagesize=(595.28,841.89),
                          rightMargin=48,leftMargin=48,topMargin=43,bottomMargin=43,
                          title='Anticipating Joystick and Ball Experiments with Tactile VMC',author='Research simulation report prepared for Yi Zhang')
    def footer(canvas,doc):
        canvas.setFont('DV',7);canvas.setFillColor(colors.HexColor('#617782'))
        canvas.drawString(48,24,'UR3e joystick / ball / switching - simulation evidence')
        canvas.drawRightString(547,24,str(doc.page))
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    (REP/'report.html').write_text('<!doctype html><html><head><meta charset="utf-8"><title>Joystick and ball EIT VMC simulations</title><style>body{max-width:1000px;margin:35px auto;padding:20px;font:16px/1.55 Georgia,serif;color:#17344b}img{width:100%}.TitleX{font-size:32px;font-weight:bold}.HeadX{font-size:23px;font-weight:bold}.CaptionX,.SmallX{font-size:13px}table{border-collapse:collapse;width:100%}td{border:1px solid #d5dfe5;padding:8px}.EqX{font-family:monospace}hr{margin:40px 0}</style></head><body>'+''.join(web)+'</body></html>')
    print(REP/'Joystick_Ball_EIT_VMC_Simulation_Report.pdf')


if __name__=='__main__':build()
