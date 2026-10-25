"""Vector schematic of the controller implemented in experiments.py."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch

FIG = Path(__file__).resolve().parents[1] / 'report' / 'figures'
INK, VIRTUAL, PHYSICAL = '#17344b', '#168c85', '#ce6537'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})


def line(ax, points, color=INK, **kw):
    p = np.array(points)
    ax.plot(p[:, 0], p[:, 1], color=color, lw=1.8, **kw)


def arrow(ax, start, end, color=INK, **kw):
    ax.annotate('', xy=end, xytext=start,
                arrowprops=dict(arrowstyle='->', color=color, lw=1.7, **kw))


def spring(ax, start, end, color=VIRTUAL, width=.16):
    a, b = np.array(start), np.array(end)
    v = b-a
    n = np.array([-v[1], v[0]]) / np.linalg.norm(v)
    t = np.r_[0, .12, np.linspace(.17, .83, 11), .88, 1]
    shift = np.r_[0, 0, np.array([(-1)**i for i in range(11)])*width, 0, 0]
    points = a+t[:, None]*v+shift[:, None]*n
    line(ax, points, color)


def damper(ax, x0, x1, y):
    # Horizontal dashpot with fixed cylinder on the left and piston on the right.
    line(ax, [(x0, y), (x0+.35, y)], VIRTUAL)
    line(ax, [(x0+.35, y+.18), (x0+.35, y-.18),
              (x0+1.0, y-.18)], VIRTUAL)
    line(ax, [(x0+.35, y+.18), (x0+1., y+.18)], VIRTUAL)
    line(ax, [(x0+.78, y-.14), (x0+.78, y+.14)], VIRTUAL)
    line(ax, [(x0+.78, y), (x1, y)], VIRTUAL)


def box(ax, x, y, w, h, text, color=INK, fontsize=10):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.06',
                              edgecolor=color, facecolor='white', lw=1.4))
    ax.text(x+w/2, y+h/2, text, ha='center', va='center', fontsize=fontsize,
            color=color, linespacing=1.5)


def main():
    fig = plt.figure(figsize=(13, 9.6), facecolor='white')
    a = fig.add_axes([.025, .40, .50, .55])
    a.set(xlim=(0, 9.7), ylim=(0, 8.0), aspect='equal')
    a.axis('off')
    a.set_title('(a) Mechanical meaning of the virtual elements', loc='left',
                fontweight='bold', fontsize=12, color=INK, pad=10)
    a.add_patch(Rectangle((3.9, 5.7), 1.4, 1.25, facecolor='#a8cedc', edgecolor=INK))
    a.text(4.6, 7.18, 'UR3e wrist', ha='center', color=INK)
    a.add_patch(Rectangle((2.25, 4.6), 5.1, 1.1, facecolor='#e3e9ed', edgecolor=INK, lw=2))
    a.plot([2.25, 7.35], [4.6, 4.6], color=VIRTUAL, lw=3)
    a.text(2.45, 5.16, 'Sensor disk', color=INK)
    a.plot(4.6, 5.7, 'o', color=INK, ms=5)
    a.text(5.05, 5.95, r'Tool origin $p_E$', color=INK)
    arrow(a, (7.2, 6.0), (5.65, 5.78), VIRTUAL, connectionstyle='arc3,rad=.5')
    a.text(7.05, 6.55, 'Virtual orientation\nspring + damper: '+r'$m_v$',
           ha='center', color=VIRTUAL, fontsize=9)
    arrow(a, (4.6, 5.64), (4.48, 4.65), color=INK)
    a.text(4.08, 5.15, r'$\hat r$', color=INK, ha='right')
    a.plot(4.45, 4.6, 'o', color=VIRTUAL, ms=8, zorder=8)
    a.text(3.6, 4.88, r'Selected point $p_c$', color=VIRTUAL, ha='right', fontsize=10)
    a.plot(6.35, 4.6, 'o', color=PHYSICAL, ms=7, zorder=8)
    a.text(7.0, 4.9, 'Actual contact', color=PHYSICAL, fontsize=10)
    a.add_patch(Circle((6.35, 4.30), .29, facecolor='#edbd9f', edgecolor=PHYSICAL, lw=2))
    line(a, [(6.35, 4.01), (6.35, 2.75)], PHYSICAL)
    spring(a, (6.35, 2.75), (6.35, 1.35), PHYSICAL, width=.17)
    line(a, [(5.85, 1.35), (6.85, 1.35)], PHYSICAL)
    for x in np.linspace(5.9, 6.8, 5):
        line(a, [(x, 1.35), (x-.12, 1.17)], PHYSICAL)
    a.text(7.0, 2.2, 'Physical support\nfixed: S2 / S3\nspring + damping:\nS1 / S4',
           color=PHYSICAL, fontsize=9, va='center', linespacing=1.5)
    spring(a, (4.45, 4.55), (4.45, 1.45), width=.19)
    a.plot(4.45, 1.25, 'o', markerfacecolor='white', markeredgecolor=VIRTUAL, ms=9, mew=2)
    line(a, [(4.45, 1.45), (4.45, 1.25)], VIRTUAL)
    a.text(4.05, 2.48, r'$K$', ha='right', color=VIRTUAL, fontsize=12)
    a.text(4.45, .72, r'Virtual target $a(t)$', color=VIRTUAL, ha='center')
    line(a, [(4.45, 4.6), (3.4, 4.6), (3.4, 3.45)], VIRTUAL)
    damper(a, 1.15, 3.4, 3.45)
    line(a, [(1.15, 3.08), (1.15, 3.82)], VIRTUAL)
    for y in np.linspace(3.1, 3.8, 4):
        line(a, [(1.15, y), (.98, y-.1)], VIRTUAL)
    a.text(2.25, 3.85, r'$D$: damps point velocity', color=VIRTUAL, ha='center', fontsize=9)
    a.text(1.05, 2.65, 'Zero-velocity\nreference', color=VIRTUAL, ha='center', fontsize=8.5)
    arrow(a, (4.92, 4.3), (4.92, 3.5), VIRTUAL)
    a.text(5.07, 3.8, r'$f_v$', color=VIRTUAL, fontsize=13)
    arrow(a, (6.75, 3.7), (6.75, 4.5), PHYSICAL)
    a.text(7.0, 3.85, 'Contact\nreaction', color=PHYSICAL, fontsize=8.5)
    a.text(.15, .08, 'Schematic side view; teal elements exist in the controller.\n'
           'The selected point can differ from the actual contact.', color=INK, fontsize=9)

    b = fig.add_axes([.565, .40, .41, .55])
    b.set(xlim=(0, 10), ylim=(0, 10)); b.axis('off')
    b.set_title('(b) Force, moment and joint torques', loc='left',
                fontweight='bold', fontsize=12, color=INK, pad=10)
    box(b, .1, 7.45, 9.7, 2.0,
        'Translation: virtual spring and viscous damping\n'
        r'$p_c=p_E+R_E\hat r,\quad v_c=J_c\dot q$'+'\n'
        r'$f_v=K(a-p_c)-D v_c$', VIRTUAL, 11)
    box(b, .1, 4.55, 9.7, 2.4,
        'Orientation: rotational spring and damping\n'
        r'$e_R=\mathrm{Log}(R_dR_E^T)^\vee$'+'\n'
        r'$m_v=K_R e_R-D_R\omega$'+'\n'
        r'$R_d$: requested orientation; $\omega=J_\omega\dot q$', VIRTUAL, 10.5)
    box(b, .1, 1.55, 9.7, 2.45,
        'Map the virtual force and moment to six joint torques\n'
        r'$\tau^*=J_c^T f_v+J_\omega^T m_v+b(q,\dot q)-0.2\dot q$'+'\n'
        r'$\tau=\mathrm{clip}(\tau^*,-\tau_{\max},\tau_{\max})$'+'\n'
        r'$\tau_{\max}=[54,54,28,9,9,9]\;\mathrm{N\,m}$', INK, 10)
    b.text(.1, .55, r'$b(q,\dot q)$ compensates model gravity and velocity terms.'+'\n'
           'Rotational gains (N m/rad; N m s/rad): S1 = 2 / 0.4; S2–S4 = 8 / 1.5.',
           color=INK, fontsize=9, linespacing=1.5)

    c = fig.add_axes([.035, .055, .94, .27])
    c.set(xlim=(0, 14), ylim=(0, 3.9)); c.axis('off')
    c.set_title('(c) Signals used by the implemented controller', loc='left',
                fontweight='bold', fontsize=12, color=INK, pad=13)
    boxes = [(0.1, 1.35, 1.7, .95, 'Synthetic EIT\nvoltages'),
             (2.1, 1.35, 1.8, .95, 'Trained estimator\n'+r'$\hat x,\hat y,\hat F$'),
             (4.25, 1.35, 1.65, .95, 'Select and latch\n'+r'$\hat r$'),
             (6.25, 1.35, 1.6, .95, 'Virtual force\nand moment'),
             (8.2, 1.35, 2.1, .95, 'Jacobian mapping\n+ bias compensation'),
             (10.65, 1.35, 3.1, .95, 'Torque limits\n'+r'$\rightarrow$ MuJoCo UR3e')]
    for x, y, w, h, text in boxes:
        box(c, x, y, w, h, text, fontsize=9)
    for first, second in zip(boxes[:-1], boxes[1:]):
        arrow(c, (first[0]+first[2]+.06, 1.825), (second[0]-.06, 1.825))
    c.text(5.1, 1.04, 'EIT position in S1–S3', ha='center', color=INK, fontsize=8)
    c.text(7.05, 3.05, r'Motion targets $a(t),R_d(t)$', ha='center', color=VIRTUAL, fontsize=10)
    arrow(c, (7.05, 2.9), (7.05, 2.36), VIRTUAL)
    c.annotate('', xy=(6.65, 2.92), xytext=(3., 2.38),
               arrowprops=dict(arrowstyle='->', lw=1.4, color=VIRTUAL,
                               linestyle='--', connectionstyle='angle,angleA=90,angleB=180,rad=6'))
    c.text(2.9, 3.55, r'S4 only: $\hat F$ adjusts the vertical target', color=VIRTUAL, fontsize=9)
    line(c, [(12.2, 1.29), (12.2, .32), (7.05, .32)], INK)
    arrow(c, (7.05, .32), (7.05, 1.29))
    c.text(9.6, .48, r'Robot state feedback: $q,\dot q$', color=INK, ha='center', fontsize=9)
    c.text(.1, .1, 'Centre and true-location conditions replace the EIT position selection.\n'
           'S4 uses the known location in every condition.', color=INK, fontsize=8.5, linespacing=1.5)

    FIG.mkdir(parents=True, exist_ok=True)
    for ext in ['png', 'pdf', 'svg']:
        fig.savefig(FIG / f'vmc_model.{ext}', dpi=200, facecolor='white', bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    main()
