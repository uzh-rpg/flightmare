import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
import quaternion
import matplotlib.colors as clrs
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import art3d
import math
from matplotlib import transforms
import matplotlib.cm as cm


plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.sans-serif": ["Computer Modern Roman"],
    'font.size': 15})

# Colors
RED = "#CF7E7B"
GRAY = "#A9A9A9"
BLUE = (0.13, 0.38, 0.55)
DARK_BLUE = '#385C9B'
GREEN = (0.13, 0.6, 0.33)
LIGHT_BLUE = "#CCE4EA"
LIGHT_RED = "#EFD4D3"
DARK_GRAY = '#6F6F6F'

reds = [(247.0/255.0, 234.0/255.0, 233.0/255.0),
        (223.0/255.0, 169.0/255.0, 167.0/255.0),
        (191.0/255.0, 83.0/255.0, 79.0/255.0)]  # R -> G -> B
blues = [(229.0/255.0, 242.0/255.0, 245.0/255.0),
         (153.0/255.0, 202.0/255.0, 213.0/255.0),
         (51.0 / 255.0, 149.0 / 255.0, 171.0 / 255.0)]  # R -> G -> B
greens = [(236.0/255.0, 239.0/255.0, 233.0/255.0),
          (179.0/255.0, 192.0/255.0, 167.0/255.0),
          (103.0 / 255.0, 128.0 / 255.0, 78.0 / 255.0)]  # R -> G -> B
cm_green = clrs.LinearSegmentedColormap.from_list('eth_green', greens, N=100)
cm_blue = clrs.LinearSegmentedColormap.from_list('eth_blue', blues, N=100)
cm_viridis = cm.get_cmap('viridis')

# Set plot colors
MAIN_COLOR = 'k'
RL_CM = cm_viridis
BASELINE_CM = cm_viridis


def divisors(n):
    divs = [1]
    for i in range(2, int(math.sqrt(n))+1):
        if n % i == 0:
            divs.extend([i, int(n/i)])
    divs.extend([n])
    return list(set(divs))


###############
# Static Gate #
###############

class StaticGate:
    """
    Static gate object fully defined by position, rotation and size.
    """
    def __init__(self, pos, rot, size, inflation=np.ones(2),
                 uncertainty=np.zeros(4), posn=None):
        """
        param: pos: center position of gate | ndarray (3,)
        param: rot: rotation of the gate | quaternion (w, x, y, z)
        param: size: size (width, height, depth) [m] | ndarray (3,)
        param: inflation: size of pillars (l, r) [m] | ndarray (2,)
        param: uncertainty: bounded displacements (x, y, z, yaw) | ndarray (4,)
        param: nominal position | ndarray (3,) used to visualize shifted gate
                                  within uncertainty bounds

        """
        self.pos = pos
        if posn is None:
            self.posn = pos
        else:
            self.posn = posn
        self.rot = rot
        self.size = size
        self.inflation = inflation
        self.uncertainty = uncertainty

        # Support waypoint
        # NOTE: Used as additional waypoint in case the angle of the gate
        # normal with the normal of the previous gate exceeds 90deg
        self.support_wp = np.zeros(2)

    def draw(self, c='k', draw_line_extension=False, draw_normal=False,
             draw_uncertainty=False, draw_support_wp=False,
             draw_inflation=False):
        """
        Draw static gate in XY plane.
        """

        # Get gate corners
        xy_l, xy_r = self.get_corners(margin=0.0)

        # Compute angle
        rot_mat = quaternion.as_rotation_matrix(self.rot)[:2, :2]
        ang = np.rad2deg(np.arctan2(rot_mat[1, 0], rot_mat[1, 1]))

        # Draw support waypoint
        if np.sum(self.support_wp) and draw_support_wp:
            circle = patches.Circle(tuple(self.support_wp), radius=3.0,
                                    color=BLUE, alpha=0.2)
            plt.gca().add_patch(circle)
            plt.scatter(*self.support_wp, c=BLUE)

        # Draw uncertainty rectangle
        if draw_uncertainty:
            uncertainty_rect = \
                patches.Rectangle((self.posn[0]-self.uncertainty[0],
                                   self.posn[1]-self.uncertainty[1]),
                                  2*self.uncertainty[0], 2*self.uncertainty[1],
                                  alpha=0.5, color=GRAY)
            plt.gca().add_patch(uncertainty_rect)

        # Compute gate line
        # NOTE: direction from left to right corner
        line = (xy_r - xy_l) / np.linalg.norm(xy_r - xy_l)

        # Draw gate incl. pillars
        if draw_inflation:
            bl = self.pos[:2] + np.array([-self.size[0]/2.0,
                                          -self.size[1]/2.0-self.inflation[1]])
            tf = transforms.Affine2D().rotate_deg_around(self.pos[0],
                                                         self.pos[1], ang)
            gate = patches.Rectangle(bl, self.size[0],
                                     self.size[1]+np.sum(self.inflation),
                                     color=BLUE, zorder=20)
            gate.set_transform(tf + plt.gca().transData)
            plt.gca().add_patch(gate)

        if draw_uncertainty:
            if self.uncertainty[3] > 0:
                wedge = patches.Wedge(self.pos[:2], 2.0,
                                      ang - self.uncertainty[3],
                                      ang + self.uncertainty[3], color=GREEN)
                plt.gca().add_patch(wedge)

        # Show gate as line segment
        bl = self.pos[:2] + np.array([-self.size[0]/2.0, -self.size[1]/2.0])
        tf = transforms.Affine2D().rotate_deg_around(self.pos[0],
                                                     self.pos[1], ang)
        gate = patches.Rectangle(bl, self.size[0], self.size[1],
                                 color=c, zorder=60)
        gate.set_transform(tf + plt.gca().transData)
        plt.gca().add_patch(gate)

        if draw_normal:
            normal = self.pos[:2] + self.get_normal()
            plt.plot([self.pos[0], normal[0]], [self.pos[1], normal[1]],
                     linewidth=3, c='k', zorder=20)

        # Draw line extension to visualize gate errors
        ext_line_length = 10.0  # m
        if draw_line_extension:
            lb_e = xy_l - line * ext_line_length / 2
            rb_e = xy_r + line * ext_line_length / 2
            plt.plot([lb_e[0], rb_e[0]], [lb_e[1], rb_e[1]], '--',
                     linewidth=1, c=c, alpha=0.2, zorder=10)

    def get_corners(self, margin=0.0):
        """
        Get gate corners in XY plane.
        param: margin: margin to actual gate corners.

        return:
        left_corner: ndarray | (2,)
        right_corner: ndarray | (2,)
        """

        # Subtract margin from gate width
        valid_width = self.size[2] - 2 * margin

        # Get gate corners by rotating line segment
        vert_line = np.array([[0.0,             0.0],
                              [valid_width / 2, -valid_width / 2]])
        rot_mat = quaternion.as_rotation_matrix(self.rot)[:2, :2]
        rot_line = np.matmul(rot_mat, vert_line)

        left_corner = self.pos[:2] + rot_line[:, 0]
        right_corner = self.pos[:2] + rot_line[:, 1]

        return left_corner, right_corner

    def get_normal(self):
        """
        Get gate normal facing previous path segment.
        """
        xy_l, xy_r = self.get_corners()
        dy = xy_l[1] - xy_r[1]
        dx = xy_l[0] - xy_r[0]
        norm = np.array([dy, -dx])
        norm /= np.linalg.norm(norm)
        return norm


def load_gates_from_cfg(cfg):

    # Create gate objects
    cfg_gates = cfg["gates"]
    n_gates = cfg_gates["N"]
    gates = []
    for gate_id in range(n_gates):

        pos = np.array(cfg_gates[f"Gate{gate_id + 1}"]["position"])
        rot = np.quaternion(*cfg_gates[f"Gate{gate_id + 1}"]["rotation"])
        size = np.array(cfg_gates[f"Gate{gate_id + 1}"]["size"])

        if cfg_gates[f"Gate{gate_id + 1}"].get("position_nominal",
                                               None) is not None:
            posn = np.array(
                cfg_gates[f"Gate{gate_id + 1}"]["position_nominal"])
        else:
            # print('No nominal position provided. Using position instead.')
            posn = pos

        try:
            inflation = np.array(cfg_gates[f"Gate{gate_id + 1}"]["inflation"])
        except KeyError:
            # print('No inflation provided. Using default: 1m')
            inflation = np.ones(2)

        try:
            uncertainty = \
                np.array(cfg_gates[f"Gate{gate_id + 1}"]["uncertainty"])
        except KeyError:
            # print('No uncertainty provided. Creating deterministic gate.')
            uncertainty = np.array([0.0, 0.0, 0.0, 0.0])

        gates.append(StaticGate(pos, rot, size, inflation, uncertainty, posn))

    # Compute support waypoints based on gate orientation
    # NOTE: Required if angle between gate normal and previous path segment
    # exceeds 90 degrees to encourage flying around the gate
    start_pos = cfg['start_pos']
    circular = np.isclose(start_pos, gates[-1].pos).all()
    start_gate = 0 if circular else 1
    for g_id in np.arange(start_gate, len(gates)):

        # compute gate normal and direction of previous path segment
        gate = gates[g_id]
        normal = gate.get_normal()
        seg_dir = (gates[g_id-1].pos - gate.pos)[:2]
        seg_dir /= np.linalg.norm(seg_dir)

        # compute angle between normals
        ang = np.arctan2(normal[0]*seg_dir[1]-normal[1]*seg_dir[0],
                         normal[0]*seg_dir[0]+normal[1]*seg_dir[1])

        r = 3.0
        if abs(ang) >= (np.pi/2.0):
            a = np.sqrt(r**2 - (gate.size[1]/2.0)**2)
            support_wp = gate.pos[:2] + a * normal
        else:
            support_wp = np.zeros(2)
        gate.support_wp = support_wp

    return gates


#############
# Draw path #
#############

def draw_path(start_pos, gates, c='k', draw_line_extension=False,
              draw_uncertainty=False, draw_normal=False, draw_inflation=False,
              draw_support_wp=False):
    """
    Draw path segments, start/end points and gates.
    :param start_pos: start position | (3, )
    :param gates: list of gate objects
    """

    for i, gate in enumerate(gates):
        if i == 0:
            plt.plot([start_pos[0], gate.pos[0]], [start_pos[1], gate.pos[1]],
                     color=c, zorder=-1, linewidth=1)
        else:
            plt.plot([gates[i-1].pos[0], gate.pos[0]],
                     [gates[i-1].pos[1], gate.pos[1]],
                     color=c, zorder=-1, linewidth=1)
        gate.draw(draw_line_extension=draw_line_extension, c=c,
                  draw_uncertainty=draw_uncertainty,
                  draw_normal=draw_normal,
                  draw_inflation=draw_inflation,
                  draw_support_wp=draw_support_wp)


####################
# Draw environment #
####################

def draw_env(cfg, c='k', draw_line_extension=True, draw_uncertainty=False,
             draw_normal=False, draw_inflation=False, draw_support_wp=False):
    """
    Plot track layout from config file incl. start position, goal position and
    all gates
    """

    # Get gates
    gates = load_gates_from_cfg(cfg)

    # Get start and goal position
    start_pos = np.array(cfg["start_pos"])

    # Draw path incl. gates and connecting line segments
    draw_path(start_pos, gates, c=c, draw_support_wp=draw_support_wp,
              draw_inflation=draw_inflation, draw_normal=draw_normal,
              draw_uncertainty=draw_uncertainty,
              draw_line_extension=draw_line_extension)

    # Set world box in 2D plot
    world_box = np.array(cfg["world_box"])
    plt.xlim([world_box[0], world_box[3]])
    plt.ylim([world_box[1], world_box[4]])
    plt.gca().set_aspect('equal')
    plt.xlabel("x [m]", rotation=0, labelpad=15)
    plt.ylabel("y [m]", rotation=90, labelpad=10)
    xmin, xmax = plt.gca().get_xlim()
    ymin, ymax = plt.gca().get_ylim()
    grid_size = 10
    x_ticks = int((xmax - xmin) // grid_size)
    y_ticks = int((ymax - ymin) // grid_size)
    plt.gca().set_xticks(np.linspace(xmin+grid_size, xmax-grid_size,
                                     x_ticks-1, endpoint=True))
    plt.gca().set_yticks(np.linspace(ymin+grid_size, ymax-grid_size,
                                     y_ticks-1, endpoint=True))


def draw_env_3d(cfg, grid=10, c='k', draw_uncertainty=False):
    """
    Plot track layout from config file incl. start position, goal position and
    all gates
    """

    try:

        # Get gates
        gates = load_gates_from_cfg(cfg)
        world_box = cfg["world_box"]

        # Get start and goal position
        start_pos = np.array(cfg["start_pos"])

        # Draw path incl. gates and connecting line segments
        for i, gate in enumerate(gates):
            # Draw line
            if i == 0:
                plt.gca().plot([start_pos[0], gate.pos[0]],
                               [start_pos[1], gate.pos[1]],
                               [start_pos[2], gate.pos[2]],
                               color=c, zorder=-10, linewidth=1)
            else:
                plt.gca().plot([gates[i - 1].pos[0], gate.pos[0]],
                               [gates[i - 1].pos[1], gate.pos[1]],
                               [gates[i - 1].pos[2], gate.pos[2]],
                               color=c, zorder=-10, linewidth=1)

            # Draw gates
            xy_l, xy_r = gate.get_corners()
            frb = xy_r + gate.get_normal()[:2] * gate.size[0]/2.0
            flb = xy_l + gate.get_normal()[:2] * gate.size[0]/2.0
            blb = xy_l - gate.get_normal()[:2] * gate.size[0]/2.0
            brb = xy_r - gate.get_normal()[:2] * gate.size[0]/2.0
            frbb = np.array([frb[0], frb[1], gate.pos[2]-gate.size[2]/2.0])
            brbb = np.array([brb[0], brb[1], gate.pos[2]-gate.size[2]/2.0])
            frbt = np.array([frb[0], frb[1], gate.pos[2]+gate.size[2]/2.0])
            brbt = np.array([brb[0], brb[1], gate.pos[2]+gate.size[2]/2.0])
            flbb = np.array([flb[0], flb[1], gate.pos[2]-gate.size[2]/2.0])
            blbb = np.array([blb[0], blb[1], gate.pos[2]-gate.size[2]/2.0])
            flbt = np.array([flb[0], flb[1], gate.pos[2]+gate.size[2]/2.0])
            blbt = np.array([blb[0], blb[1], gate.pos[2]+gate.size[2]/2.0])
            corners = np.array([brbb, frbb, frbt, brbt])
            if i in [0, 1, 7, 8, 9, 10]:
                alpha_right = 0.99
            else:
                alpha_right = 1.0
            tri = art3d.Poly3DCollection([corners], edgecolors=c,
                                         facecolors=c, alpha=alpha_right)
            plt.gca().add_collection3d(tri)
            corners = np.array([flbt, frbt, brbt, blbt])
            tri = art3d.Poly3DCollection([corners], edgecolors=c,
                                         facecolors=c)
            plt.gca().add_collection3d(tri)
            corners = np.array([flbb, frbb, brbb, blbb])
            tri = art3d.Poly3DCollection([corners], edgecolors=c,
                                         facecolors=c, alpha=0.99)
            plt.gca().add_collection3d(tri)
            corners = np.array([blbb, flbb, flbt, blbt])
            if i in [2, 3, 4, 5, 6]:
                alpha_left = 0.99
            else:
                alpha_left = 1.0
            tri = art3d.Poly3DCollection([corners], edgecolors=c,
                                         facecolors=c, alpha=alpha_left)
            plt.gca().add_collection3d(tri)

            # Draw uncertainty bounds
            fbl = np.array([-1.0, -1.0, -1.0]) * gate.uncertainty[:3] + gate.posn
            fbr = np.array([1.0, -1.0, -1.0]) * gate.uncertainty[:3] + gate.posn
            bbl = np.array([-1.0, 1.0, -1.0]) * gate.uncertainty[:3] + gate.posn
            bbr = np.array([1.0, 1.0, -1.0]) * gate.uncertainty[:3] + gate.posn
            ftl = np.array([-1.0, -1.0, 1.0]) * gate.uncertainty[:3] + gate.posn
            ftr = np.array([1.0, -1.0, 1.0]) * gate.uncertainty[:3] + gate.posn
            btl = np.array([-1.0, 1.0, 1.0]) * gate.uncertainty[:3] + gate.posn
            btr = np.array([1.0, 1.0, 1.0]) * gate.uncertainty[:3] + gate.posn
            cu = BLUE
            cw = DARK_BLUE
            if draw_uncertainty:
                corners = np.array([fbl, fbr, bbr, bbl])
                tri = art3d.Poly3DCollection([corners], facecolors=cu,
                                             alpha=0.7, zorder=23)
                plt.gca().add_collection3d(tri)
                corners = np.array([bbl, bbr, btr, btl])
                tri = art3d.Poly3DCollection([corners], facecolors=cu,
                                             alpha=0.3, zorder=23)
                plt.gca().add_collection3d(tri)
                corners = np.array([fbr, bbr, btr, ftr])
                tri = art3d.Poly3DCollection([corners], facecolors=cu,
                                             alpha=0.3, zorder=23)
                plt.gca().add_collection3d(tri)
                rot_mat = quaternion.as_rotation_matrix(gate.rot)[:2, :2]
                ang = np.rad2deg(np.arctan2(rot_mat[1, 0], rot_mat[1, 1]))
                wedge = patches.Wedge(gate.pos[:2], 3.0, ang -
                                      gate.uncertainty[3],
                                      ang + gate.uncertainty[3], color=cw,
                                      zorder=23)
                plt.gca().add_patch(wedge)
                art3d.pathpatch_2d_to_3d(wedge, z=gate.pos[2], zdir='z')

        # Set world box in 2D plot
        plt.gca().set_xlim3d([world_box[0], world_box[3]])
        plt.gca().set_ylim3d([world_box[1], world_box[4]])
        plt.gca().set_zlim3d([world_box[2], world_box[5]])

        plt.gca().xaxis.pane.fill = False
        plt.gca().yaxis.pane.fill = False
        plt.gca().zaxis.pane.fill = False

        # Now set color to white (or whatever is "invisible")
        plt.gca().xaxis.pane.set_edgecolor('w')
        plt.gca().yaxis.pane.set_edgecolor('w')
        plt.gca().zaxis.pane.set_edgecolor('w')
        plt.xlabel("x [m]", rotation=90, labelpad=15)
        plt.ylabel("y [m]", rotation=90, labelpad=15)
        zmin, zmax = plt.gca().get_zlim()
        xmin, xmax = plt.gca().get_xlim()
        ymin, ymax = plt.gca().get_ylim()
        grid_size = grid
        x_ticks = int((xmax - xmin) // grid_size)
        y_ticks = int((ymax - ymin) // grid_size)
        z_ticks = int((zmax - zmin) // grid_size)
        plt.gca().set_zticks(np.linspace(zmin, zmax, z_ticks+1,
                                         endpoint=True))
        plt.gca().set_xticks(np.linspace(xmin+grid_size, xmax-grid_size,
                                         x_ticks-1, endpoint=True))
        plt.gca().set_yticks(np.linspace(ymin+grid_size, ymax-grid_size,
                                         y_ticks-1, endpoint=True))
        plt.gca().set_zlabel('z [m]', rotation=45, labelpad=15)
        plt.xticks(rotation=0)
        plt.yticks(rotation=0)

        y_f = (ymax - ymin) / (xmax - xmin)
        z_f = (zmax - zmin) / (xmax - xmin)
        plt.gca().set_box_aspect((1, y_f, z_f))

    except KeyError:
        print("Incorrect config format. Exiting...")
        exit(1)


#####################
# Visualize rollout #
#####################

def viz_rollout(cfg, rollout_data, show_projection=False, show_3d=False):
    """
    Plot top-down view of policy rollout incl. track layout, drone position
    and velocity.
    """

    # Get environment info
    dt = cfg["environment"]["sim_dt"]
    gates = load_gates_from_cfg(cfg['track'])
    gate_heights = np.array([gate.pos[2] for gate in gates])

    # Get data from dict
    pos = rollout_data['pos']
    pos_proj = rollout_data['pos_proj']
    v = rollout_data['v']
    segment_times = rollout_data['segment_times']
    target_change_ids = rollout_data['target_change_ids']
    termination_reason = rollout_data['termination_reason']

    # Set up grid layout for figure
    fig = plt.figure(figsize=(16, 10))
    gs1 = gridspec.GridSpec(2, 1)
    gs1.update(left=0.05, right=0.45, top=0.85, bottom=0.15, hspace=0.5)
    ax1 = plt.subplot(gs1[0])
    ax2 = plt.subplot(gs1[1])
    gs2 = gridspec.GridSpec(1, 1)
    gs3 = gridspec.GridSpec(1, 1)

    gs4 = gridspec.GridSpec(1, 1)
    gs2.update(left=0.55, right=0.87, top=0.85, bottom=0.15)
    gs3.update(left=0.89, right=0.91, top=0.7, bottom=0.3)
    gs4.update(left=0.93, right=0.95, top=0.7, bottom=0.3)
    ax5 = plt.subplot(gs4[0, 0])
    gs2.update(left=0.55, right=0.91, top=0.85, bottom=0.15)
    gs3.update(left=0.93, right=0.95, top=0.7, bottom=0.3)
    if show_3d:
        ax3 = plt.subplot(gs2[0, 0], projection='3d')
    else:
        ax3 = plt.subplot(gs2[0, 0])
    ax4 = plt.subplot(gs3[0, 0])

    title_str = f'Policy Analysis'
    plt.suptitle(r"\textbf{" + title_str + "}", fontsize=18, y=0.94)

    # Plot track layout and drone path
    # -------------------------------------------------------------------------
    plt.sca(ax3)
    if show_3d:
        draw_env_3d(cfg['track'])
    else:
        draw_env(cfg['track'])

    # Display trajectory
    if show_3d:
        path_plot = ax3.scatter(xs=pos[:, 0], ys=pos[:, 1], zs=pos[:, 2],
                                c=v, s=5, zorder=1, cmap=RL_CM)
        from matplotlib import cm
        viridis = cm.get_cmap('viridis')
        v /= np.max(v)
        for i in range(len(pos)):
           drone = patches.Circle((pos[i, 0], pos[i, 1]), radius=0.34,
                                  color=viridis(v[i]))
           plt.gca().add_patch(drone)
           art3d.pathpatch_2d_to_3d(drone, z=pos[i, 2], zdir='z')
    else:
        path_plot = ax3.scatter(pos[:, 0], pos[:, 1], c=v,
                                s=25, zorder=-1, cmap=RL_CM)

    # Display projection of position onto center line
    if show_projection:
        proj_stride = 5
        plt.scatter(pos_proj[:, 0], pos_proj[:, 1], c=GRAY, s=5, alpha=0.5,
                    zorder=20)
        plt.plot([pos[:pos_proj.shape[0]:proj_stride, 0],
                  pos_proj[::proj_stride, 0]],
                 [pos[:pos_proj.shape[0]:proj_stride, 1],
                  pos_proj[::proj_stride, 1]], c=GRAY, alpha=0.5)

    # Show color bar
    cbar = plt.colorbar(path_plot, cax=ax4)
    if ax5 is None:
        cbar.set_label("v [m/s]", rotation=-90)
        ax4.yaxis.set_label_coords(2.5, 0.5)

    plt.title(f"Racing Environment | Final State: " + termination_reason,
              fontsize=15)

    # Plot segment times
    # -------------------------------------------------------------------------
    plt.sca(ax1)
    plt.bar(np.arange(1, len(segment_times) + 1), segment_times,
                color=BLUE)

    plt.xlabel("Path Segment")
    plt.ylabel("Time [s]")
    plt.xticks(np.arange(1, len(segment_times)+1))
    # Display total track time
    total_time = np.sum(segment_times)
    time_str = f'Track Time Statistics | Total: {total_time:.2f} s'
    plt.title(time_str, fontsize=15)

    # Show height profile
    # -------------------------------------------------------------------------
    world_box = np.array(cfg['track']['world_box'])
    plt.sca(ax2)

    # Get maximum episode duration
    t_total = len(pos) * dt

    # Get suitable episode duration for plotting
    if not np.floor(t_total) == t_total:
        ep_duration_int = int(t_total) + 1
    else:
        ep_duration_int = int(t_total)

    # Plot horizontal line to show gate center height
    min_gate_height = np.min(gate_heights)
    max_gate_height = np.max(gate_heights)
    plt.plot([0, ep_duration_int/dt], [min_gate_height, min_gate_height],
             c=GRAY, zorder=10)
    plt.plot([0, ep_duration_int/dt], [max_gate_height, max_gate_height],
             c=GRAY, zorder=10)

    # Plot horizontal lines to show world box bounds
    plt.plot([0, ep_duration_int/dt], [world_box[2], world_box[2]], c=BLUE,
             zorder=10)
    plt.plot([0, ep_duration_int/dt], [world_box[5], world_box[5]], c=BLUE,
             zorder=10)

    # Plot drone height profile
    plt.plot(pos[:, 2], zorder=20, c=BLUE, linewidth=3)

    # Plot vertical line to indicate time steps of gate traversals
    g_id = 0
    for c_id in target_change_ids:
        # Get vertical gate dimension
        g_b = gates[g_id].pos[2] - gates[g_id].size[2] / 2.0
        g_t = gates[g_id].pos[2] + gates[g_id].size[2] / 2.0
        g_id += 1
        g_id = g_id % len(gates)
        plt.plot([c_id, c_id], [g_b, g_t], c=BLUE)

    plt.xlabel('Time [s]')
    div = np.array(sorted(divisors(int(ep_duration_int/0.5))), dtype=np.int)
    if (div < 20).any():
        ind = np.where(div < 20)[0][-1]
        n_xticks = div[ind] + 1
    else:
        n_xticks = 3 if int(ep_duration_int/dt) % 2 == 0 else 4
    plt.xticks(np.linspace(0, ep_duration_int/dt, n_xticks),
               np.linspace(0, ep_duration_int, n_xticks))
    # Get world box dimensions including margin
    world_floor_int = int(world_box[2]) - 1
    world_ceiling_int = int(np.ceil(world_box[5])) + 1
    world_height_viz = int(world_ceiling_int-world_floor_int)
    div = np.array(sorted(divisors(world_height_viz)), dtype=np.int)
    if (div < 10).any():
        ind = np.where(div < 10)[0][-1]
        n_yticks = div[ind] + 1
    else:
        n_yticks = 3 if world_height_viz % 2 == 0 else 4
    plt.yticks(np.linspace(world_floor_int, world_ceiling_int, n_yticks))
    plt.ylabel('Height [m]')
    plt.title('Height Profile', fontsize=15)

    return fig


###############################
# Visualize states and inputs #
###############################

def viz_states_inputs(cfg, rollout_data):

    # Get rollout data
    pos = rollout_data['pos']
    dpos = rollout_data['dpos']
    euler = rollout_data['euler']
    deuler = rollout_data['deuler']
    actions = rollout_data['actions']

    fig = plt.figure(figsize=(16, 10))
    title_str = f'States and Actions'
    plt.suptitle(r"\textbf{" + title_str + "}", fontsize=18, y=0.98)
    gs = gridspec.GridSpec(5, 12)
    gs.update(left=0.02, right=0.98, top=0.95, bottom=0.02, wspace=0.3,
              hspace=0.5)

    #
    ax_x = fig.add_subplot(gs[0, 0:4])
    ax_y = fig.add_subplot(gs[0, 4:8])
    ax_z = fig.add_subplot(gs[0, 8:12])
    #
    ax_dx = fig.add_subplot(gs[1, 0:4])
    ax_dy = fig.add_subplot(gs[1, 4:8])
    ax_dz = fig.add_subplot(gs[1, 8:12])
    #
    ax_euler_x = fig.add_subplot(gs[2, 0:4])
    ax_euler_y = fig.add_subplot(gs[2, 4:8])
    ax_euler_z = fig.add_subplot(gs[2, 8:12])
    #
    ax_euler_vx = fig.add_subplot(gs[3, 0:4])
    ax_euler_vy = fig.add_subplot(gs[3, 4:8])
    ax_euler_vz = fig.add_subplot(gs[3, 8:12])
    #
    ax_action0 = fig.add_subplot(gs[4, 0:3])
    ax_action1 = fig.add_subplot(gs[4, 3:6])
    ax_action2 = fig.add_subplot(gs[4, 6:9])
    ax_action3 = fig.add_subplot(gs[4, 9:12])

    t = np.arange(0, pos.shape[0])
    ax_x.step(t, pos[:, 0], c=BLUE)
    ax_y.step(t, pos[:, 1], c=BLUE)
    ax_z.step(t, pos[:, 2], c=BLUE, label="pos [x, y, z]")
    #
    ax_dx.step(t, dpos[:, 0], c=BLUE)
    ax_dy.step(t, dpos[:, 1], c=BLUE)
    ax_dz.step(t, dpos[:, 2], c=BLUE, label=r"vel [r, $\theta$, $\phi$]")
    #
    ax_euler_x.step(t, euler[:, -1], c=BLUE)
    ax_euler_y.step(t, euler[:, 1], c=BLUE)
    ax_euler_z.step(t, euler[:, 0], c=BLUE, label=r"$\theta$ [x, y, z]")
    #
    ax_euler_vx.step(t, deuler[:, 0], c=BLUE)
    ax_euler_vy.step(t, deuler[:, 1], c=BLUE)
    ax_euler_vz.step(t, deuler[:, 2], c=BLUE, label=r"$\omega$ [x, y, z]")
    #
    ax_action0.step(t, actions[:, 0], c=BLUE)
    ax_action1.step(t, actions[:, 1], c=BLUE)
    ax_action2.step(t, actions[:, 2], c=BLUE)
    ax_action3.step(t, actions[:, 3], c=BLUE, label='action [f1, f2, f3, f4]')

    ax_z.legend()
    ax_dz.legend()
    ax_euler_z.legend()
    ax_euler_vz.legend()
    ax_action3.legend()
    #
    plt.tight_layout()

    return fig


##############################
# Visualize reward evolution #
##############################

def viz_rew_ev(cfg, rollout_data):

    # Get reward components
    reward_components_names = rollout_data['reward_components_names']
    reward_components = rollout_data['reward_components']
    assert reward_components.shape[1] == 9

    fig = plt.figure(figsize=(16, 10))
    title_str = f'Reward Evolution'
    plt.suptitle(r"\textbf{" + title_str + "}", fontsize=18, y=0.98)
    gs = gridspec.GridSpec(2, 4)
    gs.update(left=0.02, right=0.98, top=0.90, bottom=0.05, wspace=0.3,
              hspace=0.2)

    # Plot evolution of single reward component
    viz_ids = np.setdiff1d(np.arange(reward_components.shape[1]), 7).astype(np.int)
    for i, id in enumerate(viz_ids):
        ax = fig.add_subplot(gs[int(i/4), int(i % 4)])
        ax.plot(reward_components[:, id], c=BLUE)
        ax.set_title(reward_components_names[id])

    return fig