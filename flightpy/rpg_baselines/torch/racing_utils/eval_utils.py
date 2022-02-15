import numpy as np
from rpg_baselines.torch.racing_utils.viz_utils import load_gates_from_cfg


def compute_stats(cfg, rollout_data):
    """
    Compute relevant statistics for policy analysis from rollout data.
    """

    # Get relevant environment information
    dt = cfg['environment']['sim_dt']
    n_segments = cfg['track']['gates']['N']
    gates = load_gates_from_cfg(cfg['track'])
    start_pos = np.array(cfg["track"]["start_pos"])
    goal_pos = gates[-1].pos
    circular = np.isclose(start_pos, goal_pos).all()

    # Get rollout data
    pos = rollout_data['pos']
    dpos = rollout_data['dpos']
    target_wps = rollout_data['target_wps']
    modes = rollout_data['modes']
    cum_path_length = rollout_data["cum_path_length"]
    episode_length = len(pos)

    # Compute projection point on center path
    # Get start and end point of path segment
    P1 = np.zeros(pos.shape)
    P2 = np.zeros(pos.shape)
    for i in range(1, episode_length):
        # Shift of -1 since first target waypoint belongs to second observation
        s_id = min(int(target_wps[i-1]), len(cum_path_length)-1)
        if s_id == 1:
            P1[i] = start_pos
        else:
            P1[i] = gates[s_id-2].pos
        if s_id < (cum_path_length.shape[0]-1):
            P2[i] = gates[s_id-1].pos
        else:
            P2[i] = goal_pos
    # Set start and end point of segment of first time step
    P1[0] = start_pos
    P2[0] = gates[0].pos

    # Compute projection onto center line
    P12 = P2 - P1
    V = pos - P1
    Pproj = P1 + np.reshape(np.diag(np.dot(V, P12.T)) /
                            np.diag(np.dot(P12, P12.T)), (-1, 1)) * P12

    rollout_data.update({"pos_proj": Pproj})

    # Compute translational velocity
    v = np.linalg.norm(dpos, axis=1)
    rollout_data.update({"v": v})

    # Compute segment times
    segment_times = np.zeros(n_segments, dtype=np.float)
    target_change_ids = np.where(target_wps[:-1] != target_wps[1:])[0] + 1
    # Remove last target change in case of circular track and single race
    # setting
    if circular:
        target_change_ids = target_change_ids[:-1]
        target_wps[-1] = n_segments
    target_change_ids = np.append(target_change_ids, episode_length)
    target_change_ids = np.unique(target_change_ids)

    seg_cur, c_id_prev = 0, 0
    for s_id, c_id in enumerate(target_change_ids):
        seg_cur = target_wps[c_id-1] - 1
        segment_times[seg_cur] += (c_id - c_id_prev) * dt
        c_id_prev = c_id
    try:
        assert np.isclose(np.sum(segment_times), episode_length * dt)
    except AssertionError:
        print('Accumulated segment times != Episode time. Exiting...')
        print(f"Episode Length: {episode_length}")
        print(f"Segment Lengths: {segment_times}")
        exit()
    rollout_data.update({'segment_times': segment_times,
                         'target_change_ids': target_change_ids})

    # Add reward components names
    rollout_data.update({'reward_components_names':
                         cfg['rewards']['names']})

    # Set termination reason
    termination_reasons = {0: "Maximum Episode Length",
                           1: "Finish",
                           2: "Gate Error",
                           3: "Wall Crash",
                           4: "Tube Crash",
                           5: "Floor Crash",
                           6: "Ceiling Crash",
                           7: "Body Rate Constraint"}
    termination_reason = termination_reasons[modes[-1]]
    if circular and modes[-1] == 0:
        termination_reason = 'Finish'
    rollout_data.update({"termination_reason": termination_reason,
                         "termination_reasons": termination_reasons})

    return rollout_data
