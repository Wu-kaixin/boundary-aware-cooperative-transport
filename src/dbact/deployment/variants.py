"""Information-matched engineering deployment controls; pre-CONTACT_READY only."""
from dataclasses import replace
import numpy as np
from dbact.control.phase import Phase
from dbact.deployment.density import BoundaryAwareDensity

METHODS=('boundary_cvt','nearest_observed_target','local_spacing','push_cone','no_arc','no_arc_normalized','no_direction_offsets','no_exploration')


def push_cone_index(points, normals, offsets, origin, neighbor_positions, goal, *, threshold, local_radius, d_min):
    """Index of a trailing-cone sample, or None when that cone is not locally visible.

    This is an experimental target. It does not replace CVT, nearest, or spacing.
    Samples whose outward normal is not within the existing push-side threshold of
    the trailing direction are excluded. With no eligible sample the caller keeps
    the CVT centroid.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    normals = np.asarray(normals, dtype=float).reshape(-1, 2)
    offsets = np.asarray(offsets, dtype=float).reshape(-1)
    origin = np.asarray(origin, dtype=float).reshape(2)
    goal = np.asarray(goal, dtype=float).reshape(2)
    if len(points) == 0 or len(points) != len(normals) or len(points) != len(offsets):
        return None
    cone = np.flatnonzero(normals @ goal <= -float(threshold))
    if len(cone) == 0:
        return None
    targets = points + offsets[:, None] * normals
    own = np.linalg.norm(targets - origin, axis=1)
    eligible = cone[own[cone] <= float(local_radius)]
    if len(eligible) == 0:
        return None
    if len(neighbor_positions):
        neighbors = np.asarray(neighbor_positions, dtype=float).reshape(-1, 2)
        separation = np.min(np.linalg.norm(targets[:, None, :] - neighbors[None, :, :], axis=2), axis=1)
    else:
        separation = np.full(len(targets), 2 * float(d_min))
    score = np.minimum(separation, 2 * float(d_min)) - 0.25 * own
    return int(eligible[np.argmax(score[eligible])])

class DeploymentVariant:
    def __init__(self,controller,method):
        if method not in METHODS: raise ValueError(method)
        self.controller=controller; self.base=controller.cvt; self.method=method
    def compute(self,i,agents,neighbors,density,domain):
        cell=self.base.compute(i,agents,neighbors,density,domain)
        if self.method=='boundary_cvt' or self.controller.phase_monitor.reached(Phase.CONTACT_READY) or not len(density.points): return cell
        if self.method == 'push_cone':
            view = self.controller._views[agents[i].agent_id]
            goal = self.controller._goal_for(view)
            if goal is None:
                return cell
            choice = push_cone_index(
                density.points, density.normals, density.offsets, agents[i].position,
                [agents[j].position for j in neighbors], goal,
                threshold=self.controller.params.push_side_threshold,
                local_radius=self.controller.params.local_radius,
                d_min=self.controller.params.d_min,
            )
            if choice is None:
                return cell
            targets = density.points + density.offsets[:, None] * density.normals
            return replace(cell, centroid=targets[choice].copy())
        if self.method in ('nearest_observed_target','local_spacing'):
            targets=density.points+density.offsets[:,None]*density.normals
            own=np.linalg.norm(targets-agents[i].position,axis=1)
            if self.method=='nearest_observed_target': k=int(np.argmin(own))
            else:
                eligible=np.where(own<=self.controller.params.local_radius)[0]
                if not len(eligible): return cell
                separation=np.min(np.linalg.norm(targets[:,None,:]-np.asarray([agents[j].position for j in neighbors])[None,:,:],axis=2),axis=1) if neighbors else np.full(len(targets),2*self.controller.params.d_min)
                score=np.minimum(separation,2*self.controller.params.d_min)-.25*own
                k=int(eligible[np.argmax(score[eligible])])
            return replace(cell,centroid=targets[k].copy())
        view=self.controller._views[agents[i].agent_id]
        params=density.params
        if self.method in ('no_arc','no_arc_normalized'):
            arc=np.ones(len(view))
            if self.method=='no_arc_normalized' and np.sum(view.confidence)>0:
                arc*=np.sum(np.maximum(view.arc_length,0)*view.confidence)/np.sum(view.confidence)
            view=replace(view,arc_length=arc)
        if self.method=='no_direction_offsets': params=replace(params,lead_offset=None)
        if self.method=='no_exploration': params=replace(params,explore_gain=0.)
        modified=BoundaryAwareDensity.from_view(view,params,robot_positions=np.asarray([agents[i].position]+[agents[j].position for j in neighbors]),goal_direction=self.controller._goal_for(view))
        if self.method=='no_arc_normalized' and float(np.sum(modified.weights))>0:
            # Preserve total kernel mass after gap/exploration factors as well.
            modified.weights *= float(np.sum(density.weights))/float(np.sum(modified.weights))
        alternate=self.base.compute(i,agents,neighbors,modified,domain)
        # All comparisons preserve the original mass and redeployment decisions.
        return replace(cell,centroid=alternate.centroid.copy())
