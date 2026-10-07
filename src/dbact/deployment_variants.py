"""Information-matched engineering deployment controls; pre-CONTACT_READY only."""
from dataclasses import replace
import numpy as np
from dbact.phase import Phase
from dbact.boundary_density import BoundaryAwareDensity

METHODS=('boundary_cvt','nearest_observed_target','local_spacing','no_arc','no_arc_normalized','no_direction_offsets','no_exploration')

class DeploymentVariant:
    def __init__(self,controller,method):
        if method not in METHODS: raise ValueError(method)
        self.controller=controller; self.base=controller.cvt; self.method=method
    def compute(self,i,agents,neighbors,density,domain):
        cell=self.base.compute(i,agents,neighbors,density,domain)
        if self.method=='boundary_cvt' or self.controller.phase_monitor.reached(Phase.CONTACT_READY) or not len(density.points): return cell
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
        alternate=self.base.compute(i,agents,neighbors,modified,domain)
        # All comparisons preserve the original mass and redeployment decisions.
        return replace(cell,centroid=alternate.centroid.copy())
