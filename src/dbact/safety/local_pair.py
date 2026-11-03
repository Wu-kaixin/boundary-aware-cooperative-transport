"""Current one-hop responsibility negotiation during stationary acquisition.

The simulator routes synchronous offer/proposal/ACK rounds. Each decision uses
one node's incident messages. This is neither global matching nor an unknown-
boundary safety proof; finite observed object rows retain their empirical scope.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json

import numpy as np
from scipy.optimize import LinearConstraint, minimize

from dbact.geometry import clip_to_domain
from dbact.safety.qp2d import solve_min_norm_2d


TOLERANCE = 1e-7  # Same feasibility tolerance as the existing planar solver.


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode('utf-8')).hexdigest()


def _contract(params):
    return tuple(sorted(asdict(params).items()))


@dataclass(frozen=True)
class SafetyOffer:
    agent_id: str
    frame: int
    timestamp: float
    dt: float
    position: tuple
    neighbors: tuple
    domain: tuple
    contract: tuple
    nominal: tuple
    A: tuple
    b: tuple
    agent_count: int
    wall_count: int
    object_count: int
    source_json: str

    @property
    def offer_id(self):
        return _digest(asdict(self))


@dataclass(frozen=True)
class PairProposal:
    requester: str
    offers: tuple[SafetyOffer, SafetyOffer]
    velocities: tuple
    raw_velocities: tuple
    solver_status: str

    @property
    def members(self):
        return tuple(o.agent_id for o in self.offers)

    @property
    def proposal_id(self):
        return _digest(asdict(self))


@dataclass(frozen=True)
class PairAck:
    agent_id: str
    peer_id: str
    frame: int
    timestamp: float
    offer_id: str
    proposal_id: str


def build_offer(agent_id, position, neighbors, problem, frame, timestamp, dt,
                domain, support, *, acquiring):
    """Snapshot only own prepared data and the actual current one-hop inbox."""
    if not acquiring or support.get('stage') != 'stationary_acquisition':
        return None, 'outside_stationary_acquisition'
    if not support.get('applied') or problem.object_count == 0:
        return None, 'finite_support_not_applied'
    if support.get('cap_active', 0) != 0 or support.get('motion_bound') != 0.:
        return None, 'capped_or_moving_support'
    params = problem.parameters
    if params.backend != 'qp' or params.clamp_margin_to_keep_zero or problem.clamped:
        return None, 'unsupported_solver_or_margin_clamp'
    sources, stamps = support.get('source_agents', []), support.get('source_timestamps', [])
    if (agent_id not in sources or len(sources) != len(stamps)
            or not set(sources) <= {agent_id, *neighbors}
            or any(not np.isfinite(t) or abs(t-timestamp) > 1e-9 for t in stamps)):
        return None, 'missing_or_stale_local_support'
    if dt != params.dt or not np.array_equal(problem.b, problem.b_original):
        return None, 'changed_step_or_rows'
    offer = SafetyOffer(str(agent_id), int(frame), float(timestamp), float(dt),
        tuple(map(float, position)),
        tuple((str(aid), tuple(map(float, p))) for aid, p in neighbors.items()),
        tuple(map(float, domain)), _contract(params), tuple(map(float, problem.u_nom)),
        tuple(tuple(map(float, row)) for row in problem.A), tuple(map(float, problem.b)),
        problem.agent_count, problem.wall_count, problem.object_count,
        json.dumps(support, sort_keys=True, allow_nan=False))
    if not _valid_offer(offer):
        return None, 'invalid_offer'
    return offer, 'current_local_offer'


def _valid_offer(offer):
    p = dict(offer.contract)
    A, b = np.asarray(offer.A), np.asarray(offer.b)
    neighbors = dict(offer.neighbors)
    if (offer.frame < 0 or not np.isfinite(offer.timestamp) or offer.dt <= 0.
            or offer.dt != p['dt'] or offer.dt*p['gamma_agent'] > 1.+1e-9
            or len(neighbors) != len(offer.neighbors) or offer.agent_id in neighbors
            or offer.agent_count != len(neighbors) or offer.object_count <= 0
            or len(b) != offer.agent_count+offer.wall_count+offer.object_count
            or A.shape != (len(b), 2) or not np.isfinite(A).all() or not np.isfinite(b).all()
            or not np.isfinite(offer.position).all() or not np.isfinite(offer.nominal).all()
            or not np.isfinite(offer.domain).all() or len(offer.position) != 2
            or len(offer.nominal) != 2 or len(offer.domain) != 4
            or np.linalg.norm(offer.nominal) > p['max_speed']+TOLERANCE):
        return False
    xmin, xmax, ymin, ymax = offer.domain
    if not (xmin < xmax and ymin < ymax
            and xmin <= offer.position[0] <= xmax and ymin <= offer.position[1] <= ymax):
        return False
    for k, (_, peer) in enumerate(offer.neighbors):
        delta = np.asarray(offer.position)-peer
        if (not np.isfinite(peer).all() or len(peer) != 2
                or not np.allclose(A[k], 2*delta, rtol=0., atol=1e-12)
                or abs(b[k]+.5*p['gamma_agent']*(delta@delta-p['d_min']**2)) > 1e-12):
            return False
    return True


def _pair_reason(a, b):
    if not _valid_offer(a) or not _valid_offer(b):
        return 'invalid_offer'
    if (a.agent_id == b.agent_id or a.frame != b.frame
            or abs(a.timestamp-b.timestamp) > 1e-9 or a.dt != b.dt
            or a.contract != b.contract or a.domain != b.domain):
        return 'mismatched_identity_or_contract'
    na, nb = dict(a.neighbors), dict(b.neighbors)
    if b.agent_id not in na or a.agent_id not in nb:
        return 'no_mutual_current_link'
    if not (np.array_equal(na[b.agent_id], b.position) and np.array_equal(nb[a.agent_id], a.position)):
        return 'mismatched_peer_position'
    delta = np.asarray(a.position)-b.position
    if delta@delta < dict(a.contract)['d_min']**2:
        return 'already_unsafe_separation'
    return ''


def _system(a, b):
    ia = [aid for aid, _ in a.neighbors].index(b.agent_id)
    ib = [aid for aid, _ in b.neighbors].index(a.agent_id)
    A, B = np.asarray(a.A), np.asarray(b.A)
    ba, bb = np.asarray(a.b), np.asarray(b.b)
    keep_a, keep_b = np.arange(len(ba)) != ia, np.arange(len(bb)) != ib
    combined = np.vstack([np.c_[A[keep_a], np.zeros((keep_a.sum(), 2))],
        np.c_[np.zeros((keep_b.sum(), 2)), B[keep_b]], np.r_[A[ia], B[ib]]])
    rhs = np.r_[ba[keep_a], bb[keep_b], ba[ia]+bb[ib]]
    return combined, rhs, ia, ib


def _certificate(a, b, velocities):
    if _pair_reason(a, b):
        return dict(valid=False)
    values = np.asarray(velocities, dtype=float)
    if values.shape != (2, 2) or not np.isfinite(values).all():
        return dict(valid=False)
    A, rhs, ia, ib = _system(a, b)
    residuals = A@values.reshape(4)-rhs
    speeds = np.linalg.norm(values, axis=1)
    params = dict(a.contract)
    relative, rate = np.asarray(a.position)-b.position, values[0]-values[1]
    tau = float(np.clip(-(relative@rate)/(rate@rate), 0., a.dt)) if rate@rate > 0. else 0.
    separation = float(np.linalg.norm(relative+tau*rate))
    # The actually applied value, including domain clipping, must be this value.
    applied = np.array([(clip_to_domain(np.asarray(o.position)+o.dt*u, o.domain)
        -o.position)/o.dt for o, u in zip((a, b), values)])
    valid = (residuals.min() >= -TOLERANCE and speeds.max() <= params['max_speed']+TOLERANCE
        and separation >= params['d_min']-TOLERANCE
        and np.allclose(applied, values, rtol=0., atol=1e-12))
    return dict(valid=bool(valid), row_residuals=residuals.tolist(),
        minimum_residual=float(residuals.min()), speeds=speeds.tolist(),
        minimum_hold_separation=separation, minimum_hold_time=tau,
        mutual_indices=[ia, ib], relative_residual=float(residuals[-1]))


def propose(own, peer):
    reason = _pair_reason(own, peer)
    if reason:
        return None, reason
    a, b = sorted((own, peer), key=lambda o: o.agent_id)
    A, rhs, _, _ = _system(a, b)
    nominal = np.r_[a.nominal, b.nominal]
    speed = dict(a.contract)['max_speed']

    def speed_slacks(u):
        return np.array([speed**2-u[:2]@u[:2], speed**2-u[2:]@u[2:]])

    def speed_jacobian(u):
        return np.array([[-2*u[0], -2*u[1], 0., 0.], [0., 0., -2*u[2], -2*u[3]]])

    try:
        result = minimize(lambda u: float(np.sum((u-nominal)**2)), np.zeros(4),
            jac=lambda u: 2*(u-nominal), method='SLSQP',
            constraints=[LinearConstraint(A, rhs, np.inf),
                dict(type='ineq', fun=speed_slacks, jac=speed_jacobian)],
            options=dict(ftol=1e-12, maxiter=1000))
    except (ValueError, RuntimeError, FloatingPointError, np.linalg.LinAlgError) as exc:
        return None, 'numerical_failure:'+type(exc).__name__
    if not result.success or not np.isfinite(result.x).all():
        return None, 'no_numerically_verified_solution:'+str(result.message)
    raw = result.x.reshape(2, 2)
    values = np.array([(clip_to_domain(np.asarray(o.position)+o.dt*u, o.domain)
        -o.position)/o.dt for o, u in zip((a, b), raw)])
    if not _certificate(a, b, values)['valid']:
        return None, 'applied_command_failed_constraints'
    return PairProposal(own.agent_id, (a, b), tuple(map(tuple, values.tolist())),
        tuple(map(tuple, raw.tolist())), str(result.message)), 'verified_pair_proposal'


def _current_proposal(proposal, own, inbox):
    if own.agent_id not in proposal.members or len(set(proposal.members)) != 2:
        return False
    other = next(aid for aid in proposal.members if aid != own.agent_id)
    peer = inbox.get(other)
    if peer is None or _pair_reason(own, peer):
        return False
    current = {own.agent_id: own, peer.agent_id: peer}
    if (proposal.requester not in proposal.members or proposal.members != tuple(sorted(proposal.members))
            or any(o.offer_id != current[o.agent_id].offer_id for o in proposal.offers)):
        return False
    return _certificate(*proposal.offers, proposal.velocities)['valid']


def choose(own, inbox, proposals):
    """One endpoint chooses one incident proposal using its local inbox only."""
    viable = [p for p in proposals if _current_proposal(p, own, inbox)]
    if not viable:
        return None
    selected = min(viable, key=lambda p: (*p.members, p.requester))
    peer = next(aid for aid in selected.members if aid != own.agent_id)
    return PairAck(own.agent_id, peer, own.frame, own.timestamp, own.offer_id, selected.proposal_id)


def commit(proposal, own, peer, own_ack, peer_ack):
    if not _current_proposal(proposal, own, {peer.agent_id: peer}):
        return False
    for offer, other, ack in ((own, peer, own_ack), (peer, own, peer_ack)):
        if (ack is None or ack.agent_id != offer.agent_id or ack.peer_id != other.agent_id
                or ack.frame != offer.frame or ack.timestamp != offer.timestamp
                or ack.offer_id != offer.offer_id or ack.proposal_id != proposal.proposal_id):
            return False
    return True


@dataclass(frozen=True)
class PairCommand:
    proposal: PairProposal
    agent_id: str
    acknowledgements: tuple[PairAck, PairAck]

    def for_problem(self, problem):
        index = self.proposal.members.index(self.agent_id)
        own, peer = self.proposal.offers[index], self.proposal.offers[1-index]
        acks = {a.agent_id: a for a in self.acknowledgements}
        if (not commit(self.proposal, own, peer, acks.get(own.agent_id), acks.get(peer.agent_id))
                or own.contract != _contract(problem.parameters)
                or not np.array_equal(problem.A, own.A) or not np.array_equal(problem.b, own.b)
                or not np.array_equal(problem.b_original, own.b)
                or not np.array_equal(problem.u_nom, own.nominal)):
            raise ValueError('Pair command does not certify this local problem')
        _, _, mutual, other = _system(own, peer)
        velocity = np.asarray(self.proposal.velocities[index])
        peer_velocity = np.asarray(self.proposal.velocities[1-index])
        rhs = problem.b.copy()
        rhs[mutual] = own.b[mutual]+peer.b[other]-np.asarray(peer.A[other])@peer_velocity
        return velocity.copy(), rhs

    def audit_applied(self, position, velocity, dt):
        index = self.proposal.members.index(self.agent_id)
        own = self.proposal.offers[index]
        if (dt != own.dt or not np.array_equal(position, own.position)
                or not np.array_equal(velocity, self.proposal.velocities[index])
                or not _certificate(*self.proposal.offers, self.proposal.velocities)['valid']):
            raise ValueError('Pair command position, dt or applied velocity changed before execution')

    def record(self):
        return dict(proposal_id=self.proposal.proposal_id, requester=self.proposal.requester,
            members=list(self.proposal.members), offers=[asdict(o) for o in self.proposal.offers],
            raw_commands=self.proposal.raw_velocities, applied_commands=self.proposal.velocities,
            solver_status=self.proposal.solver_status,
            acknowledgements=[asdict(a) for a in self.acknowledgements],
            certificate=_certificate(*self.proposal.offers, self.proposal.velocities),
            scope='numerically_checked_current_observation_rows_not_unknown_boundary_proof')


def exchange(offers):
    """Route one-hop rounds; neither proposals nor choices see a global problem."""
    proposals, traces, inboxes = {}, {}, {}
    for aid, own in offers.items():
        inboxes[aid] = {peer: offers[peer] for peer, _ in own.neighbors if peer in offers}
        valid = aid == own.agent_id and _valid_offer(own)
        independent = (solve_min_norm_2d(np.asarray(own.nominal), np.asarray(own.A),
            np.asarray(own.b), dict(own.contract)['max_speed']) if valid else None)
        trace = dict(independent_feasible=bool(independent.feasible) if independent else None,
            independent_proposal_unexecuted=True, accepted=False, attempts=[],
            frame=own.frame, timestamp=own.timestamp, offer_id=own.offer_id)
        traces[aid] = trace
        if independent is None or independent.feasible:
            continue
        for peer_id, peer in sorted(inboxes[aid].items()):
            proposal, reason = propose(own, peer)
            trace['attempts'].append(dict(peer=peer_id, reason=reason))
            if proposal is not None:
                proposals[aid] = proposal
                break
    acknowledgements = {}
    for aid, own in offers.items():
        incoming = [proposals[p] for p in inboxes[aid] if p in proposals]
        if aid in proposals:
            incoming.append(proposals[aid])
        acknowledgements[aid] = choose(own, inboxes[aid], incoming)
    commands = {}
    for proposal in proposals.values():
        a, b = proposal.offers
        aa, bb = acknowledgements[a.agent_id], acknowledgements[b.agent_id]
        if not commit(proposal, offers[a.agent_id], offers[b.agent_id], aa, bb):
            continue
        for aid in proposal.members:
            if aid in commands:
                raise RuntimeError('Local acknowledgements produced conflicting commitments')
            commands[aid] = PairCommand(proposal, aid, (aa, bb))
            traces[aid].update(accepted=True, execution=commands[aid].record())
    return commands, traces
