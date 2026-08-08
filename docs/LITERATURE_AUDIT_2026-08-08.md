# DBACT scoped literature audit (2026-08-08)

## Audit question

Does prior work already contain the complete chain

`locally visible boundary measurements -> boundary-measure-induced density -> limited local CVT -> hard distributed CBF -> contact-driven transport of an unknown concave object`?

This is a scoped audit, not a claim that every paper in the field has been exhausted. Searches covered the intersections of coverage control, environmental/perimeter boundary monitoring, unknown-object characterization, caging/collective transport, and distributed/robust CBFs. The cut-off date is 2026-08-08.

## Closest literature by module

| Line | Closest primary source | What it establishes | Remaining gap relative to DBACT |
| --- | --- | --- | --- |
| CVT coverage | Cortes et al., [Coverage Control for Mobile Sensing Networks](https://arxiv.org/abs/math/0212212), 2004 | Locational cost, Voronoi partition, distributed Lloyd descent | Assumes a common task density; does not construct density from an unknown object boundary |
| Limited interactions | Cortes, Martinez, Bullo, [Spatially-distributed coverage optimization and control with limited-range interactions](https://arxiv.org/abs/math/0401297), 2005 | Coverage laws under limited sensing/communication | Does not address different local densities created by partial object observations |
| Adaptive density | Schwager, Rus, Slotine, [Decentralized, Adaptive Coverage Control for Networked Robots](https://journals.sagepub.com/doi/10.1177/0278364908100177), 2009 | Learns an unknown sensory distribution and shares estimates by consensus | The field is an environmental sensory function, not a boundary measure for enclosure |
| Boundary monitoring | Susca, Bullo, Martinez, [Monitoring Environmental Boundaries With a Robotic Sensor Network](https://ieeexplore.ieee.org/document/4431884), 2008 | Local-sensing agents approximate and distribute along an environmental boundary | Tracks a level-set boundary; no boundary-offset density, transport, or hard CBF safety filter |
| Online boundary estimation | Newaz, Jeong, Chong, [Online Boundary Estimation in Partially Observable Environments Using a UAV](https://doi.org/10.1007/s10846-017-0664-9), 2018 | Online partial boundary estimation | Single-UAV estimation rather than decentralized allocation and manipulation |
| Unknown-object characterization | Habibi et al., [Distributed Centroid Estimation and Motion Controllers for Collective Transport](https://zkingston.com/papers/habibi2015icra.pdf), 2015 | Distributed shape/centroid-related estimation and transport motion controllers | Relies on global aggregate quantities rather than local boundary-induced density |
| Caging transport | Pereira, Campos, Kumar, [Decentralized Algorithms for Multi-Robot Manipulation via Caging](https://repository.upenn.edu/bitstreams/89ebd9ab-d4b9-4f8b-8aca-08bd6e5e1e96/download), 2004 | Decentralized object transport using object closure/caging | Uses a stronger geometric closure framing and does not use local CVT density allocation |
| Communication-free manipulation | Wang, Schwager, [Multi-Robot Manipulation without Communication](https://sites.bu.edu/msl/files/2014/08/WangSchwagerDARS14Manipulation.pdf), 2014 | Contact-driven cooperative pushing without explicit communication | Does not solve unknown-shape boundary allocation or safety via distributed CBFs |
| Direct predecessor | [Cooperative Transportation Without Prior Object Knowledge via Adaptive Self-Allocation and Coordination](https://arxiv.org/abs/2602.19070), 2026 | Detection-agent Gaussian peaks, CVT allocation, pairwise CBF | Density peaks are located at detecting agents; full boundary geometry is not converted into a measure-induced field |
| Sensor-based safe coverage | [Sensor-based Multi-agent Coverage Control with Spatial Separation in Unstructured Environments](https://arxiv.org/abs/2403.01710), 2024/2025 version | Local point clouds, Voronoi-based coverage, obstacle-aware safe navigation | Covers a workspace/interest field rather than forming a transporting boundary enclosure |
| CBF foundation | Ames et al., [Control Barrier Functions: Theory and Applications](https://coogan.ece.gatech.edu/papers/pdf/amesecc19.pdf), 2019 | Forward invariance and optimization-based safety filters | Does not allocate pairwise responsibility or account for estimated object boundaries |
| Robust CBF | Kolathaya, Ames, [Input-to-State Safety With Control Barrier Functions](https://arxiv.org/abs/1803.03035), 2018 | Safety under bounded disturbances via ISSf-CBFs | Supplies the correct tool for a moving/estimated boundary, but not the DBACT construction |
| Responsibility allocation | Lyu, Luo, Dolan, [Responsibility-associated Multi-agent Collision Avoidance](https://arxiv.org/abs/2206.09030), 2022 | Pairwise responsibility weights with provable decentralized safety | Collision avoidance only; no unknown-object boundary density or contact transport |

## Novelty judgment

No direct prior was located that implements the entire audited chain. The defensible contribution is therefore the *specific integration and mathematical coupling*, not any individual ingredient:

1. A ray-visible local point cloud is converted into boundary targets `xi_k = b_hat_k + d_c n_hat_k` without exposing the complete polygon to the controller.
2. The discrete density weight approximates a boundary measure: `Delta s_k * confidence_k * age_k * (1 + kappa * gap_k)`. Voxel fusion prevents communication relay from multiplying mass.
3. The density is integrated over a strict limited local Voronoi cell with fixed physical grid spacing.
4. A slack-free responsibility-splitting CBF uses an estimation-error margin and an optional moving-boundary disturbance bound.
5. Concave rigid bodies are convex-decomposed and moved only by PyMunk contact impulses; the evaluator never calls scripted translation in paper transport runs.

The novelty claim must remain qualified until a broader database audit (IEEE Xplore, Scopus/Web of Science and Google Scholar citation chaining) is completed by the authors. "First" language should not appear in the paper at the current stage.

## Theory implications

- Frozen-map Lloyd descent is standard and should be a lemma, not a headline proposition.
- Because each robot has a different local map and therefore a different `phi_i`, standard common-density Lloyd convergence does not apply directly.
- The paper-level result should use a slow-time-varying/practical-stability statement: bounded density variation and bounded map disagreement imply a bounded centroid-tracking error. The frozen-map lemma is one step inside that result.
- Estimation error must be inserted into the barrier itself. With boundary-position error `epsilon_b` and normal error `epsilon_n`, use the conservative margin `epsilon = epsilon_b + 2 d_c sin(epsilon_n/2)`.
- The static-boundary hard-QP feasibility proof is constructive because `u=0` satisfies all responsibility-split and object-boundary constraints whenever the initial safe-set conditions hold. For a moving boundary, feasibility additionally depends on the object-speed bound and input box.

## Venue decision

Primary target: **IEEE Control Systems Letters (L-CSS), regular submission**. Its official author instructions impose a strict six-page IEEE two-column limit, including figures and references: [L-CSS author information](https://ieeecss.org/publication/ieee-control-systems-letters/author-information). The 2026 L-CSS-with-CDC deadline was 2026-03-17 and has passed; regular L-CSS submissions remain the relevant route.

Why L-CSS: the contribution is best defended as a compact control result (time-varying local density + hard distributed CBF + concise physics validation). Pure simulation is not itself disqualifying, but the theorem and novelty statement must be strong enough to carry the paper.

Recommended extension/fallback: **Robotics and Autonomous Systems** after a larger multi-seed/random-shape study. RA-L is not the first choice for the current sim-only evidence package, even though its page budget is also letter-sized.

Provisional title:

> **Distributed Boundary-Aware Enclosure and Contact Transport of Unknown-Shaped Objects**

Avoid using "formal caging" unless an immobilizing-cage condition is actually proved.
