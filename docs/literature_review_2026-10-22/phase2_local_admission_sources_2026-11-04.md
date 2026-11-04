# Local admission design: bounded source check

Checked 2026-11-04 Asia/Tokyo / 2026-11-03 UTC. Exploratory design after earlier
negative results; not a systematic review or proof of novelty. No local research
data or unpublished source was uploaded in the searches.

Queries used:

1. `site.proceedings.mlr.press measurement robust control barrier functions Dean Taylor Cosner Recht Ames 2021`
2. `site:arxiv.org set membership LiDAR unknown obstacles safety control barrier functions visibility occlusions`
3. `site:arxiv.org sampled data control barrier functions measurement uncertainty Breeden Garg Panagou 2022`
4. `"Breeden" "Garg" "Sampled" "Barrier"`
5. `"lidar" "set-membership" safety unknown obstacles`
6. `"control barrier" "occlusions" "unknown" lidar`

The first query's `site.` text was not a valid site filter. The cited results
were nevertheless checked at the primary sources below. Search results on
LiDAR odometry, mapping and occlusion-aware navigation were not used as evidence
that the proposed DBACT occupancy construction works.

| Primary source opened | Verified use | Limit |
| --- | --- | --- |
| [Dean et al., CoRL 2020 / PMLR 155:654–670, 2021](https://proceedings.mlr.press/v155/dean21a.html) | Measurement-robust control barrier functions are established prior work | Does not prove the DBACT polygon/coverage/contact design |
| [Breeden, Garg & Panagou, Control Barrier Functions in Sampled-Data Systems](https://arxiv.org/abs/2103.03677) | Piecewise-constant control and fixed sampling require explicit forward-invariance conditions; metadata gives IEEE Control Systems Letters 6:367–372, 2022, DOI 10.1109/LCSYS.2021.3076127 | No theorem was imported as a proof of the existing split penalty engine |
| [Kassis et al., Scientific Agent Skills](https://arxiv.org/abs/2609.00065) | Workflow attribution only; authors Kassis, Agarwal, He, Patel, Brueckner; 2026; current record v2 dated 2026-09-02 | No DBACT scientific support; cite versionless DOI, not an outcome claim |

The report's single-step statement is proved directly by set inclusion and
distance monotonicity under its declared premises. The edge/turn formula uses
the previously reviewed BV bound. Neither is claimed as original general
control or reconstruction theory. The independent review added free-ray
partitioning as a supporting module after the first comparison round; its owning
source is the repository's `src/dbact/sensing/free_space.py`, whose centroid
enclosure output is not an occupancy certificate interface.

Evidence status: established conceptual background located; full construction,
timing, task benefit and novelty of a future implementation remain unverified.
