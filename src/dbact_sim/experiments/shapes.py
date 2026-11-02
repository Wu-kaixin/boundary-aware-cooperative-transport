"""Additional deterministic geometry; legacy experiment configurations stay intact.

The two smooth outlines reproduce ``local_outline`` in the historical
``scripts/run_arbitrary_shape_monte_carlo.py`` (before its scale/yaw/placement).
Polygon discretization is part of the experiment identity, not an exact circle.
"""
import numpy as np

CARGO_FAMILIES = ('skew_box', 'chamfered_box', 't_load', 'u_tray', 'offset_load', 'double_lobe')
SHAPES = ("l_shape", "rectangle", "c_shape", "circle", "ellipse24") + CARGO_FAMILIES


def cargo_outline(family: str, setting: int = 0) -> np.ndarray:
    """Two predeclared shapes per family, each one connected rigid polygon.

    These are designed development geometries, not random draws from an
    arbitrary-shape population. Scaling/centering is the protocol's job.
    """
    if type(setting) is not int or setting not in (0, 1):
        raise ValueError('Cargo geometry setting must be 0 or 1')
    s = setting
    if family == 'skew_box':
        vertices = [(-1.2,-.65),(1.1,-.65),(.65-.25*s,.7),(-.85+.3*s,.5+.2*s)]
    elif family == 'chamfered_box':
        a,b,c = 1.3+.25*s,.48,.18+.06*s
        vertices = [(-a+c,-b),(a-c,-b),(a,-b+c),(a,b-c),
                    (a-c,b),(-a+c,b),(-a,b-c),(-a,-b+c)]
    elif family == 't_load':
        half = .35-.08*s
        vertices = [(-1.,.6),(-1.,.1),(-half,.1),(-half,-.9-.15*s),
                    (half,-.9-.15*s),(half,.1),(1.,.1),(1.,.6)]
    elif family == 'u_tray':
        half,floor = .45+.10*s,-.4-.15*s
        vertices = [(-1.,-1.),(1.,-1.),(1.,1.),(half,1.),(half,floor),
                    (-half,floor),(-half,1.),(-1.,1.)]
    elif family == 'offset_load':
        shift=.15*s
        vertices = [(-1.4,-.6),(-.3,-.6),(-.3,-.1),(.3,-.1),(.3,-.2+shift),
                    (1.4,-.2+shift),(1.4,.9+shift),(.3,.9+shift),(.3,.2),
                    (-.3,.2),(-.3,.6),(-1.4,.6)]
    elif family == 'double_lobe':
        theta=np.linspace(0,2*np.pi,48,endpoint=False)
        radius=1.+(.28+.10*s)*np.cos(2*theta)+.12*np.cos(theta)+.08*np.sin(3*theta)
        return np.c_[radius*np.cos(theta),radius*np.sin(theta)]
    else:
        raise ValueError(f'Unsupported cargo family: {family}')
    return np.asarray(vertices,dtype=float)


def smooth_outline(shape: str) -> np.ndarray:
    if shape == "circle":
        count, a, b = 64, 0.58, 0.58
    elif shape == "ellipse24":
        count, a, b = 24, 0.72, 0.42
    else:
        raise ValueError(f"Not a supported smooth outline: {shape}")
    theta = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    return np.column_stack((a * np.cos(theta), b * np.sin(theta)))
