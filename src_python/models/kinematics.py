import jax.numpy as jnp

LINK_1_MASS = 10.0
LINK_2_MASS = 4.0
LINK_2_INERTIAL_MASS = 0.8
LINK_2_LENGTH = 0.8
END_MASS = 0.5  # base end mass without object


def get_M(q: jnp.ndarray, m_true: float) -> jnp.ndarray:
    m_end = m_true + END_MASS
    c2 = jnp.cos(q[2])
    s2 = jnp.sin(q[2])
    d = q[1]

    ma = m_end + LINK_1_MASS / 4.0 + LINK_2_MASS
    mb = m_end + LINK_2_MASS / 4.0
    mc = 2.0 * m_end + LINK_2_MASS

    I1 = 0.2 * LINK_1_MASS * d * d

    M = jnp.zeros((3, 3))
    M = M.at[0, 0].set(
        I1
        + LINK_2_INERTIAL_MASS
        + ma * d * d
        + mb * LINK_2_LENGTH * LINK_2_LENGTH
        + mc * LINK_2_LENGTH * d * c2
    )
    M = M.at[0, 1].set(-0.5 * mc * LINK_2_LENGTH * s2)
    M = M.at[0, 2].set(
        LINK_2_INERTIAL_MASS
        + mb * LINK_2_LENGTH * LINK_2_LENGTH
        + 0.5 * mc * LINK_2_LENGTH * d * c2
    )
    M = M.at[1, 0].set(M[0, 1])
    M = M.at[1, 1].set(ma)
    M = M.at[1, 2].set(-0.5 * mc * LINK_2_LENGTH * s2)
    M = M.at[2, 0].set(M[0, 2])
    M = M.at[2, 1].set(M[1, 2])
    M = M.at[2, 2].set(LINK_2_INERTIAL_MASS + mb * LINK_2_LENGTH * LINK_2_LENGTH)

    return M


def get_C(q: jnp.ndarray, qdot: jnp.ndarray, m_true: float) -> jnp.ndarray:
    m_end = m_true + END_MASS
    c2 = jnp.cos(q[2])
    s2 = jnp.sin(q[2])
    d = q[1]

    ma = m_end + LINK_1_MASS / 4.0 + LINK_2_MASS
    mc = 2.0 * m_end + LINK_2_MASS

    qd0 = qdot[0]
    qd1 = qdot[1]
    qd2 = qdot[2]

    C = jnp.zeros(3)
    C = C.at[0].set(
        (2.0 * ma * d + mc * LINK_2_LENGTH * c2) * qd0 * qd1
        - mc * LINK_2_LENGTH * d * s2 * qd0 * qd2
        - 0.5 * mc * LINK_2_LENGTH * d * s2 * qd2 * qd2
    )
    C = C.at[1].set(
        -(ma * d + 0.5 * mc * LINK_2_LENGTH * c2) * qd0 * qd0
        - 0.5 * mc * LINK_2_LENGTH * c2 * qd2 * qd2
        - mc * LINK_2_LENGTH * c2 * qd0 * qd2
    )
    C = C.at[2].set(
        0.5 * mc * LINK_2_LENGTH * d * s2 * qd0 * qd0
        + mc * LINK_2_LENGTH * c2 * qd0 * qd1
    )

    return C


def get_G(q: jnp.ndarray, m_true: float) -> jnp.ndarray:
    return jnp.zeros(3)


def dynamics_residual(q, qdot, u, true_mass, tau_pred):
    M = get_M(q, true_mass)
    C = get_C(q, qdot, true_mass)
    G = get_G(q, true_mass)
    return M @ u + C + G - tau_pred
