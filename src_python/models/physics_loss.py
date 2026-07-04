from pinn_network import ResidualPINN
import jax.numpy as jnp
import jax
from kinematics import dynamics_residual

model = ResidualPINN()


def compute_total_loss(params, inputs, targets) -> jax.Array:

    predictions = model.apply(params, inputs)

    ### Level 2 NN: Physics Aware ###
    # # penalize jagged solutions by calculating the partial of tau wrt the joint space
    # # this makes sure the network doesn't cause jerk and jitter in the final compensation torque
    # # helper functions
    # def apply_single_row(x_single):
    #     return model.apply(params, x_single)
    #
    # batch_jacobian_fn = jax.vmap(jax.jacrev(apply_single_row))
    #
    # batch_jac = batch_jacobian_fn(inputs)
    #
    # # grab only qs partial from the 3x10 jacobian
    # # dimension 1 is the "stacking" dimension, the pile
    # # 2 and 3 are the "normal" jacobian 4x10
    # tau_wrt_q = batch_jac[:, :, 0:3]
    #
    # loss_physics = jnp.mean(jnp.square(tau_wrt_q))

    ### Level 3 NN: True PINN ###
    tau_predicted = predictions[:, 0:3]
    q = inputs[:, 0:3]  # the current joint state
    qdot = inputs[:, 3:6]  # the current joint velocity
    u = inputs[:, 6:9]  # commanded acceleration
    true_mass = targets[:, 3]  # the oracle true mass

    # make the batch residual fucntion
    batch_residual_fn = jax.vmap(dynamics_residual)

    residuals = batch_residual_fn(q, qdot, u, true_mass, tau_predicted)

    # get total loss

    loss_data = jnp.mean(jnp.square(targets - predictions))
    loss_physics = jnp.mean(jnp.square(residuals))
    physics_weight = 0.1

    loss = loss_data + loss_physics * physics_weight

    return loss


if __name__ == "__main__":
    key = jax.random.PRNGKey(0)
    dummy_inputs = jax.random.normal(key, (32, 10))  # A fake batch of 32 rows of data
    dummy_targets = jnp.ones((32, 4))  # A fake batch of target torques and true mass
    params = model.init(key, dummy_inputs)

    inital_loss = compute_total_loss(params, dummy_inputs, dummy_targets).item()

    loss_grad_fn = jax.value_and_grad(compute_total_loss)

    loss_value, gradients = loss_grad_fn(params, dummy_inputs, dummy_targets)

    raw_float_loss = loss_value.item()

    print(f"JAX Array Loss: {loss_value} (Type: {type(loss_value)})")
    print(f"Raw Python Float Loss: {raw_float_loss} (Type: {type(raw_float_loss)})")

    print("\n--- Gradient Check ---")
    print(f"Gradients successfully calculated!")
    print(
        f"Shape of Layer 1 Weight Gradients: {gradients['params']['Dense_0']['kernel'].shape}"
    )
