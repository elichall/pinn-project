import jax
import jax.numpy as jnp
import equinox as eqx
import optax

# 1. Hardware Verification
# This should print your NVIDIA GPU if the cuda12 plugin resolved correctly
print(f"Hardware: {jax.devices()}")


# 2. Equinox Architecture Definition
class TrivialNetwork(eqx.Module):
    linear: eqx.nn.Linear

    def __init__(self, key):
        # 2-input, 2-output linear layer
        self.linear = eqx.nn.Linear(2, 2, key=key)

    def __call__(self, x):
        return jax.nn.relu(self.linear(x))


# Initialize model and optimizer
key = jax.random.PRNGKey(0)
model = TrivialNetwork(key)
optimizer = optax.adam(learning_rate=0.01)

# Filter parameters for optimization (ignoring static boolean/integer states)
opt_state = optimizer.init(eqx.filter(model, eqx.is_inexact_array))

# 3. Define Dummy Data
x_input = jnp.array([1.0, -1.0])
y_target = jnp.array([0.5, 0.5])


# 4. Define Loss Function with Autograd
@eqx.filter_value_and_grad
def loss_fn(model, x, y):
    pred = model(x)
    return jnp.mean((pred - y) ** 2)


# 5. Execute Forward Pass, Backpropagation, and State Update
loss_value, grads = loss_fn(model, x_input, y_target)
updates, opt_state = optimizer.update(grads, opt_state, model)
model = eqx.apply_updates(model, updates)

print(f"Initial Loss computed: {loss_value:.4f}")
print("Stack validation successful. Ready to build the PINN.")
