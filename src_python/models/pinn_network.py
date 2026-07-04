import os

os.environ["JAX_PLATFORMS"] = "cpu"

import jax as jax
import jax.numpy as jnp
import flax.linen as nn


class ResidualPINN(nn.Module):
    @nn.compact
    def __call__(self, x):
        x = nn.Dense(features=128)(x)
        # allows for second derivatives to be taken by "smoothing" the function
        x = nn.tanh(x)

        x = nn.Dense(features=64)(x)
        x = nn.tanh(x)

        x = nn.Dense(features=32)(x)
        x = nn.tanh(x)

        x = nn.Dense(features=4)(x)
        return x


if __name__ == "__main__":
    model = ResidualPINN()

    # jax requires a pseudo-random number generator key
    key = jax.random.PRNGKey(0)

    dummy_input = jnp.zeros((1, 10))

    # initalize weights from untrained network
    variables = model.init(key, dummy_input)

    print("\n--- Model Initialized Successfully ---")

    # nested dictionary of generated weights:
    print(jax.tree_util.tree_map(lambda x: x.shape, variables))

    dummy_prediction = model.apply(variables, dummy_input)

    print(f"Input Shape:  {dummy_input.shape}")
    print(f"Output Shape: {dummy_prediction.shape}")
