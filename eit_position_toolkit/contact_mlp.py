"""Small NumPy MLP for normalized EIT signatures and X/Y regression."""
from __future__ import annotations

import numpy as np


class ContactMLP:
    hidden_layers = (128, 64)
    max_epochs = 1000
    learning_rate = .003
    regularization = 1e-5
    seed = 21

    def __init__(self, features, radius_mm, parameters=None):
        self.input_scale = float(np.sqrt(features))
        self.output_scale = float(radius_mm)
        if not np.isfinite(self.output_scale) or self.output_scale <= 0:
            raise ValueError("Invalid MLP position scale")
        self.layers = (features, *self.hidden_layers, 2)
        self.parameters = []
        if parameters is not None:
            for i, (incoming, outgoing) in enumerate(zip(self.layers[:-1], self.layers[1:])):
                for name, shape in ((f"mlp_weight_{i}", (incoming, outgoing)),
                                    (f"mlp_bias_{i}", (outgoing,))):
                    if name not in parameters:
                        raise ValueError(f"Saved MLP is missing {name}")
                    value = np.asarray(parameters[name], dtype=float)
                    if value.shape != shape or not np.isfinite(value).all():
                        raise ValueError(f"Invalid saved MLP parameter {name}")
                    self.parameters.append(value.copy())

    def forward(self, x):
        activations = [np.atleast_2d(x) * self.input_scale]
        for i in range(0, len(self.parameters), 2):
            z = activations[-1] @ self.parameters[i] + self.parameters[i+1]
            activations.append(np.tanh(z) if i < len(self.parameters)-2 else z)
        return activations

    def loss_gradient(self, x, y):
        """MSE on radius-scaled coordinates, plus L2 weight regularization."""
        activations = self.forward(x)
        residual = activations[-1] - np.asarray(y) / self.output_scale
        loss = float(np.mean(residual**2))
        gradient = 2 * residual / residual.size
        derivatives = [None] * len(self.parameters)
        for i in range(len(self.parameters)-2, -1, -2):
            weight = self.parameters[i]
            loss += .5 * self.regularization * float(np.sum(weight**2))
            derivatives[i] = activations[i//2].T @ gradient + self.regularization * weight
            derivatives[i+1] = gradient.sum(axis=0)
            if i:
                gradient = (gradient @ weight.T) * (1 - activations[i//2]**2)
        return loss, derivatives

    def fit(self, x, xy):
        rng = np.random.default_rng(self.seed)
        self.parameters = []
        for incoming, outgoing in zip(self.layers[:-1], self.layers[1:]):
            bound = np.sqrt(6 / (incoming + outgoing))
            self.parameters.extend([rng.uniform(-bound, bound, (incoming, outgoing)), np.zeros(outgoing)])
        first = [np.zeros_like(p) for p in self.parameters]
        second = [np.zeros_like(p) for p in self.parameters]
        x, xy = np.asarray(x), np.asarray(xy)
        step = 0
        for epoch in range(self.max_epochs):
            order = rng.permutation(len(x))
            for start in range(0, len(x), 64):
                batch = order[start:start+64]
                loss, derivatives = self.loss_gradient(x[batch], xy[batch])
                if not np.isfinite(loss) or not all(np.isfinite(g).all() for g in derivatives):
                    raise ValueError("MLP training diverged; check the captured signals")
                step += 1
                for i, derivative in enumerate(derivatives):
                    first[i] = .9 * first[i] + .1 * derivative
                    second[i] = .999 * second[i] + .001 * derivative**2
                    self.parameters[i] -= self.learning_rate * (first[i] / (1-.9**step)) / (
                        np.sqrt(second[i] / (1-.999**step)) + 1e-8)
            mse = float(np.mean((self.predict(x) - xy)**2))
            if not np.isfinite(mse):
                raise ValueError("MLP training produced non-finite predictions")
            if mse < .01:
                break
        return {"epochs_completed": epoch+1, "training_rmse_mm": float(np.sqrt(mse)),
                "max_epochs": self.max_epochs, "learning_rate": self.learning_rate,
                "regularization": self.regularization, "seed": self.seed,
                "layers": list(self.layers), "activation": "tanh", "output_activation": "linear",
                "optimizer": "Adam", "batch_size": 64}

    def predict(self, x):
        return self.forward(x)[-1] * self.output_scale

    def arrays(self):
        return {f"mlp_{kind}_{i}": self.parameters[2*i+j]
                for i in range(len(self.layers)-1) for j, kind in enumerate(("weight", "bias"))}
