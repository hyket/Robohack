"""Convert two upstream TorchScript modules and verify the NumPy export."""

import argparse
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import torch

from T3.policy import HISTORY_LENGTH, NETWORKS, OBSERVATION_COUNT, load_network, run_network


def load_module(path):
    try:
        return torch.jit.load(str(path), map_location="cpu").eval()
    except (OSError, RuntimeError, ValueError) as error:
        raise ValueError(f"Could not load TorchScript checkpoint {path}: {error}") from error


def extract(module, name):
    result = {}
    children = list(module.children())
    expected = ["Linear", "ELU"] * (len(NETWORKS[name]) - 2) + ["Linear"]
    if [child.original_name for child in children] != expected:
        raise ValueError(f"Unexpected {name} layer types")
    for layer, child in enumerate(children[::2]):
        if child.bias is None:
            raise ValueError(f"Missing bias for {name} layer {layer}")
        result[f"{name}_{layer}_weight"] = child.weight.detach().numpy()
        result[f"{name}_{layer}_bias"] = child.bias.detach().numpy()
    return result


def compare(adaptation, body, weights):
    adaptation_numpy = load_network(weights, "adaptation")
    body_numpy = load_network(weights, "body")
    rng = np.random.default_rng(2026)
    size = HISTORY_LENGTH * OBSERVATION_COUNT
    histories = np.vstack((np.zeros(size), rng.normal(scale=0.2, size=(128, size)))).astype(np.float32)
    with torch.no_grad():
        torch_histories = torch.from_numpy(histories)
        torch_latent = adaptation(torch_histories)
        torch_action = body(torch.cat((torch_histories, torch_latent), dim=-1)).numpy()
        torch_latent = torch_latent.numpy()

    if not (np.isfinite(torch_latent).all() and np.isfinite(torch_action).all()):
        raise ValueError("TorchScript produced non-finite outputs during parity check")

    latent = np.stack([run_network(adaptation_numpy, history) for history in histories])
    action = np.stack([
        run_network(body_numpy, np.concatenate((history, estimate)))
        for history, estimate in zip(histories, latent)
    ])
    np.testing.assert_allclose(latent, torch_latent, rtol=3e-5, atol=1e-4)
    np.testing.assert_allclose(action, torch_action, rtol=3e-5, atol=1e-4)
    print(f"Max absolute error across {len(histories)} histories: "
          f"latent={np.max(np.abs(latent - torch_latent)):.2e}, "
          f"action={np.max(np.abs(action - torch_action)):.2e}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("torchscript_dir", type=Path, help="directory containing the two *_latest.jit files")
    parser.add_argument("output", type=Path, help="destination for the verified NumPy checkpoint")
    args = parser.parse_args()
    torch.set_num_threads(1)
    adaptation_path = args.torchscript_dir / "adaptation_module_latest.jit"
    body_path = args.torchscript_dir / "body_latest.jit"
    try:
        output_path = args.output.resolve()
        if output_path in (adaptation_path.resolve(), body_path.resolve()):
            raise ValueError("Output path must differ from the TorchScript inputs")
        adaptation = load_module(adaptation_path)
        body = load_module(body_path)
        weights = extract(adaptation, "adaptation")
        weights.update(extract(body, "body"))
        compare(adaptation, body, weights)
        with TemporaryDirectory(dir=output_path.parent, prefix=".t3-export-") as folder:
            temporary_path = Path(folder) / output_path.name
            with temporary_path.open("wb") as output:
                np.savez_compressed(output, **weights)
            temporary_path.replace(output_path)
    except (AssertionError, OSError, RuntimeError, TypeError, ValueError) as error:
        parser.exit(1, f"{parser.prog}: {error}\n")


if __name__ == "__main__":
    main()
