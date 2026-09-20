import json
import sys
from pathlib import Path

import torch.distributed as dist

from accelerate.logging import get_logger

logger = get_logger(__name__)

_MSD_DIR = str(Path(__file__).parent / "multi_source_dataset")


def build_dataloader(cfg, dataset_py="single_source_datasets"):
    """Single entry point for building the VLA training dataloader.

    This repo only supports the single-source Unitree dataloader (the
    multi-source bucket-sampler/weighting machinery was dropped).
    """
    if dataset_py not in ("multi_source_datasets", "single_source_datasets"):
        raise ValueError(
            f"Unsupported dataset_py={dataset_py!r}; this repo only supports "
            "the single-source Unitree dataloader ('single_source_datasets')."
        )

    if _MSD_DIR not in sys.path:
        sys.path.insert(0, _MSD_DIR)
    from dataloader import create_training_dataloader  # noqa: E402

    msd_cfg = cfg.datasets.vla_data
    vla_train_dataloader, dataset = create_training_dataloader(
        config_path=msd_cfg.data_config_path,
        batch_size=msd_cfg.per_device_batch_size,
        num_workers=getattr(msd_cfg, "num_workers", 4),
        distributed=dist.is_initialized(),
        num_replicas=dist.get_world_size() if dist.is_initialized() else None,
        rank=dist.get_rank() if dist.is_initialized() else None,
    )

    if not dist.is_initialized() or dist.get_rank() == 0:
        output_dir = Path(cfg.output_dir)
        stats = {
            dataset.config.name: {
                "action": {
                    "offset": dataset._action_norm_offset.tolist(),
                    "scale": dataset._action_norm_scale.tolist(),
                },
                "state": {
                    "offset": dataset._state_norm_offset.tolist(),
                    "scale": dataset._state_norm_scale.tolist(),
                },
            }
        }
        with open(output_dir / "dataset_statistics.json", "w") as f:
            json.dump(stats, f, indent=2)
        logger.info(f"Saved dataset statistics at {output_dir / 'dataset_statistics.json'}")

    return vla_train_dataloader
