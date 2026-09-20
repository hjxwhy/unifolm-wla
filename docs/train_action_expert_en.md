# Training an Action Expert from Scratch

[Chinese](train_action_expert.md) | **English**

This guide explains how to train a UnifoLM-WLA action expert from scratch using
a pretrained UnifoLM-ER vision-language model. All commands assume that the
current working directory is the project root.

## 1. Download the Base Vision-Language Model

Download either of the following base models:

- [UnifoLM-ER-1](https://huggingface.co/unitreerobotics/UnifoLM-ER-1)
- [UnifoLM-ER-Flow](https://huggingface.co/unitreerobotics/UnifoLM-ER-Flow)

After downloading the model, open
[`examples/pretrain/train_files/run_multi_source_train_mmdit.sh`](../examples/pretrain/train_files/run_multi_source_train_mmdit.sh)
and set `base_vlm` to the full path of the local model directory. For example:

```bash
base_vlm=/path/to/UnifoLM-ER-1
```

To use UnifoLM-ER-Flow instead, set the path to its local directory:

```bash
base_vlm=/path/to/UnifoLM-ER-Flow
```

## 2. Download and Configure the Training Data

Download the training data from the
[UnifoLM-WLA-1.0 dataset collection](https://huggingface.co/collections/unitreerobotics/unifolm-wla-10).
Organize the data by the Dex1 and WBT dataset types. Each type may contain
multiple task directories. The recommended directory structure is:

```text
/path/to/unifolm_data/
├── UnifoLM_G1_Dex1_Dataset/
│   ├── G1_Dex1_MountCamera_Dataset/
│   └── G1_Dex1_Stack_Block/
└── UnifoLM_WBT_Dataset/
    ├── G1_WBT_Brainco_Pickup_Pillow/
    └── G1_WBT_Brainco_Make_The_Bed/
```

Then edit
[`unifolm_wla/dataloader/multi_source_dataset/configs/unitree.yaml`](../unifolm_wla/dataloader/multi_source_dataset/configs/unitree.yaml):

1. Set `data_base` to the common parent directory containing the Dex1 and WBT directories.
2. For Dex1 data, inherit from `*unitree_base` and set `data_path` to `UnifoLM_G1_Dex1_Dataset`.
3. For WBT data, inherit from `*unitree_fullbody_base` and set `data_path` to `UnifoLM_WBT_Dataset`.
4. Set `cache_dir` to a local cache directory with sufficient free space.

Example configuration:

```yaml
data_base: "/path/to/unifolm_data"
cache_dir: "/path/to/unifolm_cache"

datasets:
  - <<: *unitree_base
    name: "unifolm_g1_dex1"
    data_path: "UnifoLM_G1_Dex1_Dataset"
    image_keys: *unitree_img_with_stereo

  - <<: *unitree_fullbody_base
    name: "unifolm_wbt"
    data_path: "UnifoLM_WBT_Dataset"
    image_keys: *unitree_img_wo_stereo
```

When `multi_task: true`, the loader treats each task directory under
`data_path` as a sub-dataset. All sources with `enabled: true` are combined
using `ConcatDataset`, so later sources are not skipped. Sampling is
proportional to the number of samples in each source; the current implementation
does not apply the configured `weight` value.

When LeRobot data is loaded for the first time, the loader creates an Arrow
cache under `cache_dir/arrow_cache`. Subsequent runs reuse this cache. Place
`cache_dir` on a disk with sufficient capacity and good read/write performance.

## 3. Train on a Single Node

After verifying `base_vlm`, `run_root_dir`, and the dataset configuration path
in the launch script, run the following command from the project root:

```bash
bash examples/pretrain/train_files/run_multi_source_train_mmdit.sh
```

By default, the script launches one process for each GPU detected by
`nvidia-smi -L`. Set `NUM_PROCESSES` to use a specific number of processes:

```bash
NUM_PROCESSES=4 bash examples/pretrain/train_files/run_multi_source_train_mmdit.sh
```

Training outputs are written to `run_root_dir/run_id`. Batch size, training
steps, checkpoint intervals, and other training parameters can be adjusted in
the launch script.
