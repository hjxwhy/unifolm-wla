# 从头训练动作专家

**中文** | [English](train_action_expert_en.md)

本文介绍如何基于 UnifoLM-ER 系列基础视觉语言模型，从头训练
UnifoLM-WLA 动作专家。以下命令均假设当前工作目录为项目根目录。

## 1. 下载基础视觉语言模型

选择并下载以下任一基础模型：

- [UnifoLM-ER-1](https://huggingface.co/unitreerobotics/UnifoLM-ER-1)
- [UnifoLM-ER-Flow](https://huggingface.co/unitreerobotics/UnifoLM-ER-Flow)

下载完成后，打开
[`examples/pretrain/train_files/run_multi_source_train_mmdit.sh`](../examples/pretrain/train_files/run_multi_source_train_mmdit.sh)，
将 `base_vlm` 设置为模型在本机的完整路径。例如：

```bash
base_vlm=/path/to/UnifoLM-ER-1
```

也可以将其改为 UnifoLM-ER-Flow 的本地目录：

```bash
base_vlm=/path/to/UnifoLM-ER-Flow
```

## 2. 下载并配置训练数据

从 [UnifoLM-WLA-1.0 数据集合集](https://huggingface.co/collections/unitreerobotics/unifolm-wla-10)
下载训练数据。数据需要按照 Dex1 和 WBT 两种类型分别组织；同一种类型的目录下可以包含多个任务。
推荐的目录结构如下：

```text
/path/to/unifolm_data/
├── UnifoLM_G1_Dex1_Dataset/
│   ├── G1_Dex1_MountCamera_Dataset/
│   └── G1_Dex1_Stack_Block/
└── UnifoLM_WBT_Dataset/
    ├── G1_WBT_Brainco_Pickup_Pillow/
    └── G1_WBT_Brainco_Make_The_Bed/
```

随后编辑
[`unifolm_wla/dataloader/multi_source_dataset/configs/unitree.yaml`](../unifolm_wla/dataloader/multi_source_dataset/configs/unitree.yaml)：

1. 将 `data_base` 设置为同时包含 Dex1 和 WBT 目录的公共父目录。
2. Dex1 数据继承 `*unitree_base`，并将 `data_path` 设置为 `UnifoLM_G1_Dex1_Dataset`。
3. WBT 数据继承 `*unitree_fullbody_base`，并将 `data_path` 设置为 `UnifoLM_WBT_Dataset`。
4. 将 `cache_dir` 设置为具有足够空间的本地缓存目录。

配置示例：

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

`multi_task: true` 时，加载器会把 `data_path` 下的各任务目录作为子数据集加载。
所有设置为 `enabled: true` 的数据源都会通过 `ConcatDataset` 拼接，因此不会遗漏后面的数据源；
采样比例由各数据源的数据量决定，当前不会应用配置中的 `weight`。

首次加载 LeRobot 数据时，加载器会在 `cache_dir/arrow_cache` 下生成 Arrow 缓存。
后续加载会复用该缓存，因此建议将 `cache_dir` 放在容量充足、读写速度较快的磁盘上。

## 3. 单节点训练

确认脚本中的 `base_vlm`、`run_root_dir` 和数据配置路径正确后，在项目根目录执行：

```bash
bash examples/pretrain/train_files/run_multi_source_train_mmdit.sh
```

脚本默认使用当前节点上 `nvidia-smi -L` 检测到的全部 GPU。若只希望启动指定数量的进程，
可以通过 `NUM_PROCESSES` 覆盖，例如：

```bash
NUM_PROCESSES=4 bash examples/pretrain/train_files/run_multi_source_train_mmdit.sh
```

训练输出默认写入 `run_root_dir/run_id`。批大小、训练步数、保存间隔和其他训练参数可在启动脚本中调整。
