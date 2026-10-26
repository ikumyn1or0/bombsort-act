# 元のファイル: matsuolab/pai_training（コミット b4d6b54） の scripts/train.py
# pai_training には LICENSE がないため、このファイルは本リポジトリの MIT ライセンスの対象外。
# 変えた点:
#  - 予測の刻みを引数にする（--action-dt、既定 0.1 秒）
#  - 設定を引数で受け取る（元は configs/train_config.yaml 固定）
#  - 観測のキーで時間方向の次元が付いているものは [:, -1] で落とす（元は state と environment_state だけ決め打ち）
#  - 1ステップの時間を「データ待ち」と「計算（順伝播・逆伝播・更新）」に分けて測る
#  - wandb は使わない
import argparse, importlib.util, statistics, time
from pathlib import Path
import torch
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.common.datasets.utils import dataset_to_policy_features
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.configs.types import FeatureType


def load_config_py(path):
    spec = importlib.util.spec_from_file_location('features_cfg', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset-root', required=True)
    ap.add_argument('--features', required=True)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--steps', type=int, default=100)
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--chunk', type=int, default=16)
    ap.add_argument('--action-dt', type=float, default=0.1, help='予測の刻み（秒）。講義の設定は 0.0〜1.5 を 0.1 刻み')
    ap.add_argument('--lr', type=float, default=1e-4)
    ap.add_argument('--out', default='')
    ap.add_argument('--warmup', type=int, default=5, help='平均から除く最初のステップ数')
    ap.add_argument('--epochs', type=float, default=0, help='>0 なら --steps の代わりに、データを何周するかで長さを決める')
    ap.add_argument('--save-every', type=int, default=0, help='>0 なら、この周数ごとに <out>/ep<周数> に保存する')
    a = ap.parse_args()
    t_start = time.time()
    dev = torch.device(a.device)
    cfg_py = load_config_py(a.features)
    meta = LeRobotDatasetMetadata(cfg_py.repo_id, root=a.dataset_root)
    feats = dataset_to_policy_features(cfg_py.features)
    out_f = {k: f for k, f in feats.items() if f.type is FeatureType.ACTION}
    in_f = {k: f for k, f in feats.items() if k not in out_f}
    print('input_features:', in_f)
    cfg = ACTConfig(input_features=in_f, output_features=out_f, chunk_size=a.chunk, n_action_steps=1)
    policy = ACTPolicy(cfg, dataset_stats=meta.stats)
    policy.train(); policy.to(dev)
    n_params = sum(p.numel() for p in policy.parameters())
    obs_keys = [k for k in in_f]
    delta = {k: [0.0] for k in obs_keys}
    delta['action'] = [round(i * a.action_dt, 6) for i in range(a.chunk)]
    print('delta_timestamps.action:', delta['action'])
    ds = LeRobotDataset(cfg_py.repo_id, root=a.dataset_root, delta_timestamps=delta)
    print(f'dataset: {ds.num_episodes} episodes, {ds.num_frames} frames, fps={meta.fps}; params={n_params/1e6:.1f}M')
    opt = torch.optim.Adam(policy.parameters(), lr=a.lr)
    dl = torch.utils.data.DataLoader(ds, num_workers=a.workers, batch_size=a.batch, shuffle=True,
                                     pin_memory=dev.type == 'cuda', drop_last=True)
    spe = len(dl)
    if a.epochs > 0:
        a.steps = int(round(a.epochs * spe))
    print(f'steps_per_epoch={spe} total_steps={a.steps}', flush=True)
    ep_losses = []
    if dev.type == 'cuda':
        torch.cuda.reset_peak_memory_stats()
    t_setup = time.time() - t_start
    sync = (lambda: torch.cuda.synchronize()) if dev.type == 'cuda' else (lambda: None)
    waits, comps, step = [], [], 0
    t_loop = time.time()
    t_prev = time.perf_counter()
    while step < a.steps:
        for batch in dl:
            t_got = time.perf_counter()
            batch = {k: (v.to(dev, non_blocking=True) if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
            for k in obs_keys:
                # 動画のキーは delta が1つだと時間の次元が付かずに届く。付いているものだけ落とす
                if batch[k].ndim == len(in_f[k].shape) + 2:
                    batch[k] = batch[k][:, -1]
            loss, _ = policy.forward(batch)
            loss.backward(); opt.step(); opt.zero_grad()
            sync()
            t_done = time.perf_counter()
            waits.append(t_got - t_prev); comps.append(t_done - t_got); t_prev = t_done
            if step % 100 == 0:
                print(f'step {step} loss {loss.item():.3f} wait {waits[-1]*1000:.0f}ms comp {comps[-1]*1000:.0f}ms', flush=True)
            ep_losses.append(loss.item())
            step += 1
            if step % spe == 0:
                e = step // spe
                print(f'EPOCH {e} mean_loss {statistics.mean(ep_losses):.4f} elapsed {time.time() - t_loop:.0f}s', flush=True)
                ep_losses = []
                if a.out and a.save_every and e % a.save_every == 0:
                    policy.save_pretrained(f'{a.out}/ep{e}')
                    print(f'saved to {a.out}/ep{e}', flush=True)
            if step >= a.steps:
                break
    t_train = time.time() - t_loop
    w, c = waits[a.warmup:], comps[a.warmup:]
    tot = [x + y for x, y in zip(w, c)]
    print(f'RESULT device={a.device} batch={a.batch} workers={a.workers} steps={a.steps} '
          f'step_mean={statistics.mean(tot):.3f}s step_median={statistics.median(tot):.3f}s '
          f'wait_mean={statistics.mean(w):.3f}s comp_mean={statistics.mean(c):.3f}s '
          f'(n={len(tot)}, warmup {a.warmup} excluded) setup={t_setup:.1f}s loop={t_train:.1f}s final_loss={loss.item():.3f}')
    if dev.type == 'cuda':
        print(f'RESULT gpu_peak_mem={torch.cuda.max_memory_allocated()/2**20:.0f}MiB')
    if a.out:
        Path(a.out).mkdir(parents=True, exist_ok=True)
        policy.save_pretrained(a.out)
        print('saved to', a.out)


if __name__ == '__main__':
    main()
