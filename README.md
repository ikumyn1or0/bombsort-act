# bombsort-act

ロボットアーム CRANE+ V2 が、Gazebo の中で、カメラ画像だけを見て赤と黒の円柱を色ごとの陣地へ仕分ける。
方策は模倣学習の ACT（LeRobot）で、お手本は、物体の正しい位置を知っているスクリプトに作らせた。

松尾研究室「Physical AI 応用講座1」の最終課題として作成した。
着想元は、任天堂「スーパーマリオ64DS」のミニゲーム「ボムへいをわけろ！」である（キャラクターの名前と見た目は使っていない）。

- 動画：https://youtu.be/IV1W7EmeI8U
- 説明資料：講座に提出した PDF

## 課題の設定

- 机の上のフィールドに、円柱が3個（赤と黒の組み合わせ）静止して置かれる。これを1組とする
- 方策の入力は、斜め上のカメラの画像（128×128 RGB）と関節角5つ。物体の位置は入力しない
- 方策は、どの物体から取るかも含めて、3個を色ごとの陣地へ運ぶ
- 組を出してから 45 秒（シミュレータの時刻）以内に3個とも陣地へ置ければ、組の成功とする

掴みは簡略化している。手を閉じたときに、手先と物体の水平距離が上限（10 mm）以内なら、物体を手先の真下へ寄せて固定する。

## 結果

提出の版（10 周学習した checkpoint）を 20 組評価した。配置は、学習データの記録（乱数の種 41）とは別の種（110）で作った。

| 指標 | 値 |
|---|---|
| 組の成功 | 17 / 20 |
| 個の成功 | 54 / 60 |
| 1個あたりの所要時間（中央値） | 10.7 秒（シミュレータの時刻） |
| 失敗の内訳（個） | 時間切れ 4、フィールドの外へ出た 2 |

## 構成

```
bombsort/   シミュレータ側：起動ファイル、ワールド、物体の位置を中継するプログラム
bs/         審判、お手本、変換、学習、推論、評価と学習の手順
```

| ファイル | 役割 |
|---|---|
| `bombsort/bombsort.launch.py` | Gazebo、ロボット、コントローラ、通信の中継を起動する |
| `bombsort/bombsort_v6m.sdf` | 評価と学習に使うワールド（カメラ、壁、陣地、円柱6個） |
| `bombsort/bombsort_v6m_view.sdf` | 上に、録画用のカメラを足したもの |
| `bombsort/pose_relay.cc` | 物体の位置の設定と読み出しを Gazebo と中継する（ビルドは `build_relay.sh`） |
| `bs/referee.py` | 審判。物体を出し、掴みの固定と外しを受け持ち、成否を判定する |
| `bs/demo.py` | お手本のスクリプト。審判が出す正しい状態を読み、決まった手順で運ぶ |
| `bs/rosbag2lerobot_bs.py` | 記録（rosbag）を LeRobot のデータセットに変換する |
| `bs/train.py` | ACT を学習する |
| `bs/infer.py` | 学習した方策で腕を動かす推論ノード |
| `bs/run.sh` | ワールドを立て、審判と、お手本か方策を組んで1回まわす |
| `bs/m4b_pipeline.sh` | 学習の通し：お手本で 100 組を記録 → 変換 → 30 周学習 → 途中の周を評価 |
| `bs/t9d_am10.sh` | 提出の版の最終評価（上の結果） |

ほかのシェル（`m4_eval.sh` など）は、上の2つから呼ばれる部品である。

## 動かし方

講義の演習と同じ Docker イメージの中で動かす。コードの中のパスは `/workspace/bs`・`/workspace/bombsort`・`/home/ubuntu/...` に決め打ちしてあるので、このリポジトリを `/workspace` にマウントする。

### 1. コンテナを起動する

```bash
git clone https://github.com/ikumyn1or0/bombsort-act.git
cd bombsort-act
docker run -d --name pai_ros2_gpu --gpus all -p 6080:80 --shm-size=512m \
  -v "$(pwd)":/workspace airobotbook/ros2-desktop-ai-robot-book-humble:latest
```

このイメージには、書籍のサンプル（AI-Robot-Book-Humble）のワークスペース `~/airobot_ws` が入っている。CRANE+ V2 のパッケージと、逆運動学に使う chapter6 もここにある。
ブラウザで `http://localhost:6080` を開くと、コンテナのデスクトップが見える。

以下は、コンテナの中で `ubuntu` ユーザーとして実行する（`docker exec -it -u ubuntu pai_ros2_gpu bash`）。

### 2. 学習環境（pai_training）を用意する

```bash
cd ~ && git clone --recursive https://github.com/matsuolab/pai_training
cd pai_training && git checkout b4d6b54
curl -fsSL https://pixi.sh/install.sh | bash
~/.pixi/bin/pixi install
```

変換・学習・推論は、この pixi 環境で動かす（`bs/px.sh` と `bs/agent.sh` が呼ぶ）。
このリポジトリの `bs/` にある変換・学習・推論のスクリプトは pai_training のスクリプトを改造したもので、pai_training の中のファイルは書き換えない。

### 3. 物体の位置の中継をビルドする

```bash
bash /workspace/bombsort/build_relay.sh   # /home/ubuntu/pose_relay ができる
```

### 4. 学習する

```bash
bash /workspace/bs/m4b_pipeline.sh
```

お手本で 100 組を記録し（種 41）、変換し、30 周学習する。学習と並行して、5 周ごとの checkpoint を評価する。
記録は `~/dataset/rosbags/`、データセットは `~/dataset/lerobot_dataset/`、checkpoint は `~/checkpoints/` にできる。
進み具合は `/workspace/bs/logs/m4b_pipeline.log` に出る。済んだ段は飛ばすので、落ちたら同じコマンドで続きから回る。

Gazebo は実時間でしか進まない。手元（RTX 4060）で提出の版を作ったときは、記録に 65 分、変換に 7 分、学習 30 周に 91 分かかった（学習は評価と並行）。

### 5. 評価する

```bash
bash /workspace/bs/t9d_am10.sh
```

10 周の checkpoint を、審判の条件 `--k 3 --time-limit 15 --placement dir --balance --batch-limit 45 --attach-max 0.010` で 20 組評価する（種 110）。
結果は `/workspace/bs/results/` に、1個1行の jsonl と集計の json で出る。
このスクリプトは、比べるために試した別の版（256×256 の画像）も続けて評価する。その checkpoint はこのリポジトリの手順では作らないので、その段は失敗する。

## 授業のコードとの対応

授業で使ったコードのうち、改造したファイルだけをこのリポジトリに置いた。各ファイルの冒頭に、元のファイルと変えた点を書いてある。元のリポジトリは、上の手順の中で clone する。

| このリポジトリのファイル | 元のファイル | 変えた点 | 理由 |
|---|---|---|---|
| `bs/rosbag2lerobot_bs.py` | [matsuolab/pai_training](https://github.com/matsuolab/pai_training) の `scripts/rosbag2lerobot.py` | 1つの記録に入った複数のエピソードを、審判が出す開始から初期姿勢に戻るまでで切り分ける。成功して初期姿勢に戻ったエピソードだけを使う。シミュレータの時刻で 20 Hz に標本化する | 100 組を1本の記録で続けて取るため。失敗した動きを学ばせないため |
| `bs/train.py` | 同 `scripts/train.py` | 設定を引数で受け取る。予測の刻みを 0.1 秒にする。1ステップの時間を、データ待ちと計算に分けて測る | 条件を変えた学習を、設定ファイルを書き換えずに回すため |
| `bs/infer.py` | 同 `scripts/infer.py` | 指令をシミュレータの時刻で 20 Hz に合わせる。起動したら初期姿勢へ送ってから方策に切り替える | お手本と変換が、シミュレータの時刻で 20 Hz だから。審判は、腕が初期姿勢に着くまで物体を出さないから |
| `bombsort/bombsort.launch.py` | [AI-Robot-Book-Humble/crane_plus](https://github.com/AI-Robot-Book-Humble/crane_plus) の `crane_plus_gazebo/launch/crane_plus_with_table.launch.py` | ワールドを差し替える。手先に物体を固定するための関節を加える。カメラなどの通信の中継を足す | 課題のフィールドとカメラを使い、掴みを簡略化するため |
| `bombsort/bombsort_v6m.sdf`、`bombsort_v6m_view.sdf` | 同 `crane_plus_gazebo/worlds/table.sdf` | カメラ、フィールドの壁、陣地、円柱を足す | 課題のフィールドを作るため |
| `bs/referee.py`、`bs/demo.py` ほか | 自作 | — | — |

逆運動学には、書籍のサンプル [AI-Robot-Book-Humble/chapter6](https://github.com/AI-Robot-Book-Humble/chapter6) の `kinematics.py` を、コンテナの中のものから import して使う（このリポジトリには複製していない）。

## ライセンス

自作のファイルは MIT License（`LICENSE`）とする。

次のファイルは対象外である。

- `bs/rosbag2lerobot_bs.py`、`bs/train.py`、`bs/infer.py`：matsuolab/pai_training を改造したもの。元のリポジトリにはライセンスの表示がない
- `bombsort/bombsort.launch.py`、`bombsort/bombsort_v6m.sdf`、`bombsort/bombsort_v6m_view.sdf`：AI-Robot-Book-Humble/crane_plus を改造したもの。元のライセンス（Apache License 2.0）に従う
