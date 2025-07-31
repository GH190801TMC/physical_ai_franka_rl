# FRANKA ROS2 強化学習シミュレータ

[![Docker](https://img.shields.io/badge/Docker-Container-blue.svg)](https://www.docker.com/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange.svg)](https://ubuntu.com/)
[![Python](https://img.shields.io/badge/Python-3.10-green.svg)](https://www.python.org/)

## 🎯 プロジェクト概要

FRANKA FR3ロボットアームの強化学習環境を提供するROS2ベースのシミュレータです。Cube把持・輸送タスクの学習に特化しています。

### 主要機能
- 🤖 **FRANKA FR3モデル**: 公式franka_gazebo_bringup完全統合
- 🎮 **強化学習環境**: OpenAI Gym + PPO (Stable Baselines3)
- 📊 **高速シミュレーション**: Gazebo物理エンジン統合
- 🎬 **録画システム**: 学習過程の可視化

## 🚀 クイックスタート

```bash
# プロジェクトセットアップ
cd ~/ros2_robot_simulator

# Dockerコンテナ起動
docker compose up -d

# 公式FRANKAパッケージビルド確認
docker exec franka_ros2 bash -c "cd /ros2_ws && source install/setup.bash && ros2 pkg list | grep franka"

# FR3モデル表示デモ
cd robot_test_framework
python3 franka_official_recorder.py

# TDDテスト実行
cd ~/ros2_robot_simulator
./tdd
```

## 🏗️ システムアーキテクチャ

```
Ubuntu 24.04 Host
├── Docker Container (ROS2 Humble)
│   ├── FRANKA公式パッケージ
│   │   ├── franka_description
│   │   ├── franka_gazebo_bringup
│   │   ├── franka_hardware
│   │   └── franka_msgs
│   ├── Gazeboシミュレータ
│   └── MoveIt2統合
└── 強化学習フレームワーク
    ├── Gym環境
    ├── PPOエージェント
    └── 可視化システム
```

## 📚 強化学習実装計画

### MVP-0.6.x シリーズ
- **MVP-0.6.1**: 強化学習環境基盤（OpenAI Gym）
- **MVP-0.6.2**: Cube物理シミュレーション
- **MVP-0.6.3**: 状態・行動空間定義（20次元観測、8次元行動）
- **MVP-0.6.4**: 報酬関数設計（速度最適化重視）
- **MVP-0.6.5**: IK統合・軌道計画（MoveIt2）
- **MVP-0.6.6**: PPO実装（Stable Baselines3）
- **MVP-0.6.7**: 学習管理システム
- **MVP-0.6.8**: 可視化・モニタリング（TensorBoard）
- **MVP-0.6.9**: 評価・ベンチマーク
- **MVP-0.6.10**: 統合デモンストレーション

詳細は [強化学習実装計画書](docs/reinforcement_learning/RL_IMPLEMENTATION_PLAN.md) を参照。

## 🛠️ 必要環境

- **OS**: Ubuntu 24.04 LTS
- **Docker**: 20.10以上
- **GPU**: NVIDIA GPU（推奨）
- **メモリ**: 16GB以上推奨

## 📂 プロジェクト構造

```
ros2_robot_simulator/
├── robot_test_framework/      # テストフレームワーク
│   ├── franka_official_recorder.py  # FR3録画システム
│   ├── franka_test_runner.py       # 統合テストランナー
│   └── tests/                       # MVP別テストスイート
├── docs/                      # ドキュメント
│   └── reinforcement_learning/      # 強化学習関連
├── config/                    # 設定ファイル
└── CLAUDE.md                  # Claude Code設定
```

## 🔄 開発ワークフロー

1. **コード変更実施**（最大50行）
2. **TDD実行**: `./tdd`（必須）
3. **結果確認**: HTMLレポート確認
4. **コミット**: 成功率100%確認後

## 📊 現在の状態

- ✅ FR3モデルGazebo表示成功
- ✅ 公式franka_gazebo_bringupビルド完了  
- ✅ 動画録画システム構築
- ✅ TDDテスト最適化（28テスト中26テスト成功）
- 🚧 強化学習環境実装準備中

## 🤝 コントリビューション

1. 公式FRANKAパッケージ使用必須
2. TDD 100%準拠
3. 日本語コメント推奨
4. 小規模変更（50行以内）

## 📄 ライセンス

MIT License

---

**問い合わせ**: [Issues](https://github.com/your-repo/issues)