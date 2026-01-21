# FRC2026-Public

FRC Team 9427 的 2026 賽季機器人程式碼。

## 功能特色

### 底盤系統 (Swerve Drive)
- 四輪獨立轉向 Swerve Drive (SDS MK5N 模組)
- 基於 WPILib Kinematics 的運動學計算
- CANcoder 絕對值編碼器校正
- 電壓補償與電流限制

### 自動駕駛 (Autonomous)
- CSV 軌跡讀取與追蹤
- PID + Feedforward 軌跡追蹤器
- Team 254 風格的 Action/AutoMode 框架

### 控制系統
- Delta-U MPC 控制器 (JNI 原生實作)
- 狀態空間建模 (State Space)
- 擴增狀態觀測器 (Disturbance Observer)

### 子系統
- **Intake Wheel**: 單馬達滾輪進球
- **Intake Pivot**: 雙馬達四連桿機構
- **Indexer**: 雙滾輪傳送機構

### 架構設計
- IO 抽象層 (支援模擬測試)
- Looper 週期性任務管理
- 子系統狀態機設計

## 硬體配置

| 元件 | 型號 |
|------|------|
| 驅動馬達 | Kraken X60 |
| 轉向馬達 | Kraken X44 |
| 絕對值編碼器 | CANcoder |
| IMU | Pigeon 2 |

## 建置

```bash
.\gradlew build
.\gradlew deploy
```

## 授權

基於 WPILib 開源授權。
