# FRC2026-Public: 航太級控制系統框架

![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen) ![Version](https://img.shields.io/badge/Version-2026.1.0-blue) ![Control](https://img.shields.io/badge/Control-StateSpace%2BMPC-orange)

## 📌 專案概述 (Project Overview)

本專案 (`FRC2026-Public`) 是一個為 FRC 機器人設計的高性能控制軟體框架。不同於傳統的 FRC 程式碼，本系統採用**航太級 (Aerospace-Grade) 軟體架構**，強調確定性 (Determinism)、硬體抽象化 (Hardware Abstraction) 與基於第一性原理 (First Principles) 的控制演算法。

核心亮點包含基於 **Delta-U 擴增狀態空間 (Augmented State Space) 的模型預測控制 (MPC)**、抗干擾觀測器 (Disturbance Observer) 以及嚴格分離的 I/O 層。

---

## 🏗 系統架構 (System Architecture)

本框架深受 FRC Team 254 (The Cheesy Poofs) 架構啟發，並針對現代非線性控制需求進行了深度客製。

### 1. 狀態機與循環 (State Machines & Loops)
系統依靠 `Looper` 與 `Subsystem` 介面管理週期性任務。所有子系統 (Subsystem) 運行於嚴格的 20ms 控制迴路中，確保控制律 (Control Law) 的時間一致性。
- **確定性執行**：避免 Java Garbage Collection (GC) 造成的時序抖動。
- **分離式架構**：邏輯層 (Logic) 與硬體層 (IO) 完全解耦，支援 Simulation-in-the-Loop (SIL) 驗證。

### 2. 硬體抽象層 (Hardware Abstraction Layer - HAL)
所有感測器與致動器皆透過 `IO` 介面 (如 `DriveIO`) 存取。
- `DriveIOReal`: 實際硬體實作 (CTRE Phoenix 6, REV).
- `DriveIOSim`: 物理模擬實作，用於單元測試與演算法驗證。

---

## 🚀 核心控制演算法 (Core Control Algorithms)

我們拒絕使用 "Magic Numbers" 或單純的 PID 試誤法。所有控制策略皆源自物理建模。

### 1. Delta-U 模型預測控制 (Delta-U MPC)
位於 `libraries/lib9427/controllers`，這是本框架的控制核心。
- **問題定義**：傳統 LQR 對靜態誤差 (Steady-State Error) 敏感。
- **解決方案**：採用 Delta-U Formulation，將控制增量 $\Delta u$ 作為輸入，狀態 $u$ 擴增為狀態變數。這自然引入了積分作用 (Integral Action)，完美消除靜態誤差。
- **抗干擾觀測器 (Disturbance Observer, DOB)**：實時估計未建模的外部力矩 (摩擦、碰撞)，並在前饋項 (Feedforward) 中主動補償。
- **JNI 加速**：核心矩陣運算透過 JNI 呼叫 C++ 原生庫執行，確保計算延遲 < 1ms。

### 2. 軌跡追蹤 (Trajectory Following)
位於 `auto/trajectory`，採用前饋主導 (Feedforward-Dominant) 的控制策略。
- **理論基礎**：$u(t) = K_p(r - x) + \dot{r}_{ff}$
- **設計哲學**：前饋項 ($\dot{r}_{ff}$) 負責 90% 的控制輸出，PID 僅負責修正微小的隨機誤差。這消除了傳統 PID 造成的相位滯後 (Phase Lag)。

### 3. Swerve 驅動與動力學 (Swerve Drive Dynamics)
位於 `subsystems/drive`。
- **SwerveSetpointGenerator**：基於 254 的動力學約束生成器，防止模組在高速轉向時發生物理上不可行的指令 (如無限加速度)，避免輪胎打滑與電流過載。
- **電壓補償 (Voltage Compensation)**：即時測量匯流排電壓，動態調整 PWM 輸出，確保在電池電壓下降時控制增益 ($K_v$) 保持恆定。

---

## 🛠 建置與部署 (Build & Deploy)

本專案使用 Gradle 進行依賴管理與建置。

```bash
# Windows
.\gradlew build

# 部署至機器人
.\gradlew deploy
```

---

## 💡 反直覺工程觀點 (Counter-Intuitive Engineering Insight)

### 為什麼「完美的模型」不如「優秀的觀測器」？

在控制工程中，新手常花費數週試圖建立完美的物理模型 (精確測量質量、轉動慣量、摩擦係數)。然而，**這通常是徒勞的**。

**現實是骯髒的**：地毯磨損會改變摩擦力 (.8 -> .9)，電池內阻隨溫度變化，潤滑油黏度隨時間改變。一個「完美」的模型在比賽開始 30 秒後就失效了。

**我們的策略**：
我們接受模型是**錯誤的** (Imperfect Model)。
我們將所有模型誤差 (Model Mismatch) 與外部干擾 (External Disturbance) 視為一個總集訊號 $d(t)$，並使用**擴增狀態觀測器 (Augmented State Observer)** 即時估計它。

$$ \hat{d}_{k+1} = \hat{d}_k + L (y_k - C \hat{x}_k) $$

與其花費 100 小時追求 99% 準確的模型，不如花 10 小時設計一個能處理 20% 模型誤差的強健 (Robust) 觀測器。這就是為什麼我們的機器人能在機構老化、電壓下降的情況下，依然保持公分級的控制精度。

---

## 📜 版權 (Credentials)

Copyright © 2026 FRC Team / William-season. All Rights Reserved.
Based on WPILib & Team 254 Cheesy Poofs Architecture.
