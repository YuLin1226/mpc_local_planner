# 模型預測控制（MPC）與離散化網格詳解

## 一、MPC/預測控制器的基本概念

模型預測控制（Model Predictive Control, MPC）是一種高級控制策略，它通過求解線上優化問題來生成控制指令。MPC的基本流程是：

1. **預測**：基於當前狀態和系統模型，預測未來一段時間內系統的行為
2. **優化**：找到使性能指標最優的控制序列
3. **執行**：僅執行第一個控制指令
4. **滾動優化**：時間向前推進，重複以上步驟

### 預測控制器的 `step` 函數

預測控制器的 `step` 函數通常執行以下操作：

```cpp
bool PredictiveController::step(const StateVector& x, ReferenceTrajectoryInterface& xref, 
                               ReferenceTrajectoryInterface& uref, const Duration& dt, 
                               const Time& t, TimeSeries::Ptr u_sequence, 
                               TimeSeries::Ptr x_sequence, ...)
```

1. **狀態測量/估計**：獲取系統當前狀態（x）
2. **最佳化問題求解**：
   - 預測系統未來行為
   - 最小化代價函數（平衡跟踪性能和控制代價）
   - 滿足系統約束
3. **控制應用**：僅應用優化序列中的第一個控制指令
4. **滾動時域實現**：下一時間步再次重複整個過程

## 二、離散化網格（Discretization Grid）

### 離散化網格的概念

離散化網格是將連續時間的最佳控制問題轉換為離散時間問題的方法：

1. **時間離散化**：
   - 將連續時間區間 [t₀, tf] 分割為有限個時間點：`t₀, t₁, t₂, ..., tₙ`
   - 相鄰時間點之間的間隔通常稱為 dt

2. **全離散化（Full Discretization）**：
   - 在每個時間點上定義狀態變量：`x₀, x₁, x₂, ..., xₙ`
   - 定義控制輸入：`u₀, u₁, u₂, ..., uₙ₋₁`
   - 系統動態方程轉換為離散等式約束

### 離散化網格的數據結構

離散化網格通常使用以下數據結構：

1. **狀態序列**：`std::vector<StateVertex>` 或 `std::vector<Eigen::VectorXd>`
2. **控制序列**：`std::vector<ControlVertex>` 或 `std::vector<Eigen::VectorXd>`
3. **時間步長**：單一變量（固定時間步長）或數組（變動時間步長）
4. **目標狀態**：終點狀態可能單獨存儲
5. **網格點與邊緣約束的圖結構**：頂點是狀態和控制變量，邊是約束

### 網格更新函數

網格更新函數 `grid.update()` 的主要任務：

```cpp
GridUpdateResult update(const Eigen::VectorXd& x0, ReferenceTrajectoryInterface& xref, 
                        ReferenceTrajectoryInterface& uref, NlpFunctions& nlp_fun, 
                        OptimizationEdgeSet& edges, SystemDynamicsInterface::Ptr dynamics, ...)
```

1. **網格自適應調整**：根據需要調整離散化網格
2. **參考軌跡預計算**：為參考軌跡在網格點上預先計算值
3. **設置先前控制**：設置之前的控制輸入
4. **軌跡初始化**：初始化狀態和控制序列
5. **暖啟動處理**：使用前一次的解進行暖啟動
6. **更新非線性規劃（NLP）函數**：準備優化問題
7. **創建邊緣約束**：設置動力學約束和代價函數

## 三、實際應用中的軌跡生成過程

在實際的機器人控制中，MPC常用於點到點控制：

### 簡化的參考軌跡

```cpp
corbo::StaticReference xref(xf);  // 僅包含目標狀態
corbo::ZeroReference uref(_dynamics->getInputDimension());  // 期望零控制輸入
```

1. **僅使用兩個關鍵點**：
   - 起點：目前機器人的姿態
   - 終點：局部目標點（來自全局路徑的最後一個點）
   
2. **靜態參考**：
   - `xref` 是一個靜態參考，只包含最終目標狀態
   - `uref` 通常是零參考，表示希望最小化控制輸入

### 中間軌跡的生成

中間軌跡不是直接使用全局路徑中的點，而是通過優化生成：

1. **初始化策略**：
   ```cpp
   generateInitialStateTrajectory(x, xf, initial_plan, backward);
   ```
   - 使用全局路徑 `initial_plan` 作為初始猜測
   - 這只是優化的起點，不是硬性約束

2. **物理模型約束**：
   - 使用機器人動力學模型確保生成的軌跡物理可行
   - 處理各種約束，如速度限制、加速度限制等

3. **優化求解**：
   ```cpp
   _ocp_successful = PredictiveController::step(x, xref, uref, dt, time, u_seq, x_seq, ...);
   ```
   - 內部調用 OCP 的 compute 方法
   - 使用 grid 離散化問題
   - 使用 solver 求解最佳化問題

## 四、MPC變數與優化流程

### 最佳化問題中的變數

1. **優化變數**：
   - 未來的控制序列：`u₀, u₁, ..., u_{N-1}`
   - 在全離散化方法中，未來狀態也是變數：`x₁, x₂, ..., x_N`
   
2. **已知量**：
   - 當前狀態 `x₀`
   - 參考軌跡 `xref` 和 `uref`
   - 系統動力學模型

### 優化流程

1. **問題設置**：
   - `grid.update()` 負責設置問題
   - 更新參考軌跡、初始化變數、設定約束
   
2. **求解過程**：
   - `solver.solve()` 真正進行優化
   - 找到最佳的狀態和控制序列
   
3. **應用控制**：
   - 僅應用第一個控制輸入到系統
   - 獲取新的系統狀態，重複整個過程

## 五、優勢與應用

這種"稀疏參考 + 優化填充"的方法優勢：

1. **路徑靈活性**：不強制機器人跟隨可能不符合動力學限制的預定義路徑
2. **平滑高效**：控制器能找到更平滑、更高效的路徑
3. **適應性**：能夠適應變化的環境和擾動
4. **物理合理性**：生成的軌跡符合機器人的物理約束

這種方法在機器人導航中非常常見，特別是當高層規劃器提供的全局路徑可能不完全符合機器人物理約束時。
