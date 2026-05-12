# GA-algorithm-for-considering-kinematics-and-dynamics

純 R 接頭機構的最佳化分析。此次考量了搖撼力（Shaking Force）的影響，架構仍遵循著「自動模組化 $\rightarrow$ 力學分析」的邏輯。

在實作中，這類的架構稱為 **「Kinetostatics」（動靜力分析）**，也就是先解出機構運動學的所有資訊後，才計算力學分析矩陣（仍可以跑靜力與動力問題）。以下是各函式的名稱與簡介，後續會持續更新補完：

* **`GA.m`**：主運行程式。類似 `main` 函式，根據基因演算法（Genetic Algorithm）的邏輯架構所做出的 Interface。
* **`solver_multi_objects.m`**：求解器。負責運算當代中一組染色體的機構運動學與力學，並計算其適應值（Fitness value）。
