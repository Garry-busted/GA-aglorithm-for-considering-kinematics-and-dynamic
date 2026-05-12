# GA-aglorithm-for-considering-kinematics-and-dynamic

純R接頭的機構的最佳化分析。
此次考慮了搖撼力的影響，架構仍遵循著「自動模組化」-「力學分析」的架構。
在實作中，這類的架構稱為「kinetostatic」，也就是先有運動學所有資訊後才計算力學分析（仍可以跑靜力與動力問題）。
以下是各函式的名稱與簡介，可能不會一次打完。
* `GA.m`：主運行程式，類似main，根據基因演算法的邏輯架構所做出的Interface。
* `solver_multi_objects.m`：求解器，負責運算當代中一組染色體的機構運動學與力學並計算其適應值。
