# 握手動作重心穩定功能說明

## 問題描述

機器人執行握手動作時，會將前左腿（FL，Front-Left）抬起。此時機器人僅以其餘三腳支撐，支撐三角形由以下三腳構成：

| 腿名稱 | 代號 | 位置索引 |
|--------|------|----------|
| 前右腿 | FR（Front-Right） | col 0 |
| 後右腿 | RR（Rear-Right）  | col 2 |
| 後左腿 | RL（Rear-Left）   | col 3 |

原本程式直接抬腿，未進行任何重心補償，導致機器人的重心（CoM）落在支撐三角形**外側**，造成傾倒。

---

## 解決方法

在抬起 FL 腿之前，先讓機器人身體向支撐三角形的重心方向平移，使 CoM 落入支撐三角形內部，維持靜態穩定性；握手完成後再平順地復原身體位置。

此做法不依賴外部函式庫（如 placo），而是利用既有的自製 IK（`Triceratops_IK`）實現。

---

## 幾何計算

**預設站姿各腳位置（body frame，單位：公尺）：**

| 腿  | 位置索引 | X（前後） | Y（左右） |
|-----|---------|----------|----------|
| FR  | 0       | +0.09067 | −0.085   |
| FL  | 1       | +0.09067 | +0.085   |
| RR  | 2       | −0.09067 | −0.085   |
| RL  | 3       | −0.09067 | +0.085   |

**握手時支撐三角形（FL 被抬起）重心：**

```
centroid_X = (0.09067 + (-0.09067) + (-0.09067)) / 3 ≈ -0.0302 m
centroid_Y = (-0.085  + (-0.085)  + 0.085)        / 3 ≈ -0.0283 m
```

原始重心位於 (0, 0)，需往 (−0.030, −0.028) 方向偏移。  
考量安全邊界取 60%，實際平移量為：

```
COM_SHIFT_X = -0.018 m
COM_SHIFT_Y = -0.017 m
```

**身體平移的實現方式：**  
在 body frame 中，身體往 (dx, dy) 移動，等效於所有腳目標位置往反方向移動 (−dx, −dy)，再重新計算 IK 即可。

---

## 程式碼修改說明

**修改檔案：**  
`Controller.py`

**影響範圍：** 僅限握手動作，步態、IK、Config 等所有其他函式均未改動。

---

### 新增方法：`_shift_body(x_offset, y_offset, steps=20)`

```python
def _shift_body(self, x_offset, y_offset, steps=20):
    """將身體平移至 (x_offset, y_offset)，透過反向移動所有腳位置後跑 IK。僅供 handshake 使用。"""
    baseline = self.config.default_stance.copy()
    for i in range(1, steps + 1):
        progress = i / steps
        shifted = baseline.copy()
        shifted[0, :] -= x_offset * progress
        shifted[1, :] -= y_offset * progress
        joint_angles = self.inverse_kinematics.four_legs_inverse_kinematics(shifted)
        goal = (
            joint_angles * 180 / 3.14 * DEGREE_TO_SERVO
            + self.config.leg_center_position
        )
        self.control_cmd.motor_position_control(goal)
        time.sleep(0.05)
```

**說明：**
- `baseline`：以 `config.default_stance` 作為基準（不依賴步態當下的 state，確保握手前後行為一致）
- 迴圈 `steps=20` 步、每步間隔 0.05 秒，總過渡時間約 1 秒，使動作平順不急促
- `_shift_body(0, 0)` 即代表回到預設站姿，可作為恢復動作使用

---

### 修改方法：`handshake()`

**修改前流程：**
```
stop_gait → reset_to_original → 抬 FL 腿（20 步插值）→ 保持 2 秒 → reset_to_original → start_gait
```

**修改後流程：**
```
stop_gait → reset_to_original → 重心平移（_shift_body）→ 抬 FL 腿（20 步插值）→ 保持 2 秒 → 恢復重心（_shift_body）→ reset_to_original → start_gait
```

新增的兩個步驟：

| 時機 | 呼叫 | 說明 |
|------|------|------|
| 抬腿前 | `_shift_body(-0.018, -0.017)` | 身體往後右偏移，CoM 進入支撐三角形 |
| 放腿後 | `_shift_body(0, 0)` | 身體平順回到中心，再交給 reset_to_original |

---

## 參數調整指引

若機器人在握手時仍有傾斜，可調整 `Controller.py` 中的以下兩個常數（位於 `handshake()` 方法內）：

```python
COM_SHIFT_X = -0.018   # 往後（負 X）偏移量，單位：公尺
COM_SHIFT_Y = -0.017   # 往右（負 Y）偏移量，單位：公尺
```

**調整原則：**

| 狀況 | 調整方向 |
|------|----------|
| 仍然往 FL 方向傾倒 | 將數值往 −0.025 方向加大（例如 −0.022, −0.021） |
| 馬達回報錯誤或卡頓 | 縮小數值，或將 `_shift_body` 的 `steps` 從 20 增加到 30 |
| 平移過大、其他腿快碰到關節極限 | 縮小數值至 −0.012 左右 |

理論上限為支撐三角形重心的 100%（X=−0.030, Y=−0.028），但建議最多使用 80% 以保留安全餘量。

---

## 不受影響的功能

以下功能完全未改動：

- 步態控制：`puppy_move()`、`start_gait()`、`stop_gait()`
- 步態計算：`GaitController`、`StanceController`、`SwingController`
- 逆向運動學：`Triceratops_IK`
- 機器人設定：`Triceratops_Config`
- 腰部晃動：`sway()`
- 所有 ROS2 訂閱與指令介面
