# robstride-rs 詳細仕様

Rust crate `robstride-rs` の通信プロトコル / API 詳細仕様書。

- 対象: Robstride RS-00 / RS-01 / RS-02 / RS-03 / RS-04 / RS-05 / RS-06 (Edulite01 / 02 / 05 を含む)
- 物理層: CAN 2.0B (標準ボーレート 1 Mbps、29-bit 拡張ID)
- 当面の動作確認ターゲット: Edulite05 (= RS-05)

> プロトコルの記載値は公開仕様書および sandbox 実装からの転記です。ファームウェアリビジョンによって差異がある可能性があるため、本番投入前にお手元の機体マニュアルと突き合わせて検証してください。

---

## 1. クレート構成

```
robstride-rs/
├── Cargo.toml                       # workspace ルート
├── crates/
│   ├── robstride-protocol/          # no_std 純粋プロトコル層
│   ├── robstride/                   # socketcan 同期ドライバ
│   └── robstride-cli/               # テスト用 CLI バイナリ
└── doc/
    └── spec.md                      # 本書
```

| クレート             | 役割                                                                                  | `no_std`           |
| -------------------- | ------------------------------------------------------------------------------------- | ------------------ |
| `robstride-protocol` | CAN ID / フレーム / MIT エンコード / レスポンスパーサ。I/O・alloc 不使用。           | yes                |
| `robstride`          | Linux SocketCAN ベースの同期ドライバ (`Motor` API + バススキャン)。                  | no                 |
| `robstride-cli`      | プロトコル動作確認用の CLI バイナリ。                                                | no                 |

`robstride-protocol` は I/O を行わないので、STM32 や RP2040 等の embedded HAL と組み合わせて再利用できます。

---

## 2. 物理層 / バス設定

| 項目         | 値 (デフォルト)                                                |
| ------------ | -------------------------------------------------------------- |
| 物理層       | CAN 2.0B (差動 CAN-H / CAN-L)                                  |
| ビットレート | 1 Mbps (Robstride 標準)                                        |
| フレーム種別 | **拡張フレーム (29-bit ID) のみ**                              |
| データ長     | **常に 8 バイト** (`DATA_LEN = 8`)                             |
| 終端抵抗     | バス両端に 120 Ω                                              |
| ホスト ID    | デフォルト `0xFD` (`DEFAULT_HOST_ID`)。バス上の最大モータID より大きい値を推奨 |
| モータ ID    | 1..=127 (CAN ID として 0 / 0xFF 等は非推奨)                    |

### 2.1 SLCAN (USB-CAN) アダプタ例

```bash
sudo slcand -o -s8 -t hw -S 1000000 /dev/ttyUSB0 can0
sudo ip link set can0 up
```

### 2.2 ネイティブ CAN

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

---

## 3. CAN ID レイアウト (29-bit)

```
bits  28..24       23..8           7..0
     +---------+----------------+-----------+
     | comm(5) |  extra_data(16)| dev_id(8) |
     +---------+----------------+-----------+
```

- `comm` (5 bit): 通信タイプ (CommType)
- `extra_data` (16 bit): コマンドごとに用途が異なる。要求側ではホスト ID やトルクフィードフォワードを格納し、応答側では `[ステータスビット(8) | デバイスID(8)]` を返す。
- `dev_id` (8 bit): 要求では宛先モータ ID、応答では宛先ホスト ID。

エンコード/デコード API:

| 関数                                                                | 役割                                                       |
| ------------------------------------------------------------------- | ---------------------------------------------------------- |
| `build_can_id(comm: CommType, extra: u16, dev: u8) -> u32`          | enum 経由で 29-bit ID を組み立てる                         |
| `build_can_id_raw(comm_u5: u8, extra: u16, dev: u8) -> u32`         | 解析済み整数から再構築する場合に使用                       |
| `parse_can_id(id: u32) -> (u8, u16, u8)`                            | `(comm_type, extra_data, device_id)` を取り出す            |

---

## 4. CommType (5-bit)

| 値 | 名称 (`CommType`)    | 方向    | 説明                                                          |
| -- | -------------------- | ------- | ------------------------------------------------------------- |
| 0  | `GetDeviceId`        | Host→M  | デバイス情報取得 (応答に MCU UUID 8byte)                      |
| 1  | `OperationControl`   | Host→M  | MIT モード制御 (pos/vel/kp/kd in data, torque in extra_data)  |
| 2  | `OperationStatus`    | M→Host  | モータステータス応答 (pos/vel/torque/temperature)             |
| 3  | `Enable`             | Host→M  | モータ有効化                                                  |
| 4  | `Disable`            | Host→M  | モータ無効化 (フリーラン)                                     |
| 6  | `SetZeroPosition`    | Host→M  | 現在位置を機械的ゼロに設定 (data[0] = 1)                       |
| 7  | `SetDeviceId`        | Host→M  | デバイス CAN ID 変更                                          |
| 17 | `ReadParameter`      | Host↔M  | パラメータ読み出し                                            |
| 18 | `WriteParameter`     | Host→M  | パラメータ書き込み (f32 / i8 / 他)                             |
| 21 | `FaultReport`        | M→Host  | 故障通知                                                      |
| 22 | `SaveParameters`     | Host→M  | 全パラメータを Flash に保存                                   |
| 23 | `SetBaudrate`        | Host→M  | CAN ボーレート変更                                            |
| 24 | `ActiveReport`       | M→Host  | 自発ステータス通知                                            |
| 25 | `SetProtocol`        | Host→M  | プロトコル種別の切り替え                                      |

実装は [`crates/robstride-protocol/src/comm_type.rs`](../crates/robstride-protocol/src/comm_type.rs)。

---

## 5. RunMode (制御モード)

| 値 | 名称 (`RunMode`) | 説明                                                                                        |
| -- | ---------------- | ------------------------------------------------------------------------------------------- |
| 0  | `Mit`            | MIT モード。`OperationControl` フレームで pos / vel / kp / kd / τ_ff を一発指令              |
| 1  | `Position`       | 位置制御モード。目標位置を `LocRef` パラメータに書き込む                                    |
| 2  | `Velocity`       | 速度制御モード。目標速度を `SpdRef` パラメータに書き込む                                    |
| 3  | `Torque`         | トルク (電流) 制御モード。目標 Iq を `IqRef` パラメータに書き込む                            |

切替は `WriteParameter` で `ParamIndex::RunMode` (= `0x7005`) に i8 を書き込みます (`build_run_mode_frame`)。

---

## 6. MIT モード値エンコード

MIT モード制御では、固定スケールの 16-bit 値 (`u16`) を介して連続値をやり取りします。

### 6.1 符号付き量 (position / velocity / torque)

範囲 `[-scale, +scale]` を `[0x0000, 0xFFFF]` にマップし、`0x7FFF` を 0 として扱います。

```text
encode: u16 = clamp((value / scale + 1.0) * 0x7FFF, 0, 0xFFFF)
decode: f32 = (raw / 0x7FFF - 1.0) * scale
```

### 6.2 符号無し量 (kp / kd)

範囲 `[0, scale]` を `[0x0000, 0xFFFF]` にマップ。

```text
encode: u16 = clamp((value / scale) * 0xFFFF, 0, 0xFFFF)
decode: f32 = (raw / 0xFFFF) * scale
```

### 6.3 モデル別スケーリング表 (`MitScales::for_model`)

| モデル | position [rad] | velocity [rad/s] | torque [Nm] | kp     | kd    |
| ------ | -------------- | ---------------- | ----------- | ------ | ----- |
| RS-00  | ±4π           | ±50.0           | ±17.0      | 500.0  | 5.0   |
| RS-01  | ±4π           | ±44.0           | ±17.0      | 500.0  | 5.0   |
| RS-02  | ±4π           | ±44.0           | ±17.0      | 500.0  | 5.0   |
| RS-03  | ±4π           | ±50.0           | ±60.0      | 5000.0 | 100.0 |
| RS-04  | ±4π           | ±15.0           | ±120.0     | 5000.0 | 100.0 |
| **RS-05 (Edulite05)** | ±4π | ±33.0       | ±17.0      | 500.0  | 5.0   |
| RS-06  | ±4π           | ±20.0           | ±60.0      | 5000.0 | 100.0 |

`MotorModel::from_name()` は以下のエイリアスを受け付けます (大文字小文字は区別しません):

- `RS-00`, `RS-01`, ..., `RS-06`
- `RS00`, `RS01`, ..., `RS06`
- `Edulite01` ↔ `RS-01`
- `Edulite02` ↔ `RS-02`
- `Edulite05` ↔ `RS-05`

---

## 7. フレームペイロード詳細 (8 バイト固定)

すべての要求/応答フレームはデータ部 8 バイト固定です。`robstride-protocol` のビルダは `[u8; 8]` を返し、`alloc` を要求しません。

### 7.1 Ping (`GetDeviceId`)

| 要素      | 値                                         |
| --------- | ------------------------------------------ |
| comm      | 0                                          |
| extra     | host_id (16-bit にゼロ拡張)                |
| dev       | 対象モータ ID                              |
| data[0..8]| すべて 0                                   |
| 応答      | `comm=0`, data に MCU UUID (8 バイト相当)  |

### 7.2 Enable / Disable

| 要素      | Enable               | Disable               |
| --------- | -------------------- | --------------------- |
| comm      | 3                    | 4                     |
| extra     | host_id              | host_id               |
| dev       | 対象モータ ID        | 対象モータ ID         |
| data      | 8 バイトすべて 0     | 8 バイトすべて 0      |
| 応答      | `OperationStatus`     | `OperationStatus`     |

### 7.3 SetZeroPosition

| 要素   | 値                                                   |
| ------ | ---------------------------------------------------- |
| comm   | 6                                                    |
| extra  | host_id                                              |
| dev    | 対象モータ ID                                        |
| data[0]| `0x01` (機械ゼロ設定指示)                            |
| data[1..]| 0                                                  |

応答は仕様上即座に返らないため、ドライバ側では送信後に短い `sleep(50 ms)` を挟みます。

### 7.4 OperationControl (MIT モード)

| 要素     | 値                                                                |
| -------- | ----------------------------------------------------------------- |
| comm     | 1                                                                 |
| extra    | `encode_mit_signed(torque, scales.torque)` (16-bit)               |
| dev      | 対象モータ ID                                                     |
| data[0..2] | `pos_u16` (big-endian)                                          |
| data[2..4] | `vel_u16` (big-endian)                                          |
| data[4..6] | `kp_u16`  (big-endian)                                          |
| data[6..8] | `kd_u16`  (big-endian)                                          |
| 応答     | `OperationStatus` (1 フレーム)                                    |

> ポイント: トルクフィードフォワードは **CAN ID の `extra_data` 側** に乗ります。data 部 8 バイトを使い切っているためです。

### 7.5 ReadParameter

| 要素     | 値                                                          |
| -------- | ----------------------------------------------------------- |
| comm     | 17                                                          |
| extra    | host_id                                                     |
| dev      | 対象モータ ID                                               |
| data[0..2] | `param_index` (little-endian, `u16`)                      |
| data[2..8] | 0                                                         |
| 応答     | comm=17, data[0..2] = index, data[4..8] = `f32` (LE)        |

### 7.6 WriteParameter (f32)

| 要素     | 値                                                          |
| -------- | ----------------------------------------------------------- |
| comm     | 18                                                          |
| extra    | host_id                                                     |
| dev      | 対象モータ ID                                               |
| data[0..2] | `param_index` (LE)                                        |
| data[2..4] | 予約 (0)                                                  |
| data[4..8] | 値 `f32` (LE)                                             |
| 応答     | 機種により無応答 / `OperationStatus` 等。ドライバは 5 ms 待機後リターン |

### 7.7 WriteParameter (i8)

`RunMode` 切替や 1 バイト系パラメータに使用。

| 要素     | 値                                  |
| -------- | ----------------------------------- |
| comm     | 18                                  |
| extra    | host_id                             |
| dev      | 対象モータ ID                       |
| data[0..2] | `param_index` (LE)                |
| data[2..4] | 予約                              |
| data[4]    | 値 (`i8` を `u8` キャスト)        |
| data[5..8] | 0                                 |

### 7.8 OperationStatus / FaultReport の応答パース

| フィールド           | 範囲 / 形式                                                                  |
| -------------------- | ---------------------------------------------------------------------------- |
| `extra_data` 上位 8b | ステータスビット (詳細下表)                                                  |
| `extra_data` 下位 8b | モータの自分自身の ID                                                        |
| `data[0..2]`         | `pos_u16` (BE) — `decode_mit_signed` で position [rad]                       |
| `data[2..4]`         | `vel_u16` (BE) — velocity [rad/s]                                            |
| `data[4..6]`         | `torque_u16` (BE) — torque [Nm]                                              |
| `data[6..8]`         | `temp_u16` (BE) — temperature [°C] = `temp_u16 * 0.1`                        |

ステータスビット (`extra_data` 上位 8b の各ビット):

| ビット | フィールド                | 意味                          |
| ------ | ------------------------- | ----------------------------- |
| 15..14 | `mode` (2 bit)            | 現在の RunMode                |
| 13     | `uncalibrated`            | エンコーダ未キャリブレーション|
| 12     | `stall`                   | スタール検知                  |
| 11     | `magnetic_encoder_fault`  | 磁気エンコーダ異常            |
| 10     | `overtemperature`         | 過温度                        |
| 9      | `overcurrent`             | 過電流                        |
| 8      | `undervoltage`            | 低電圧                        |

`parse_status_frame()` は `comm_type ∈ {OperationStatus(2), FaultReport(21)}` のフレームのみ受理します。

---

## 8. ParamIndex

| 名称              | 値       | 型   | 説明                                       |
| ----------------- | -------- | ---- | ------------------------------------------ |
| `MechOffset`      | `0x2005` | f32  | 機械原点オフセット                         |
| `MeasuredPosition`| `0x3016` | f32  | 計測位置                                   |
| `MeasuredVelocity`| `0x3017` | f32  | 計測速度                                   |
| `MeasuredTorque`  | `0x302C` | f32  | 計測トルク                                 |
| `RunMode`         | `0x7005` | i8   | 制御モード切替                             |
| `IqRef`           | `0x7006` | f32  | Iq 目標値 (トルクモード)                   |
| `SpdRef`          | `0x700A` | f32  | 速度目標値 (速度モード)                    |
| `LimitTorque`     | `0x700B` | f32  | トルク上限                                 |
| `CurKp`           | `0x7010` | f32  | 電流ループ Kp                              |
| `CurKi`           | `0x7011` | f32  | 電流ループ Ki                              |
| `CurFiltGain`     | `0x7014` | f32  | 電流フィルタゲイン                         |
| `LocRef`          | `0x7016` | f32  | 位置目標値 (位置モード)                    |
| `LimitSpd`        | `0x7017` | f32  | 速度上限 (位置モード時)                    |
| `LimitCur`        | `0x7018` | f32  | 電流上限                                   |
| `MechPos`         | `0x7019` | f32  | 機械角位置                                 |
| `IqFilt`          | `0x701A` | f32  | Iq フィルタ後                              |
| `MechVel`         | `0x701B` | f32  | 機械角速度                                 |
| `Vbus`            | `0x701C` | f32  | バス電圧                                   |
| `LocKp`           | `0x701E` | f32  | 位置ループ Kp                              |
| `SpdKp`           | `0x701F` | f32  | 速度ループ Kp                              |
| `SpdKi`           | `0x7020` | f32  | 速度ループ Ki                              |
| `SpdFiltGain`     | `0x7021` | f32  | 速度フィルタゲイン                         |
| `AccRad`          | `0x7022` | f32  | 加速度 [rad/s²]                            |
| `VelMax`          | `0x7024` | f32  | 最大速度                                   |
| `AccSet`          | `0x7025` | f32  | 加速度設定値                               |
| `CanTimeout`      | `0x7028` | f32  | CAN タイムアウト                           |
| `ZeroState`       | `0x7029` | f32  | ゼロ状態                                   |

---

## 9. ドライバ API (`robstride` クレート)

### 9.1 `Motor` 構造体

```rust
pub struct Motor {
    /* private */
}
```

| メソッド                                                                                                                  | 戻り値                       | 役割                                         |
| ------------------------------------------------------------------------------------------------------------------------- | ---------------------------- | -------------------------------------------- |
| `Motor::open(interface, motor_id, model)`                                                                                 | `Result<Motor>`              | デフォルトホスト ID で開く                   |
| `Motor::open_with_host(interface, motor_id, host_id, model)`                                                              | `Result<Motor>`              | 任意のホスト ID で開く                       |
| `set_timeout(timeout)`                                                                                                    | `Result<()>`                 | 受信タイムアウト変更                         |
| `motor_id()` / `host_id()` / `model()` / `scales()`                                                                       | アクセサ                     |                                              |
| `is_enabled()` / `current_run_mode()`                                                                                     | 内部状態                     |                                              |
| `ping()`                                                                                                                  | `Result<(u16, Vec<u8>)>`     | GetDeviceId 送信 → `(extra, payload)`        |
| `enable()` / `disable()`                                                                                                  | `Result<MotorFeedback>`      | 有効化 / 無効化                              |
| `set_zero()`                                                                                                              | `Result<()>`                 | 現在位置を機械ゼロに                          |
| `set_run_mode(mode)`                                                                                                      | `Result<()>`                 | RunMode 切替                                 |
| `mit_control(pos, vel, kp, kd, torque)`                                                                                   | `Result<MotorFeedback>`      | MIT モード 1 ショット                         |
| `set_position(pos)` / `set_position_speed_limit(speed)` / `set_torque_limit(τ)` / `set_current_limit(i)`                 | `Result<()>`                 | 位置モード関連                                |
| `set_velocity(v)` / `set_torque(iq)`                                                                                      | `Result<()>`                 | 速度 / トルクモード関連                       |
| `read_param(param)`                                                                                                       | `Result<f32>`                | パラメータ読み出し                            |
| `write_param_f32(param, value)`                                                                                           | `Result<()>`                 | f32 パラメータ書き込み                        |
| `read_status()`                                                                                                           | `Result<MotorFeedback>`      | ゼロ振幅 MIT で状態のみ取得                  |
| `read_position()` / `read_velocity()` / `read_current()` / `read_vbus()`                                                  | `Result<f32>`                | 個別ショートカット                            |

### 9.2 `Drop` 動作

`Motor` が `enabled == true` の状態で drop されたときは、自動的に `Disable` フレームを送信します。CLI で「enable したまま終了したい」場合は `std::mem::forget(motor)` で抑止できます。

### 9.3 バススキャン

```rust
pub fn scan_bus(
    interface: &str,
    host_id: u8,
    id_range: RangeInclusive<u8>,
    timeout_per_id: Duration,
    on_progress: Option<&mut dyn FnMut(usize, usize, u8)>,
) -> Result<Vec<ScanResult>>;

pub fn dump_bus(interface: &str, duration: Duration) -> Result<Vec<(u32, Vec<u8>)>>;
```

- `scan_bus` は ID ごとに `GetDeviceId` を投げ、応答ペイロードを `ScanResult { motor_id, payload }` で返します。
- `on_progress` を渡すと進捗 (`idx`, `total`, `motor_id`) をリアルタイム通知します。
- `dump_bus` は指定時間バスを受動監視して全フレームを返します。

### 9.4 エラー型

```rust
#[derive(thiserror::Error, Debug)]
pub enum Error {
    CanSocket(#[from] std::io::Error),
    Timeout       { motor_id: u8 },
    InvalidResponse(String),
    MotorFault    { motor_id: u8, extra_data: u16 },
    NotEnabled    { motor_id: u8 },
    InvalidFrame(&'static str),
}
```

`MotorFault` の `extra_data` には `FaultReport` 受信時の生 16-bit 値が入っているので、ビット展開して詳細を確認できます。

---

## 10. CLI 仕様 (`robstride-cli`)

### 10.1 グローバルオプション

| フラグ              | 短縮 | デフォルト   | 役割                                  |
| ------------------- | ---- | ------------ | ------------------------------------- |
| `--interface`       | `-i` | `can0`       | SocketCAN インターフェース名          |
| `--motor-id`        | `-m` | `1`          | 対象モータ CAN ID                     |
| `--host-id`         |      | `0xFD` (253) | ホスト CAN ID                         |
| `--model`           |      | `Edulite05`  | モータモデル (`RS-XX` / `EduliteXX`)  |

### 10.2 サブコマンド

| サブコマンド                  | 説明                                                              |
| ----------------------------- | ----------------------------------------------------------------- |
| `status`                      | 1 回ステータス取得                                               |
| `enable`                      | 有効化 (ドライバ Drop による自動 disable を抑止)                  |
| `disable`                     | 無効化                                                           |
| `set-zero`                    | 現在位置を機械ゼロに                                             |
| `move-to <pos> [--speed --tolerance --timeout]` | 位置モードで指定 rad へ移動 (到達 or タイムアウトで停止)          |
| `spin <vel> [--duration]`     | 速度モードで連続回転 (Ctrl-C / duration 経過で停止)               |
| `torque <τ> [--duration]`     | トルクモードで電流指令 (Ctrl-C / duration 経過で停止)             |
| `mit --pos --vel --kp --kd --torque` | MIT モードで 1 ショット指令                                |
| `scan [--from --to --timeout]` | ID 1..32 (デフォルト) を ping して応答を列挙                      |
| `dump [--duration]`           | バスをパッシブ監視                                               |
| `monitor [--interval]`        | 周期的にステータスを表示                                         |

### 10.3 出力フォーマット

`print_feedback` は以下の 1 行を出力します:

```text
status: motor=  1 pos=+0.123 rad  vel=+0.000 rad/s  τ=+0.000 Nm  T=24.5°C  mode=1
```

ステータスビットでフラグが立っていると 2 行目に `flags: ...` が追加されます。

### 10.4 シグナル処理

`spin` / `torque` / `monitor` / `move-to` は `ctrlc` クレートで Ctrl-C を捕捉し、`AtomicBool` を経由してループを抜けます。`spin` / `torque` 終了時は必ず `disable()` を呼びます。

---

## 11. タイミング / リトライ

| 項目                        | 値          | 出典                                    |
| --------------------------- | ----------- | --------------------------------------- |
| 受信タイムアウト (default) | 100 ms      | `DEFAULT_TIMEOUT`                       |
| `set_zero` 後ウェイト       | 50 ms       | sandbox 実装の経験値                    |
| `set_run_mode` 後ウェイト   | 10 ms       | RunMode 反映待ち                        |
| `write_param_f32` 後ウェイト| 5 ms        | 連続書き込みでバッファ詰まり防止        |

これらは現状ハードコードです。実機検証で必要があれば調整可能にします (例: `Motor::set_post_write_delay(...)` の追加など)。

---

## 12. 既知の制限・未実装

- 同期 API のみ。`tokio` 等 async ランタイムへのアダプタは未提供。
- マルチドロップ送信 (1 フレームで複数モータ駆動) は未対応。Robstride にブロードキャスト ID 仕様があるかは要確認。
- 実機 (Edulite05) でのソークテスト未実施。`robstride_sandbox` 側がハードウェア検証済みのリファレンス。
- `SetDeviceId` / `SetBaudrate` / `SaveParameters` / `SetProtocol` / `ActiveReport` 等の管理系コマンドはプロトコル enum には載っているが、ドライバ層のヘルパは未実装。

---

## 13. 参考実装 / 出典

- `robstride_sandbox` ([../../robstride_sandbox/](../../robstride_sandbox/)): 当 crate のロジック元になった実機検証済み sandbox 実装。
- `lkmotor-rs` ([../../lkmotor-rs/](../../lkmotor-rs/)): 同じく sibling crate。protocol/driver の 2 層構成パターンを踏襲している。
- Robstride 公式プロトコル仕様書 (公開 PDF): CommType / ParamIndex / MIT スケール表の一次出典。

