# How to make a drone

If anything wrong please contact [1107@siwg.top](mailto:1107@siwg.top).

Stay safe, have fun!

## Disclaimer

All content provided in this note(*How to Make a Drone*) is for learning, communication, and sharing purposes only, and should be used for reference only. Readers may use the content of this note for personal study, research, appreciation, and other non-commercial or non-profit purposes. At the same time, readers must comply with applicable open-source licenses and must not infringe upon the legitimate rights and interests of the note author or any related rights holders.

This note does not guarantee the completeness, accuracy, or timeliness of its content.

For the convenience of readers, this note may contain links to third-party websites. The content of such links does not represent the views of this note. By clicking these links, you may leave this note and enter third-party websites. This note and its author are not responsible for any risks, losses, or damages arising from third-party websites.

Some operations described in this note may involve safety risks. The note and its author shall not be held responsible for any injury, damage, or loss caused by improper operation or otherwise. Please proceed with caution and pay close attention to safety.

By continuing to read this note, you acknowledge and accept the above disclaimer and agree to assume all associated risks yourself.

The final interpretation of the above statements, within the limits permitted by law, belongs to this note and its author.

## Contributor

Author: 1107

## Content

1. Preparation for material
2. Tools&Techs for assemble
3. INS (Inertial Navigation System) design
4. FireflyLink Communication Protocol

## Preparation for Material

### 1.0 Introduction

This chapter introduces the option for material and MCU, aiming to find cheap, practial and fit suitable for the usage scenario material, as ~~**I am poor**~~



### 1.1 Options and Purchase for material

### Electronic Speed Controler(ECS)

We need ECS for transferring the control signal(Usually PWM) into the control of current in order to control the motor rotation.

**How do we Obtain parameter of ECS?**

Manufacturers usually provide

- Rated Voltage
- Continuous current
- BEC output
- ......

> RPM
> Voltage
> Current
> Temperature
> Consumption

### Brushless Mortor

Comparing to Brush Mortor, 

## INS (Inertial Navigation System) design

### MEKF — Multiplicative Extended Kalman Filter

The drone uses an MEKF to estimate 3D attitude by fusing gyroscope and accelerometer data. The state is a **quaternion** `q = [x, y, z, w]` (rotation from inertial to body frame), and a **3×3 covariance matrix `P`** over the attitude error vector. This avoids the singularity of a 4×4 quaternion covariance under the unit-norm constraint.

**Files:** `KalmanFilter.c` / `KalmanFilter.h`, quaternion math in `quaternion.c` / `quaternion.h`.

---

#### Data Structures

```c
typedef struct {
    double w, x, y, z;
} Quaternion;

typedef struct {
    Quaternion q;      // attitude quaternion
    double P[3][3];    // 3×3 error-state covariance
} MEKF;
```

#### Initialization: `mekf_init`

- `q = [0, 0, 0, 1]` — identity rotation (level, nose forward)
- `P = diag(1e-4, 1e-4, 1e-4)` — small initial uncertainty

---

#### Utility Functions

**`cross_product(a, b, res)`** — 3D cross product: `res = a × b`.

**`skew(vec, res)`** — builds the 3×3 skew-symmetric matrix:
```
        [  0   -vz   vy ]
[v]_× = [  vz   0   -vx ]
        [ -vy   vx   0  ]
```
Encodes cross product as matrix multiply: `[a]_× · b = a × b`.

**`matrix3_inverse(A, invA)`** — closed-form 3×3 inverse via cofactor expansion. Returns `false` if `|det| < 1e-12`.

**`errvec2quat(vec, dquat)`** — exponential map from `so(3)` to `SO(3)`:
```
θ = |vec|
if θ < 1e-8:  dq = [vec_x/2, vec_y/2, vec_z/2, 1]   (small-angle approx)
else:         dq = [sin(θ/2)·vec/θ, cos(θ/2)]
```
Result is always normalized.

---

#### Predict Step: `mekf_predict(model, omega, dt, Q)`

**Gyro integration:**
```
φ = ω · dt                 (body-frame rotation over timestep)
dq = errvec2quat(φ)        (convert to quaternion)
q_new = dq ⊗ q             (left-multiply: body-frame increment applied to attitude)
Normalize(q_new)
```

`quatMultiply(dquat, model->q)` computes `dquat * q` — left-multiplication applies a body-frame rotation correctly.

**Covariance propagation:**
```
P = P + Q
```

Uses `F ≈ I` approximation (valid for short timesteps). Q should be scaled by `dt²`: `Q = σ_gyro² · dt² · I₃`. The full `-[ω]_× P - P [ω]_×ᵀ` term is omitted as a common simplification.

---

#### Update Step: `mekf_update(model, z, v_I, R)`

The caller normalizes the accelerometer `z` and uses `v_I = [0, 0, 1]` (gravity in NED).

**1. Predicted measurement:**
```
A(q) = rotation matrix from quaternion:
       [ 1-2(y²+z²)   2(xy-zw)     2(xz+yw)  ]
       [ 2(xy+zw)     1-2(x²+z²)   2(yz-xw)  ]
       [ 2(xz-yw)     2(yz+xw)     1-2(x²+y²) ]

vB = A(q) · v_I           (predicted gravity in body frame)
```

**2. Innovation:**
```
y = z - vB
```

**3. Measurement Jacobian:**
```
H = -[vB]_×
```
Derivation: `A(δq)·v_I ≈ vB + [vB]_×·δθ`, so `y = -[vB]_×·δθ + noise`.

**4. Kalman gain:**
```
S = H·P·Hᵀ + R
If |S| < 1e-12 → skip update
K = P·Hᵀ·S⁻¹
```

**5. Attitude correction:**
```
a = K · y                 (error rotation vector)
dq = errvec2quat(a)
q_new = dq ⊗ q
Normalize(q_new)
```

**6. Covariance update (Joseph form):**
```
P = (I - KH)·P·(I - KH)ᵀ  +  K·R·Kᵀ
```
Then explicitly symmetrized: `P[i][j] = P[j][i] = ½(P[i][j] + P[j][i])`.

The Joseph form guarantees symmetry and positive-definiteness under floating-point roundoff.

---

#### Combined Step: `MEKF_step`

```c
void MEKF_step(MEKF *model, const double omega[3], double dt,
               const double z[3], const double v_I[3],
               const double Q[3][3], const double R[3][3])
```

Convenience wrapper — calls `mekf_predict` then `mekf_update`. The caller in `AttitudeCtrl.c` uses the two functions separately to normalize the accelerometer in between.

---

#### Data Flow

```
MPU6050 → raw accel/gyro → scale to physical units
                                │
                 ┌──────────────┴──────────────┐
                 ▼                              ▼
         omega (rad/s)              accel (normalized)
                 │                              │
                 ▼                              ▼
         mekf_predict(q, P)          mekf_update(q, P)
                 │                              │
                 └──────────┬───────────────────┘
                            ▼
                  updated quaternion q
                            │
                            ▼
                 atan2 formulas → roll, pitch, yaw (°)
                            │
                            ▼
                    nRF24L01 telemetry
```

---

#### Tuning Parameters

| Parameter | Meaning | Typical Value |
|-----------|---------|---------------|
| `P_init` (diagonal) | Initial attitude uncertainty | `1e-4` |
| `Q` (diagonal) | Gyro process noise | `1e-6` ~ `1e-4` |
| `R` (diagonal) | Accelerometer measurement noise | `0.1` ~ `1.0` |

- **Higher Q** → trusts gyro more, smoother, slower convergence
- **Higher R** → trusts accelerometer less, less vibration-sensitive
- **Note:** Q should incorporate `dt²`; halving the loop period means quartering Q.

---

#### Euler Angle Extraction

From `AttitudeCtrl.c`, after MEKF produces `q`:

```
roll  = atan2(2(qw·qx + qy·qz), 1 - 2(qx² + qy²)) · 180/π
pitch = asin(2(qw·qy - qx·qz)) · 180/π
yaw   = atan2(2(qw·qz + qx·qy), 1 - 2(qy² + qz²)) · 180/π
```

The MEKF corrects roll and pitch via the gravity vector; yaw is unobservable from the accelerometer and relies on gyro integration alone (drift expected without a magnetometer).


## FireflyLink Communication Protocol

FireflyLink is a custom wireless protocol for the nRF24L01 module, featuring CRC16 error detection and Hamming(7,4) forward error correction to ensure reliable data transmission between drone and remote control.

**Files:** `FireflyLink.c` / `FireflyLink.h`

### Wire Format

nRF24L01 max payload = 32 bytes. FireflyLink packet structure:

```
Wire format (max 31 bytes for 13-byte payload):
[Header:1] [Seq:1] [Len:1] [Payload:N] [ECC:N] [CRC16:2]
  0xFD
```

| Field | Bytes | Description |
|-------|-------|-------------|
| Header | 1 | Fixed `0xFD`, identifies FireflyLink packet |
| Seq | 1 | Sequence number for drop detection |
| Len | 1 | Payload length N (max 13) |
| Payload | N | Data (attitude angles, battery, etc.) |
| ECC | N | Hamming(7,4) code, 1 ECC byte per payload byte |
| CRC16 | 2 | CRC16-CCITT over [Seq + Len + Payload + ECC] |

Max payload = 13 bytes: (32 − 5) / 2 = 13.

### Hamming(7,4) Encoding

Each payload byte is split into high nibble (bits[7:4]) and low nibble (bits[3:0]). Each nibble generates 3 parity bits:

- **Data bits:** 4 (d1 d2 d3 d4, mapping to nibble bits[3:0])
- **Parity bits:** 3 (p1 p2 p3)
  - p1 = d1 ⊕ d2 ⊕ d4
  - p2 = d1 ⊕ d3 ⊕ d4
  - p3 = d2 ⊕ d3 ⊕ d4

Both nibble parities are packed into 1 ECC byte:

```
ECC byte layout: [p1_hi p2_hi p3_hi p1_lo p2_lo p3_lo 0 0]
                 bit[5]  bit[4]  bit[3]  bit[2]  bit[1]  bit[0]
```

### Hamming(7,4) Decoding / Correction

The receiver computes a syndrome to locate and correct single-bit errors:

```
s1 = p1_received ⊕ p1_calculated
s2 = p2_received ⊕ p2_calculated
s3 = p3_received ⊕ p3_calculated
pos = (s3 << 2) | (s2 << 1) | s1
```

Syndrome-to-position mapping:

| s3:s2:s1 | Codeword Position | Action |
|----------|-------------------|--------|
| 000 | No error | — |
| 001 / 010 / 100 | Parity bit error | Data correct, no fix needed |
| 011 | d1 (bit 3) | Flip data bit 3 |
| 101 | d2 (bit 2) | Flip data bit 2 |
| 110 | d3 (bit 1) | Flip data bit 1 |
| 111 | d4 (bit 0) | Flip data bit 0 |

### Data Flow

#### Transmit Side

```
Sensor data → fireflylink_init → fireflylink_pack → fireflylink_serialize → nrf24_send
                                     │                      │
                                     ▼                      ▼
                              Compute ECC + CRC      Serialize to wire bytes
```

#### Receive Side

```
nrf24_recv → fireflylink_parse → fireflylink_unpack → Application data
                                      │
                                      ▼
                              CRC check → Hamming repair → CRC recheck
```

### Error Correction Flow

1. Compute CRC16 of received data, compare against received CRC
2. CRC match → data intact, fast-path return (`FIREFLYLINK_OK`)
3. CRC mismatch → attempt Hamming single-bit correction
4. Recompute CRC after correction
5. CRC matches → correction successful, return number of bits corrected
6. Still mismatched → unrecoverable, return `FIREFLYLINK_ERR_CRC`

### API Reference

| Function | Description |
|----------|-------------|
| `fireflylink_init(pkt, seq)` | Initialize packet, zero-fill and set sequence number |
| `fireflylink_pack(pkt, data, len)` | Pack data, compute ECC + CRC |
| `fireflylink_unpack(pkt, data_out, len_out)` | Unpack: CRC verify first, then Hamming repair on failure |
| `fireflylink_serialize(pkt, buf, size)` | Serialize to wire-format byte buffer |
| `fireflylink_parse(buf, size, pkt)` | Parse wire-format bytes into packet struct |
| `fireflylink_crc16(data, len)` | Direct CRC16-CCITT computation |
| `fireflylink_wire_size(pkt)` | Get wire-format byte count |

### Status Codes

| Return | Meaning |
|--------|---------|
| ≥ 0 | Success (unpack returns bits corrected) |
| `FIREFLYLINK_ERR_CRC (-1)` | CRC mismatch, data unrecoverable |
| `FIREFLYLINK_ERR_LEN (-2)` | Invalid payload length |
| `FIREFLYLINK_ERR_HEAD (-3)` | Wrong wire-format header byte |
| `FIREFLYLINK_ERR_SIZE (-4)` | Buffer size insufficient |
