# Marker follow: cánh tay bám theo marker (version 1)

Script: `marker_follow.py`. Marker dùng: ArUco **DICT_4X4_50, ID 0**. Camera: RealSense D415.

## Nguyên lý
- Robot về **HOME** (tư thế gập trong `home_pose.json`), rồi sang tư thế **START**: đầu kẹp cách 20 cm về phía trước, cao 15 cm, chúi xuống 45°. HOME gập gần như không tiến lên được nên không dùng làm mốc.
- Nhấn **SPACE** khi robot đã ở START: vị trí 3D hiện tại của marker được lấy làm **mốc**.
- Từ đó: `đầu kẹp = START + (marker hiện tại − marker mốc)`, tỉ lệ 1:1.
- Không cần biết camera đặt ở đâu, chỉ cần camera **nhìn cùng hướng với robot**:

| Marker (theo góc nhìn của camera) | Đầu kẹp robot |
|---|---|
| Ra xa camera | Tiến tới |
| Sang phải trong ảnh | Sang phải (nhìn từ sau robot) |
| Lên trên trong ảnh | Lên |

Nếu bạn muốn chiều trái/phải ngược lại (kiểu soi gương), thêm `--mirror`.

## Chạy
```bash
conda activate lerobot
cd ~/robot/SO101
python marker_follow.py --dry     # bước 1: chỉ camera + tính toán, robot không chạy
python marker_follow.py           # bước 2: robot thật
```

Bước 1 (`--dry`): nhấn SPACE, đưa marker lần lượt theo từng hướng và kiểm tra dòng `move fwd / left / up` có đúng dấu không.

## Phím (bấm vào cửa sổ ảnh trước)
| Phím | Tác dụng |
|---|---|
| SPACE | Về START, lấy mốc, bắt đầu bám. Nhấn lại để lấy mốc mới |
| p | Tạm dừng / chạy tiếp |
| h | Dừng và về START |
| q | Thoát (robot về HOME trước) |

## Màn hình
- Khung **viền đỏ** + `OUT OF REACH`: marker đã ra ngoài vùng với tới. Đầu kẹp dừng ở điểm xa nhất với được trên đường tới đích, và không bao giờ thấp hơn 1 cm so với mặt bàn.
- `MARKER LOST`: không thấy marker, robot đứng yên.
- `(tilt held)`: đầu kẹp giữ nguyên góc chúi 45°. `(tilt free)`: góc chúi phải thả tự do để với tới điểm đó.

## Độ mượt (chỉnh nếu cần)
| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `--mincutoff` | 1.0 | Nhỏ hơn: mượt hơn khi đứng yên, nhưng trễ hơn |
| `--beta` | 4.0 | Lớn hơn: ít trễ hơn khi di chuyển nhanh |
| `--speed` | 0.15 | Tốc độ tối đa của đầu kẹp (m/s) |

Ngoài ra mỗi khớp bị giới hạn tối đa 2°/lệnh (60°/s).

## Đã kiểm tra (offline, chưa có camera/robot)
- Từ START: với được khoảng ±30 cm trái/phải, lên 30 cm, xuống 13 cm, lùi 4–5 cm (theo mô hình MuJoCo).
- Nhiễu độ sâu 3 mm → sau lọc còn khoảng 0,9 mm. Khi marker vẽ vòng tròn 10 cm, sai lệch bám tối đa khoảng 9 mm.
- Chạy toàn bộ vòng lặp với camera giả: marker sang phải 8 cm → đầu kẹp sang phải 80 mm.
