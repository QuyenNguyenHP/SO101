# Hand follow: cánh tay bắt chước bàn tay

Script: `hand_follow.py`. Camera: RealSense D415. Nhận diện tay: MediaPipe (`models/hand_landmarker.task`).

## Bàn tay → robot
| Bàn tay | Robot |
|---|---|
| Di chuyển (trái/phải, lên/xuống, xa/gần camera) | Đầu kẹp di chuyển tương ứng, tỉ lệ 1:1 |
| Nắm lại | Gripper kẹp lại |
| Mở ra | Gripper mở. Mở một nửa thì gripper cũng mở một nửa |
| Xoay lòng bàn tay (vặn cổ tay quanh trục ngón tay) | `wrist_roll` xoay đúng góc đó |

- Vị trí lấy ở **gốc lòng bàn tay** (gần cổ tay), là chỗ không bị ngón tay che khi nắm lại, nên nắm tay không làm robot giật.
- Robot bám theo cả khi tay đang nắm, nhờ vậy có thể kẹp đồ rồi mang đi.
- Hướng trục giống `marker_follow.py`: camera nhìn cùng hướng với robot. Dùng `--mirror` để đảo trái/phải.

## Chạy
```bash
conda activate lerobot
cd ~/robot/SO101
python hand_follow.py --dry     # kiểm tra hướng trước, robot không chạy
python hand_follow.py           # robot thật
```
Nhấn **SPACE**: robot ra START (đầu kẹp cách 20 cm về phía trước, cao 15 cm, gripper mở). Đưa **bàn tay mở** ra trước camera để lấy mốc, sau đó robot bắt đầu bám theo.

Phím: `SPACE` lấy mốc lại · `p` tạm dừng/chạy tiếp · `h` về START · `q` thoát (về HOME trước).

## Hiệu chỉnh (xem các số trên màn hình)
| Vấn đề | Cách sửa |
|---|---|
| Wrist xoay ngược chiều | `--roll-sign -1` |
| Không muốn wrist xoay | `--no-roll` |
| Nắm tay mà gripper chưa đóng hết | Đọc số `ratio` lúc nắm tay, đặt `--fist` cao hơn số đó một chút (mặc định 1.2) |
| Xòe tay mà gripper chưa mở hết | Đọc số `ratio` lúc xòe, đặt `--open` thấp hơn số đó một chút (mặc định 1.75) |
| Rung / trễ | Như `marker_follow.py`: `--mincutoff`, `--beta`, `--speed` |

## Đã kiểm tra (offline, chưa có camera/robot)
- Toàn bộ vòng lặp chạy với camera và bàn tay giả. Tay sang phải 6 cm, lên 4 cm, ra xa 5 cm → đầu kẹp sang phải 60, lên 40, tiến 50 mm.
- Nắm tay: gripper đóng, đầu kẹp không xê dịch.
- Xoay lòng bàn tay 40° → wrist_roll xoay 40°.
- Ngưỡng `ratio` mới ước lượng trên bàn tay mô phỏng (xòe ≈ 1.85, nắm ≈ 0.8). Cần xem lại với tay thật.
