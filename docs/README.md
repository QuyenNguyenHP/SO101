# SO101 — các script thường dùng

```bash
conda activate lerobot
cd ~/robot/SO101
```

## Bốn chương trình chính

| Thứ tự | Chương trình | Chạy nhanh | Công dụng |
|---|---|---|---|
| 1 | [hand_over.py](../hand_over.py) | `python hand_over.py` | Gắp vật và đưa vào lòng bàn tay; SPACE bắt đầu/tạm dừng, s dừng tay, q thoát |
| 2 | [camera_pick.py](../camera_pick.py) | `python camera_pick.py` | Click vật → click chỗ đặt → SPACE để chạy |
| 3 | [table_calib.py](../table_calib.py) | `python table_calib.py` | Chạm đầu hàm cố định vào dấu X → SPACE → đưa tay ra → click dấu X; ít nhất 6 cặp rồi c để tính và lưu |
| 4 | [roll_check.py](../roll_check.py) | `python roll_check.py` | Kiểm tra hướng kẹp, a/d chỉnh cổ tay, o lưu offset |

`roll_check.py` là chương trình kiểm tra **wrist roll** được nhắc tới là “row check”. Thứ tự trên là thứ tự ưu tiên sử dụng; khi thiết lập lại, hiệu chuẩn servo trước, rồi table calibration → kiểm tra bằng `Reference/verify_click.py` → roll check → camera pick/handover.

Các chương trình trên điều khiển tay thật. Đỡ tay trước khi tắt torque theo lời nhắc. Với `hand_over.py`, SPACE bắt đầu vòng lặp tự động; với `roll_check.py`, click vật có thể làm tay di chuyển. Có thể thêm `--dry` để dùng tay giả, nhưng vẫn cần camera; không dùng dữ liệu tay giả để lưu hiệu chuẩn thật.

## Script hỗ trợ trong `Reference/`

| Script | Vai trò |
|---|---|
| `Reference/snap_target.py` | Chụp ảnh vật mẫu cho handover, lưu trong `targets/` |
| `Reference/record_home.py` | Ghi tư thế home dùng chung |
| `Reference/goto.py` | Đưa đầu kẹp tới tọa độ để kiểm tra vị trí |
| `Reference/verify_click.py` | Kiểm tra hiệu chuẩn bằng cách click dấu trên bàn; cung cấp hàm xử lý tọa độ camera |
| `Reference/so101_real.py` | Kết nối robot, đổi đơn vị, giới hạn khớp, nạp offset và home |
| `Reference/keyboard_control.py` | Điều khiển bàn phím và các hàm IK dùng chung |
| `Reference/pick_place.py` | Lập kế hoạch gắp/đặt và các hàm chuyển động dùng chung |
| `Reference/detector.py` | Tìm vật và che robot khỏi ảnh |
| `Reference/handeye_calib.py` | Cung cấp `open_camera` cho các chương trình chính; cũng có quy trình hiệu chuẩn marker |

Thư mục chính chỉ còn bốn script Python ở bảng trên. Các chương trình tự nạp module dùng chung từ `Reference/`.

Các file JSON nằm trong `json/` (gồm hiệu chuẩn, home, mẫu đo và bản sao lưu trong `json/debug/`). Tài liệu Markdown nằm trong `docs/`. Các file `.npy`, `models/`, `targets/` và ảnh/log trong `debug/` giữ nguyên vị trí. Lệnh trong tài liệu được chạy từ thư mục `~/robot/SO101`.

## Reference

Toàn bộ script hỗ trợ, module dùng chung, thiết lập phần cứng và demo được gom vào [Reference/](REFERENCE.md). Chạy từ thư mục dự án, ví dụ:

```bash
python Reference/scan_motors.py
python Reference/real_twin.py
```

Hướng dẫn: [cài đặt servo](SETUP_ROBOT.md), [hiệu chuẩn bàn](SETUP_TABLE_2D_CALIB.md), [cách dùng camera pick và table calibration](NHAT_KY_2026-11-21_camera_pick.md#13-cách-dùng-nhanh).

## Dữ liệu NumPy

- `T_base_cam.npy`: giữ tại thư mục chính; handover và camera pick đọc ma trận hiệu chuẩn này.
- `npy/`: các kế hoạch `square_plan.npy`, `wave_plan.npy`, `pick_place_plan.npy`, bản sao `T_base_cam_handeye.npy` và các bản hiệu chuẩn cũ trong `npy/debug/`. Các file này không cần để chạy handover/camera pick.
