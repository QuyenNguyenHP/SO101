# Script tham khảo

Chạy các lệnh từ `~/robot/SO101`, trong môi trường `lerobot`. Các script này vẫn dùng được; script kết nối robot có thể bật/tắt torque hoặc di chuyển tay theo quy trình riêng.

| Script | Công dụng |
|---|---|
| `scan_motors.py` | Dò ID servo trên bus |
| `set_one_motor.py` | Đặt ID cho servo |
| `test_gripper.py` | Thử chuyển động tay gắp bằng đơn vị thô |
| `real_twin.py` | Xoay tay thật bằng tay, xem mô hình mô phỏng theo sau |
| `marker_check.py` | Chẩn đoán nhận diện marker ArUco; vẫn lưu ảnh vào `debug/` ở thư mục dự án |
| `make_square_plan.py` | Tạo `npy/square_plan.npy` tại thư mục dự án |
| `make_wave_plan.py` | Tạo `npy/wave_plan.npy` tại thư mục dự án |
| `real_play.py` | Chạy kế hoạch trên robot thật; mặc định đọc `npy/square_plan.npy` tại thư mục dự án |
| `sim_to_real.py` | Điều khiển tay thật từ thanh trượt MuJoCo hoặc kế hoạch |

Ví dụ tạo và dùng kế hoạch (lệnh cuối điều khiển tay thật):

```bash
python Reference/make_wave_plan.py
python Reference/real_play.py npy/wave_plan.npy
```

Các module dùng chung cũng nằm trong thư mục này; bốn chương trình chính tự nạp chúng. File JSON hiệu chuẩn và home nằm trong `json/`; bản sao lưu JSON nằm trong `json/debug/`. Models, targets, ảnh/log debug và `T_base_cam.npy` vẫn ở thư mục dự án SO101. Các file kế hoạch và bản sao hiệu chuẩn `.npy` nằm trong `npy/`. Tài liệu Markdown nằm trong `docs/`.

## Script hỗ trợ và module dùng chung

| Script | Công dụng |
|---|---|
| `snap_target.py` | Chụp ảnh mẫu cho handover |
| `record_home.py` | Ghi tư thế home |
| `goto.py` | Đi tới tọa độ để kiểm tra |
| `verify_click.py` | Kiểm tra hiệu chuẩn và hiệu chỉnh XY |
| `handeye_calib.py` | Hiệu chuẩn marker và mở camera dùng chung |
| `so101_real.py` | Kết nối robot, giới hạn, offset và home |
| `keyboard_control.py` | Điều khiển bàn phím và IK |
| `pick_place.py` | Lập kế hoạch gắp/đặt |
| `detector.py` | Nhận diện vật và che robot |

Chạy công cụ hỗ trợ từ thư mục SO101, ví dụ `python Reference/record_home.py` hoặc `python Reference/verify_click.py`.
