# Đưa chuyển động từ MuJoCo sang tay robot SO101 thật

Hướng dẫn chi tiết từng bước để chạy chuyển động từ MuJoCo trên tay SO101 thật: hình vuông, vẫy tay, điều khiển bằng bàn phím, gắp và đặt vật. Kèm các lỗi đã gặp thật trong quá trình làm và cách sửa.

> [!WARNING]
> Tay robot thật có thể kẹp tay người, hất đổ đồ vật và làm cháy servo. Dọn sạch bàn, bắt vít đế robot xuống bàn, để một tay ở **công tắc nguồn hoặc phích cắm**. Mọi chuyển động mới đều phải chạy **chậm** trước.

---

## 1. Mục tiêu và ý tưởng

Bạn đã có chuyển động chạy được trong MuJoCo, ví dụ vẽ hình vuông (`draw_square.py`) hay vẫy tay (`wave_hello.py`). Mục tiêu là cho tay SO101 thật làm đúng chuyển động đó.

Ý tưởng chính: **MuJoCo tính toán, tay thật chỉ làm theo.**

```
MuJoCo (rad)  ──►  đổi sang độ, kẹp giới hạn, hạn chế tốc độ  ──►  LeRobot  ──►  servo STS3215
     ▲                                                                              │
     └────────────────  digital twin: đọc góc thật đưa ngược về sim  ◄──────────────┘
```

- **Mô phỏng** lo phần khó: động học ngược (IK), lập kế hoạch đường đi, kiểm tra va chạm và kiểm tra vật lý khi gắp.
- **LeRobot** (thư viện của Hugging Face) lo phần giao tiếp với 6 servo Feetech qua cổng USB.
- **`Reference/so101_real.py`** là cầu nối giữa hai bên. Nó đổi đơn vị, cắt lệnh theo giới hạn đã calibrate, và giới hạn mỗi lệnh chỉ được đổi vài độ để tay không bị giật.

Hai quy ước được dùng xuyên suốt hướng dẫn:

| | Mô phỏng (MuJoCo) | Tay thật (LeRobot, `use_degrees=True`) |
|---|---|---|
| 5 khớp tay | radian | độ, 0° ở giữa khoảng đã calibrate |
| Tay gắp | −0,17 … 1,75 rad | 0 … 100 (0 = đóng, 100 = mở) |
| Quan hệ đã kiểm chứng | `SIGN = [1,1,1,1,1]`, `OFFSET = 0` | độ = degrees(rad) |

---

## 2. Trước khi bắt đầu

1. Tay robot đã cài đặt xong theo [SETUP_ROBOT.md](SETUP_ROBOT.md): 6 servo trả lời, đã calibrate `my_follower`, `Reference/real_twin.py` khớp với sim.
2. Mở terminal, chạy `conda activate lerobot` rồi `cd ~/robot/SO101`.
3. *(Không bắt buộc)* Camera đã hiệu chuẩn theo [SETUP_TABLE_2D_CALIB.md](SETUP_TABLE_2D_CALIB.md), nếu muốn camera tự tìm vật.

---

## 3. Chạy một kế hoạch có sẵn: hình vuông và vẫy tay

Một **kế hoạch** (plan) là file `.npy` chứa N dòng. Mỗi dòng là một lệnh, gồm 5 cột (5 khớp) hoặc 6 cột (thêm tay gắp), đơn vị radian, chạy ở 30 lệnh mỗi giây.

**Bước 1. Tạo kế hoạch trong sim**

```bash
python Reference/make_square_plan.py
```

Lệnh này tạo hình vuông cạnh 6 cm (nửa cạnh 3 cm). Tham số `0.05` cho hình vuông cạnh 10 cm. Các cạnh là đường thẳng vì IK được giải ở từng bước. Kết quả lưu vào `npy/square_plan.npy`.

```bash
python Reference/make_wave_plan.py
```

Lệnh này tạo động tác vẫy tay 10 giây, có tay gắp, và chuyển động tăng dần ở đầu, giảm dần ở cuối trong 1,5 s. Tham số `20` cho 20 giây. Kết quả lưu vào `npy/wave_plan.npy`.

> [!IMPORTANT]
> `draw_square.py` trong sim dùng hình vuông 40 cm, quá tầm với của tay thật. Trên tay thật, bắt đầu từ 6 cm.

**Bước 2. Chạy trên tay thật**

```bash
python Reference/real_play.py npy/square_plan.npy
```

```bash
python Reference/real_play.py npy/wave_plan.npy
```

1. Script cảnh báo nếu kế hoạch vượt giới hạn sim hoặc giới hạn đã calibrate.
2. Robot đi từ tư thế hiện tại tới điểm đầu của kế hoạch trong 3 s, chuyển động mượt.
3. Nhấn **Enter** để chạy. Mặc định chạy nửa tốc độ. Thêm `1.0` ở cuối lệnh để chạy đủ tốc độ, nhưng chỉ khi đã chạy chậm ổn.
4. Khi xong, script in **sai số khớp cuối cùng** (độ). Lần chạy hình vuông đầu tiên ra `[0.5 -0.2 -0.9 -0.3 0.5]`, tức dưới 1°.
5. Script hỏi trước khi tắt torque: **hãy đỡ tay robot**, rồi nhấn Enter.

**Bước 3 (tuỳ chọn). Sim và tay thật chạy cùng lúc**

```bash
python Reference/sim_to_real.py
```

Đây là chế độ **thanh trượt**. Mở bảng *Control* bên phải cửa sổ MuJoCo và kéo thanh trượt: tay thật làm theo.

```bash
python Reference/sim_to_real.py npy/wave_plan.npy 0.5
```

Đây là chế độ **kế hoạch**: kế hoạch chạy trên sim và tay thật cùng lúc. Dòng trạng thái in độ lệch lớn nhất giữa sim và tay thật. Thêm `--dry` vào bất kỳ lệnh nào để thử mà không cần tay robot.

---

## 4. Điều khiển đầu gắp bằng bàn phím

`Reference/keyboard_control.py` giải IK cho **đầu ngón gắp** (site `gripperframe`). Bạn ra lệnh "đi tới trước 5 mm", script tự tính góc 5 khớp, rồi sim và tay thật cùng đi.

```bash
python Reference/keyboard_control.py
```

Muốn thử chỉ trên sim, thêm `--dry`. **Phím được đọc từ terminal**, nên phải bấm chuột vào terminal trước khi gõ phím, rồi nhìn sang cửa sổ MuJoCo.

| Phím | Tác dụng |
|---|---|
| ↑ / ↓ | đầu gắp tiến / lùi (+X / −X) |
| ← / → | đầu gắp sang trái / phải (+Y / −Y, nhìn từ phía sau robot) |
| `w` / `s` | lên / xuống (+Z / −Z) |
| `i` / `k` | ngóc / chúc mũi tay gắp |
| `j` / `l` | xoay cổ tay |
| `o` / `c` / space | mở / đóng từng chút / bật tắt mở hết – đóng hết |
| `p` | khoá góc nghiêng: tay gắp giữ nguyên độ nghiêng khi di chuyển |
| `1` `2` `3` | bước 2 / 5 / 10 mm |
| `h` | về tư thế home |
| `q` | thoát |

- Quả cầu xanh trong sim là điểm đích.
- Dòng trạng thái in `tip x… y… z… mm`. **Đây chính là toạ độ dùng cho phần gắp vật ở mục 5.**
- Nếu IK không đến được trong phạm vi 3 mm, lệnh bị từ chối, trạng thái hiện `OUT OF REACH` và tay đứng yên ở tư thế cũ.
- Đầu gắp không bao giờ xuống thấp hơn 1 cm so với mặt bàn.

---

### 4a. Ghi tư thế home: `Reference/record_home.py`

Tư thế **home** là chỗ tay quay về:
- khi nhấn `h` trong `Reference/keyboard_control.py`, `Reference/verify_click.py`, `camera_pick.py`, `Reference/goto.py`;
- ở cuối mỗi lần gắp và đặt.

Mặc định home là `[0, -20, 30, 40, 0]` độ. Muốn tự chọn tư thế home:

```bash
python Reference/record_home.py
```

1. **Giữ tay robot bằng một tay** rồi nhấn Enter. Torque tắt và tay trở nên mềm.
2. Đưa tay tới tư thế muốn làm home. Dòng trạng thái liên tục hiện góc servo và vị trí tâm kẹp (x, y, z).
3. Nhấn **Enter** để ghi. Torque bật lại và tay **giữ nguyên tư thế đó**, nên bạn có thể buông tay.
4. Script cảnh báo nếu một khớp nằm sát giới hạn, hoặc ngón kẹp thấp hơn 5 cm so với mặt bàn. Nhấn **Y** để lưu vào `json/home_pose.json`.

| Lệnh | Tác dụng |
|---|---|
| `python Reference/record_home.py --current` | Ghi tư thế tay đang giữ, không tắt torque. Dùng khi đã đưa tay tới bằng `Reference/goto.py` hoặc `Reference/keyboard_control.py` |
| `python Reference/record_home.py --test` | Sau khi ghi: nâng lên 3 cm rồi quay về home để thử |
| `python Reference/record_home.py --show` | Xem home đang lưu (không cần tay robot) |
| `python Reference/record_home.py --reset` | Bỏ home đã ghi, quay về mặc định |

Ghi chú:
- Home được lưu theo **góc servo thật**. Nếu sau này `json/joint_offsets.json` thay đổi, home vẫn là đúng tư thế đó ngoài đời.
- File cũ được giữ trong `json/debug/home_pose_<thời gian>.json`.
- Mọi script đọc home lúc khởi động. Vì vậy ghi home xong thì **khởi động lại** các script đang chạy.

Một home tốt:
- tay kẹp cách mặt bàn 10–20 cm;
- không che camera nhìn xuống vùng làm việc;
- các khớp không ở sát giới hạn;
- khuỷu tay gập, để tay giữ tư thế nhẹ nhàng, không bị gồng.

## 5. Gắp và đặt vật (pick & place)

`Reference/pick_place.py` lập kế hoạch đầy đủ, **kiểm tra bằng vật lý trong MuJoCo** (một khối hộp cùng kích thước với vật thật), và chỉ chạy trên tay thật khi kiểm tra đạt.

**Bước 1. Đo vị trí vật**

Chạy `Reference/keyboard_control.py`. Đưa đầu gắp tới ngay trên vật, mũi chúc xuống, rồi đọc **x, y** trên dòng trạng thái. Làm tương tự cho chỗ đặt. Đơn vị là mm, đo trong hệ của robot: x hướng ra trước, y sang trái.

**Bước 2. Xem trước trên sim**

```bash
python Reference/pick_place.py 250 0  220 -100 --height 50 --width 30
```

Lệnh này gắp ở (250, 0) và đặt ở (220, −100). Script in ra kết quả kiểm tra vật lý: vật được nhấc cao bao nhiêu, đặt lệch bao nhiêu mm, có còn đứng thẳng không.

**Bước 3. Chạy trên tay thật**

```bash
python Reference/pick_place.py 250 0  220 -100 --height 50 --width 30 --real
```

| Tuỳ chọn | Ý nghĩa | Mặc định |
|---|---|---|
| `--height` | chiều cao vật (mm) | 60 |
| `--width` | bề rộng vật, đo giữa hai hàm kẹp (mm) | 40 |
| `--yaw` | vật nằm xoay bao nhiêu độ trên bàn; cổ tay tự xoay theo | 0 |
| `--speed` | tốc độ trên tay thật (tối đa 2,0; mô phỏng kiểm tra ở đúng tốc độ này) | 1,0 |
| `--dry` | đi cùng `--real`: chạy thử đường đi của tay thật mà không cần tay | |

Script lần lượt thực hiện các bước: **di chuyển tới trên vật → hạ xuống → đóng kẹp → nhấc lên → mang đi → căn chỉnh → hạ xuống → mở kẹp → nhấc ra → về home**. Tên bước được in ra khi tay thực hiện. Trước khi chạy, script hỏi Enter để xác nhận. Nhấn Ctrl+C để dừng bất cứ lúc nào.

**Cách chọn vật và vị trí (quyết định độ thành công)**

- Vật **nhẹ, dạng hộp, rộng 25–35 mm**, ví dụ khối gỗ hoặc hộp diêm dựng đứng. Đây là cỡ được đặt chính xác nhất khi thử trong sim.
- Đặt vật cách đế khoảng **200–300 mm**. Gần hơn khoảng 190 mm thì tay không thể đi thẳng lên xuống được. Script sẽ báo `Cannot move straight up … Try a spot a little further away` và không chạy.
- Đo x, y bằng `Reference/keyboard_control.py` (mục 4), đừng ước lượng bằng mắt.

**Kết quả thử trong sim**

| Thử nghiệm | Kết quả |
|---|---|
| 5 cỡ vật (rộng 20–40 mm, cao 30–60 mm) × 3 cặp vị trí | 15/15 thành công, nhấc 77–84 mm, đặt lệch 1–5 mm, vật vẫn đứng thẳng |
| Vật nằm xoay 30° (`--yaw 30`) | Thành công, lệch 4 mm |
| `--real --dry` | Chạy đủ 10 bước, thoát bình thường |

**Các chỉnh sửa có trong script.** Mỗi chỉnh sửa sửa một lỗi mà mô phỏng đã phát hiện:

| Hằng số / cơ chế | Vấn đề nó sửa |
|---|---|
| `PITCH = 80` (không phải 90) | Ở đúng 90°, IK mất khả năng điều khiển góc nghiêng |
| `roll_for` và `--yaw` | Hàm kẹp không song song với mặt vật khi đế xoay |
| `CLEAR = 8 mm` | Hàm cố định đáp lên đỉnh vật |
| Hạ xuống bằng đúng đường nhấc lên đảo ngược | Hạ xiên làm đụng vật |
| `PLACE_ABOVE = 10 mm` | Vật tụt trong kẹp khoảng 8 mm, nên khi đặt bị ấn xuống bàn |
| `place_shift(width)` | Khi mở kẹp, hàm động kéo vật rộng 40 mm trượt khoảng 15 mm |

Các giá trị này được chỉnh trên sim. Tay thật có ma sát và độ chính xác khác, nên có thể phải chỉnh lại `CLEAR` và `PLACE_ABOVE` sau vài lần thử.

---

## 6. Đánh giá kết quả

| Đo gì | Tốt | Chấp nhận được | Cần xem lại |
|---|---|---|---|
| Sai số khớp cuối (`Reference/real_play.py`) | dưới 1° | 1–3° | trên 3°: tay bị cản, nguồn yếu, hoặc giới hạn bị cắt |
| Độ lệch sim–thật (`Reference/sim_to_real.py`) | dưới 3° khi chạy | dưới 6° lúc tăng tốc | lớn và tăng dần: giảm tốc độ |
| Pick & place trên sim | lệch dưới 5 mm | 5–10 mm | trên 15 mm hoặc vật đổ: đổi vị trí hoặc kích thước |
| Pick & place trên tay thật | vật vào đúng chỗ | lệch dưới 1 cm | trượt khỏi kẹp: xem mục 8 |

Phần sai số còn lại chủ yếu đến từ độ rơ của servo STS3215 và sai lệch của bước calibrate. Với SO101, sai lệch vài mm ở đầu gắp đã là kết quả tốt.

---

## 7. Các file trong thư mục

| File | Dùng để |
|---|---|
| `Reference/so101_real.py` | Module dùng chung: lớp `RealArm` (đọc, gửi, đóng), đổi đơn vị sim ↔ thật, giới hạn khớp từ file calibrate |
| `Reference/scan_motors.py` | Xem servo nào trả lời trên bus |
| `Reference/set_one_motor.py` | Đặt ID cho servo vừa cắm thêm |
| `Reference/test_gripper.py` | Thử tay gắp bằng đơn vị thô, dùng được trước khi calibrate |
| `Reference/real_twin.py` | Digital twin: xoay tay thật bằng tay, sim vẽ theo |
| `Reference/make_square_plan.py`, `Reference/make_wave_plan.py` | Tạo kế hoạch `.npy` |
| `Reference/real_play.py` | Chạy một kế hoạch trên tay thật |
| `Reference/sim_to_real.py` | Sim điều khiển tay thật trực tiếp (thanh trượt hoặc kế hoạch) |
| `Reference/keyboard_control.py` | Điều khiển đầu gắp theo toạ độ bằng bàn phím |
| `Reference/record_home.py`, `json/home_pose.json` | Ghi tư thế home bằng cách cầm tay đưa tới (mục 4a); mọi script dùng home này |
| `Reference/pick_place.py` | Gắp và đặt vật, có kiểm tra vật lý |
| `Reference/handeye_calib.py` | Hiệu chuẩn camera D415: thu mẫu marker ArUco, tính `T_base_cam.npy` (xem SETUP_TABLE_2D_CALIB.md) |
| `camera_pick.py` | Click vật trên ảnh camera, tự đo cao/rộng/góc xoay, click chỗ đặt, kiểm tra trong sim rồi gắp bằng tay thật |
| `Reference/detector.py` | Tìm mọi vật trên bàn bằng ảnh độ sâu (loại tay robot ra) và gọi tên bằng CLIP; dùng qua `camera_pick.py --detect "..."` |
| `Reference/goto.py` | Đưa tay gắp tới một điểm 3D (x y z mm, có pitch), rồi chờ lệnh tiếp theo |
| `Reference/verify_click.py` | Click một điểm trên ảnh camera, tay đi tới phía trên (4 cm, đổi bằng `+`/`-`) để kiểm tra hiệu chuẩn |
| `table_calib.py` | Hiệu chuẩn camera trên mặt bàn: chạm đầu hàm cố định vào các dấu X rồi click chúng |

Dùng `RealArm` trong script của riêng bạn:

```python
from so101_real import RealArm
arm = RealArm(dry=False)   # dry=True: tay giả, không có gì chuyển động
q = arm.read_sim()         # 6 giá trị (rad): 5 khớp + tay gắp, đơn vị sim
arm.send_sim(q)            # bị cắt theo giới hạn và giới hạn tốc độ (mặc định 2°/lệnh = 60°/s)
arm.close()                # hỏi bạn đỡ tay robot, rồi tắt torque
```

---

## 8. Lỗi thường gặp

| Triệu chứng | Nguyên nhân hay gặp | Cách sửa |
|---|---|---|
| Lỗi kết nối, ID servo, calibrate, sim quay ngược | Phần cài đặt chưa xong | Xem bảng lỗi trong [SETUP_ROBOT.md](SETUP_ROBOT.md) |
| `Device or resource busy` | Một chương trình khác đang giữ cổng (một cửa sổ calibrate, twin hay teleop khác) | Đóng chương trình đó, chỉ chạy một script mỗi lần |
| Tay giật, dừng giữa chừng | `max_relative_target` hoặc giới hạn tốc độ đang chặn bước nhảy lớn | Giảm `--speed`, hoặc tăng số bước trong kế hoạch |
| Servo nóng, kêu rè, tay bị võng | Giữ tư thế chống trọng lực hoặc ép xuống bàn | Thu nhỏ chuyển động, kiểm tra điện áp và dòng của nguồn |
| `OUT OF REACH` liên tục | Điểm đích ngoài tầm với hoặc dưới `Z_MIN` | Lùi lại vài bước, hoặc nhấn `h` về home |
| Pick & place từ chối chạy trên tay thật | Kiểm tra vật lý trên sim không đạt | Dời điểm gắp xa đế hơn (200–300 mm), nhập đúng `--width` và `--height` |
| Vật trượt khỏi kẹp trên tay thật | Ma sát thấp, vật nặng, hoặc `--width` nhập lớn hơn thật | Đo lại bề rộng, dùng vật nhẹ hơn hoặc nhám hơn |
| Cửa sổ MuJoCo báo `Segmentation fault` khi thoát | Viewer bị lỗi lúc Python tắt | Vô hại: các script đã tắt an toàn trước khi thoát |

---

## 9. Danh sách kiểm tra cho lần chạy đầu tiên

Làm lại danh sách này **mỗi lần** chạy một chuyển động mới:

- [ ] Chuyển động đã chạy ổn trên sim, không đụng bàn
- [ ] Đã chạy `Reference/real_twin.py` sau lần calibrate gần nhất
- [ ] Chạy chậm (`0.5` hoặc thấp hơn), chuyển động nhỏ
- [ ] Không có gì trong tầm với của tay, đế đã bắt vít
- [ ] Tay để sẵn ở phích nguồn, sẵn sàng nhấn `Ctrl+C`
- [ ] Đã xem hết lần chạy đầu trước khi tăng tốc độ
- [ ] Đỡ tay robot trước khi nhấn Enter để tắt torque

---

## 10. Bước tiếp theo: thêm camera

Hiện tại vị trí vật được đo bằng tay qua `Reference/keyboard_control.py`. Bước tiếp theo là để **camera tự tìm vật**: gắn một RealSense D415 cố định, hiệu chuẩn ma trận `T_base_cam` bằng marker ArUco dán trên tay gắp, rồi đưa toạ độ vật thẳng vào `Reference/pick_place.py`. Chi tiết trong [SETUP_TABLE_2D_CALIB.md](SETUP_TABLE_2D_CALIB.md).

---

## Tài liệu tham khảo

- Hướng dẫn SO101 của LeRobot: `~/robot/lerobot/docs/source/so101.mdx` (bản online trên huggingface.co/docs/lerobot/so101)
- Phần cứng SO-ARM100/101: https://github.com/TheRobotStudio/SO-ARM100
- Mô hình MuJoCo: `~/robot/mujoco_menagerie/robotstudio_so101/so101.xml`
