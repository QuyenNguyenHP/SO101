# Cài đặt tay robot SO101 (phần cứng, servo, calibrate)

Hướng dẫn chi tiết từng bước, đưa một tay SO101 vừa lắp xong tới trạng thái sẵn sàng nhận lệnh từ MuJoCo. Làm xong file này thì chuyển sang:

- [SETUP_TABLE_2D_CALIB.md](SETUP_TABLE_2D_CALIB.md) để setup tay trên mặt bàn và căn chỉnh bằng điểm 2D với camera D415 (không bắt buộc);
- [DEPLOY_REAL_SO101.md](DEPLOY_REAL_SO101.md) để chạy chuyển động: hình vuông, vẫy tay, bàn phím, gắp và đặt vật.

> [!WARNING]
> Tay robot thật có thể kẹp tay người, hất đổ đồ vật và làm cháy servo. Dọn sạch bàn, bắt vít đế robot xuống bàn, để một tay ở **công tắc nguồn hoặc phích cắm**.

---

## 1. Mục tiêu

Sau khi làm xong file này, bạn có:

1. Sáu servo trả lời đúng ID 1–6 trên bus.
2. File calibrate `my_follower.json` đã được lưu.
3. Đã kiểm chứng: tư thế trong MuJoCo trùng với tư thế tay thật (`SIGN`, `OFFSET`).

Hai quy ước được dùng xuyên suốt:

| | Mô phỏng (MuJoCo) | Tay thật (LeRobot, `use_degrees=True`) |
|---|---|---|
| 5 khớp tay | radian | độ, 0° ở giữa khoảng đã calibrate |
| Tay gắp | −0,17 … 1,75 rad | 0 … 100 (0 = đóng, 100 = mở) |
| Quan hệ đã kiểm chứng | `SIGN = [1,1,1,1,1]`, `OFFSET = 0` | độ = degrees(rad) |

---

## 2. Chuẩn bị

**Phần cứng**

- Tay SO101 **follower** đã lắp, gồm 6 servo Feetech **STS3215**, ID 1–6.
- Board điều khiển servo đi kèm kit (chip CH343, cắm USB-C vào máy tính, hiện ra là `/dev/ttyACM0`).
- Nguồn **12 V** cho loại servo 12 V. Xem nhãn trên servo: loại 7,4 V thì dùng nguồn 5 V. **Sai điện áp sẽ làm hỏng servo.**
- Máy tính Linux.
- *(Không bắt buộc)* tay **leader** nếu muốn điều khiển bằng tay (teleop).

**Bảng khớp**: tên khớp giống nhau ở MuJoCo và LeRobot.

| # | Khớp | Khoá trong LeRobot | ID servo | Khoảng trong sim (rad) |
|---|---|---|---|---|
| 0 | `shoulder_pan` (xoay đế) | `shoulder_pan.pos` | 1 | −1,92 … 1,92 |
| 1 | `shoulder_lift` (vai) | `shoulder_lift.pos` | 2 | −1,75 … 1,75 |
| 2 | `elbow_flex` (khuỷu) | `elbow_flex.pos` | 3 | −1,69 … 1,69 |
| 3 | `wrist_flex` (gập cổ tay) | `wrist_flex.pos` | 4 | −1,66 … 1,66 |
| 4 | `wrist_roll` (xoay cổ tay) | `wrist_roll.pos` | 5 | −2,74 … 2,84 |
| 5 | `gripper` (tay gắp) | `gripper.pos` | 6 | −0,17 … 1,75 |

**Phần mềm**: cài Miniforge (conda), sau đó tạo một môi trường riêng tên `lerobot`. Môi trường riêng giúp các phiên bản thư viện mà LeRobot ghim không làm hỏng phần MuJoCo cũ.

```bash
bash ~/Miniforge3-Linux-x86_64.sh -b -p ~/miniforge3
```

```bash
~/miniforge3/bin/conda init bash && source ~/.bashrc
```

```bash
conda create -y -n lerobot python=3.12
```

```bash
conda activate lerobot
```

```bash
git clone https://github.com/huggingface/lerobot.git ~/robot/lerobot
```

```bash
cd ~/robot/SO101
```

```bash
pip install -e "$HOME/robot/lerobot[feetech]"
```

Lệnh cài chạy ngay trong `~/robot/SO101`, không cần `cd` vào thư mục `lerobot`. Thư mục `~/robot/lerobot` chỉ là nơi để mã nguồn LeRobot; bạn không cần mở hay sửa gì trong đó.

```bash
pip install mujoco
```

Cài MuJoCo vào **cùng** môi trường thì một script có thể chạy cả mô phỏng lẫn tay thật.

**Mô hình MuJoCo**: các script dùng `~/robot/mujoco_menagerie/robotstudio_so101/`. Hai file cảnh là `scene.xml`, và `scene_box.xml` có thêm một khối hộp cho phần gắp vật.

> [!NOTE]
> Mỗi lần mở terminal mới, chạy `conda activate lerobot` rồi `cd ~/robot/SO101`. Nếu báo `conda: command not found`, chạy lại `~/miniforge3/bin/conda init bash` rồi mở terminal mới.

---

## 3. Kết nối và tìm cổng USB

1. Cắm nguồn vào board trước, sau đó mới cắm USB vào máy tính.
2. Tìm cổng: chạy lệnh dưới, rút cáp USB khi được hỏi rồi nhấn Enter.

```bash
lerobot-find-port
```

Lệnh in ra tên cổng, ví dụ `/dev/ttyACM0`. Tất cả script đều dùng `PORT = '/dev/ttyACM0'`. Nếu cổng của bạn khác, sửa hằng số này trong `Reference/so101_real.py` và trong các script có hằng số `PORT`.

3. Cấp quyền dùng cổng serial. Chỉ cần làm một lần, sau đó đăng xuất rồi đăng nhập lại:

```bash
sudo usermod -aG dialout $USER
```

Muốn thử ngay mà không đăng xuất thì dùng `sudo chmod 666 /dev/ttyACM0`. Lệnh này chỉ có tác dụng tới lần rút cáp tiếp theo.

---

## 4. Đặt ID cho servo

Servo mới xuất xưởng đều có ID 1. Bus cần 6 ID khác nhau (1–6) ở tốc độ 1 000 000 baud. **Hai servo trùng ID thì cả bus sẽ báo lỗi**, kể cả các servo đúng.

**Cách 1, dùng công cụ của LeRobot** (khi tất cả servo đều mới):

```bash
lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0
```

Làm theo lời nhắc: mỗi lần chỉ cắm **đúng một** servo mà nó gọi tên (bắt đầu từ gripper, kết thúc ở shoulder_pan), rồi nhấn Enter.

**Cách 2, cắm dần từng servo dọc theo chuỗi.** Dùng cách này khi kit đã được đặt ID sẵn nhưng có servo bị sai. Đây là cách đã sửa được tay này: servo 1 mang nhầm ID 6, trùng với gripper.

1. Rút hết dây servo khỏi board. Chỉ cắm **servo 1** (shoulder_pan):

```bash
python Reference/set_one_motor.py shoulder_pan
```

2. **Giữ nguyên** servo 1, cắm thêm servo 2 vào chuỗi:

```bash
python Reference/set_one_motor.py shoulder_lift
```

3. Tiếp tục như vậy, mỗi lần thêm một servo: `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`.

`Reference/set_one_motor.py` dò mọi tốc độ baud. Servo có ID nhỏ hơn ID cần đặt được coi là đã xong và để nguyên. Servo mới duy nhất được đặt ID, rồi kiểm tra bằng 10 lần đọc liên tiếp. Script **dừng lại mà không ghi gì** nếu thấy trùng ID, không có servo mới, hoặc có hơn một servo mới.

**Kiểm tra**: cả 6 ID phải trả lời.

```bash
python Reference/scan_motors.py
```

**Thử tay gắp** trước khi calibrate. Script dùng đơn vị thô và giới hạn lực ở 40%:

```bash
python Reference/test_gripper.py
```

Tay gắp mở ra rồi đóng vào khoảng 13° (150 tick). Thêm tham số `300` nếu muốn biên độ lớn hơn.

---

## 5. Calibrate servo

Bước calibrate cho LeRobot biết điểm giữa và hai đầu giới hạn của từng khớp. **Bước này cũng quyết định góc 0° của tay thật có trùng với 0 rad trong MuJoCo hay không**, nên cần làm cẩn thận.

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower
```

1. Khi được hỏi, dùng tay đưa robot về **giữa khoảng của mọi khớp** rồi nhấn Enter. Đó là tư thế trong tài liệu SO101 của LeRobot: tay dựng đứng và gập, tay gắp mở một nửa.
2. Sau đó xoay **từng khớp chậm hết cả hai đầu** rồi nhấn Enter. **File chỉ được lưu sau lần nhấn Enter này.**

Kết quả được lưu vào:

```
~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json
```

Từ đây về sau luôn dùng đúng `id` là `my_follower` để LeRobot tìm thấy file.

Các script đọc file này để lấy **giới hạn thật** của từng khớp: một nửa khoảng đã calibrate trừ đi `MARGIN = 3°`. Mọi lệnh vượt giới hạn sẽ bị cắt bớt. Phải làm vậy vì ở chế độ độ, LeRobot **không** tự cắt lệnh.

> [!TIP]
> Mô hình MuJoCo `robotstudio_so101` được làm khớp với cách calibrate mới của LeRobot, nên sau khi calibrate, 0 rad trong sim thường gần 0° trên tay thật. Mục 7 sẽ kiểm tra lại điều đó.

---

## 6. (Không bắt buộc) Điều khiển bằng tay leader

Cách kiểm tra nhanh phần cứng và calibrate. Trước tiên calibrate tay leader một lần bằng lệnh ở mục 5, đổi thành `--teleop.type=so101_leader --teleop.id=my_leader`. Sau đó chạy:

```bash
lerobot-teleoperate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower --teleop.type=so101_leader --teleop.port=/dev/ttyACM1 --teleop.id=my_leader
```

Di chuyển tay leader thì tay follower phải làm theo y như vậy.

---

## 7. Digital twin: kiểm tra sim và tay thật có khớp nhau

Trước khi gửi bất kỳ lệnh nào, phải biết góc thật tương ứng với góc sim ra sao. `Reference/real_twin.py` **tắt torque**: tay robot bị thả lỏng, bạn dùng tay cầm xoay nó. Cửa sổ MuJoCo vẽ lại tư thế thật theo thời gian thực, cả tay gắp.

```bash
python Reference/real_twin.py
```

**Cách kiểm tra `SIGN` và `OFFSET`**

1. Xoay **từng khớp một** bằng tay. Nếu khớp trong sim quay **ngược chiều**, đặt `SIGN` của khớp đó thành `-1`.
2. Đưa tay thật về đúng tư thế 0 của sim, tức mọi khớp bằng 0. Đọc số độ được in ra: các số đó chính là `OFFSET`. Nếu tất cả nằm trong khoảng ±5° thì để `OFFSET = 0`.
3. Thử nhiều tư thế cho tới khi tay trong cửa sổ trùng với tay thật ở mọi tư thế.

Với tay này, kết quả là `SIGN = [1, 1, 1, 1, 1]` và `OFFSET = 0`. Hai giá trị này đã được ghi vào `Reference/so101_real.py`. Nếu calibrate lại, hãy chạy `Reference/real_twin.py` lại để kiểm tra.

---

## 8. Đánh giá

| Kiểm tra | Đạt khi | Nếu không đạt |
|---|---|---|
| `Reference/scan_motors.py` | Cả 6 ID trả lời | Mục 4, hoặc xem bảng lỗi bên dưới |
| `Reference/test_gripper.py` | Tay gắp mở, đóng êm, về vị trí cũ | Kiểm tra dây và nguồn của servo 6 |
| Calibrate | File `my_follower.json` tồn tại | Chạy lại và nhấn Enter ở bước cuối |
| `Reference/real_twin.py` | Sim trùng tay thật ở mọi tư thế, sai lệch dưới khoảng 5° | Sửa `SIGN`/`OFFSET`, hoặc calibrate lại |

Cả bốn dòng đều đạt thì chuyển sang [DEPLOY_REAL_SO101.md](DEPLOY_REAL_SO101.md).

---

## 9. Lỗi thường gặp

| Triệu chứng | Nguyên nhân hay gặp | Cách sửa |
|---|---|---|
| `conda: command not found` | Thư mục cài conda bị xoá, hoặc `.bashrc` mất đoạn `conda init` | Cài lại Miniforge vào `~/miniforge3`, chạy `conda init bash`, tạo lại môi trường (mục 2) |
| `Permission denied: '/dev/ttyACM0'` | Chưa vào nhóm `dialout` | Mục 3, rồi đăng xuất và đăng nhập lại |
| `Device or resource busy` | Một chương trình khác đang giữ cổng (một cửa sổ calibrate, twin hay teleop khác) | Đóng chương trình đó, chỉ chạy một script mỗi lần |
| Cổng thành `/dev/ttyACM1` sau khi cắm lại | Linux đánh số lại | Chạy lại `lerobot-find-port`, hoặc tạo udev rule để có tên cố định |
| `Missing motor IDs`, `gripper not found` | Mất nguồn, lỏng dây, hoặc servo chưa có ID | Kiểm tra đèn servo nháy khi bật nguồn, cắm chặt dây, chạy `Reference/scan_motors.py` |
| `Failed to set up … Operation timed out` | **Hai servo trùng ID** trên bus | Cắm dần từng servo với `Reference/set_one_motor.py` (mục 4) |
| `ImportError … so101_follower` | LeRobot đã đổi tên module | Dùng `from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig` |
| Không tìm thấy file calibrate | Dùng `id` khác nhau giữa các lệnh | Luôn dùng `my_follower` |
| Một khớp quay **ngược** chiều sim | `SIGN` sai | Mục 7 |
| Tay thật lệch sim một góc cố định | `OFFSET` sai, hoặc calibrate chưa chuẩn | Mục 7, hoặc calibrate lại (mục 5) |

---

## Tài liệu tham khảo

- Hướng dẫn SO101 của LeRobot: `~/robot/lerobot/docs/source/so101.mdx` (bản online trên huggingface.co/docs/lerobot/so101)
- Phần cứng SO-ARM100/101: https://github.com/TheRobotStudio/SO-ARM100
- Mô hình MuJoCo: `~/robot/mujoco_menagerie/robotstudio_so101/so101.xml`
