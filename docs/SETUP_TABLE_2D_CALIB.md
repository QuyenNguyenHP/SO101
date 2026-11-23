# Setup tay SO101 trên mặt phẳng bàn: căn chỉnh bằng điểm 2D (camera D415)

Hướng dẫn trọn vẹn, từ lúc gắn camera tới lúc **click vào một điểm trên ảnh thì tay gắp đi đúng tới điểm đó**.

**Mục tiêu cuối cùng:** chạy `Reference/verify_click.py`, click vào một dấu X trên bàn, tay gắp đi tới ngay phía trên dấu X.

- **Trục X và Y phải chính xác:** lệch không quá 3–5 mm.
- **Trục Z được phép lệch ít**, vì tay chỉ dừng cách mặt bàn khoảng 4 cm.

Làm phần này **sau khi** tay robot đã cài đặt xong theo [SETUP_ROBOT.md](SETUP_ROBOT.md): servo đúng ID, đã calibrate, `Reference/real_twin.py` khớp với sim. Mọi lệnh đều chạy trong `~/robot/SO101`, môi trường conda `lerobot`.

---

## Tổng quan quy trình

| Bước (mục) | Làm gì | Script | Kết quả |
|---|---|---|---|
| 2 | Chuẩn bị, cài phần mềm | | |
| 3 | Gắn camera, giải phóng camera | `realsense-viewer` | camera thấy cả vùng làm việc |
| 4 | **Hiệu chuẩn trên mặt bàn** (bắt buộc) | `table_calib.py` | `T_base_cam.npy` |
| 5 | Kiểm tra bằng click | `Reference/verify_click.py` | `debug/verify_log.txt` |
| 6 | **Căn chỉnh X/Y** nếu còn lệch | `Reference/verify_click.py` (phím `i` `k` `j` `l`, `o`) | `json/camera_offset.json` |
| 7 | Gắp vật, đi tới điểm 3D | `camera_pick.py`, `Reference/goto.py`, `Reference/pick_place.py` | |
| Phụ lục | Hiệu chuẩn bằng marker ArUco (không bắt buộc) | `Reference/handeye_calib.py`, `Reference/marker_check.py` | |

> [!IMPORTANT]
> Sau khi hiệu chuẩn, **camera, đế robot và mặt bàn không được xê dịch**. Lệch 1–2 mm ở giá camera thành vài mm ở đầu gắp. Bị xê dịch thì làm lại bước 4.

---

## 1. Ý tưởng và hệ toạ độ

Camera D415 cho mỗi pixel một khoảng cách (ảnh độ sâu). Khi bạn click một pixel, script tính ra điểm 3D **trong hệ camera**. Ma trận 4×4 **`T_base_cam`** đổi điểm đó sang **hệ đế robot**:

```
p_base = T_base_cam · p_cam + (hiệu chỉnh XY trong json/camera_offset.json)
```

Sau đó, động học ngược (IK, trên mô hình MuJoCo) tính góc 5 khớp để đưa đầu gắp tới `p_base`.

**Hệ toạ độ đế robot** (giống `Reference/keyboard_control.py` và `Reference/pick_place.py`, đơn vị mm):

- **x** hướng ra phía trước robot, **y** sang trái (nhìn từ phía sau robot), **z** hướng lên, z = 0 là mặt bàn.
- Gốc x = 0 nằm **lùi sau trục xoay của đế khoảng 39 mm**. Đừng đo toạ độ bằng thước từ tâm trục đế.

**Hai điểm trên tay gắp**:

| Điểm | Ở đâu | Dùng khi |
|---|---|---|
| **Đầu mút hàm cố định** (`fixed_jaw_sph_tip1`) | Đầu cuối của hàm **không** di chuyển, chạm được bằng tay | Hiệu chuẩn (bước 4) và kiểm tra (bước 5, mặc định): nhìn thấy rõ, dễ so với dấu X |
| **Tâm kẹp** (`gripperframe`) | Giữa hai hàm, phía trước đầu hàm cố định khoảng 20 mm | `Reference/pick_place.py` (vật nằm giữa hai hàm), `verify_click.py --tcp` |

Hai điểm này cách nhau một khoảng cố định theo mô hình, nên hiệu chuẩn bằng điểm này thì dùng được cho cả điểm kia.

**Vì sao cần hiệu chuẩn trên mặt bàn?** Lần đầu, tay này được hiệu chuẩn bằng marker ArUco (phụ lục A). Sai số trên giấy chỉ 5 mm, nhưng khi kiểm tra thật thì mặt bàn bị tính ở z = −32 mm và lệch khoảng 30 mm theo x. Lý do:

- marker được đo ở trên cao (z từ 76 mm trở lên), còn khi gắp thì vị trí vật lấy từ **ảnh độ sâu trên mặt bàn**;
- hai cách đo này lệch nhau vài phần trăm.

`table_calib.py` hiệu chuẩn **đúng cách đo dùng khi gắp**: điểm độ sâu trên mặt bàn ↔ vị trí thật của robot.

---

## 2. Chuẩn bị

**Phần cứng**

- Tay SO101 follower đã calibrate (`id` = `my_follower`), **đế bắt vít** cố định vào bàn.
- RealSense D415, giá đỡ hoặc tripod chắc chắn.
- **Cáp và cổng USB 3** (cổng màu xanh). Ở USB 2 vẫn chạy được, nhưng ảnh màu kèm độ sâu chỉ được 640×480, nên điểm click kém chính xác hơn.
- Bút dạ, thước, (tuỳ chọn) một tờ giấy trắng dán trên bàn để vẽ dấu X.

**Phần mềm**

```bash
conda activate lerobot && cd ~/robot/SO101
```

```bash
pip install pyrealsense2 opencv-contrib-python scipy
```

Kiểm tra đã đủ thư viện:

```bash
python -c "import pyrealsense2, cv2, scipy; print('OK', cv2.__version__)"
```

---

## 3. Gắn camera

1. Đặt D415 cao hơn mặt bàn khoảng **35–50 cm**, chiếu chéo xuống khoảng 45°. Camera phải thấy **toàn bộ vùng làm việc**, tức cách đế 180–320 mm, cả trái lẫn phải.
2. Khoảng cách từ camera tới vùng làm việc nên **35–60 cm**. D415 đo độ sâu kém khi gần hơn khoảng 30 cm, và kém chính xác dần khi xa hơn.
3. **Vặn chặt giá.** Robot cũng phải bắt vít cố định.
4. **Tránh ánh sáng ngược.** Không để cửa sổ sáng nằm sau vùng làm việc: camera sẽ tự giảm độ sáng và vùng bàn bị tối. Mặt bàn đen bóng hoặc phản quang thì không đo được độ sâu; dán giấy mờ lên những chỗ đó.
5. **Giải phóng camera.** Mỗi lúc chỉ một chương trình được dùng camera. Máy này có dịch vụ `d415-lab.service` (app `DepthCamera-D415`) tự chạy và chiếm camera. Tắt nó trước:

```bash
systemctl --user stop d415-lab.service
```

Kiểm tra không còn ai giữ camera (lệnh không in gì là đã trống):

```bash
fuser -v /dev/video*
```

6. Mở `realsense-viewer`, bật luồng Color và Depth, kiểm tra vùng làm việc có độ sâu (không có lỗ đen) ở mọi chỗ. Kiểm tra xong thì **đóng `realsense-viewer`**.

---

## 4. Hiệu chuẩn trên mặt bàn (`table_calib.py`)

**Ý tưởng:** đặt đầu hàm cố định đúng vào một dấu X. Robot cho biết vị trí của nó (tính từ góc servo), camera cho biết vị trí của dấu X (tính từ ảnh độ sâu). Với **8–10 cặp điểm** như vậy, script tính cùng lúc:

- **`T_base_cam`**: camera → robot, kèm hệ số tỉ lệ độ sâu (D415 có thể lệch vài phần trăm);
- **độ lệch góc nhỏ của khớp** vai, khuỷu, cổ tay (`json/joint_offsets.json`). Mọi lần chạm đều nằm trên mặt bàn, nên độ cao đầu hàm phải bằng nhau ở mọi điểm. Nếu robot báo gần đế z = −15 mm, xa đế z = −29 mm, tức là góc khớp đang lệch vài độ. File này được `Reference/so101_real.py` nạp tự động, nên **mọi script** (`keyboard_control`, `verify_click`, `pick_place`…) đều được sửa theo.

**Chuẩn bị dấu X**

- Vẽ **6–10 dấu X** nhỏ, rõ (nét 1–2 mm), rải khắp vùng làm việc: gần đế (khoảng 180 mm), xa đế (khoảng 300 mm), bên trái, bên phải, ở giữa.
- **Không** vẽ tất cả trên một đường thẳng: các điểm phải trải ra thành một vùng.
- Khi chạm, **thay đổi độ nghiêng của tay gắp** giữa các điểm: có lúc gần thẳng đứng, có lúc nghiêng 30–40°. Như vậy script mới tách được độ lệch của từng khớp.

**Chạy**

```bash
python table_calib.py
```

1. Đỡ tay robot rồi nhấn **Enter** trong terminal: **torque tắt**, tay mềm ra để bạn cầm di chuyển.
2. Với **mỗi dấu X**:
   1. Cầm tay robot, đặt **đầu mút của hàm cố định** đúng vào **tâm dấu X**, chạm nhẹ mặt bàn. Giữ thật yên.
   2. Bấm vào cửa sổ ảnh rồi nhấn **SPACE**. Script đọc góc servo 10 lần, ra dòng `jaw tip at x … y … z …`. Nếu báo `arm moved`, giữ yên hơn và nhấn lại.
   3. Nhấc tay robot ra để **camera nhìn thấy dấu X**, rồi **click chuột vào tâm dấu X** trên ảnh.
   4. Màn hình báo `pair N saved`.

   Thứ tự SPACE và click có thể đảo. Nhấn **u** để bỏ cặp vừa làm nếu lỡ tay.
3. Đủ ít nhất 6 cặp (nên 8–10) thì nhấn **c**. Terminal in ra:
   - **fine joint offsets**: độ lệch góc tìm được cho vai, khuỷu, cổ tay. Vài độ là bình thường; trên 12° thì script không lưu và yêu cầu calibrate servo lại;
   - **touch heights**: độ cao các lần chạm trước và sau khi sửa. Sau khi sửa, các độ cao phải gần bằng nhau;
   - **sai số từng điểm**; điểm sai nhiều bị đánh dấu `<- check this one`;
   - **XY error predicting each touch from the others**: con số quan trọng nhất. Mỗi điểm được dự đoán chỉ từ các điểm còn lại, giống hệt một lần click mới. **Dưới 4 mm là GOOD**;
   - **vị trí camera** so với đế robot (mm): kiểm tra nhanh bằng thước, phải khớp trong vài cm.
   - Trong lúc thu, nếu chạm và click không phải cùng một dấu X, cặp đó bị **REJECTED** ngay. Dấu X đã làm hiện màu xanh kèm số, và không click lại được.
4. Kết quả lưu vào `T_base_cam.npy` và `json/joint_offsets.json`. Bản hiệu chuẩn marker cũ được giữ ở `npy/T_base_cam_handeye.npy`. Các cặp điểm (kèm góc servo thô) lưu ở `json/table_points.json`.
5. Nhấn **q**, đỡ tay robot, nhấn **Enter**.

**Nếu có điểm bị đánh dấu `<- check this one`:** thường do click lệch tâm dấu X hoặc đầu hàm không nằm đúng tâm. Chạy lại có giữ các cặp cũ, rồi làm thêm vài điểm:

```bash
python table_calib.py --resume
```

Tính lại từ các cặp đã lưu (không cần camera, không cần robot):

```bash
python table_calib.py --solve-only
```

> [!TIP]
> Click chính xác quan trọng ngang với đặt đầu hàm chính xác. Ở 640×480, mỗi pixel ứng với khoảng 0,7–1 mm trên bàn. Cứ click đúng tâm dấu X, đừng click vào chỗ có bóng của tay robot.

---

## 5. Kiểm tra bằng `Reference/verify_click.py`

```bash
python Reference/verify_click.py
```

Torque **bật** khi script kết nối: tay giữ cứng ở tư thế hiện tại. Mọi thao tác đều làm trong cửa sổ ảnh: **dòng chữ vàng** trên cùng cho biết đang làm gì, dòng dưới cùng hiện số fps và hiệu chỉnh XY đang dùng.

Với từng dấu X:

1. **Click** vào tâm dấu X. Dòng trạng thái hiện `point x … y … z … mm`. Trên mặt bàn, **z phải gần 0** (trong khoảng ±10 mm). Nếu z sai nhiều thì bước 4 chưa tốt.
2. Nhấn **`g`**. Nếu báo `IK ok`, nhấn **`y`** để đi (hoặc **`n`** để huỷ). Tay đi chậm trong 3 giây tới vị trí **đầu hàm cố định cách dấu X 4 cm phía trên**.
3. Chờ khoảng 2 giây. Script **tự bù độ võng servo**: tay vươn ra bị trọng lực kéo, servo dừng thiếu 1–2°, script đọc góc thật rồi chỉnh lại.
4. **Nhìn thẳng từ trên xuống**: đầu hàm cố định phải nằm ngay trên tâm dấu X.
5. Nhấn **`h`** để về home rồi thử dấu X khác.

Thử **ít nhất 4–5 dấu X** ở các vùng khác nhau. Mỗi lần tới nơi, script ghi vào `debug/verify_log.txt`:

```
--- arrived (move) ---
  target               x    204  y     40  z     40 mm   (nudge so far x +0 y +0 mm)
  fixed-jaw tip (servos)  x    204  y     40  z     41 mm   -> off by x +0 y +0 z +1 mm
  joint sag before correction (deg): [0.4 1.4 1.8 0.5 0.6]  [pan lift elbow wrist roll]
```

Cách đọc:

- **`target`**: vị trí camera tính ra (cộng 4 cm theo z).
- **`(servos)`**: vị trí thật của đầu hàm, tính từ góc servo đọc về. Nếu dòng này gần bằng `target` thì **robot đã đi đúng lệnh**. Khi đó, mọi độ lệch bạn nhìn thấy bằng mắt đều đến từ **phía camera**.
- **`joint sag`**: số độ servo bị võng trước khi được bù.

---

## 6. Căn chỉnh khi X hoặc Y còn lệch

Trước hết, xác định **loại lệch** bằng 4–5 dấu X ở các vùng khác nhau.

| Bạn thấy | Nguyên nhân | Cách sửa |
|---|---|---|
| Lệch **cùng hướng, cùng độ lớn** ở mọi dấu X (ví dụ luôn hụt 8 mm về phía đế) | Sai lệch cố định còn lại sau hiệu chuẩn | **6a. Hiệu chỉnh XY bằng phím** |
| Lệch **khác nhau** theo vùng (gần đế đúng, xa đế lệch; hoặc trái đúng, phải lệch) | Hiệu chuẩn bàn chưa tốt: ít điểm, điểm dồn một chỗ, click lệch | **6b. Làm lại bước 4** với nhiều điểm hơn, rải đều hơn |
| Dòng `(servos)` lệch xa `target` (trên 5 mm) | Robot không tới đúng lệnh: võng quá nhiều, nguồn yếu, vướng dây | **6c. Kiểm tra phía robot** |
| Trước đúng, giờ sai | Camera, đế robot hoặc tờ giấy trên bàn bị xê dịch | Vặn chặt lại, làm lại bước 4 |
| X, Y đúng, chỉ Z lệch vài mm | Độ sâu trên mặt bàn hơi sai | Chấp nhận được với mục tiêu của bước này. Khi gắp, `Reference/pick_place.py` dùng chiều cao vật bạn nhập |

### 6a. Hiệu chỉnh XY bằng phím (sai lệch cố định)

1. Trong `Reference/verify_click.py`, click một dấu X **ở giữa vùng làm việc**, nhấn `g`, rồi `y`, và chờ tay tới nơi.
2. Nhìn từ trên xuống rồi **đẩy đầu hàm cho trùng dấu X**. Mỗi lần nhấn dịch 2 mm, tay đi ngay:

   | Phím | Dịch đầu hàm |
   |---|---|
   | **`i`** | +x: ra xa đế |
   | **`k`** | −x: về phía đế |
   | **`j`** | +y: sang trái (nhìn từ phía sau robot) |
   | **`l`** | −y: sang phải |

   Dòng trạng thái hiện tổng đã dịch, ví dụ `nudge x +8 y -2 mm`.
3. Khi đầu hàm nằm **đúng trên tâm dấu X**, nhấn **`o`**: phần đã dịch được **cộng vào hiệu chỉnh XY** và lưu vào `json/camera_offset.json`. Từ đó, **mọi lần click** đều tự áp dụng hiệu chỉnh này, kể cả sau khi tắt và mở lại script.
4. **Kiểm tra lại** bằng 3–4 dấu X **khác**: click, `g`, `y`. Lúc này tay phải tới đúng mà không cần đẩy thêm.
   - Nếu dấu X nào còn lệch nhẹ, cùng hướng, thì đẩy thêm và nhấn `o` lần nữa. Các lần lưu **cộng dồn**.
   - Nếu mỗi dấu X cần đẩy theo **hướng khác nhau**, đó không còn là sai lệch cố định. Nhấn **`0`** để xoá hiệu chỉnh, rồi làm **6b**.

Mở file để xem hiệu chỉnh đang dùng:

```bash
cat json/camera_offset.json
```

> [!NOTE]
> Hiệu chỉnh XY chỉ sửa được **phần lệch giống nhau ở mọi nơi**. Nếu cần hiệu chỉnh lớn hơn khoảng 15 mm, nên làm lại bước 4 cẩn thận hơn thay vì bù bằng phím.

### 6b. Làm lại hiệu chuẩn bàn

1. Thêm dấu X ở **chỗ đang lệch nhiều nhất**, đặc biệt là các góc của vùng làm việc.
2. Chạy `python table_calib.py --resume` để giữ các cặp cũ và thêm cặp mới. Hoặc chạy không có `--resume` để làm lại từ đầu, nếu nghi các cặp cũ sai.
3. Nhấn `c`, xem các điểm bị đánh dấu. Mục tiêu là `mean error` dưới 4 mm.
4. **Xoá hiệu chỉnh XY cũ**, vì hiệu chuẩn mới đã khác. Trong `Reference/verify_click.py` nhấn **`0`**, hoặc xoá file `json/camera_offset.json`.
5. Làm lại bước 5. Nếu vẫn còn lệch cố định thì làm 6a.

### 6c. Kiểm tra phía robot

- So dòng `(servos)` với `target` trong log. Nếu lệch trên 5 mm dù đã bù võng, xem dòng `joint sag`: khớp nào võng nhiều (trên 3°)?
- **Nguồn 12 V** đủ dòng chưa? Dây servo có bị căng, kéo tay robot không?
- Xem `json/joint_offsets.json`: độ lệch lớn (trên 8–10°) nghĩa là calibrate servo chưa tốt.
- Chạy `python Reference/real_twin.py`: tư thế trong sim phải trùng tay thật. Nếu không trùng, calibrate servo lại theo [SETUP_ROBOT.md](SETUP_ROBOT.md), rồi làm lại bước 4 của file này.
- Muốn so sánh khi không bù võng thì chạy `python Reference/verify_click.py --no-sag`.

### 6d. Kẹp xuống bị chéo so với vật: căn khớp xoay cổ tay (`roll_check.py`)

Góc 0° của khớp **wrist_roll** là tư thế bạn để cổ tay lúc calibrate LeRobot, nên nó có thể lệch so với mô phỏng 20–45°. `table_calib.py` không đo được độ lệch này. Khi có lệch, robot xoay cổ tay đúng theo mô phỏng nhưng ngoài đời hai hàm kẹp vẫn nằm chéo với vật.

```bash
python roll_check.py
```

1. Đặt một khối lập phương cách đế 150–280 mm, rồi click vào mặt trên của nó.
   - Robot đưa kẹp (đang mở) lên phía trên khối, cổ tay xoay theo góc đo được.
   - **Đường vàng** trên ảnh là vị trí hai hàm kẹp theo mô phỏng (chấm tròn là hàm cố định).
2. Nhìn thẳng từ trên xuống. Nhấn **a / d** để xoay thêm 1° (**A / D** là 5°) cho tới khi hai hàm kẹp **vuông góc với mặt khối**. Nhấn **l** để hạ xuống thử, **u** để nhấc lên.
3. Nhấn **o** để lưu độ lệch vào `json/joint_offsets.json` (mục `wrist_roll`). Robot về home.
4. Kiểm tra: xoay khối đi 30–45°, rồi click lại. Lần này kẹp phải tự vuông góc với khối. Nếu kẹp xoay **ngược chiều** với khối thì dấu của khớp đang sai: hãy báo lại.

Ngoài ra, góc của vật giờ được lấy từ **cạnh trong ảnh màu**, kết hợp với ảnh độ sâu (`refine_yaw` trong `camera_pick.py`). Log của `hand_over.py` in ra cả hai giá trị: `yaw … (depth alone …, edges …)`.

---

## 7. Dùng để gắp vật và đi tới điểm 3D

Khi bước 5–6 đã đạt (X/Y lệch dưới ~5 mm), có ba cách dùng hiệu chuẩn.

### 7a. Gắp tự động bằng camera: `camera_pick.py` (khuyên dùng)

```bash
python camera_pick.py
```

1. **Click lên mặt trên của vật.** Script tự đo vật từ ảnh độ sâu:
   - vật là vùng nhô cao hơn mặt bàn trên 8 mm, được viền xanh;
   - script tính **tâm**, **chiều cao**, **bề rộng** (cạnh ngắn, là chiều hai hàm kẹp lại) và **góc xoay**, rồi in ra.
2. **Click chỗ muốn đặt** trên bàn (dấu chữ thập đỏ).
3. Nhấn **SPACE**. Script lập kế hoạch, chạy thử toàn bộ trong mô phỏng vật lý MuJoCo với hộp đúng kích thước và góc đo được.
   - Mô phỏng đạt: tay thật chạy luôn.
   - Mô phỏng không đạt: script báo lý do và không chạy.

Các phím:

| Phím | Tác dụng |
|---|---|
| `s` | Dừng tay ngay, giữ nguyên tư thế |
| `r` | Chọn lại từ đầu |
| `h` | Về tư thế home |
| `q` | Thoát |

Tuỳ chọn:
- `--speed 1.5`: tốc độ (mặc định 1.0, khoảng 16 giây cho một lần gắp và đặt; tối đa 2.0, khoảng 8 giây). Mô phỏng kiểm tra ở đúng tốc độ này, và giới hạn tốc độ servo cũng tăng theo. Lần đầu nên thử 1.0, sau đó 1.5.
  - `Reference/goto.py` cũng có `--speed` (1.0 = 8 cm/s khi đi thẳng).
- `--lower 20`: tâm kẹp xuống thấp hơn **nửa chiều cao vật** bao nhiêu mm (mặc định 20).
  - Đầu ngón kẹp luôn được giữ cách bàn ít nhất 4 mm, nên với vật thấp, tay chỉ xuống tới mức đó.
  - Dòng `> descend to object (grip centre … mm above the table)` cho biết độ cao thực tế đã dùng.
  - `--lower 0` là kiểu kẹp cũ, ở giữa thân vật.
- `--dry`: tay giả, chỉ kiểm tra camera và phép đo.

Log nằm ở `debug/camera_pick_log.txt`.

Vật phù hợp:
- Cao khoảng 15–60 mm. Vật cao hơn sẽ vượt tầm lúc mang đi.
- Rộng 10–65 mm theo chiều kẹp.
- Đứng trên bàn, cách đế 180–300 mm.
- Không bóng loáng hay trong suốt, vì D415 không đo được độ sâu trên các bề mặt này.

Nếu phép đo sai (viền xanh lẫn sang vật bên cạnh hoặc lẫn tay robot), hãy để các vật cách nhau trên 3 cm rồi nhấn `r` và click lại.

### 7a+. Nhận diện nhiều vật theo tên: `camera_pick.py --detect`

```bash
python camera_pick.py --detect "green cube, white cube, tape measure, black cylinder"
```

Cách hoạt động:
1. **Tìm vật bằng ảnh độ sâu.** Mọi thứ nhô cao hơn mặt bàn trên 8 mm và nằm cách đế 10–42 cm được coi là vật. **Tay robot được loại ra:** mô hình 3D của tay, đặt đúng theo góc khớp đang đọc được, được chiếu vào ảnh camera.
2. **Gọi tên bằng CLIP.** Mỗi vật được cắt ra từ ảnh màu, rồi so với danh sách tên bạn gõ. Vật nào trông giống "robot arm" hoặc "empty table" thì bị bỏ. Tên có dấu `?` nghĩa là CLIP không chắc (xác suất dưới 0,35).
3. **Đo vật:** tâm, chiều cao, bề rộng và góc xoay, đo bằng ảnh độ sâu như ở 7a.
4. Trên ảnh, mỗi vật có **số thứ tự**:
   - **khung xanh:** gắp được;
   - **khung đỏ:** không gắp được, kèm lý do (quá rộng, quá thấp, ngoài tầm với).
5. Nhấn **phím số 1–9** (hoặc click vào khung) để chọn vật, click chỗ đặt, rồi nhấn **SPACE**.

Mẹo đặt tên:
- Tên tiếng Anh, ngắn, mô tả nhìn thấy được: màu + hình dạng, ví dụ "green cube", "red tape measure".
- Nên gõ tên cho **mọi vật trên bàn**, kể cả vật không định gắp. CLIP luôn chọn tên gần nhất trong danh sách, nên thiếu tên thì vật lạ sẽ bị gán nhầm tên khác.

Giới hạn:
- Vật mỏng dưới khoảng 8–10 mm (USB, giấy) không có trong ảnh độ sâu nên không được tìm thấy.
- Hai vật chạm nhau sẽ thành một: hãy để chúng cách nhau trên 1–2 cm.
- Việc nhận diện tạm dừng trong lúc tay đang chạy, và mất khoảng 0,3 giây mỗi lần trên CPU.

`--backend yolo` dùng YOLO-World (`models/yolov8s-worldv2.pt`) thay cho ảnh độ sâu. Trên camera của bạn (640x480, nhìn từ trên xuống), YOLO-World nhận kém: không thấy khối nhỏ và gọi nhầm tên. Vì vậy cách độ sâu + CLIP là mặc định.

Thử nhanh trên một ảnh, kết quả lưu ở `debug/detect.jpg`:

```bash
python Reference/detector.py "green cube, white cube, tape measure"
```

### 7b. Gắp với số đo tay: `Reference/pick_place.py`

1. Chạy `python Reference/verify_click.py --tcp`, click lên **tâm của vật** và đọc `x`, `y`.
2. Đo chiều cao và bề rộng của vật bằng thước.
3. Chạy lệnh, ví dụ:

```bash
python Reference/pick_place.py 245 -30  220 100 --height 50 --width 30 --real
```

### 7c. Đưa tay gắp tới một điểm trong không gian: `Reference/goto.py`

```bash
python Reference/goto.py 250 0 80
```

Lệnh trên đưa **tâm kẹp** tới x 250, y 0, z 80 mm, chúi xuống 80°.
- `--pitch 30`: nghiêng 30° dưới phương ngang (0 = nằm ngang, 90 = thẳng đứng).
- `--jaw`: ngắm đầu hàm cố định thay vì tâm kẹp, giống `Reference/verify_click.py`.

Sau mỗi lần di chuyển, tay **giữ nguyên tư thế** và chờ lệnh tiếp theo:

| Gõ | Tác dụng |
|---|---|
| `200 -50 120` | Đi tới x y z (mm) |
| `200 -50 120 45` | Đi tới x y z, pitch 45° |
| `o` / `c` / `g 30` | Mở / đóng / đặt tay kẹp ở mức 30 (0 = đóng, 100 = mở) |
| `w` | In vị trí hiện tại |
| `h` | Về home |
| `q` | Thoát |

Cách tay di chuyển:
- Nếu tay đang thấp (dưới 6 cm), nó **nâng thẳng lên** trước.
- Sau đó nó xoay tới phía trên điểm đích, rồi **hạ thẳng xuống**, nên không quét qua vật trên bàn.
- Pitch yêu cầu không với tới được thì script tự chọn pitch gần nhất với tới được và báo ra.
- Script từ chối các điểm làm đầu ngón kẹp chạm bàn.
- Tới nơi, script bù độ võng servo và in sai số so với điểm đích. Sai số này tính theo góc servo, không phải theo camera.

Trong `Reference/verify_click.py`, phím **`+` / `-`** đổi độ cao phía trên điểm click, mỗi lần 1 cm, từ 5 mm tới 20 cm. Nếu tay đang ở đó, nó di chuyển luôn tới độ cao mới.

### 7d. Đưa vật vào lòng bàn tay: `hand_over.py`

Bạn chìa bàn tay mở ra, robot tự gắp vật đã chọn, thả vào lòng bàn tay, rồi về home. Rút tay ra và chìa lại thì robot làm vòng tiếp theo.

**Chọn vật** bằng ảnh mẫu. CLIP so sánh ảnh mẫu với từng vật trên bàn: điểm giống nằm trong khoảng 0–1 và hiện trên mỗi khung.

```bash
python hand_over.py --ref targets/cube.jpg     # một hoặc nhiều ảnh, hoặc một thư mục
python hand_over.py                            # mọi ảnh trong targets/
```

**Nhiều vật, có thứ tự ưu tiên:** đặt **số ở đầu tên file**. Các ảnh cùng số là cùng một vật:

```
targets/1_green_cube.jpg      ưu tiên 1
targets/2_white_cube.jpg      ưu tiên 2
targets/2_white_cube_b.jpg    thêm một ảnh của khối trắng (góc khác, ánh sáng khác)
```

Robot đưa khối xanh trước, khi khối xanh còn trên bàn. Nó chỉ đưa khối trắng khi không còn khối xanh, hoặc khi khối xanh đang nằm ở chỗ không gắp được. Mỗi khung trên màn hình hiện tên vật mà nó giống nhất cùng điểm giống. Muốn đổi thứ tự thì chỉ cần đổi số trong tên file.

Chụp ảnh mẫu bằng `python Reference/snap_target.py`. Lệnh này cắt mỗi vật ra `targets/candidates/obj_N.jpg`, xem ảnh tổng ở `debug/snap_overview.jpg`. Sau đó lưu ảnh mình chọn, ví dụ `python Reference/snap_target.py --keep 4 2_white_cube`. Có thể dạy ngay trong `hand_over.py`: nhấn **t**, rồi click vào khung của vật. Ảnh được lưu thành `targets/N_taught_….jpg` với **ưu tiên thấp nhất**.

**Chạy.** Chương trình khởi động ở trạng thái **PAUSED**. Nhấn **SPACE** để bắt đầu vòng tự động:

1. Robot chờ tới khi thấy **vật đích** (khung xanh `TARGET`) và **một bàn tay mở, giữ yên 1 giây**. Bàn tay phải cách trục đế 140–310 mm và không cao hơn mặt bàn quá 11 cm.
2. Robot lập kế hoạch gắp, kiểm tra trong MuJoCo, rồi gắp. Bộ lập kế hoạch giống `camera_pick.py`.
3. Robot nhấc vật lên và **tìm lại bàn tay** (lấy vị trí mới). Nó đưa vật tới phía trên lòng bàn tay, hạ chậm 6 cm cuối, mở kẹp khi **đáy vật còn cách lòng bàn tay khoảng 3 cm**, đi lên rồi về home.
4. Robot chờ bạn **rút tay ra** (không thấy tay trong 1 giây), rồi bắt đầu vòng mới.

Các trường hợp đặc biệt:

- Nếu tay **dịch hơn 5 cm** trong lúc robot đang hạ xuống, robot dừng, vẫn giữ vật và ngắm lại.
- Nếu **30 giây** không thấy tay, robot đặt vật về chỗ cũ và chuyển sang PAUSED.

| Phím | Tác dụng |
|---|---|
| SPACE | Bắt đầu hoặc tạm dừng vòng tự động. Sau `s` khi đang cầm vật: tiếp tục đưa vào tay |
| s | Dừng tay robot ngay (giữ nguyên tư thế) và tạm dừng |
| t | Dạy vật đích: click vào khung của vật |
| x | Quên các ảnh đã dạy (file vẫn còn trong `targets/`) |
| + / − | Tăng hoặc giảm ngưỡng giống (mặc định `--sim 0.80`) |
| h | Về home (khi đang PAUSED) |
| q | Thoát |

**Màu trên màn hình**

- **Bàn tay:** xanh lá = sẵn sàng (mở, yên, trong tầm); vàng = mở nhưng chưa yên hoặc ngoài tầm; xám = nắm lại.
- **Vật:** xanh lá dày = vật đích; cam = giống nhưng không gắp được; xám = vật khác.

Nhận bàn tay dùng MediaPipe Hands (`models/hand_landmarker.task`, chạy offline trên CPU). Vị trí 3D của lòng bàn tay lấy từ ảnh độ sâu. Mọi bước được ghi vào `debug/hand_over_log.txt`.

> [!WARNING]
> Robot di chuyển tới tay người. Vài lần đầu, hãy:
> - chạy `--speed 0.5`;
> - để tay kia sẵn trên phím **s**;
> - giữ lòng bàn tay **phẳng, ngửa lên, đứng yên**;
> - để mặt tránh xa tay robot.
>
> Không dùng với vật sắc, nóng, hoặc nặng hơn khoảng 100 g.

### Tự động hoàn toàn

Thay bước click bằng bộ nhận diện vật (lọc màu HSV, YOLO…): lấy pixel tâm vật, rồi gọi `measure()` trong `camera_pick.py` (đo vật) hoặc `pixel_to_base()` trong `Reference/verify_click.py` (chỉ lấy toạ độ).

---

## 8. Các file

| File | Nội dung |
|---|---|
| `table_calib.py` | Hiệu chuẩn trên mặt bàn (bước 4) |
| `Reference/verify_click.py` | Click để kiểm tra, đẩy và lưu hiệu chỉnh XY (bước 5, 6) |
| `camera_pick.py` | Gắp tự động: click vật và chỗ đặt (bước 7a) |
| `Reference/goto.py` | Đưa tay gắp tới điểm 3D (bước 7c) |
| `Reference/detector.py`, `models/yolov8s-worldv2.pt` | Nhận diện nhiều vật theo tên (bước 7a+) |
| `hand_over.py`, `models/hand_landmarker.task`, `targets/` | Đưa vật vào lòng bàn tay (bước 7d) |
| `roll_check.py` | Căn khớp xoay cổ tay với mô phỏng (bước 6d) |
| `Reference/snap_target.py` | Chụp ảnh mẫu vật cho `hand_over.py` |
| `T_base_cam.npy`, `json/T_base_cam.json` | Kết quả hiệu chuẩn camera → robot |
| `json/table_points.json` | Các cặp điểm của hiệu chuẩn bàn |
| `json/camera_offset.json` | Hiệu chỉnh XY (mm), cộng vào mọi điểm camera |
| `json/joint_offsets.json` | Độ lệch góc nhỏ của các khớp (độ), mọi script nạp qua `Reference/so101_real.py`. Xoá file này để quay về không lệch |
| `debug/verify_log.txt` | Nhật ký các lần kiểm tra |
| `npy/T_base_cam_handeye.npy` | Kết quả hiệu chuẩn marker cũ (dự phòng) |
| `Reference/handeye_calib.py`, `Reference/marker_check.py`, `json/samples.json` | Hiệu chuẩn bằng marker (phụ lục A) |

---

## 9. Lỗi thường gặp

| Triệu chứng | Nguyên nhân hay gặp | Cách sửa |
|---|---|---|
| `Device or resource busy`, hoặc `The camera is used by another program` | Chương trình khác đang giữ camera; thường là `d415-lab.service` (tự bật lại sau 5 giây nếu chỉ tắt `app.py`) | `systemctl --user stop d415-lab.service`; xem ai đang giữ camera bằng `fuser -v /dev/video*` |
| `Couldn't resolve requests` | Camera cắm USB 2 không hỗ trợ chế độ được yêu cầu (bản script cũ) | Bản hiện tại tự chọn chế độ phù hợp; nên cắm USB 3 |
| `No RealSense camera found` | Chưa cắm camera, hoặc cáp lỏng | Cắm lại, kiểm tra bằng `realsense-viewer` |
| Cửa sổ ảnh như bị treo | Script đang chờ ở terminal (bản cũ), hoặc đang ở câu hỏi `Support the arm…` khi thoát | Bản hiện tại làm mọi thứ trong cửa sổ ảnh; khi thoát thì đỡ tay robot rồi nhấn Enter ở terminal |
| `no depth at that pixel` | Mặt bóng, đen, quá gần camera, hoặc bị tay robot che | Click lệch ra một chút, dán giấy mờ, đưa tay robot ra chỗ khác |
| Click mặt bàn mà z khác 0 nhiều (trên 10 mm) | Hiệu chuẩn chưa đúng ở mặt bàn | Làm bước 4 (hiệu chuẩn bàn) |
| `out of reach` | Điểm quá gần hoặc quá xa đế | Chọn điểm cách đế 180–300 mm |
| Lệch cố định theo X/Y | Sai lệch còn lại | Mục 6a |
| Lệch khác nhau theo vùng | Hiệu chuẩn bàn thiếu điểm hoặc có điểm sai | Mục 6b |
| Hôm qua đúng, hôm nay lệch | Camera hoặc robot bị xê dịch | Vặn chặt, làm lại bước 4, xoá hiệu chỉnh XY |
| Số fps thấp (dưới 10) | Băng thông USB 2 | Cắm USB 3 |

---

## Phụ lục A. Hiệu chuẩn bằng marker ArUco (không bắt buộc)

Đây là cách hiệu chuẩn đầu tiên đã dùng: marker dán trên tay gắp, camera đo vị trí marker bằng `solvePnP` ở 15–25 tư thế. Với tay này, kết quả **không đủ chính xác trên mặt bàn** (xem mục 1), nên hiện chỉ dùng để tham khảo hoặc kiểm tra chéo. Hiệu chuẩn bàn (bước 4) không cần marker.

**Marker**: từ điển `DICT_4X4_50`, **ID 1** (mặc định của script), cạnh phần đen khoảng 30 mm.

- **Bắt buộc có viền trắng ít nhất 5 mm** quanh phần đen. Không có viền trắng thì OpenCV không nhận ra marker. Nếu lỡ cắt mất viền, dán marker lên giấy trắng hoặc bìa trắng rộng hơn khoảng 5 mm mỗi bên.
- In ở tỉ lệ 100%, giấy mờ, dán **phẳng** lên mặt ngoài **hàm cố định**.
- Có thể vẽ tay. Mẫu 6×6 ô của ID 1 (■ = đen):

```
■ ■ ■ ■ ■ ■
■ ■ ■ ■ ■ ■
■ □ □ □ □ ■
■ □ ■ ■ □ ■
■ □ ■ □ ■ ■
■ ■ ■ ■ ■ ■
```

**Kiểm tra camera nhận marker** (in lời khuyên nếu không nhận được: sai ID, quá nhỏ, mờ, loá, mất viền):

```bash
python Reference/marker_check.py
```

**Thu mẫu** (torque tắt; SPACE ghi mẫu, `u` bỏ mẫu vừa ghi, `c` tính, `x` bỏ mẫu sai nhất, `q` thoát). Nên thu 15–25 mẫu, rải đều vị trí, độ cao và góc cổ tay:

```bash
python Reference/handeye_calib.py --marker-size 0.0298
```

**Nếu script in `HINT: the data fits best with --marker-size …`**, nghĩa là camera "thấy" marker nhỏ hơn cạnh đo bằng thước kẹp. Với tay này, camera thấy khoảng 27 mm trong khi thước kẹp đo 29,8 mm. Kiểm tra độc lập bằng cảm biến độ sâu:

```bash
python Reference/marker_check.py --depth 0.0298
```

Tính lại với cạnh hiệu dụng, không cần thu mẫu lại:

```bash
python Reference/handeye_calib.py --solve-only --marker-size 0.0269
```

**Lưu ý:** `Reference/handeye_calib.py` ghi đè `T_base_cam.npy`. Sau khi chạy nó, phải làm lại bước 4 rồi mới kiểm tra (bước 5).
