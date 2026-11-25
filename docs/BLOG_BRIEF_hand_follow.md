# Brief viết blog: Điều khiển cánh tay robot bằng bàn tay (hand retargeting)

## Thông tin cho người viết
- Trang: dqtech.cloud · category: `tech-tips` · ngôn ngữ: tiếng Việt
- Đối tượng: người mới tìm hiểu robot/AI, sinh viên, người thích DIY
- Giọng văn: thân thiện, dễ hiểu. Mỗi thuật ngữ đều giải thích bằng một câu đời thường
- Trạng thái: chương trình đã viết xong và chạy thử trên máy (mô phỏng), CHƯA chạy với camera và robot thật. Bài phải ghi rõ là "đang thử nghiệm", không được viết như thể đã chạy thật
- Gợi ý tiêu đề: "Điều khiển cánh tay robot bằng bàn tay: không găng tay, chỉ cần một camera"

## Phần cứng và phần mềm
- Cánh tay robot: SO-101 (mã nguồn mở, 5 khớp + 1 gripper)
- Camera chiều sâu: Intel RealSense D415
- Phần mềm: Python, OpenCV (nhận diện marker ArUco), MediaPipe Hands (nhận diện bàn tay), MuJoCo (mô hình robot để tính động học), LeRobot (driver điều khiển robot)

## Ý tưởng chính: điều khiển TƯƠNG ĐỐI
- Robot đứng ở vị trí xuất phát. Camera ghi lại vị trí bàn tay (hoặc marker) tại thời điểm đó làm MỐC.
- Sau đó chỉ đo ĐỘ DỊCH của tay so với mốc, rồi cho đầu kẹp robot dịch đúng bằng như vậy (tỉ lệ 1:1).
- Lợi ích: không cần đo hay hiệu chuẩn vị trí camera so với robot. Chỉ cần camera nhìn CÙNG HƯỚNG với robot.
- Quy ước trục: tay ra xa camera → robot tiến tới; tay sang phải trong ảnh → robot sang phải; tay lên → robot lên.

## Hai phiên bản đã làm
### 1. Theo marker (marker_follow.py)
- Marker: ArUco DICT_4X4_50, ID 0, gắn trên một tay cầm in 3D.
- Vị trí 3D = tâm marker trong ảnh + độ sâu trung vị trên marker. Không cần biết kích thước marker.
- Chỉ điều khiển vị trí (trái/phải, lên/xuống, trước/sau); góc chúi của gripper giữ cố định.

### 2. Theo bàn tay (hand_follow.py)
| Bàn tay | Robot |
|---|---|
| Di chuyển | Đầu kẹp di chuyển theo, 1:1 |
| Nắm lại / mở ra | Gripper kẹp / mở (mở nửa chừng thì gripper mở nửa chừng) |
| Xoay lòng bàn tay | Cổ tay robot (wrist roll) xoay đúng góc đó |
- Vị trí lấy ở gốc lòng bàn tay, nên khi nắm tay robot không bị giật.
- Độ mở bàn tay = tỉ lệ (cổ tay → đầu ngón) / (cổ tay → khớp gốc ngón): xòe ≈ 1.8, nắm ≈ 1.0.
- Góc xoay = góc quay của đường nối khớp ngón trỏ → khớp ngón út, quanh trục dọc ngón tay.
- Robot vẫn bám theo khi tay đang nắm, nên có thể kẹp đồ rồi mang đi.

## Các bài học / chi tiết hay để kể
- Vị trí "home" gập gọn của robot là chỗ xuất phát tồi: ở tư thế đó cánh tay gần như duỗi thẳng đứng (gần điểm kỳ dị), chỉ tiến được khoảng 1 cm. Giải pháp: thêm tư thế START ở giữa vùng làm việc (đầu kẹp cách 20 cm phía trước, cao 15 cm, chúi 45°). Từ START robot với được khoảng ±30 cm sang hai bên, lên 30 cm, xuống 13 cm.
- Chống rung có 3 lớp: bộ lọc One-Euro → giới hạn tốc độ đầu kẹp 0,15 m/s → giới hạn mỗi khớp 2°/lệnh.
- An toàn:
  - Tay ra ngoài tầm với: màn hình viền đỏ, robot dừng ở mép vùng làm việc.
  - Không bao giờ xuống thấp hơn 1 cm so với mặt bàn.
  - Mất dấu tay: robot đứng yên.
  - Luôn chạy chế độ `--dry` (robot không cử động) trước để kiểm tra hướng.
- Kết quả thử mô phỏng:
  - Tay sang phải 6 cm, lên 4 cm, ra xa 5 cm → đầu kẹp sang phải 60 mm, lên 40 mm, tiến 50 mm.
  - Nắm tay → gripper đóng, robot không xê dịch.
  - Xoay tay 40° → cổ tay robot xoay 40°.
  - Nhiễu độ sâu 3 mm, sau lọc còn khoảng 0,9 mm.

## Cách chạy (code block trong bài)
```bash
conda activate lerobot
cd ~/robot/SO101
python hand_follow.py --dry   # chỉ camera + tính toán, robot đứng yên
python hand_follow.py         # robot thật
```
Phím: SPACE = về START và lấy mốc (đưa bàn tay MỞ ra trước camera) · p = tạm dừng · h = về START · q = thoát (robot về home).
Tùy chọn hay dùng: `--mirror` (đảo trái/phải), `--roll-sign -1` (đảo chiều xoay cổ tay), `--open` / `--fist` (ngưỡng xòe/nắm).

## Thuật ngữ cần giải thích
- **Teleoperation**: điều khiển robot từ xa; người làm, robot làm theo.
- **Vision-based teleoperation**: teleoperation chỉ dùng camera, không găng tay cảm biến, không tay điều khiển.
- **Motion / hand retargeting**: "dịch" chuyển động của người sang robot, vì hai bên khác nhau về kích thước và số khớp (ví dụ 5 ngón tay → 1 gripper).
- **Điều khiển tương đối & clutching**: chỉ dùng độ dịch so với mốc; lấy mốc lại giống nhấc chuột lên đặt chỗ khác.
- **Camera chiều sâu (depth camera)**: camera đo được khoảng cách tới từng điểm ảnh, nên biết vật ở xa hay gần.
- **ArUco marker**: ô vuông đen trắng giống mã QR đơn giản, máy tính nhận ra rất nhanh và chính xác.
- **MediaPipe Hands**: thư viện của Google, tìm 21 điểm khớp trên bàn tay trong ảnh, có cả tọa độ 3D.
- **Động học ngược (Inverse Kinematics, IK)**: biết đầu kẹp cần tới đâu, tính ngược ra mỗi khớp phải quay bao nhiêu độ.
- **Điểm kỳ dị (singularity)**: tư thế (ví dụ tay duỗi thẳng) mà robot khó hoặc không thể di chuyển theo một số hướng.
- **Bộ lọc One-Euro**: bộ lọc làm mượt tín hiệu; mượt khi tay đứng yên, phản ứng nhanh khi tay di chuyển.
- **Imitation learning**: robot học bằng cách xem người làm mẫu; teleoperation là cách thu dữ liệu mẫu.

## Dự án tham khảo (nhắc tên, kèm nguồn chính thức)
AnyTeleop, DexPilot (NVIDIA), thư viện dex-retargeting, Open-TeleVision, Bunny-VisionPro.

## Gợi ý hình minh họa
1. Sơ đồ: camera D415 → máy tính (MediaPipe + IK) → cánh tay SO-101.
2. Bảng/hình "bàn tay → robot": di chuyển / nắm-mở / xoay.
(Ảnh hoặc video thật của robot nên bổ sung sau khi chạy thật.)

## Hướng tiếp theo (kết bài)
- Chạy trên robot thật và chỉnh ngưỡng xòe/nắm.
- Thêm gập cổ tay lên/xuống (wrist flex).
- Ghi lại dữ liệu để huấn luyện robot tự làm (imitation learning).
