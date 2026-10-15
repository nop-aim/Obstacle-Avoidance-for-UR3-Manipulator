# Obstacle Avoidance for UR3 Manipulator

Repository này chứa mã nguồn, mô phỏng và tài liệu nghiên cứu cho đề tài **Nghiên cứu & Thiết kế Thuật toán Tránh Vật cản cho Cánh tay Robot UR3 (Obstacle Avoidance for Collaborative Robots)**.

---

## 📂 Cấu trúc & Thành phần Dự án

| Thành phần / Tệp tin | Loại | Mô tả chi tiết |
| :--- | :---: | :--- |
| **`main_real/`** | Thư mục | Mã nguồn Python/C++ triển khai thuật toán tránh vật cản trực tiếp trên cánh tay robot UR3 thật. |
| **`JCK_simulink/`** | Thư mục | Các mô hình mô phỏng Động học ngược (Inverse Kinematics) & Bộ điều khiển trên MATLAB & Simulink. |
| **`data_coll/`** | Thư mục | Tập dữ liệu thu thập (góc khớp, vận tốc, vị trí Cartesian, dữ liệu cảm biến vật cản). |
| **`NCKH/`** | Thư mục | Tài liệu, báo cáo tiến độ và kết quả Nghiên cứu Khoa học. |
| **`print/`** | Thư mục | Bản vẽ kỹ thuật, sơ đồ hệ thống, mô hình 3D chuẩn bị cho in ấn/báo cáo. |
| **`BaoCao_ThuatToan_TrachVatCan_UR3.docx`** | Tài liệu | Báo cáo thuyết minh chi tiết về cơ sở lý thuyết và thuật toán tránh vật cản cho UR3. |
| **`Khansari_Billard_AR12_1.pdf`** | Tài liệu | Bài báo khoa học nền tảng về thuật toán tránh vật cản dựa trên Hệ thống Động lực học (Dynamical Systems). |
| **`ICMT2025_98.pdf`** | Tài liệu | Bài báo tham khảo từ Hội thảo Quốc tế ICMT 2025. |
| **`Obstacle_Avoidance_for_a_UR3_Robot...pdf`** | Tài liệu | Tài liệu nghiên cứu chuyên sâu về bài toán tránh vật cản ứng dụng riêng trên UR3. |
| **`s41598-025-17740-5.pdf`** | Tài liệu | Bài báo khoa học bổ trợ thuật toán điều khiển và quỹ đạo. |

---

## 🚀 Công nghệ & Cấu hình Hệ thống

| Tiêu chí | Thông tin chi tiết |
| :--- | :--- |
| **Thiết bị phần cứng** | Cánh tay robot công nghiệp **UR3 (Universal Robots)** |
| **Ngôn ngữ lập trình** | Python, C++, MATLAB Script |
| **Môi trường mô phỏng** | MATLAB / Simulink, ROS / Gazebo |
| **Thuật toán chính** | Avoidance based on Dynamical Systems / Kinematics Control |

---

## 📝 Hướng dẫn Chạy Hệ thống

| Bước | Hành động | Lệnh / Thao tác |
| :---: | :--- | :--- |
| **1** | **Chạy Mô phỏng** | Mở MATLAB $\rightarrow$ Truy cập thư mục `JCK_simulink/` $\rightarrow$ Mở và chạy file `.slx` chính. |
| **2** | **Chạy Robot Thực** | Ket nối robot UR3 cùng mạng LAN $\rightarrow$ Truy cập `main_real/` $\rightarrow$ Chạy lệnh: `python main.py` |

---
*Đồ án Tốt nghiệp / Nghiên cứu Khoa học - Chuyên ngành Điều khiển & Tự động hóa.*