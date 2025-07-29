# ROS2 NODE IMPLEMENTATION

## Deskripsi Program

Program ini berfungsi sebagai jembatan komunikasi antara Node `twist_command_randomizer` dan `movement_reader` yang terdapat pada package `magang_2025`

Node ini harus:
- **Subscribe** ke topic `/autonomous_vel`
- **Forward** data Twist ke `/cmd_vel`
- **Publish** string `autonomous` ke topic `/cmd_type`

---

## Struktur Workspace

```bash
└── ros_ws
  └── src
    ├── ms2-tubes1-cakrai17
    └── ms2-tb1-cakrai17-13524104
      └── pkg_13524104
        ├── scripts
          └── connector_node.py
        ├── config
          └── bridge_config.yaml
        ├── launch
          └── connector_launch.py
        ├── CMakeLists.txt
        └── package.xml
```

---

## Spesifikasi Bonus 1

Menambahkan:
- Launch file: `connector_launch.py`
- Config file: `connector_config.yaml`

### Penjelasan

Bonus ini bertujuan untuk menjalankan Node menggunakan **launch file** dan **YAML file**.

Dengan adanya `connector_launch.py`, Node `connector_node.py` dapat dijalankan hanya dengan satu perintah:

```bash
ros2 launch pkg_13524104 connector_launch.py
```
YAML config `connector_config.yaml` dapat digunakan untuk menyimpan parameter tambahan yang dapat dibaca Node. Namun, dalam kasus ini hanya ada 1 parameter yang digunakan, yaitu `/autonomous_vel` sehingga tidak terlihat perbedaannya antara membuat YAML file dengan langsung membuatnya di file python

---

## Spesifikasi Bonus 2

Untuk memenuhi spesifikasi bonus kedua, dibuat sebuah Node bernama `multiplexer_node.py` yang bertugas sebagai multiplexer dari tiga topik input berisi Twist, yaitu:

- /keyboard_vel
- /joy_vel
- /autonomous_vel

Node ini akan meneruskan pesan Twist ke topik output /cmd_vel berdasarkan prioritas berikut:
```bash
keyboard_vel > jov_vel > autonomous_vel
```

Selain itu, Node ini juga melakukan publikasi jenis sumber ke topik /cmd_type dengan nilai:

- "keyboard" jika sumber berasal dari /keyboard_vel
- "joy" jika dari /joy_vel
- "autonomous" jika dari /autonomous_vel

## Cara Kerja

Setiap 0.2 detik (5Hz), Node akan:

1. Mengecek apakah terdapat pesan dari topik `keyboard_vel`, jika ada akan diteruskan langsung ke `/cmd_vel`.
2. Jika tidak ada pesan dari `keyboard_vel`, Node akan memeriksa `joy_vel`.
3. Jika tidak ada juga, baru memproses `autonomous_vel`.
4. Setelah mengirim pesan Twist, Node juga mengirim String ke `/cmd_type` untuk memberitahu sumber data aktif.

## Contoh Output

```bash
[KEYBOARD] ➤ Lin.x = +0.45 m/s, Ang.z = -0.30 rad/s
[JOY] ➤ Lin.x = +0.20 m/s, Ang.z = +0.10 rad/s
[AUTONOMOUS] ➤ Lin.x = +0.50 m/s, Ang.z = -0.60 rad/s
```
