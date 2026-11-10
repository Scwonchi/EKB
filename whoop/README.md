# whoop — микродрон «Сверх»

## Модель распознавания: человек и красное окно

`yolo_train_and_export.ipynb` — ноутбук для Google Colab: обучает YOLO11n на датасете из Roboflow
и переводит модель в `.cvimodel` — формат нейроускорителя бортового компьютера (процессор cv181x).
Сделан по ноутбуку производителя; команды конвертации (tpu-mlir 1.14) те же, настройки собраны в
первую ячейку. **В Colab целиком ещё не запускался.**

1. Соберите датасет в Roboflow: классы `person` (манекены, ростовые фигуры, плакаты, часть наполовину
   закрыта предметами) и `window` (красное окно). Снимайте камерой микродрона.
2. Откройте ноутбук в Colab, выберите среду с GPU, впишите ссылку на датасет в первой ячейке и
   запустите все ячейки. Модель появится на Google Диске.
3. Скопируйте модель на дрон и запустите детектор:

```bash
scp rescue.cvimodel root@10.10.0.1:/root/        # с Windows — 10.10.1.1
ssh root@10.10.0.1                               # пароль root
cp /root/config_sample.yaml /root/config.yaml    # model.path, model.class_cnt, classes.names
killall face_recognizer
/usr/opt/tiny_control/yolo_detector /root/config.yaml
```

4. Видео с рамками: `ffplay -fflags nobuffer -flags low_delay -rtsp_transport udp rtsp://10.10.0.1/h264`.
5. В программе полёта результаты читает `drone.get_detections()` (кадр 1280 × 720).

Документация: https://edu.sverk.tech/learn/whoop/neural-training
