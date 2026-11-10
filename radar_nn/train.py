"""
Обучение нейросети по записям радара и экспорт в model.h для ESP32-S3.

Пример:
    python train.py --person dvizhetsya,nepodvizhen
    python train.py --data data_capsule --out capsule/model.h        # капсула (capsule/)
    python train.py --data data_capsule_sim --out capsule/model.h    # синтетика tools/capsule_sim

Читает data/<метка>/*.csv (сделаны record.py), обучает небольшую сеть
и пишет:
    ld2410c_nn/model.h   — веса и константы для скетча (копируется рядом со скетчем)

Установка: pip install numpy   (TensorFlow не нужен: сеть небольшая, обучается на numpy)
В папке данных файл SYNTHETIC — записи синтетические: в model.h ставится NN_SYNTHETIC 1,
и капсула по умолчанию только оценивает сетью (nn advise), а не решает.
"""
import argparse
import glob
import os
import sys

import numpy as np

# ---- Параметры окна и признаков (C++ берёт их из model.h) ----
FS = 10            # Гц, частота кадров логгера
WIN = 100          # окно 10 с
STEP = 10          # сдвиг окна 1 с
N_CH = 18
BANDS = [(1, 6), (7, 20), (21, 50)]   # номера бинов ДПФ: 0,1–0,6 Гц (дыхание), 0,7–2 Гц, 2,1–5 Гц
N_FEAT = N_CH * (2 + len(BANDS))
H1, H2 = 32, 16
TRAIN_FRAC = 0.8


def window_features(w: np.ndarray) -> np.ndarray:
    """w: (WIN, 18). Ровно то же считает nn_core.h на ESP32."""
    feats = []
    for c in range(N_CH):
        x = w[:, c].astype(np.float64)
        mean = x.mean()
        std = x.std()
        d = x - mean
        p = np.abs(np.fft.rfft(d)) ** 2 / WIN
        feats.append(mean)
        feats.append(std)
        for lo, hi in BANDS:
            feats.append(np.log1p(p[lo:hi + 1].sum()))
    return np.array(feats, dtype=np.float32)


def load(data_dir):
    labels = sorted(d for d in os.listdir(data_dir) if os.path.isdir(os.path.join(data_dir, d)))
    if len(labels) < 2:
        sys.exit(f"В {data_dir} нужно минимум две папки-метки")
    Xtr, ytr, Xva, yva = [], [], [], []
    print("Записи:")
    for li, lab in enumerate(labels):
        files = sorted(glob.glob(os.path.join(data_dir, lab, "*.csv")))
        ntr = nva = 0
        secs = 0
        for f in files:
            a = np.loadtxt(f, delimiter=",", skiprows=1, ndmin=2)
            if a.shape[1] != N_CH:
                print(f"  пропуск {f}: {a.shape[1]} столбцов")
                continue
            T = len(a)
            secs += T / FS
            if T < WIN:
                print(f"  пропуск {f}: короче окна ({T / FS:.0f} с)")
                continue
            # Разбиение по времени внутри записи: окна не перекрываются между train и val
            cut = int(T * TRAIN_FRAC)
            for s in range(0, T - WIN + 1, STEP):
                e = s + WIN
                if e <= cut:
                    Xtr.append(window_features(a[s:e])); ytr.append(li); ntr += 1
                elif s >= cut:
                    Xva.append(window_features(a[s:e])); yva.append(li); nva += 1
        print(f"  {lab:15s} файлов {len(files):3d}, {secs / 60:5.1f} мин, окон train {ntr}, val {nva}")
    return labels, np.array(Xtr), np.array(ytr), np.array(Xva), np.array(yva)


def c_array(name, arr, per_line=8):
    flat = np.asarray(arr, dtype=np.float32).ravel()
    body = ",\n  ".join(
        ", ".join(f"{v:.8e}f" for v in flat[i:i + per_line]) for i in range(0, len(flat), per_line)
    )
    return f"static const float {name}[{len(flat)}] = {{\n  {body}\n}};\n"


def export_header(path, weights, mu, sd, labels, person, synthetic=False, info=""):
    W1, b1, W2, b2, W3, b3 = weights
    is_person = [1 if l in person else 0 for l in labels]
    with open(path, "w", encoding="utf-8") as f:
        f.write("// Сгенерировано train.py — не редактировать вручную\n#pragma once\n\n")
        f.write(f"#define NN_SYNTHETIC {1 if synthetic else 0}\n#define NN_INFO \"{info}\"\n")
        f.write(f"#define NN_FS {FS}\n#define NN_WIN {WIN}\n#define NN_STEP {STEP}\n#define NN_CH {N_CH}\n")
        f.write(f"#define NN_FEAT {N_FEAT}\n#define NN_H1 {H1}\n#define NN_H2 {H2}\n#define NN_CLASSES {len(labels)}\n")
        f.write(f"#define NN_NBANDS {len(BANDS)}\n")
        f.write("static const int NN_BAND_LO[NN_NBANDS] = {" + ", ".join(str(b[0]) for b in BANDS) + "};\n")
        f.write("static const int NN_BAND_HI[NN_NBANDS] = {" + ", ".join(str(b[1]) for b in BANDS) + "};\n")
        f.write("static const char *NN_LABELS[NN_CLASSES] = {" + ", ".join(f'"{l}"' for l in labels) + "};\n")
        f.write("static const int NN_IS_PERSON[NN_CLASSES] = {" + ", ".join(map(str, is_person)) + "};\n\n")
        f.write(c_array("NN_MU", mu))
        f.write(c_array("NN_SD", sd))
        f.write(c_array("NN_W1", W1)); f.write(c_array("NN_B1", b1))
        f.write(c_array("NN_W2", W2)); f.write(c_array("NN_B2", b2))
        f.write(c_array("NN_W3", W3)); f.write(c_array("NN_B3", b3))
    return W1, b1, W2, b2, W3, b3


def read_header(path):
    """Веса обратно из model.h — проверка, что записанные числа дают те же ответы."""
    import re
    txt = open(path, encoding="utf-8").read()
    get = lambda n: np.array([float(v.rstrip("f")) for v in
                              re.search(rf"{n}\[\d+\] = \{{(.*?)\}};", txt, re.S).group(1).replace("\n", "").split(",")],
                             dtype=np.float64)
    return (get("NN_W1").reshape(N_FEAT, H1), get("NN_B1"), get("NN_W2").reshape(H1, H2), get("NN_B2"),
            get("NN_W3").reshape(H2, -1), get("NN_B3"))


def train_numpy(Xtr, ytr, Xva, yva, n_cls, epochs, seed):
    """Сеть N_FEAT -> H1 -> H2 -> классы (ReLU, softmax), Adam, веса классов поровну,
    лучшие веса по потерям на проверке. Возвращает (W1, b1, W2, b2, W3, b3)."""
    rng = np.random.default_rng(seed)
    he = lambda i, o: rng.normal(0, np.sqrt(2 / i), (i, o))
    P = [he(N_FEAT, H1), np.zeros(H1), he(H1, H2), np.zeros(H2), he(H2, n_cls), np.zeros(n_cls)]
    m = [np.zeros_like(x) for x in P]
    v = [np.zeros_like(x) for x in P]
    counts = np.bincount(ytr, minlength=n_cls)
    cw = len(ytr) / (n_cls * np.maximum(counts, 1))
    lr, l2, bs, step = 1e-3, 1e-4, 32, 0
    best, bestP, wait = 1e9, None, 0

    def fwd(X):
        h1 = np.maximum(X @ P[0] + P[1], 0)
        h2 = np.maximum(h1 @ P[2] + P[3], 0)
        o = h2 @ P[4] + P[5]
        o = np.exp(o - o.max(axis=1, keepdims=True))
        return h1, h2, o / o.sum(axis=1, keepdims=True)

    for ep in range(epochs):
        idx = rng.permutation(len(Xtr))
        for s in range(0, len(idx), bs):
            b = idx[s:s + bs]
            x, y = Xtr[b], ytr[b]
            h1, h2, pr = fwd(x)
            d = pr.copy(); d[np.arange(len(b)), y] -= 1
            d *= (cw[y] / len(b))[:, None]
            g = [None] * 6
            g[4], g[5] = h2.T @ d, d.sum(0)
            d2 = (d @ P[4].T) * (h2 > 0)
            g[2], g[3] = h1.T @ d2, d2.sum(0)
            d1 = (d2 @ P[2].T) * (h1 > 0)
            g[0], g[1] = x.T @ d1, d1.sum(0)
            step += 1
            for i in range(6):
                gi = g[i] + (l2 * P[i] if i % 2 == 0 else 0)
                m[i] = 0.9 * m[i] + 0.1 * gi
                v[i] = 0.999 * v[i] + 0.001 * gi * gi
                P[i] -= lr * (m[i] / (1 - 0.9 ** step)) / (np.sqrt(v[i] / (1 - 0.999 ** step)) + 1e-8)
        pv = fwd(Xva)[2]
        loss = float(np.mean(cw[yva] * -np.log(pv[np.arange(len(yva)), yva] + 1e-9)))
        if loss < best - 1e-4:
            best, bestP, wait = loss, [x.copy() for x in P], 0
        else:
            wait += 1
            if wait >= 30:                                     # как EarlyStopping(patience=30)
                break
    print(f"Обучение: эпох {ep + 1}, лучшие потери на проверке {best:.4f}")
    return tuple(bestP)


def numpy_forward(x, mu, sd, W1, b1, W2, b2, W3, b3):
    """То же, что predict() в nn_core.h — для проверки экспорта."""
    z = (x - mu) / sd
    h = np.maximum(z @ W1 + b1, 0)
    h = np.maximum(h @ W2 + b2, 0)
    o = h @ W3 + b3
    o = np.exp(o - o.max(axis=-1, keepdims=True))
    return o / o.sum(axis=-1, keepdims=True)


def main():
    ap = argparse.ArgumentParser(description="Обучение сети по данным радара")
    ap.add_argument("--data", default="data")
    ap.add_argument("--person", default="dvizhetsya,nepodvizhen",
                    help="метки, которые означают «человек есть» (через запятую)")
    ap.add_argument("--out", default=os.path.join("ld2410c_nn", "model.h"))
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    labels, Xtr, ytr, Xva, yva = load(args.data)
    person = [p.strip() for p in args.person.split(",") if p.strip()]
    unknown = [p for p in person if p not in labels]
    if unknown:
        print(f"Внимание: меток {unknown} нет среди папок {labels}")
    if len(Xtr) == 0:
        sys.exit("Нет окон для обучения: записи слишком короткие")
    if len(Xva) == 0:
        sys.exit("Нет окон для проверки: делайте записи длиннее ~60 с")

    mu = Xtr.mean(axis=0)
    sd = Xtr.std(axis=0)
    sd[sd < 1e-3] = 1.0           # постоянные признаки (например, отключённые зоны)

    weights = train_numpy((Xtr - mu) / sd, ytr, (Xva - mu) / sd, yva, len(labels), args.epochs, args.seed)
    prob_va = numpy_forward(Xva, mu, sd, *weights)
    pred = prob_va.argmax(axis=1)
    acc = (pred == yva).mean()
    print(f"\nТочность на проверочных окнах: {acc * 100:.1f}%")
    cm = np.zeros((len(labels), len(labels)), dtype=int)
    for t, p in zip(yva, pred):
        cm[t, p] += 1
    w = max(len(l) for l in labels) + 2
    print("\nМатрица ошибок (строки — правда, столбцы — ответ сети):")
    print(" " * w + "".join(f"{l[:10]:>12s}" for l in labels))
    for i, l in enumerate(labels):
        print(f"{l:<{w}s}" + "".join(f"{v:12d}" for v in cm[i]))

    is_p = np.array([l in person for l in labels])
    if is_p.any() and (~is_p).any():
        tp = np.isin(yva, np.where(is_p)[0])
        pp = np.isin(pred, np.where(is_p)[0])
        print(f"\nЧеловек найден в {pp[tp].mean() * 100:.0f}% окон, где он есть; "
              f"ложная тревога в {pp[~tp].mean() * 100:.0f}% пустых окон")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    synthetic = os.path.exists(os.path.join(args.data, "SYNTHETIC"))
    info = (f"{'синтетика' if synthetic else 'записи'} ({os.path.basename(os.path.abspath(args.data))}), "
            f"{len(Xtr)} окон, точность на проверке {acc * 100:.0f}%")
    export_header(args.out, weights, mu, sd, labels, person, synthetic, info)
    # Проверка: из записанных чисел (как на ESP) получаются те же ответы
    ref = numpy_forward(Xva, mu, sd, *read_header(args.out))
    print(f"\nПроверка экспорта: макс. расхождение {np.abs(ref - prob_va).max():.2e}")
    print(f"Записано: {args.out}\n  {info}")
    if synthetic:
        print("  Синтетика: на устройстве сеть только оценивает (nn advise), пока не обучите на записях")


if __name__ == "__main__":
    main()
