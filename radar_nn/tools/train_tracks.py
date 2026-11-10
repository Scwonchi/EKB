"""
Обучение нейросети треков LD2450 и экспорт в rescue_scout/track_model.h.

    python tools/train_tracks.py                    # все записи из data_tracks/
    python tools/train_tracks.py --no-synth         # только свои записи (без sim_*)

Данные — data_tracks/<папка>/*.csv: строки F (признаки трека, их считает устройство
в режиме rec; record_tracks.py сохраняет) и строки R (сырые кадры, для track_data replay).
Метка трека:
  - в строке F стоит 0/1 (синтетика) — берётся она;
  - папка начинается с «pusto» — в записи людей не было, все треки ложные;
  - иначе в записи был человек, а заголовок файла (# zone=2.0-3.5 angle=-30..30)
    говорит, где он: треки в зоне — человек, вне зоны — ложные. Без зоны —
    треки, видимые в ≥ 50% кадров, — человек, остальные не используются.

Проверка — на последних 25% каждой записи (соседние строки почти одинаковые,
поэтому перемешивать нельзя: проверка вышла бы завышенной).

Нужен только numpy:  pip install numpy
"""
import argparse
import datetime as dt
import glob
import os
import re
import sys

import numpy as np

NF = 12                     # должно совпадать с tnn::NF в track_nn.h
H1, H2 = 16, 8
VAL_FRAC = 0.25
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FEATURES = ["дальность", "доля_кадров", "разброс_по_дальности", "разброс_вбок", "скорость",
            "разброс_скорости", "доля_нулевой_скорости", "смещение", "шаг", "самый_длинный_пропуск",
            "смены_знака_скорости", "до_соседа"]


def parse_range(s):
    m = re.fullmatch(r"\s*(-?[\d.]+)\s*(?:-|\.\.)\s*(-?[\d.]+)\s*", s or "")
    return (float(m.group(1)), float(m.group(2))) if m else None


def load_file(path, folder):
    meta, rows = {}, []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("#"):
                for k, v in re.findall(r"(\w+)=(\S+)", line):
                    meta[k] = v
            elif line.startswith("F,"):
                c = line.strip().split(",")
                if len(c) != 6 + NF:
                    continue
                try:
                    rows.append((float(c[3]), float(c[4]), c[5], [float(x) for x in c[6:]]))
                except ValueError:
                    pass
    if not rows:
        return None
    zone, ang = parse_range(meta.get("zone")), parse_range(meta.get("angle"))
    empty = folder.lower().startswith("pusto") or folder.lower().startswith("sim_pusto")
    X, y = [], []
    skipped = 0
    for rng, a, lab, feat in rows:
        if lab in ("0", "1"):
            t = int(lab)
        elif empty:
            t = 0
        elif zone:
            inside = zone[0] <= rng <= zone[1] and (ang is None or ang[0] <= a <= ang[1])
            t = 1 if inside else 0
        else:
            if feat[1] < 0.5:
                skipped += 1
                continue
            t = 1
        X.append(feat)
        y.append(t)
    return np.array(X, np.float32), np.array(y, np.float32), meta.get("source", "device"), skipped


def load(data_dir, use_synth):
    Xtr, ytr, Xva, yva = [], [], [], []
    sources = set()
    print("Записи:")
    for folder in sorted(os.listdir(data_dir)):
        fd = os.path.join(data_dir, folder)
        if not os.path.isdir(fd) or (folder.startswith("sim_") and not use_synth):
            continue
        n = [0, 0]
        files = sorted(glob.glob(os.path.join(fd, "*.csv")))
        for p in files:
            r = load_file(p, folder)
            if r is None:
                continue
            X, y, src, skipped = r
            sources.add(src)
            cut = int(len(X) * (1 - VAL_FRAC))
            Xtr.append(X[:cut]); ytr.append(y[:cut]); Xva.append(X[cut:]); yva.append(y[cut:])
            n[0] += int((y == 0).sum()); n[1] += int((y == 1).sum())
            if skipped:
                print(f"    {os.path.basename(p)}: нет зоны — {skipped} слабых треков пропущено")
        print(f"  {folder:16s} файлов {len(files):3d}, треков-людей {n[1]:6d}, ложных {n[0]:6d}")
    if not Xtr:
        sys.exit(f"В {data_dir} нет записей. Синтетика: tools/track_data synth data_tracks")
    cat = lambda a: np.concatenate(a) if a else np.zeros((0, NF), np.float32)
    return cat(Xtr), np.concatenate(ytr), cat(Xva), np.concatenate(yva), sources


# ---------- Сеть: 12 → 16 → 8 → 1, numpy ----------
def init(rng):
    he = lambda i, o: rng.normal(0, np.sqrt(2 / i), (i, o)).astype(np.float32)
    return {"W1": he(NF, H1), "b1": np.zeros(H1, np.float32), "W2": he(H1, H2), "b2": np.zeros(H2, np.float32),
            "W3": he(H2, 1), "b3": np.zeros(1, np.float32)}


def forward(P, x):
    h1 = np.maximum(0, x @ P["W1"] + P["b1"])
    h2 = np.maximum(0, h1 @ P["W2"] + P["b2"])
    z = (h2 @ P["W3"] + P["b3"])[:, 0]
    return 1 / (1 + np.exp(-np.clip(z, -30, 30))), (h1, h2)


def train(Xtr, ytr, Xva, yva, epochs, seed=1):
    rng = np.random.default_rng(seed)
    P = init(rng)
    m = {k: np.zeros_like(v) for k, v in P.items()}
    v = {k: np.zeros_like(v) for k, v in P.items()}
    pos = max(ytr.mean(), 1e-3)
    w_pos, w_neg = 0.5 / pos, 0.5 / (1 - pos)              # классы поровну
    lr, l2, bs, step = 3e-3, 1e-4, 256, 0
    best, bestP = 1e9, None
    for ep in range(epochs):
        idx = rng.permutation(len(Xtr))
        for s in range(0, len(idx), bs):
            b = idx[s:s + bs]
            x, y = Xtr[b], ytr[b]
            p, (h1, h2) = forward(P, x)
            w = np.where(y > 0.5, w_pos, w_neg)
            dz = (w * (p - y) / len(b))[:, None]                # d(BCE)/dz
            g = {"W3": h2.T @ dz, "b3": dz.sum(0)}
            d2 = (dz @ P["W3"].T) * (h2 > 0)
            g["W2"], g["b2"] = h1.T @ d2, d2.sum(0)
            d1 = (d2 @ P["W2"].T) * (h1 > 0)
            g["W1"], g["b1"] = x.T @ d1, d1.sum(0)
            step += 1
            for k in P:
                gk = g[k] + (l2 * P[k] if k[0] == "W" else 0)
                m[k] = 0.9 * m[k] + 0.1 * gk
                v[k] = 0.999 * v[k] + 0.001 * gk * gk
                P[k] -= lr * (m[k] / (1 - 0.9 ** step)) / (np.sqrt(v[k] / (1 - 0.999 ** step)) + 1e-8)
        pv, _ = forward(P, Xva)
        wv = np.where(yva > 0.5, w_pos, w_neg)
        loss = float(np.mean(wv * -(yva * np.log(pv + 1e-7) + (1 - yva) * np.log(1 - pv + 1e-7))))
        if loss < best:
            best, bestP = loss, {k: x.copy() for k, x in P.items()}
        if ep % 10 == 0 or ep == epochs - 1:
            print(f"  эпоха {ep:3d}: потери на проверке {loss:.4f}")
    return bestP


def report(name, p, y, thr):
    pred = p >= thr
    tp, fn = int((pred & (y > 0.5)).sum()), int((~pred & (y > 0.5)).sum())
    fp, tn = int((pred & (y < 0.5)).sum()), int((~pred & (y < 0.5)).sum())
    rec = tp / max(tp + fn, 1)
    fpr = fp / max(fp + tn, 1)
    acc = (tp + tn) / max(len(y), 1)
    print(f"  {name}: точность {acc * 100:.1f}%, людей найдено {rec * 100:.1f}% ({tp}/{tp + fn}), "
          f"ложных пропущено как людей {fpr * 100:.1f}% ({fp}/{fp + tn})")
    return acc, rec, fpr


def c_arr(name, a):
    a = np.asarray(a, np.float32).ravel()
    body = ",".join(f"{x:.7g}f" for x in a)
    out, cur = [], ""
    for tok in body.split(","):
        if len(cur) + len(tok) > 100:
            out.append(cur); cur = ""
        cur += tok + ","
    out.append(cur.rstrip(","))
    return f"static const float {name}[{len(a)}] = {{\n  " + "\n  ".join(out) + "\n};\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.join(ROOT, "data_tracks"))
    ap.add_argument("--out", default=os.path.join(ROOT, "rescue_scout", "track_model.h"))
    ap.add_argument("--no-synth", action="store_true", help="не брать синтетику (папки sim_*)")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--threshold", type=float, default=0.5)
    a = ap.parse_args()

    Xtr, ytr, Xva, yva, sources = load(a.data, not a.no_synth)
    print(f"Обучение: {len(Xtr)} строк (людей {int(ytr.sum())}), проверка: {len(Xva)} (людей {int(yva.sum())})")
    mu = Xtr.mean(0)
    sd = Xtr.std(0) + 1e-3
    n = lambda X: (X - mu) / sd
    P = train(n(Xtr), ytr, n(Xva), yva, a.epochs)

    print("Проверка (данные, которых сеть не видела):")
    pv, _ = forward(P, n(Xva))
    acc, rec, fpr = report("все треки            ", pv, yva, a.threshold)
    strong = Xva[:, 1] >= 0.5                                 # треки, которые проходят правила трекера
    if strong.any():
        report("треки, прошедшие правила", pv[strong], yva[strong], a.threshold)
    # Что важнее всего: насколько хуже, если перемешать один признак
    base = float(np.mean((pv >= a.threshold) == (yva > 0.5)))
    rng = np.random.default_rng(0)
    imp = []
    for i in range(NF):
        X = Xva.copy(); X[:, i] = rng.permutation(X[:, i])
        p2, _ = forward(P, n(X))
        imp.append(base - float(np.mean((p2 >= a.threshold) == (yva > 0.5))))
    order = np.argsort(imp)[::-1][:5]
    print("  важнее всего: " + ", ".join(f"{FEATURES[i]} (−{imp[i] * 100:.1f}%)" for i in order))

    synthetic = sources <= {"synth"}
    src = "синтетика" if synthetic else ("записи + синтетика" if "synth" in sources else "записи с устройства")
    info = f"{src}, {len(Xtr)} строк, проверка: людей найдено {rec * 100:.0f}%, ложных пропущено {fpr * 100:.0f}%, {dt.date.today()}"
    with open(a.out, "w", encoding="utf-8") as f:
        f.write("// Сгенерировано tools/train_tracks.py — не редактировать вручную.\n")
        f.write(f"// Обучено: {info}\n#pragma once\n\n")
        f.write(f"#define TNN_NF {NF}\n#define TNN_H1 {H1}\n#define TNN_H2 {H2}\n")
        f.write(f"#define TNN_SYNTHETIC {1 if synthetic else 0}\n#define TNN_INFO \"{info}\"\n\n")
        f.write(c_arr("TNN_MU", mu) + c_arr("TNN_SD", sd))
        f.write(c_arr("TNN_W1", P["W1"]) + c_arr("TNN_B1", P["b1"]))
        f.write(c_arr("TNN_W2", P["W2"]) + c_arr("TNN_B2", P["b2"]))
        f.write(c_arr("TNN_W3", P["W3"]))
        f.write(f"static const float TNN_B3 = {float(P['b3'][0]):.7g}f;\n")
    # Для сверки с C++: track_data check
    chk = os.path.join(ROOT, "tools", "track_check.csv")
    sel = np.linspace(0, len(Xva) - 1, min(200, len(Xva))).astype(int)
    pc, _ = forward(P, n(Xva[sel].astype(np.float64)))
    with open(chk, "w") as f:
        for x, p in zip(Xva[sel], pc):
            f.write(",".join(f"{v:.9g}" for v in x) + f",{p:.9g}\n")
    print(f"Записано: {a.out}\n  {info}")
    if synthetic:
        print("  Модель на синтетике: в прошивке по умолчанию только оценивает (nn advise).")


if __name__ == "__main__":
    main()
