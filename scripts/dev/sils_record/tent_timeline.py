import csv, math, sys
R = sys.argv[1]
rows = [r for r in csv.DictReader(open(f'{R}/noclimb_ca.csv')) if None not in r.values()]
f = lambda r, k: float(r[k]) if r[k] not in ('', 'nan') else float('nan')
t0 = None; last = -9
for r in rows:
    if r['mode'] != 'CA': continue
    t = f(r, 't'); t0 = t0 or t
    if t - t0 < last + 0.6: continue
    last = t - t0
    X, Y = 35 + f(r, 'x'), -8 - f(r, 'y')
    if X < 38 or X > 60 or t - t0 > 35: continue
    print(f"{t-t0:5.1f} X {X:5.1f} Y {Y:5.1f} v {math.hypot(f(r,'vx'),f(r,'vy')):3.1f} pol {f(r,'speed_policy'):3.1f} dimg {f(r,'d_img_min'):5.2f}")
