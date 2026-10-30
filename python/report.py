"""Generate a mission report + plots from telemetry.csv and events.csv.

Usage:  python python/report.py [telemetry.csv] [events.csv]
Requires: pandas, matplotlib
"""
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

tel_path = sys.argv[1] if len(sys.argv) > 1 else "telemetry.csv"
evt_path = sys.argv[2] if len(sys.argv) > 2 else "events.csv"

tel = pd.read_csv(tel_path)
try:
    evt = pd.read_csv(evt_path)
except pd.errors.EmptyDataError:
    evt = pd.DataFrame(columns=["t", "id", "type", "message"])

# ---- Tracking error (measurement and filtered track vs truth), metres ----
m_per_deg = 111_195.0
def err_m(lat, lon, tlat, tlon):
    dy = (lat - tlat) * m_per_deg
    dx = (lon - tlon) * m_per_deg * np.cos(np.radians(tlat))
    return np.hypot(dx, dy)

tel["meas_err_m"] = err_m(tel.meas_lat, tel.meas_lon, tel.true_lat, tel.true_lon)
tel["trk_err_m"] = err_m(tel.trk_lat, tel.trk_lon, tel.true_lat, tel.true_lon)

print("=== MISSION REPORT ===")
print(f"Duration: {tel.t.max():.0f} s   Aircraft: {tel.id.nunique()}   Records: {len(tel)}\n")
summary = tel.groupby("id").agg(
    records=("t", "count"),
    mean_meas_err_m=("meas_err_m", "mean"),
    mean_trk_err_m=("trk_err_m", "mean"),
    final_dist_km=("dist_to_dest_m", lambda s: s.iloc[-1] / 1000),
)
print(summary.round(1).to_string(), "\n")
print(f"Events logged: {len(evt)}")
if len(evt):
    print(evt.groupby("type").size().to_string(), "\n")
    print(evt.to_string(index=False))

# ---- Plots ----
fig, ax = plt.subplots(1, 3, figsize=(17, 5))
for aid, g in tel.groupby("id"):
    ax[0].plot(g.true_lon, g.true_lat, label=aid)
    ax[0].scatter(g.true_lon.iloc[0], g.true_lat.iloc[0], marker="o")
    ax[1].plot(g.t, g.true_alt, label=aid)
    ax[2].plot(g.t, g.dist_to_dest_m / 1000, label=aid)
ax[0].set(title="Ground tracks", xlabel="Longitude", ylabel="Latitude")
ax[1].set(title="Altitude (m)", xlabel="t (s)")
ax[2].set(title="Distance to destination (km)", xlabel="t (s)")
for a in ax:
    a.legend(); a.grid(alpha=0.3)
for _, e in evt.iterrows():
    ax[1].axvline(e.t, color="red", alpha=0.3, ls="--")
plt.tight_layout()
plt.savefig("mission_overview.png", dpi=120)
print("\nSaved mission_overview.png")
