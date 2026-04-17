"""
arm_ver_3 워크스페이스 분석기 v4
- 점(dot)으로만 표시 (스틱/STL 없음)
- joint1-joint2 간격 18cm 반영
- 논문 스타일 흰 배경
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from scipy.spatial import ConvexHull
import xml.etree.ElementTree as ET
import os, glob, argparse, time
from dataclasses import dataclass, field
from typing import List, Optional, Dict


# ══════════════════════════════════════════════
# 변환 행렬
# ══════════════════════════════════════════════
def rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rpy2rot(rpy):
    return rz(rpy[2]) @ ry(rpy[1]) @ rx(rpy[0])

def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3,:3] = rpy2rot(rpy)
    T[:3, 3] = xyz
    return T

def rodrigues_T(axis, angle):
    ax = np.asarray(axis, float)
    n  = np.linalg.norm(ax)
    if n < 1e-9: return np.eye(4)
    ax /= n
    c,s = np.cos(angle), np.sin(angle)
    t = 1-c; x,y,z = ax
    R = np.array([
        [t*x*x+c,   t*x*y-s*z, t*x*z+s*y],
        [t*x*y+s*z, t*y*y+c,   t*y*z-s*x],
        [t*x*z-s*y, t*y*z+s*x, t*z*z+c  ]
    ])
    T = np.eye(4); T[:3,:3] = R
    return T


# ══════════════════════════════════════════════
# URDF 파서
# ══════════════════════════════════════════════
@dataclass
class Joint:
    name: str; jtype: str
    parent: str; child: str
    xyz: np.ndarray; rpy: np.ndarray
    axis: np.ndarray
    lower: float; upper: float

def parse_urdf(path):
    root = ET.parse(path).getroot()
    joints = []
    for j in root.findall("joint"):
        orig = j.find("origin")
        xyz = np.array([float(v) for v in orig.get("xyz","0 0 0").split()]) if orig is not None else np.zeros(3)
        rpy = np.array([float(v) for v in orig.get("rpy","0 0 0").split()]) if orig is not None else np.zeros(3)
        ax_el = j.find("axis")
        axis = np.array([float(v) for v in ax_el.get("xyz","0 0 1").split()]) if ax_el is not None else np.array([0.,0.,1.])
        lim = j.find("limit")
        lo = float(lim.get("lower",-np.pi)) if lim is not None else -np.pi
        hi = float(lim.get("upper", np.pi)) if lim is not None else  np.pi
        joints.append(Joint(
            name=j.get("name"), jtype=j.get("type","fixed"),
            parent=j.find("parent").get("link"),
            child =j.find("child").get("link"),
            xyz=xyz, rpy=rpy, axis=axis, lower=lo, upper=hi
        ))
    return joints

def build_chain(joints):
    children = {j.child for j in joints}
    parents  = {j.parent for j in joints}
    root = next(iter(parents - children))
    def dfs(lnk, path):
        nexts = [j for j in joints if j.parent == lnk]
        if not nexts: return path
        best = path
        for j in nexts:
            c = dfs(j.child, path+[j])
            if len(c) > len(best): best = c
        return best
    return dfs(root, [])


# ══════════════════════════════════════════════
# FK
# ══════════════════════════════════════════════
def fk(chain, q):
    T = np.eye(4)
    frames = [T.copy()]
    qi = 0
    for j in chain:
        T = T @ make_T(j.xyz, j.rpy)
        if j.jtype in ("revolute","continuous"):
            T = T @ rodrigues_T(j.axis, q[qi]); qi += 1
        elif j.jtype == "prismatic":
            Tp = np.eye(4); Tp[:3,3] = j.axis * q[qi]; qi += 1
            T = T @ Tp
        frames.append(T.copy())
    return T[:3,3].copy(), frames


# ══════════════════════════════════════════════
# 워크스페이스 샘플링
# ══════════════════════════════════════════════
def sample_workspace(chain, n=100_000, seed=42):
    rng = np.random.default_rng(seed)
    active = [j for j in chain if j.jtype in ("revolute","continuous","prismatic")]
    lo = np.array([j.lower for j in active])
    hi = np.array([j.upper for j in active])
    print(f"  DOF: {len(active)}  /  samples: {n:,}")
    for i,j in enumerate(active):
        print(f"  [{i}] {j.name:12s}  {np.degrees(lo[i]):+7.1f}deg ~ {np.degrees(hi[i]):+7.1f}deg")
    pts = np.empty((n,3))
    bs = 5_000
    for s in range(0, n, bs):
        e = min(s+bs, n)
        qs = rng.uniform(lo, hi, size=(e-s, len(active)))
        for k,q in enumerate(qs):
            pts[s+k], _ = fk(chain, q)
    return pts


# ══════════════════════════════════════════════
# Z 슬라이스 면적
# ══════════════════════════════════════════════
def slice_area(pts, z_center_m, band=0.025):
    mask = np.abs(pts[:,2] - z_center_m) < band
    sub  = pts[mask, :2]
    if len(sub) < 6:
        return None, 0.0
    try:
        hull = ConvexHull(sub)
        return sub[hull.vertices], hull.volume
    except:
        return None, 0.0


# ══════════════════════════════════════════════
# 시각화
# ══════════════════════════════════════════════
SLICE_COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]

def setup_style():
    plt.rcParams.update({
        "font.family":      "DejaVu Sans",
        "font.size":        9,
        "axes.linewidth":   0.8,
        "axes.edgecolor":   "#333333",
        "axes.labelcolor":  "#111111",
        "axes.titlesize":   10,
        "axes.titleweight": "bold",
        "xtick.color":      "#333333",
        "ytick.color":      "#333333",
        "xtick.labelsize":  8,
        "ytick.labelsize":  8,
        "grid.color":       "#dddddd",
        "grid.linewidth":   0.5,
        "figure.facecolor": "white",
        "axes.facecolor":   "white",
        "savefig.facecolor":"white",
        "legend.framealpha":0.9,
        "legend.edgecolor": "#bbbbbb",
        "legend.fontsize":  8,
    })

def style2d(ax, xl, yl, title):
    ax.set_xlabel(xl); ax.set_ylabel(yl); ax.set_title(title)
    ax.axhline(0, color="#bbbbbb", lw=0.6, ls="--", zorder=0)
    ax.axvline(0, color="#bbbbbb", lw=0.6, ls="--", zorder=0)
    ax.grid(True, lw=0.4, zorder=0)
    ax.set_aspect("equal", adjustable="datalim")

def plot_joints_dots(ax3, frames, label="Home pose (q=0)"):
    """관절 위치를 점으로만 표시"""
    jp = np.array([f[:3,3] for f in frames])
    # 관절 연결선 (얇게)
    ax3.plot(jp[:,0], jp[:,1], jp[:,2],
             color="#aaaaaa", lw=0.8, alpha=0.5, zorder=5)
    # 각 관절 점
    colors_j = ["#333333","#e41a1c","#ff7f00","#4daf4a",
                "#377eb8","#984ea3","#a65628"]
    for k, (p, c) in enumerate(zip(jp, colors_j)):
        ax3.scatter(*p, color=c, s=40, zorder=10,
                    depthshade=False,
                    label=f"J{k}" if k==0 else f"J{k}")
    # 마지막 점(endpoint) 강조
    ax3.scatter(*jp[-1], color="#e41a1c", s=80,
                marker="*", zorder=11, label="Endpoint")

def plot_all(chain, pts, frames_home, robot_name="arm_ver_3"):
    setup_style()

    x,y,z = pts[:,0], pts[:,1], pts[:,2]
    r    = np.sqrt(x**2+y**2+z**2)
    r_xy = np.sqrt(x**2+y**2)

    # 통계
    print("\n" + "="*58)
    print(f"  {robot_name}  Workspace Analysis  (joint1-2 gap: 180mm)")
    print("="*58)
    for tag,(lo,hi) in [("X",(x.min(),x.max())),("Y",(y.min(),y.max())),
                         ("Z",(z.min(),z.max())),
                         ("3D Reach",(r.min(),r.max())),
                         ("XY Reach",(r_xy.min(),r_xy.max()))]:
        print(f"  {tag:12s}  {lo*1000:+8.1f} mm ~ {hi*1000:+8.1f} mm  (span {(hi-lo)*1000:.1f} mm)")
    print(f"  Max reach  :  {r.max()*1000:.1f} mm")
    print("="*58)

    # Z 슬라이스
    z_min_mm, z_max_mm = z.min()*1000, z.max()*1000
    slice_zs = np.linspace(z_min_mm + (z_max_mm-z_min_mm)*0.15,
                            z_max_mm - (z_max_mm-z_min_mm)*0.15, 4)
    print("\n  Z-slice cross-section areas:")
    slice_results = []
    for zv in slice_zs:
        verts, area = slice_area(pts, zv/1000)
        slice_results.append((zv, verts, area))
        print(f"    Z = {zv:+7.1f} mm   {area*1e4:8.1f} cm²")

    # 다운샘플
    vis_n = min(50_000, len(pts))
    idx   = np.random.choice(len(pts), vis_n, replace=False)
    vpts  = pts[idx]

    fig = plt.figure(figsize=(18, 11))
    gs  = gridspec.GridSpec(2, 3, figure=fig,
                            left=0.05, right=0.97,
                            top=0.91, bottom=0.07,
                            wspace=0.32, hspace=0.38)

    # ─────────────────────────────
    # 1) 3D — 워크스페이스 점 + 관절 점
    # ─────────────────────────────
    ax3 = fig.add_subplot(gs[:,0], projection="3d")

    # 워크스페이스 점 (빨간 점)
    ax3.scatter(vpts[:,0], vpts[:,1], vpts[:,2],
                c="#CC2222", s=0.4, alpha=0.10,
                rasterized=True, zorder=1)

    # 슬라이스 윤곽선
    for (zv, verts, area), col in zip(slice_results, SLICE_COLORS):
        if verts is not None:
            zv_m = zv/1000
            closed_x = np.append(verts[:,0], verts[0,0])
            closed_y = np.append(verts[:,1], verts[0,1])
            ax3.plot(closed_x, closed_y,
                     zs=zv_m, zdir='z',
                     color=col, lw=1.5, alpha=0.9,
                     label=f"Z={zv:.0f}mm ({area*1e4:.0f}cm²)")

    # 관절 위치 점
    plot_joints_dots(ax3, frames_home)

    ax3.set_xlabel("X (m)", labelpad=3)
    ax3.set_ylabel("Y (m)", labelpad=3)
    ax3.set_zlabel("Z (m)", labelpad=3)
    ax3.set_title("3D Workspace", pad=8)
    ax3.legend(loc="upper left", fontsize=6, markerscale=1.5)
    ax3.xaxis.pane.fill = False
    ax3.yaxis.pane.fill = False
    ax3.zaxis.pane.fill = False
    ax3.xaxis.pane.set_edgecolor("#cccccc")
    ax3.yaxis.pane.set_edgecolor("#cccccc")
    ax3.zaxis.pane.set_edgecolor("#cccccc")
    ax3.grid(True, lw=0.3, color="#dddddd")

    # ─────────────────────────────
    # 2) Top view XY
    # ─────────────────────────────
    ax_xy = fig.add_subplot(gs[0,1])
    ax_xy.scatter(vpts[:,0]*1000, vpts[:,1]*1000,
                  c="#CC2222", s=0.4, alpha=0.12, rasterized=True)
    theta = np.linspace(0, 2*np.pi, 300)
    ax_xy.plot(r_xy.max()*1000*np.cos(theta),
               r_xy.max()*1000*np.sin(theta),
               "--", color="#555555", lw=0.9,
               label=f"Max XY reach: {r_xy.max()*1000:.0f} mm")
    # 관절 위치 점 (top)
    jp = np.array([f[:3,3] for f in frames_home])
    ax_xy.scatter(jp[:,0]*1000, jp[:,1]*1000,
                  c=["#333333","#e41a1c","#ff7f00","#4daf4a",
                     "#377eb8","#984ea3","#a65628"][:len(jp)],
                  s=35, zorder=5, edgecolors="#333333", linewidths=0.4)
    ax_xy.legend(fontsize=7)
    style2d(ax_xy, "X (mm)", "Y (mm)", "Top View (XY plane)")

    # ─────────────────────────────
    # 3) Front view XZ
    # ─────────────────────────────
    ax_xz = fig.add_subplot(gs[0,2])
    ax_xz.scatter(vpts[:,0]*1000, vpts[:,2]*1000,
                  c="#CC2222", s=0.4, alpha=0.12, rasterized=True)
    for (zv, _, _), col in zip(slice_results, SLICE_COLORS):
        ax_xz.axhline(zv, color=col, lw=0.9, ls="--", alpha=0.9,
                      label=f"Z={zv:.0f}mm")
    ax_xz.scatter(jp[:,0]*1000, jp[:,2]*1000,
                  c=["#333333","#e41a1c","#ff7f00","#4daf4a",
                     "#377eb8","#984ea3","#a65628"][:len(jp)],
                  s=35, zorder=5, edgecolors="#333333", linewidths=0.4)
    ax_xz.legend(fontsize=6, loc="upper right")
    style2d(ax_xz, "X (mm)", "Z (mm)", "Front View (XZ plane)")

    # ─────────────────────────────
    # 4) Side view YZ
    # ─────────────────────────────
    ax_yz = fig.add_subplot(gs[1,1])
    ax_yz.scatter(vpts[:,1]*1000, vpts[:,2]*1000,
                  c="#CC2222", s=0.4, alpha=0.12, rasterized=True)
    for (zv, _, _), col in zip(slice_results, SLICE_COLORS):
        ax_yz.axhline(zv, color=col, lw=0.9, ls="--", alpha=0.9)
    ax_yz.scatter(jp[:,1]*1000, jp[:,2]*1000,
                  c=["#333333","#e41a1c","#ff7f00","#4daf4a",
                     "#377eb8","#984ea3","#a65628"][:len(jp)],
                  s=35, zorder=5, edgecolors="#333333", linewidths=0.4)
    style2d(ax_yz, "Y (mm)", "Z (mm)", "Side View (YZ plane)")

    # ─────────────────────────────
    # 5) Z-slice 단면
    # ─────────────────────────────
    ax_sl = fig.add_subplot(gs[1,2])
    for (zv, verts, area), col in zip(slice_results, SLICE_COLORS):
        if verts is not None:
            closed = np.vstack([verts, verts[0]])
            ax_sl.fill(closed[:,0]*1000, closed[:,1]*1000,
                       alpha=0.15, color=col)
            ax_sl.plot(closed[:,0]*1000, closed[:,1]*1000,
                       color=col, lw=1.5,
                       label=f"Z={zv:.0f}mm  |  {area*1e4:.1f} cm²")
    ax_sl.axhline(0, color="#bbbbbb", lw=0.6, ls="--")
    ax_sl.axvline(0, color="#bbbbbb", lw=0.6, ls="--")
    ax_sl.set_xlabel("X (mm)"); ax_sl.set_ylabel("Y (mm)")
    ax_sl.set_title("Horizontal Cross-Section at Z levels")
    ax_sl.legend(fontsize=7, loc="upper right")
    ax_sl.set_aspect("equal", adjustable="datalim")
    ax_sl.grid(True, lw=0.4)
    for sp in ax_sl.spines.values(): sp.set_linewidth(0.8)

    # 타이틀
    active = [j for j in chain if j.jtype in ("revolute","continuous","prismatic")]
    fig.suptitle(
        f"{robot_name}  —  {len(active)}-DOF Workspace  "
        f"({len(pts):,} samples,  max reach: {r.max()*1000:.0f} mm)"
        f"  [joint1-2 gap: 180 mm]",
        fontsize=11, fontweight="bold", color="#111111"
    )

    out = "/home/lmh/ros2_ws/src/final_project/urdf/arm_workspace_v4.png"
    plt.savefig(out, dpi=200, bbox_inches="tight", facecolor="white")
    print(f"\n  Saved -> {out}")
    plt.show()


# ══════════════════════════════════════════════
# main
# ══════════════════════════════════════════════
def main():
    global args
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf",    default="/home/lmh/ros2_ws/src/final_project/urdf/arm_3.urdf")
    ap.add_argument("--samples", type=int, default=100_000)
    args = ap.parse_args()

    t0 = time.time()

    # URDF 수정: joint_8 간격 18cm
    import tempfile, shutil
    tree = ET.parse(args.urdf)
    root = tree.getroot()
    modified = False
    for j in root.findall("joint"):
        if j.get("name") == "joint_8":
            orig = j.find("origin")
            if orig is not None:
                old_xyz = np.array([float(v) for v in orig.get("xyz","0 0 0").split()])
                old_dist = np.linalg.norm(old_xyz)
                new_xyz  = old_xyz * (0.18 / old_dist)
                orig.set("xyz", f"{new_xyz[0]:.6f} {new_xyz[1]:.6f} {new_xyz[2]:.6f}")
                print(f"  joint_8 gap: {old_dist*1000:.1f}mm -> 180.0mm")
                print(f"  new xyz: {new_xyz}")
                modified = True

    # 임시 파일로 저장 후 파싱
    tmp = args.urdf.replace(".urdf", "_180mm.urdf")
    tree.write(tmp, encoding="unicode", xml_declaration=True)
    print(f"  Modified URDF: {tmp}")

    joints = parse_urdf(tmp)
    chain  = build_chain(joints)
    active = [j for j in chain if j.jtype in ("revolute","continuous","prismatic")]
    print(f"  Links/Joints/DOF: {len(joints)} joints, {len(active)} active\n")

    # Home position
    q0 = np.zeros(len(active))
    _, frames_home = fk(chain, q0)
    print("  Home position (q=0):")
    link_order = [chain[0].parent] + [j.child for j in chain]
    for ln, fr in zip(link_order, frames_home):
        p = fr[:3,3]
        print(f"    {ln:12s}  ({p[0]*1000:+7.1f}, {p[1]*1000:+7.1f}, {p[2]*1000:+7.1f}) mm")

    print(f"\n  Sampling ({args.samples:,})...")
    pts = sample_workspace(chain, n=args.samples)
    print(f"  Done: {time.time()-t0:.1f}s")

    plot_all(chain, pts, frames_home)


if __name__ == "__main__":
    main()