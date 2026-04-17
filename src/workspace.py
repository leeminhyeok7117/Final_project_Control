"""
arm_ver_3 워크스페이스 분석기 v6
==============================
변경사항:
  - 양팔 TF 점 대칭 올바르게 표시
  - Figure 2: 단일 Z 높이 단면 (arm1 / arm2 / 교집합 각각 표시)
  - --j2_dist : joint_8 링크 길이 직접 지정 (mm), 기본 200mm
  - --j2_xy   : base XY 기준 관절2 거리로 역산 (선택)

사용법:
    python workspace_v6.py
    python workspace_v6.py --j2_dist 200
    python workspace_v6.py --z_desk 850 --j2_dist 180
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Polygon as MplPolygon, FancyArrowPatch
from matplotlib.path import Path as MplPath
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull
import xml.etree.ElementTree as ET
import os, argparse, time, tempfile
from dataclasses import dataclass
from typing import List, Optional, Tuple


# ══════════════════════════════════════════════
# 변환 행렬
# ══════════════════════════════════════════════
def rx(a):
    c,s=np.cos(a),np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def rpy2rot(rpy): return rz(rpy[2]) @ ry(rpy[1]) @ rx(rpy[0])
def make_T(xyz, rpy):
    T=np.eye(4); T[:3,:3]=rpy2rot(rpy); T[:3,3]=xyz; return T
def rodrigues_T(axis, angle):
    ax=np.asarray(axis,float); n=np.linalg.norm(ax)
    if n<1e-9: return np.eye(4)
    ax/=n; c,s=np.cos(angle),np.sin(angle); t=1-c; x,y,z=ax
    R=np.array([[t*x*x+c,t*x*y-s*z,t*x*z+s*y],
                [t*x*y+s*z,t*y*y+c,t*y*z-s*x],
                [t*x*z-s*y,t*y*z+s*x,t*z*z+c]])
    T=np.eye(4); T[:3,:3]=R; return T


# ══════════════════════════════════════════════
# URDF 파서 + joint_8 거리 수정
# ══════════════════════════════════════════════
@dataclass
class Joint:
    name:str; jtype:str; parent:str; child:str
    xyz:np.ndarray; rpy:np.ndarray; axis:np.ndarray
    lower:float; upper:float

def load_and_modify_urdf(path, j2_xy_mm=200.0):
    """joint_8 길이를 조정해서 joint2의 base XY 거리 = j2_xy_mm 이 되도록 역산"""
    tree = ET.parse(path)
    root = tree.getroot()

    # 원본 joint_8 xyz 읽기
    orig_xyz = None
    for j in root.findall("joint"):
        if j.get("name") == "joint_8":
            orig_xyz = np.array([float(v) for v in j.find("origin").get("xyz").split()])
            break
    orig_mag = np.linalg.norm(orig_xyz)  # 원본 크기

    # joint2 XY를 FK로 계산하는 함수 (joint_8 스케일 k)
    def joint2_xy(k):
        # joint_7 rpy=(-pi/2,0,0) → rx(-pi/2) = [[1,0,0],[0,0,1],[0,-1,0]]
        # world_offset = R7 @ (orig_xyz * k)
        local = orig_xyz * k
        world_x = local[0]                  # R7 x행
        world_y = local[2]                  # R7 y행 = local z
        base_y  = -0.10875                  # joint_7 Y offset
        return np.sqrt((base_y + world_y)**2 + world_x**2 + local[1]**2)

    # 이분법으로 k 역산
    lo, hi = 0.01, 20.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if joint2_xy(mid) * 1000 < j2_xy_mm:
            lo = mid
        else:
            hi = mid
    k_sol = (lo + hi) / 2
    new_mag_m = k_sol * orig_mag
    new_xyz = orig_xyz * k_sol

    # URDF 수정
    for j in root.findall("joint"):
        if j.get("name") == "joint_8":
            j.find("origin").set("xyz", f"{new_xyz[0]:.8f} {new_xyz[1]:.8f} {new_xyz[2]:.8f}")

    print(f"  joint_8 링크: {orig_mag*1000:.1f}mm → {new_mag_m*1000:.1f}mm")
    print(f"  joint2 XY from base: {joint2_xy(k_sol)*1000:.1f}mm (목표: {j2_xy_mm:.0f}mm)")

    tmp = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w")
    tree.write(tmp, encoding="unicode", xml_declaration=True)
    tmp.close()
    return tmp.name

def parse_urdf(path):
    root = ET.parse(path).getroot()
    joints = []
    for j in root.findall("joint"):
        orig  = j.find("origin")
        xyz   = np.array([float(v) for v in orig.get("xyz","0 0 0").split()]) if orig is not None else np.zeros(3)
        rpy   = np.array([float(v) for v in orig.get("rpy","0 0 0").split()]) if orig is not None else np.zeros(3)
        ax_el = j.find("axis")
        axis  = np.array([float(v) for v in ax_el.get("xyz","0 0 1").split()]) if ax_el is not None else np.array([0.,0.,1.])
        lim   = j.find("limit")
        lo    = float(lim.get("lower",-np.pi)) if lim is not None else -np.pi
        hi    = float(lim.get("upper", np.pi)) if lim is not None else  np.pi
        joints.append(Joint(
            name=j.get("name"), jtype=j.get("type","fixed"),
            parent=j.find("parent").get("link"),
            child =j.find("child").get("link"),
            xyz=xyz, rpy=rpy, axis=axis, lower=lo, upper=hi))
    return joints

def build_chain(joints):
    children={j.child for j in joints}; parents={j.parent for j in joints}
    root=next(iter(parents-children))
    def dfs(lnk,path):
        nexts=[j for j in joints if j.parent==lnk]
        if not nexts: return path
        best=path
        for j in nexts:
            c=dfs(j.child,path+[j])
            if len(c)>len(best): best=c
        return best
    return dfs(root,[])


# ══════════════════════════════════════════════
# FK + XZ 미러
# ══════════════════════════════════════════════
def fk(chain, q):
    T=np.eye(4); frames=[T.copy()]; qi=0
    for j in chain:
        T=T@make_T(j.xyz,j.rpy)
        if j.jtype in ("revolute","continuous"):
            T=T@rodrigues_T(j.axis,q[qi]); qi+=1
        elif j.jtype=="prismatic":
            Tp=np.eye(4); Tp[:3,3]=j.axis*q[qi]; qi+=1; T=T@Tp
        frames.append(T.copy())
    return T[:3,3].copy(), frames

def mirror_y(pts):
    """XZ 평면 대칭: Y → -Y"""
    m=pts.copy(); m[:,1]*=-1; return m

def mirror_frames(frames):
    """FK 프레임 리스트 XZ 대칭 (위치만 Y 반전, 시각화용)"""
    return [np.array([
        [T[0,0], T[0,1], T[0,2], T[0,3]],
        [T[1,0], T[1,1], T[1,2],-T[1,3]],  # Y translation 반전
        [T[2,0], T[2,1], T[2,2], T[2,3]],
        [0,      0,      0,      1      ]
    ]) for T in frames]


# ══════════════════════════════════════════════
# 샘플링
# ══════════════════════════════════════════════
def sample_workspace(chain, n=100_000, seed=42):
    rng=np.random.default_rng(seed)
    active=[j for j in chain if j.jtype in ("revolute","continuous","prismatic")]
    lo=np.array([j.lower for j in active]); hi=np.array([j.upper for j in active])
    print(f"  DOF: {len(active)}  /  samples: {n:,}")
    for i,j in enumerate(active):
        print(f"  [{i}] {j.name:12s}  {np.degrees(lo[i]):+7.1f}° ~ {np.degrees(hi[i]):+7.1f}°")
    pts=np.empty((n,3)); bs=5_000
    for s in range(0,n,bs):
        e=min(s+bs,n)
        qs=rng.uniform(lo,hi,size=(e-s,len(active)))
        for k,q in enumerate(qs): pts[s+k],_=fk(chain,q)
    return pts


# ══════════════════════════════════════════════
# Z 슬라이스 ConvexHull
# ══════════════════════════════════════════════
def slice_hull(pts, z_center, band=0.025):
    mask=np.abs(pts[:,2]-z_center)<band
    sub=pts[mask,:2]
    if len(sub)<6: return None, 0.0
    try:
        hull=ConvexHull(sub); return sub[hull.vertices], hull.volume
    except: return None, 0.0

def find_widest_z(pts, n_steps=50):
    zs=np.linspace(pts[:,2].min(), pts[:,2].max(), n_steps)
    best_z, best_a = zs[0], 0.0
    for zv in zs:
        _,a=slice_hull(pts,zv)
        if a>best_a: best_a=a; best_z=zv
    return best_z, best_a

def hull_intersection_2d(verts_a, verts_b, grid_n=300):
    """두 ConvexHull polygon의 교집합 영역 (raster 방식, shapely 불필요)"""
    all_v=np.vstack([verts_a,verts_b])
    xmin,ymin=all_v.min(axis=0)-0.01; xmax,ymax=all_v.max(axis=0)+0.01
    xx,yy=np.meshgrid(np.linspace(xmin,xmax,grid_n), np.linspace(ymin,ymax,grid_n))
    pts_grid=np.column_stack([xx.ravel(),yy.ravel()])

    def inside(verts, pts):
        closed=np.vstack([verts,verts[0]])
        path=MplPath(closed)
        return path.contains_points(pts)

    in_a=inside(verts_a, pts_grid)
    in_b=inside(verts_b, pts_grid)
    both=in_a & in_b
    cell=((xmax-xmin)*(ymax-ymin))/(grid_n**2)
    inter_pts=pts_grid[both]
    area=both.sum()*cell
    # 교집합 외곽 Hull
    if len(inter_pts)>3:
        try:
            h=ConvexHull(inter_pts); return inter_pts[h.vertices], area
        except: pass
    return inter_pts, area


# ══════════════════════════════════════════════
# 공통 스타일
# ══════════════════════════════════════════════
def setup_style():
    plt.rcParams.update({
        "font.family":"DejaVu Sans","font.size":9,
        "axes.linewidth":0.8,"axes.edgecolor":"#333333",
        "axes.labelcolor":"#111111","axes.titlesize":10,
        "axes.titleweight":"bold","xtick.color":"#333333",
        "ytick.color":"#333333","xtick.labelsize":8,"ytick.labelsize":8,
        "grid.color":"#dddddd","grid.linewidth":0.5,
        "figure.facecolor":"white","axes.facecolor":"white",
        "savefig.facecolor":"white",
        "legend.framealpha":0.9,"legend.edgecolor":"#bbbbbb","legend.fontsize":8,
    })

JCOLORS = ["#333333","#e41a1c","#ff7f00","#4daf4a","#377eb8","#984ea3","#a65628"]
SLICE_COLORS = ["#e41a1c","#377eb8","#4daf4a","#984ea3"]

def draw_joints(ax3, frames, size=40, label_prefix=""):
    jp=np.array([f[:3,3] for f in frames])
    ax3.plot(jp[:,0],jp[:,1],jp[:,2], color="#aaaaaa",lw=0.7,alpha=0.5,zorder=4)
    for k,(p,c) in enumerate(zip(jp,JCOLORS[:len(jp)])):
        lbl = f"{label_prefix}J{k}" if k==0 else None
        ax3.scatter(*p, color=c, s=size, zorder=10, depthshade=False,
                    edgecolors="white", linewidths=0.3, label=lbl)

def style2d(ax, xl, yl, title):
    ax.set_xlabel(xl); ax.set_ylabel(yl); ax.set_title(title)
    ax.axhline(0,color="#bbbbbb",lw=0.5,ls="--",zorder=0)
    ax.axvline(0,color="#bbbbbb",lw=0.5,ls="--",zorder=0)
    ax.grid(True,lw=0.4,zorder=0)
    ax.set_aspect("equal",adjustable="datalim")


# ══════════════════════════════════════════════
# Figure 1: 기존 5패널 (양팔 빨간 + 교집합 파란)
# ══════════════════════════════════════════════
def plot_figure1(chain, pts1, pts2, frames1, frames2,
                 slice_results, inter_pts, z_desk_m):
    vis=min(30_000,len(pts1))
    vp1=pts1[np.random.choice(len(pts1),vis,replace=False)]
    vp2=pts2[np.random.choice(len(pts2),vis,replace=False)]
    vi =inter_pts[np.random.choice(len(inter_pts),min(12_000,len(inter_pts)),replace=False)] if len(inter_pts)>0 else inter_pts

    jp1=np.array([f[:3,3] for f in frames1])
    jp2=np.array([f[:3,3] for f in frames2])

    fig=plt.figure(figsize=(18,11), num="Fig 1 — Dual Arm Workspace (5-panel)")
    gs =gridspec.GridSpec(2,3,figure=fig,left=0.05,right=0.97,
                           top=0.91,bottom=0.07,wspace=0.32,hspace=0.38)

    # ── 3D ──
    ax3=fig.add_subplot(gs[:,0],projection="3d")
    ax3.scatter(vp1[:,0],vp1[:,1],vp1[:,2],c="#CC2222",s=0.4,alpha=0.08,rasterized=True)
    ax3.scatter(vp2[:,0],vp2[:,1],vp2[:,2],c="#CC2222",s=0.4,alpha=0.08,rasterized=True)
    if len(vi)>0:
        ax3.scatter(vi[:,0],vi[:,1],vi[:,2],c="#1155CC",s=1.5,alpha=0.5,rasterized=True,label="Intersection")
    for (zv, v1, a1, v2, a2), col in zip(slice_results, SLICE_COLORS):
        if v1 is not None:
            cx = np.append(v1[:,0], v1[0,0])
            cy = np.append(v1[:,1], v1[0,1])
            ax3.plot(cx, cy, zs=zv/1000, zdir='z', color=col, lw=1.2, alpha=0.8,
                    label=f"Z={zv:.0f}mm")
    draw_joints(ax3,frames1,label_prefix="A1-")
    draw_joints(ax3,frames2,label_prefix="A2-")
    ax3.axhline  # dummy
    ax3.set_xlabel("X (m)",labelpad=3); ax3.set_ylabel("Y (m)",labelpad=3)
    ax3.set_zlabel("Z (m)",labelpad=3); ax3.set_title("3D Workspace (both arms)",pad=8)
    ax3.legend(loc="upper left",fontsize=6,markerscale=2)
    for p in (ax3.xaxis.pane,ax3.yaxis.pane,ax3.zaxis.pane):
        p.fill=False; p.set_edgecolor("#cccccc")
    ax3.grid(True,lw=0.3,color="#dddddd")

    # ── Top XY ──
    ax_xy=fig.add_subplot(gs[0,1])
    ax_xy.scatter(vp1[:,0]*1000,vp1[:,1]*1000,c="#CC2222",s=0.4,alpha=0.10,rasterized=True,label="Arm 1&2")
    ax_xy.scatter(vp2[:,0]*1000,vp2[:,1]*1000,c="#CC2222",s=0.4,alpha=0.10,rasterized=True)
    if len(vi)>0:
        ax_xy.scatter(vi[:,0]*1000,vi[:,1]*1000,c="#1155CC",s=0.8,alpha=0.3,rasterized=True,label="Intersection")
    ax_xy.scatter(jp1[:,0]*1000,jp1[:,1]*1000,c=JCOLORS[:len(jp1)],s=30,zorder=6,edgecolors="#333333",linewidths=0.4)
    ax_xy.scatter(jp2[:,0]*1000,jp2[:,1]*1000,c=JCOLORS[:len(jp2)],s=30,zorder=6,edgecolors="#333333",linewidths=0.4)
    ax_xy.legend(fontsize=7,markerscale=4)
    style2d(ax_xy,"X (mm)","Y (mm)","Top View (XY)")

    # ── Front XZ ──
    ax_xz=fig.add_subplot(gs[0,2])
    ax_xz.scatter(vp1[:,0]*1000,vp1[:,2]*1000,c="#CC2222",s=0.4,alpha=0.10,rasterized=True)
    ax_xz.scatter(vp2[:,0]*1000,vp2[:,2]*1000,c="#CC2222",s=0.4,alpha=0.10,rasterized=True)
    if len(vi)>0:
        ax_xz.scatter(vi[:,0]*1000,vi[:,2]*1000,c="#1155CC",s=0.8,alpha=0.25,rasterized=True)
    for (zv,v1,a1,v2,a2),col in zip(slice_results,SLICE_COLORS):
        ax_xz.axhline(zv,color=col,lw=0.9,ls="--",alpha=0.8,label=f"Z={zv:.0f}mm")
    ax_xz.axhline(z_desk_m*1000,color="#226622",lw=1.3,ls="-",label=f"Desk Z={z_desk_m*1000:.0f}mm")
    ax_xz.scatter(jp1[:,0]*1000,jp1[:,2]*1000,c=JCOLORS[:len(jp1)],s=30,zorder=6,edgecolors="#333333",linewidths=0.4)
    ax_xz.scatter(jp2[:,0]*1000,jp2[:,2]*1000,c=JCOLORS[:len(jp2)],s=30,zorder=6,edgecolors="#333333",linewidths=0.4)
    ax_xz.legend(fontsize=6,loc="upper right")
    style2d(ax_xz,"X (mm)","Z (mm)","Front View (XZ)")

    # ── Side YZ ──
    ax_yz=fig.add_subplot(gs[1,1])
    ax_yz.scatter(vp1[:,1]*1000,vp1[:,2]*1000,c="#CC2222",s=0.4,alpha=0.10,rasterized=True)
    ax_yz.scatter(vp2[:,1]*1000,vp2[:,2]*1000,c="#CC2222",s=0.4,alpha=0.10,rasterized=True)
    if len(vi)>0:
        ax_yz.scatter(vi[:,1]*1000,vi[:,2]*1000,c="#1155CC",s=0.8,alpha=0.25,rasterized=True)
    ax_yz.axhline(z_desk_m*1000,color="#226622",lw=1.3,ls="-",label=f"Desk Z")
    ax_yz.scatter(jp1[:,1]*1000,jp1[:,2]*1000,c=JCOLORS[:len(jp1)],s=30,zorder=6,edgecolors="#333333",linewidths=0.4)
    ax_yz.scatter(jp2[:,1]*1000,jp2[:,2]*1000,c=JCOLORS[:len(jp2)],s=30,zorder=6,edgecolors="#333333",linewidths=0.4)
    ax_yz.legend(fontsize=7)
    style2d(ax_yz,"Y (mm)","Z (mm)","Side View (YZ)")

    # ── Z 슬라이스 단면 ──
    ax_sl=fig.add_subplot(gs[1,2])
    for (zv, v1, a1, v2, a2), col in zip(slice_results, SLICE_COLORS):
        if v1 is not None:
            cl1 = np.vstack([v1, v1[0]])
            ax_sl.fill(cl1[:,0]*1000, cl1[:,1]*1000, alpha=0.12, color=col)
            ax_sl.plot(cl1[:,0]*1000, cl1[:,1]*1000, color=col, lw=1.5,
                    label=f"Z={zv:.0f}mm  {a1*1e4:.0f}cm²")
        if v2 is not None:
            cl2 = np.vstack([v2, v2[0]])
            ax_sl.fill(cl2[:,0]*1000, cl2[:,1]*1000, alpha=0.12, color=col)
            ax_sl.plot(cl2[:,0]*1000, cl2[:,1]*1000, color=col, lw=1.5, ls="--")
    ax_sl.axhline(0,color="#bbbbbb",lw=0.5,ls="--")
    ax_sl.axvline(0,color="#bbbbbb",lw=0.5,ls="--")
    ax_sl.set_xlabel("X (mm)"); ax_sl.set_ylabel("Y (mm)")
    ax_sl.set_title("Cross-Sections")
    ax_sl.legend(fontsize=7); ax_sl.set_aspect("equal",adjustable="datalim")
    ax_sl.grid(True,lw=0.4)

    r1=np.sqrt((pts1**2).sum(axis=1))
    active=[j for j in chain if j.jtype in ("revolute","continuous","prismatic")]
    fig.suptitle(
        f"arm_ver_3  —  {len(active)}-DOF  Dual Arm (XZ mirror)  "
        f"({len(pts1):,} samples/arm)  |  Desk Z={z_desk_m*1000:.0f}mm  "
        f"|  Max reach={r1.max()*1000:.0f}mm",
        fontsize=11,fontweight="bold",color="#111111")
    return fig


# ══════════════════════════════════════════════
# Figure 2: 3D + 단일 Z 단면 (arm1/arm2/교집합/책상)
# ══════════════════════════════════════════════
def plot_figure2(pts1, pts2, frames1, frames2,
                 z_desk_m, band=0.025):

    # --- 슬라이스 계산 ---
    verts1, area1 = slice_hull(pts1, z_desk_m, band)
    verts2, area2 = slice_hull(pts2, z_desk_m, band)

    inter_verts, area_inter = (None, 0.0)
    if verts1 is not None and verts2 is not None:
        inter_verts, area_inter = hull_intersection_2d(verts1, verts2)

    print(f"\n  Z={z_desk_m*1000:.0f}mm 단면:")
    print(f"    Arm 1 area  : {area1*1e4:.1f} cm²")
    print(f"    Arm 2 area  : {area2*1e4:.1f} cm²")
    print(f"    Intersection: {area_inter*1e4:.1f} cm²")

    vis=min(25_000,len(pts1))
    vp1=pts1[np.random.choice(len(pts1),vis,replace=False)]
    vp2=pts2[np.random.choice(len(pts2),vis,replace=False)]
    jp1=np.array([f[:3,3] for f in frames1])
    jp2=np.array([f[:3,3] for f in frames2])

    fig=plt.figure(figsize=(17,8), num="Fig 2 — Cross-Section + Desk")
    gs =gridspec.GridSpec(1,2,figure=fig,left=0.04,right=0.97,
                           top=0.90,bottom=0.07,wspace=0.30)

    # ─────────────────────────────────────────
    # 왼쪽: 3D (양팔 + 책상 면 + 관절점)
    # ─────────────────────────────────────────
    ax3=fig.add_subplot(gs[0,0],projection="3d")
    ax3.scatter(vp1[:,0],vp1[:,1],vp1[:,2],c="#CC2222",s=0.4,alpha=0.07,
                rasterized=True,label="Arm 1 & 2")
    ax3.scatter(vp2[:,0],vp2[:,1],vp2[:,2],c="#CC2222",s=0.4,alpha=0.07,rasterized=True)

    # 책상 면 (초록) - Z=z_desk 평면
    all_xy=np.vstack([vp1[:,:2], vp2[:,:2]])
    xmn,ymn=all_xy.min(axis=0)-0.03; xmx,ymx=all_xy.max(axis=0)+0.03
    desk_c=np.array([[xmn,ymn,z_desk_m],[xmx,ymn,z_desk_m],
                      [xmx,ymx,z_desk_m],[xmn,ymx,z_desk_m]])
    desk_poly=Poly3DCollection([list(zip(desk_c[:,0],desk_c[:,1],desk_c[:,2]))],
                                facecolor="#44BB44",edgecolor="#228822",alpha=0.30,lw=0.8)
    ax3.add_collection3d(desk_poly)
    cx_d=np.append(desk_c[:,0],desk_c[0,0]); cy_d=np.append(desk_c[:,1],desk_c[0,1])
    ax3.plot(cx_d,cy_d,zs=z_desk_m,zdir='z',color="#228822",lw=1.5,
             label=f"Desk Z={z_desk_m*1000:.0f}mm")

    draw_joints(ax3,frames1,size=35)
    draw_joints(ax3,frames2,size=35)

    ax3.set_xlabel("X (m)",labelpad=3); ax3.set_ylabel("Y (m)",labelpad=3)
    ax3.set_zlabel("Z (m)",labelpad=3)
    ax3.set_title("3D: Both Arms + Desk Plane",pad=8)
    ax3.legend(loc="upper left",fontsize=7,markerscale=2)
    for p in (ax3.xaxis.pane,ax3.yaxis.pane,ax3.zaxis.pane):
        p.fill=False; p.set_edgecolor("#cccccc")
    ax3.grid(True,lw=0.3,color="#dddddd")

    # ─────────────────────────────────────────
    # 오른쪽: XY 단면 (Z=z_desk)
    # ─────────────────────────────────────────
    ax2=fig.add_subplot(gs[0,1])

    # 책상 전체 배경 (연한 초록)
    desk_bg=MplPolygon(desk_c[:,:2]*1000,closed=True,
                        facecolor="#ddffdd",edgecolor="none",alpha=0.6,zorder=0)
    ax2.add_patch(desk_bg)

    # Arm 1 영역 (빨간 테두리, 연한 붉은 채움)
    if verts1 is not None:
        cl1=np.vstack([verts1,verts1[0]])
        ax2.fill(cl1[:,0]*1000,cl1[:,1]*1000,color="#CC2222",alpha=0.12,zorder=2,
                 label=f"Arm 1  ({area1*1e4:.0f} cm²)")
        ax2.plot(cl1[:,0]*1000,cl1[:,1]*1000,color="#CC2222",lw=2.0,zorder=3)

    # Arm 2 영역 (미러, 같은 빨간)
    if verts2 is not None:
        cl2=np.vstack([verts2,verts2[0]])
        ax2.fill(cl2[:,0]*1000,cl2[:,1]*1000,color="#CC2222",alpha=0.12,zorder=2,
                 label=f"Arm 2  ({area2*1e4:.0f} cm²)")
        ax2.plot(cl2[:,0]*1000,cl2[:,1]*1000,color="#CC2222",lw=2.0,zorder=3,ls="--")

    # 교집합 (파란 채움 + 테두리)
    if inter_verts is not None and len(inter_verts)>2:
        cl_i=np.vstack([inter_verts,inter_verts[0]])
        ax2.fill(cl_i[:,0]*1000,cl_i[:,1]*1000,color="#1155CC",alpha=0.40,zorder=4,
                 label=f"Intersection  ({area_inter*1e4:.0f} cm²)")
        ax2.plot(cl_i[:,0]*1000,cl_i[:,1]*1000,color="#1155CC",lw=2.0,zorder=5)

    # 책상 테두리
    desk_bdr=MplPolygon(desk_c[:,:2]*1000,closed=True,
                         facecolor="none",edgecolor="#228822",lw=1.5,
                         ls="-",zorder=6,label=f"Desk plane")
    ax2.add_patch(desk_bdr)

    # 관절 점 (top view)
    ax2.scatter(jp1[:,0]*1000,jp1[:,1]*1000,c=JCOLORS[:len(jp1)],
                s=40,zorder=8,edgecolors="#333333",linewidths=0.5)
    ax2.scatter(jp2[:,0]*1000,jp2[:,1]*1000,c=JCOLORS[:len(jp2)],
                s=40,zorder=8,edgecolors="#333333",linewidths=0.5,marker="D")

    # 면적 annotation
    ax2.text(0.02,0.97,
             f"Z = {z_desk_m*1000:.0f} mm  (band ±{band*1000:.0f}mm)\n"
             f"Arm 1 : {area1*1e4:.1f} cm²\n"
             f"Arm 2 : {area2*1e4:.1f} cm²\n"
             f"∩ Overlap : {area_inter*1e4:.1f} cm²\n"
             f"Overlap ratio : {area_inter/max(min(area1,area2),1e-9)*100:.1f}%",
             transform=ax2.transAxes,va="top",ha="left",fontsize=9,
             bbox=dict(boxstyle="round,pad=0.4",facecolor="white",
                       edgecolor="#cccccc",alpha=0.9))

    ax2.axhline(0,color="#bbbbbb",lw=0.5,ls="--",zorder=1)
    ax2.axvline(0,color="#bbbbbb",lw=0.5,ls="--",zorder=1)
    ax2.set_xlabel("X (mm)"); ax2.set_ylabel("Y (mm)")
    ax2.set_title(f"XY Cross-Section at Z = {z_desk_m*1000:.0f} mm",fontsize=10,fontweight="bold")
    ax2.legend(fontsize=8,loc="lower right")
    ax2.set_aspect("equal",adjustable="datalim")
    ax2.grid(True,lw=0.4,zorder=0)

    fig.suptitle(
        f"arm_ver_3  —  Dual Arm Cross-Section & Desk  "
        f"|  Desk Z = {z_desk_m*1000:.0f} mm  "
        f"|  Intersection = {area_inter*1e4:.1f} cm²",
        fontsize=11,fontweight="bold",color="#111111")
    return fig


# ══════════════════════════════════════════════
# main
# ══════════════════════════════════════════════
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--urdf",    default="/home/lmh/ros2_ws/src/final_project/urdf/arm_3.urdf")
    ap.add_argument("--samples", type=int,   default=100_000)
    ap.add_argument("--j2_xy", type=float, default=200.0,
                    help="joint2 XY 수평거리 (base 원점 기준, mm)")
    ap.add_argument("--z_desk",  type=float, default=700,
                    help="단면 Z 높이 (mm). 미지정시 가장 넓은 단면 자동 선택")
    ap.add_argument("--save_dir",default=None)
    args=ap.parse_args()

    setup_style()
    t0=time.time()

    # URDF 수정 (joint_8 = j2_dist mm)
    tmp_path = load_and_modify_urdf(args.urdf, j2_xy_mm=args.j2_xy)
    joints=parse_urdf(tmp_path)
    chain =build_chain(joints)
    active=[j for j in chain if j.jtype in ("revolute","continuous","prismatic")]
    print(f"  DOF: {len(active)}")

    # Home FK + XY base 거리 출력
    q0=np.zeros(len(active))
    _,frames1=fk(chain,q0)
    frames2=mirror_frames(frames1)
    jp=np.array([f[:3,3] for f in frames1])
    print(f"\n  Home position (q=0, joint_8={args.j2_xy:.0f}mm):")
    names=["base","joint1","joint2","joint3","joint4","joint5","endpoint"]
    for i,(nm,p) in enumerate(zip(names,jp)):
        xy=np.sqrt(p[0]**2+p[1]**2)
        d3=np.linalg.norm(p)
        print(f"  {nm:10s} ({p[0]*1000:+7.1f},{p[1]*1000:+7.1f},{p[2]*1000:+7.1f})mm | XY={xy*1000:.1f}mm | 3D={d3*1000:.1f}mm")

    # 샘플링
    print(f"\n  Sampling ({args.samples:,})...")
    pts1=sample_workspace(chain, n=args.samples)
    pts2=mirror_y(pts1)
    print(f"  Done: {time.time()-t0:.1f}s")

    r1=np.sqrt((pts1**2).sum(axis=1))
    print(f"\n  Max reach: {r1.max()*1000:.1f}mm")

    # 가장 넓은 Z
    best_z, best_area=find_widest_z(pts1)
    print(f"  Widest Z: {best_z*1000:.1f}mm  ({best_area*1e4:.0f}cm²)")

    z_desk_m=(args.z_desk/1000.0) if args.z_desk else best_z
    print(f"  Desk Z: {z_desk_m*1000:.1f}mm {'(manual)' if args.z_desk else '(auto-widest)'}")

    # Figure 1용 슬라이스
    z_min,z_max=pts1[:,2].min()*1000, pts1[:,2].max()*1000
    slice_zs=np.linspace(z_min+(z_max-z_min)*0.15, z_max-(z_max-z_min)*0.15, 4)
    slice_results=[]
    print("\n  4-slice areas:")
    for zv in slice_zs:
        v1, a1 = slice_hull(pts1, zv/1000)
        v2, a2 = slice_hull(pts2, zv/1000)
        slice_results.append((zv, v1, a1, v2, a2))
        print(f"    Z={zv:+7.1f}mm  arm1={a1*1e4:.0f}cm²  arm2={a2*1e4:.0f}cm²")

    # 교집합 포인트 (Figure 1용, Z > z_desk)
    mask1=pts1[:,2]>z_desk_m; mask2=pts2[:,2]>z_desk_m
    
    def voxel_inter(pa, pb, vm=15.0):
        v=vm/1000.0; org=np.vstack([pa,pb]).min(axis=0)
        def to_v(p): return set(map(tuple,((p-org)/v).astype(int)))
        common=to_v(pa)&to_v(pb)
        if not common: return np.empty((0,3))
        return np.array(list(common))*v+org+v/2
    inter_pts=voxel_inter(pts1[mask1],pts2[mask2])
    print(f"\n  Z>{z_desk_m*1000:.0f}mm intersection voxels: {len(inter_pts)}")

    # Figure 생성
    fig1=plot_figure1(chain,pts1,pts2,frames1,frames2,slice_results,inter_pts,z_desk_m)
    fig2=plot_figure2(pts1,pts2,frames1,frames2,z_desk_m)

    # 저장
    sdir=args.save_dir or os.path.dirname(args.urdf)
    p1=os.path.join(sdir,"workspace_v6_fig1.png")
    p2=os.path.join(sdir,"workspace_v6_fig2.png")
    fig1.savefig(p1,dpi=180,bbox_inches="tight",facecolor="white")
    fig2.savefig(p2,dpi=180,bbox_inches="tight",facecolor="white")
    print(f"\n  Saved: {p1}\n  Saved: {p2}")
    print(f"  Total: {time.time()-t0:.1f}s")

    plt.show()
    os.unlink(tmp_path)

if __name__=="__main__":
    main()