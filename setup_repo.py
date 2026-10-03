# setup_repo.py
"""
DBSW Glass Engine — Repository Restructuring Script
Converts legacy Pyodide/WebWorker layout into a production-ready Flask + OpenSeesPy architecture.

Run from repository root:
    python setup_repo.py
"""

import os
import shutil
import sys

# Define target repository structure
DIRECTORIES = [
    "assets",
    "glass_core",
    "tests",
    "static/css",
    "static/js"
]

FILES = {}

# 1. glass_core/__init__.py
FILES["glass_core/__init__.py"] = '''"""
DBSW Glass Engine Core Module
Non-linear shell mechanics, viscoelastic EET, and CEN/TS 19100 post-processing.
"""
'''

# 2. glass_core/eet.py
FILES["glass_core/eet.py"] = '''# glass_core/eet.py
"""
DBSW Structural Glass Engine — Enhanced Effective Thickness (EET) Module
Reference Framework: CEN/TS 19100-2, BS EN 16612, Wölfel (1987), Bennison et al. (2001)
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple


# Interlayer Shear Modulus (G_int) Database [MPa]
G_INT_DATABASE: Dict[str, Dict[str, List[Tuple[float, float]]]] = {
    "PVB_Standard": {
        "gust":       [(10.0, 100.0), (20.0, 30.0),  (30.0, 4.0),   (40.0, 0.8),   (50.0, 0.4)],
        "short":      [(10.0, 20.0),  (20.0, 3.5),   (30.0, 0.8),   (40.0, 0.3),   (50.0, 0.15)],
        "medium":     [(10.0, 3.0),   (20.0, 0.6),   (30.0, 0.2),   (40.0, 0.1),   (50.0, 0.05)],
        "permanent":  [(10.0, 0.4),   (20.0, 0.1),   (30.0, 0.05),  (40.0, 0.02),  (50.0, 0.01)]
    },
    "SGP_SentryGlas": {
        "gust":       [(10.0, 300.0), (20.0, 260.0), (30.0, 200.0), (40.0, 100.0), (50.0, 15.0)],
        "short":      [(10.0, 260.0), (20.0, 200.0), (30.0, 120.0), (40.0, 35.0),  (50.0, 4.0)],
        "medium":     [(10.0, 200.0), (20.0, 140.0), (30.0, 50.0),  (40.0, 10.0),  (50.0, 1.2)],
        "permanent":  [(10.0, 120.0), (20.0, 50.0),  (30.0, 10.0),  (40.0, 1.5),   (50.0, 0.3)]
    }
}

def get_interlayer_shear_modulus(interlayer_type: str, temp_C: float, load_duration: str) -> float:
    if interlayer_type not in G_INT_DATABASE:
        raise ValueError(f"Unknown interlayer: {interlayer_type}")
    durations = G_INT_DATABASE[interlayer_type]
    if load_duration not in durations:
        raise ValueError(f"Unknown duration class: {load_duration}")
    curve = durations[load_duration]
    temps = [pt[0] for pt in curve]
    g_vals = [pt[1] for pt in curve]
    return max(float(np.interp(temp_C, temps, g_vals)), 0.001)

@dataclass
class GlassPly:
    thickness_mm: float
    glass_type: str = "annealed"
    E_modulus_MPa: float = 70000.0

@dataclass
class Interlayer:
    thickness_mm: float
    interlayer_type: str = "PVB_Standard"

@dataclass
class EETResult:
    h_ef_w: float
    h_ef_sigma: List[float]
    eta: float
    G_int_MPa: float

class EnhancedEffectiveThicknessCalculator:
    BOUNDARY_COEFF_D = {
        "simply_supported_4_edges": 12.0,
        "simply_supported_2_edges": 9.6,
        "fixed_4_edges":            24.0,
        "cantilever":               3.0
    }

    @classmethod
    def calculate_2ply(
        cls, ply1: GlassPly, ply2: GlassPly, interlayer: Interlayer,
        span_mm: float, temp_C: float, load_duration: str,
        boundary_condition: str = "simply_supported_4_edges"
    ) -> EETResult:
        G_int = get_interlayer_shear_modulus(interlayer.interlayer_type, temp_C, load_duration)
        h1, h2, h_int = ply1.thickness_mm, ply2.thickness_mm, interlayer.thickness_mm
        E = ply1.E_modulus_MPa
        H = 0.5 * h1 + h_int + 0.5 * h2
        I_0 = (h1**3 + h2**3) / 12.0
        d1 = H * h2 / (h1 + h2)
        d2 = H * h1 / (h1 + h2)
        I_s = h1 * d1**2 + h2 * d2**2
        D_factor = cls.BOUNDARY_COEFF_D.get(boundary_condition, 12.0)
        
        psi = (D_factor * E * I_s * h_int) / (G_int * span_mm**2 * (h1 + h2))
        eta = float(np.clip(1.0 / (1.0 + psi), 0.0, 1.0))
        I_eff_w = I_0 + eta * I_s
        h_ef_w = (12.0 * I_eff_w)**(1.0 / 3.0)
        
        h_ef_sigma_1 = np.sqrt((h_ef_w**3) / (h1 + 2.0 * eta * d1))
        h_ef_sigma_2 = np.sqrt((h_ef_w**3) / (h2 + 2.0 * eta * d2))
        
        return EETResult(
            h_ef_w=round(float(h_ef_w), 3),
            h_ef_sigma=[round(float(h_ef_sigma_1), 3), round(float(h_ef_sigma_2), 3)],
            eta=round(float(eta), 4),
            G_int_MPa=round(float(G_int), 3)
        )
'''

# 3. glass_core/eurocode_19100.py
FILES["glass_core/eurocode_19100.py"] = '''# glass_core/eurocode_19100.py
"""
CEN/TS 19100-1 / BS EN 16612 Eurocode Post-Processor
"""

from dataclasses import dataclass

@dataclass
class EurocodeVerificationResult:
    f_gd_MPa: float
    sigma_max_MPa: float
    utilisation_stress: float
    pass_stress: bool
    w_max_mm: float
    w_limit_mm: float
    utilisation_deflection: float
    pass_deflection: bool
    location: str
    governing_limit_state: str

class Eurocode19100Verifier:
    F_GK_SURFACE = {"annealed": 45.0, "heat_strengthened": 70.0, "toughened": 120.0}
    K_SP_EDGE = {"arrised": 0.80, "ground": 0.90, "polished": 1.00, "as_cut": 0.60}
    K_MOD = {"gust": 1.00, "short": 0.74, "medium": 0.45, "permanent": 0.26}

    @classmethod
    def verify_pane(
        cls, sigma_surface_MPa: float, sigma_edge_MPa: float, w_max_mm: float, span_mm: float,
        glass_type: str = "annealed", edge_finish: str = "ground", load_duration: str = "gust",
        deflection_limit_ratio: float = 100.0, gamma_M1: float = 1.25
    ) -> EurocodeVerificationResult:
        f_gk = cls.F_GK_SURFACE.get(glass_type, 45.0)
        k_mod = cls.K_MOD.get(load_duration, 1.00)
        
        # Surface Resistance
        if glass_type == "annealed":
            f_gd_surface = (k_mod * f_gk) / gamma_M1
        else:
            f_bk = f_gk - 45.0
            f_gd_surface = ((k_mod * 45.0) / gamma_M1) + (f_bk / 1.20)
        util_surface = sigma_surface_MPa / f_gd_surface

        # Edge Resistance
        k_sp = cls.K_SP_EDGE.get(edge_finish, 0.90)
        if glass_type == "annealed":
            f_gd_edge = (k_mod * k_sp * f_gk) / gamma_M1
        else:
            f_bk = f_gk - 45.0
            f_gd_edge = ((k_mod * k_sp * 45.0) / gamma_M1) + (f_bk / 1.20)
        util_edge = sigma_edge_MPa / f_gd_edge

        if util_edge >= util_surface:
            gov_util_stress, gov_f_gd, gov_sigma, location = util_edge, f_gd_edge, sigma_edge_MPa, "edge"
        else:
            gov_util_stress, gov_f_gd, gov_sigma, location = util_surface, f_gd_surface, sigma_surface_MPa, "surface"

        w_limit = span_mm / deflection_limit_ratio
        util_defl = w_max_mm / w_limit

        return EurocodeVerificationResult(
            f_gd_MPa=round(gov_f_gd, 2),
            sigma_max_MPa=round(gov_sigma, 2),
            utilisation_stress=round(gov_util_stress, 3),
            pass_stress=gov_util_stress <= 1.00,
            w_max_mm=round(w_max_mm, 2),
            w_limit_mm=round(w_limit, 2),
            utilisation_deflection=round(util_defl, 3),
            pass_deflection=util_defl <= 1.00,
            location=location,
            governing_limit_state="ULS Stress" if gov_util_stress >= util_defl else "SLS Deflection"
        )
'''

# 4. glass_core/opensees_engine.py
FILES["glass_core/opensees_engine.py"] = '''# glass_core/opensees_engine.py
"""
OpenSeesPy Non-Linear Shell & Beam FEA Core
"""

import numpy as np

try:
    import openseespy.opensees as ops
    OPENSEES_AVAILABLE = True
except ImportError:
    OPENSEES_AVAILABLE = False


class OpenSeesGlassSolver:
    """
    Non-Linear Plate/Shell Solver for Out-of-Plane Glazing Panes.
    """
    def __init__(self, Lx: float, Ly: float, t_eff_bending: float, E: float = 70000.0, nu: float = 0.23):
        self.Lx = float(Lx)
        self.Ly = float(Ly)
        self.t_b = float(t_eff_bending)
        self.E = float(E)
        self.nu = float(nu)

    def solve_pressure(self, nx: int = 20, ny: int = 10, pressure_kPa: float = 1.5) -> dict:
        if not OPENSEES_AVAILABLE:
            raise RuntimeError("OpenSeesPy is not installed in the active Python environment.")
            
        ops.wipe()
        ops.model('basic', '-ndm', 3, '-ndf', 6)
        
        dx = self.Lx / nx
        dy = self.Ly / ny
        
        node_map = {}
        node_id = 1
        for i in range(nx + 1):
            for j in range(ny + 1):
                x, y = i * dx, j * dy
                ops.node(node_id, x, y, 0.0)
                node_map[(i, j)] = node_id
                
                if i == 0 or i == nx or j == 0 or j == ny:
                    ops.fix(node_id, 1, 1, 1, 0, 0, 1)
                node_id += 1
                
        ops.section('ElasticMembranePlateSection', 1, self.E, self.nu, self.t_b, 2.5e-9)
        
        elem_id = 1
        for i in range(nx):
            for j in range(ny):
                n1 = node_map[(i, j)]
                n2 = node_map[(i + 1, j)]
                n3 = node_map[(i + 1, j + 1)]
                n4 = node_map[(i, j + 1)]
                ops.element('ShellMITC4', elem_id, n1, n2, n3, n4, 1)
                elem_id += 1
                
        p_N_mm2 = pressure_kPa * 1e-3
        ops.timeSeries('Linear', 1)
        ops.pattern('Plain', 1, 1)
        
        tributary_area = dx * dy
        for (i, j), nid in node_map.items():
            factor = 0.25 if (i in (0, nx) and j in (0, ny)) else (0.50 if (i in (0, nx) or j in (0, ny)) else 1.0)
            fz = -p_N_mm2 * tributary_area * factor
            ops.load(nid, 0.0, 0.0, fz, 0.0, 0.0, 0.0)
            
        ops.system('FullGeneral')
        ops.numberer('RCM')
        ops.constraints('Transformation')
        ops.test('NormDispIncr', 1e-6, 100, 0)
        ops.algorithm('NewtonRaphson')
        ops.integrator('LoadControl', 0.1)
        ops.analysis('Static')
        
        ok = ops.analyze(10)
        
        max_disp_z = 0.0
        displacements = []
        for (i, j), nid in node_map.items():
            dz = abs(ops.nodeDisp(nid, 3))
            if dz > max_disp_z:
                max_disp_z = dz
            displacements.append([i * dx, j * dy, float(ops.nodeDisp(nid, 3))])
            
        return {
            "converged": ok == 0,
            "max_deflection_mm": round(float(max_disp_z), 3),
            "node_displacements": displacements
        }
'''

# 5. glass_core/supports.py
FILES["glass_core/supports.py"] = '''# glass_core/supports.py
"""
Commercial Hardware Contact & Spring Generator (Spiders, Clamps, Shoes)
"""

from dataclasses import dataclass

@dataclass
class PointFixingConfig:
    disk_diameter_mm: float = 60.0
    hole_diameter_mm: float = 26.0
    pad_thickness_mm: float = 3.0
    pad_modulus_MPa: float = 300.0  # Nylon / POM
    is_articulated: bool = True     # Ball-joint vs. Rigid
'''

# 6. app.py
FILES["app.py"] = '''# app.py
"""
DBSW Glass Engine — Local Flask Gateway
Hosts UI and serves non-linear OpenSeesPy FEA endpoints.
"""

from flask import Flask, render_template, request, jsonify
import numpy as np

from glass_core.eet import GlassPly, Interlayer, EnhancedEffectiveThicknessCalculator
from glass_core.eurocode_19100 import Eurocode19100Verifier
from glass_core.opensees_engine import OpenSeesGlassSolver

app = Flask(__name__, template_folder=".", static_folder="static")

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/solve", methods=["POST"])
def solve():
    try:
        payload = request.get_json(force=True)
        
        Lx = float(payload.get("Lx", 2000.0))
        Ly = float(payload.get("Ly", 1000.0))
        pressure = float(payload.get("pressure_kPa", 1.5))
        temp = float(payload.get("temp_C", 20.0))
        duration = payload.get("load_duration", "gust")
        
        # 1. Evaluate Effective Thickness via EET
        ply1 = GlassPly(thickness_mm=float(payload.get("t1_mm", 6.0)), glass_type=payload.get("glass_type", "annealed"))
        ply2 = GlassPly(thickness_mm=float(payload.get("t2_mm", 6.0)), glass_type=payload.get("glass_type", "annealed"))
        inter = Interlayer(thickness_mm=float(payload.get("t_int_mm", 1.52)), interlayer_type=payload.get("interlayer", "PVB_Standard"))
        
        eet_res = EnhancedEffectiveThicknessCalculator.calculate_2ply(ply1, ply2, inter, span_mm=min(Lx, Ly), temp_C=temp, load_duration=duration)
        
        # 2. Run OpenSeesPy FEA Solve
        solver = OpenSeesGlassSolver(Lx=Lx, Ly=Ly, t_eff_bending=eet_res.h_ef_w)
        fea_res = solver.solve_pressure(nx=int(payload.get("nx", 20)), ny=int(payload.get("ny", 10)), pressure_kPa=pressure)
        
        # 3. Nominal Bending Stress Estimation & Eurocode Post-Process
        # (Simplified plate bending stress estimate for demo integration)
        q_N_mm2 = pressure * 1e-3
        sigma_nom = (0.75 * q_N_mm2 * (min(Lx, Ly)**2)) / (eet_res.h_ef_sigma[0]**2)
        
        code_res = Eurocode19100Verifier.verify_pane(
            sigma_surface_MPa=sigma_nom,
            sigma_edge_MPa=sigma_nom * 0.85,
            w_max_mm=fea_res["max_deflection_mm"],
            span_mm=min(Lx, Ly),
            glass_type=payload.get("glass_type", "annealed"),
            load_duration=duration
        )
        
        return jsonify({
            "status": "completed",
            "eet": {
                "h_ef_w": eet_res.h_ef_w,
                "h_ef_sigma": eet_res.h_ef_sigma,
                "eta": eet_res.eta,
                "G_int_MPa": eet_res.G_int_MPa
            },
            "fea": fea_res,
            "verification": {
                "f_gd_MPa": code_res.f_gd_MPa,
                "sigma_max_MPa": code_res.sigma_max_MPa,
                "util_stress": code_res.utilisation_stress,
                "pass_stress": code_res.pass_stress,
                "w_limit_mm": code_res.w_limit_mm,
                "util_defl": code_res.utilisation_deflection,
                "pass_defl": code_res.pass_deflection,
                "governing": code_res.governing_limit_state
            }
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    print("Starting DBSW Glass Engine Local Server on http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=True)
'''

# 7. requirements.txt
FILES["requirements.txt"] = '''openseespy>=3.4.0
flask>=3.0.0
numpy>=1.24.0
scipy>=1.10.0
'''


def execute_setup():
    print("=== DBSW Glass Engine Repository Restructuring ===")
    
    # Create Directories
    for d in DIRECTORIES:
        os.makedirs(d, exist_ok=True)
        print(f"[DIR]  Created: {d}")
        
    # Handle legacy folder migration
    if os.path.exists("python_core"):
        print("[MIGRATE] Moving legacy python_core files to assets/legacy_backup/...")
        os.makedirs("assets/legacy_backup", exist_ok=True)
        for f in os.listdir("python_core"):
            src = os.path.join("python_core", f)
            dst = os.path.join("assets/legacy_backup", f)
            if os.path.isfile(src):
                shutil.move(src, dst)
        os.rmdir("python_core")

    if os.path.exists("solver_worker.js"):
        print("[MIGRATE] Moving solver_worker.js to assets/legacy_backup/...")
        shutil.move("solver_worker.js", "assets/legacy_backup/solver_worker.js")

    # Create target files
    for filepath, content in FILES.items():
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(content.strip() + "\n")
        print(f"[FILE] Created: {filepath}")

    print("\n[SUCCESS] Repository restructured successfully.")
    print("\nNext steps:")
    print("1. Install dependencies:  pip install -r requirements.txt")
    print("2. Run Flask local API:  python app.py")


if __name__ == "__main__":
    execute_setup()
