# app.py
"""
DBSW Glass Engine — Local Flask API Gateway
Hosts WebGL UI and routes non-linear OpenSeesPy FEA requests.
"""

import sys
import traceback
from flask import Flask, render_template, request, jsonify

from glass_core.eet import GlassPly, Interlayer, EnhancedEffectiveThicknessCalculator
from glass_core.eurocode_19100 import Eurocode19100Verifier
from glass_core.opensees_engine import OpenSeesGlassSolver

app = Flask(__name__, template_folder=".", static_folder="static")

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/solve", methods=["POST"])
def solve():
    """
    Primary API Endpoint called by index.html when user clicks 'Execute Analysis'.
    """
    try:
        payload = request.get_json(force=True)
        
        # 1. Parse Geometry & Load Data
        Lx = float(payload.get("Lx", 2000.0))
        Ly = float(payload.get("Ly", 1000.0))
        pressure_kPa = float(payload.get("pressure_kPa", 1.5))
        temp_C = float(payload.get("temp_C", 20.0))
        load_duration = payload.get("load_duration", "gust")
        glass_type = payload.get("glass_type", "annealed")
        edge_finish = payload.get("edge_finish", "ground")
        
        # 2. Evaluate Viscoelastic Enhanced Effective Thickness (EET)
        t1 = float(payload.get("t1_mm", 6.0))
        t2 = float(payload.get("t2_mm", 6.0))
        t_int = float(payload.get("t_int_mm", 1.52))
        interlayer_type = payload.get("interlayer_type", "PVB_Standard")
        
        ply1 = GlassPly(thickness_mm=t1, glass_type=glass_type)
        ply2 = GlassPly(thickness_mm=t2, glass_type=glass_type)
        interlayer = Interlayer(thickness_mm=t_int, interlayer_type=interlayer_type)
        
        eet_result = EnhancedEffectiveThicknessCalculator.calculate_2ply(
            ply1, ply2, interlayer,
            span_mm=min(Lx, Ly),
            temp_C=temp_C,
            load_duration=load_duration
        )
        
        # 3. Execute Geometrically Non-Linear OpenSeesPy Shell Solve
        nx = int(payload.get("nx", 20))
        ny = int(payload.get("ny", 10))
        
        solver = OpenSeesGlassSolver(
            Lx=Lx, Ly=Ly,
            t_eff_bending=eet_result.h_ef_w,
            t_eff_membrane=eet_result.h_ef_w  # Updated post-mesh stress recovery
        )
        
        fea_result = solver.solve_pressure(nx=nx, ny=ny, pressure_kPa=pressure_kPa)
        
        # 4. Eurocode Post-Processing (Surface vs Edge Failure)
        # Extract peak surface stress and edge stress from FEA output
        sigma_surface_max = fea_result["max_surface_stress_MPa"]
        sigma_edge_max = fea_result["max_edge_stress_MPa"]
        
        ec_result = Eurocode19100Verifier.verify_pane(
            sigma_surface_MPa=sigma_surface_max,
            sigma_edge_MPa=sigma_edge_max,
            w_max_mm=fea_result["max_deflection_mm"],
            span_mm=min(Lx, Ly),
            glass_type=glass_type,
            edge_finish=edge_finish,
            load_duration=load_duration
        )
        
        return jsonify({
            "status": "completed",
            "eet": {
                "h_ef_w": eet_result.h_ef_w,
                "h_ef_sigma": eet_result.h_ef_sigma,
                "eta": eet_result.eta,
                "G_int_MPa": eet_result.G_int_MPa
            },
            "fea": {
                "converged": fea_result["converged"],
                "max_deflection_mm": fea_result["max_deflection_mm"],
                "nodes": fea_result["nodes"],
                "elements": fea_result["elements"],
                "displacements": fea_result["displacements"],
                "stresses_MPa": fea_result["stresses_MPa"]
            },
            "verification": {
                "f_gd_MPa": ec_result.f_gd_MPa,
                "sigma_max_MPa": ec_result.sigma_max_MPa,
                "utilisation_stress": ec_result.utilisation_stress,
                "pass_stress": ec_result.pass_stress,
                "w_limit_mm": ec_result.w_limit_mm,
                "utilisation_deflection": ec_result.utilisation_deflection,
                "pass_deflection": ec_result.pass_deflection,
                "governing_location": ec_result.location,
                "governing_limit_state": ec_result.governing_limit_state
            }
        })
        
    except Exception as err:
        traceback.print_exc()
        return jsonify({"status": "error", "message": str(err)}), 500

if __name__ == "__main__":
    print("Starting DBSW Structural Glass Engine (Option C Local API)...")
    print("Serving UI at http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=True)
