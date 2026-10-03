# test/test_app.py
import pytest
from glass_core.eet import GlassPly, Interlayer, EnhancedEffectiveThicknessCalculator
from glass_core.eurocode_19100 import Eurocode19100Verifier
from glass_core.opensees_engine import OpenSeesGlassSolver

def test_full_pipeline_execution():
    # 1. EET Check
    ply1 = GlassPly(thickness_mm=8.0, glass_type="annealed")
    ply2 = GlassPly(thickness_mm=8.0, glass_type="annealed")
    pvb = Interlayer(thickness_mm=1.52, interlayer_type="PVB_Standard")
    
    eet = EnhancedEffectiveThicknessCalculator.calculate_2ply(
        ply1, ply2, pvb, span_mm=1500.0, temp_C=20.0, load_duration="gust"
    )
    assert eet.h_ef_w > 8.0
    assert eet.h_ef_w <= 17.52

    # 2. OpenSeesPy Solve Check
    solver = OpenSeesGlassSolver(Lx=2000.0, Ly=1500.0, t_eff_bending=eet.h_ef_w)
    res = solver.solve_pressure(nx=12, ny=8, pressure_kPa=1.2)
    
    assert res["converged"] is True
    assert res["max_deflection_mm"] > 0.0

    # 3. Eurocode 19100 Verification
    verif = Eurocode19100Verifier.verify_pane(
        sigma_surface_MPa=res["max_surface_stress_MPa"],
        sigma_edge_MPa=res["max_edge_stress_MPa"],
        w_max_mm=res["max_deflection_mm"],
        span_mm=1500.0,
        glass_type="annealed",
        edge_finish="ground",
        load_duration="gust"
    )
    
    assert verif.utilisation_stress > 0.0
    assert verif.f_gd_MPa > 0.0
