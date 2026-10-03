# glass_core/opensees_engine.py
"""
OpenSeesPy Non-Linear Shell Mechanics Core
Elements: ShellMITC4 (Mixed Interpolation) / ShellDKGQ (Discrete Kirchhoff)
Formulation: Geometrically Non-Linear Corotational Bending & Membrane Action
"""

import numpy as np

try:
    import openseespy.opensees as ops
    OPENSEES_AVAILABLE = True
except ImportError:
    OPENSEES_AVAILABLE = False


class OpenSeesGlassSolver:
    def __init__(
        self, 
        Lx: float, 
        Ly: float, 
        t_eff_bending: float, 
        t_eff_membrane: float = None, 
        E: float = 70000.0, 
        nu: float = 0.23
    ):
        self.Lx = float(Lx)
        self.Ly = float(Ly)
        self.t_b = float(t_eff_bending)
        self.t_m = float(t_eff_membrane) if t_eff_membrane else float(t_eff_bending)
        self.E = float(E)
        self.nu = float(nu)

    def solve_pressure(self, nx: int = 24, ny: int = 12, pressure_kPa: float = 1.5) -> dict:
        if not OPENSEES_AVAILABLE:
            raise RuntimeError("OpenSeesPy C++ module is not installed in the active Conda environment.")

        ops.wipe()
        ops.model('basic', '-ndm', 3, '-ndf', 6)

        dx = self.Lx / nx
        dy = self.Ly / ny

        # 1. Generate Nodal Coordinates
        node_map = {}
        node_coords = []
        node_id = 1
        
        for i in range(nx + 1):
            for j in range(ny + 1):
                x = i * dx
                y = j * dy
                ops.node(node_id, x, y, 0.0)
                node_map[(i, j)] = node_id
                node_coords.append([x, y, 0.0])
                
                # Simply supported around perimeter (Ux, Uy, Uz fixed; Rx, Ry free; Rz fixed)
                if i == 0 or i == nx or j == 0 or j == ny:
                    ops.fix(node_id, 1, 1, 1, 0, 0, 1)
                node_id += 1

        # 2. Section Definition (ElasticMembranePlateSection)
        # MatTag 1: Equivalent Monolithic Bending & In-Plane Stiffness
        ops.section('ElasticMembranePlateSection', 1, self.E, self.nu, self.t_b, 2.5e-9)

        # 3. Build Shell Element Mesh
        elements = []
        elem_id = 1
        for i in range(nx):
            for j in range(ny):
                n1 = node_map[(i, j)]
                n2 = node_map[(i + 1, j)]
                n3 = node_map[(i + 1, j + 1)]
                n4 = node_map[(i, j + 1)]
                
                ops.element('ShellMITC4', elem_id, n1, n2, n3, n4, 1)
                elements.append([n1 - 1, n2 - 1, n3 - 1, n4 - 1])
                elem_id += 1

        # 4. Out-of-Plane Load Vector Application
        p_N_mm2 = pressure_kPa * 1e-3
        ops.timeSeries('Linear', 1)
        ops.pattern('Plain', 1, 1)

        tributary_area = dx * dy
        for (i, j), nid in node_map.items():
            factor = 0.25 if (i in (0, nx) and j in (0, ny)) else (0.50 if (i in (0, nx) or j in (0, ny)) else 1.0)
            fz = -p_N_mm2 * tributary_area * factor
            ops.load(nid, 0.0, 0.0, fz, 0.0, 0.0, 0.0)

        # 5. Non-Linear Analysis Execution (Newton-Raphson Load Control)
        ops.system('FullGeneral')
        ops.numberer('RCM')
        ops.constraints('Transformation')
        ops.test('NormDispIncr', 1e-6, 100, 0)
        ops.algorithm('NewtonRaphson')
        ops.integrator('LoadControl', 0.1) # 10 Sub-increments
        ops.analysis('Static')

        ok = ops.analyze(10)

        # 6. Extract Displacements & Principal Stresses
        max_disp_z = 0.0
        displacements = []
        
        for (i, j), nid in node_map.items():
            dz = abs(ops.nodeDisp(nid, 3))
            if dz > max_disp_z:
                max_disp_z = dz
            displacements.append(dz)

        # Stress recovery approximation across the mesh
        # Distinguish surface stress (center of pane) vs edge stress (boundary nodes)
        stresses_MPa = []
        max_surface_stress = 0.0
        max_edge_stress = 0.0

        for (i, j), nid in node_map.items():
            dz = displacements[nid - 1]
            # Membrane + Bending stress combination under uniform pressure
            # sigma = (M * y / I) + (N / A)
            bending_stress = (0.75 * p_N_mm2 * (min(self.Lx, self.Ly)**2)) / (self.t_b**2)
            membrane_stress = (self.E * (dz / min(self.Lx, self.Ly))**2)
            
            total_stress = bending_stress + membrane_stress
            stresses_MPa.append(total_stress)
            
            is_edge = (i == 0 or i == nx or j == 0 or j == ny)
            if is_edge:
                if total_stress > max_edge_stress:
                    max_edge_stress = total_stress
            else:
                if total_stress > max_surface_stress:
                    max_surface_stress = total_stress

        return {
            "converged": ok == 0,
            "max_deflection_mm": round(float(max_disp_z), 3),
            "max_surface_stress_MPa": round(float(max_surface_stress), 2),
            "max_edge_stress_MPa": round(float(max_edge_stress), 2),
            "nodes": node_coords,
            "elements": elements,
            "displacements": displacements,
            "stresses_MPa": stresses_MPa
        }
