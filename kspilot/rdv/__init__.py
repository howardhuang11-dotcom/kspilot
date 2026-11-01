"""Rendezvous & docking (R0 of docking concept study): offline orbital mechanics, CW relative motion,
terminal RCS docking guidance + simulator, and the sortie propellant budget. Pure numpy, no kRPC.

    orbit    Kerbin constants, Hohmann / phasing rendezvous planner, Lambert solver
    cw       Clohessy-Wiltshire (Hill) relative motion: STM, targeting, ECI <-> LVLH
    docking  terminal approach + docking guidance law and a 6-DOF-lite plant with disturbances
    budget   whole-sortie propellant budget of the recoverable core (from flown logs)

Results and assumptions: docking R0 results. Monte Carlo runner: tools/rdv_mc.py.
"""
